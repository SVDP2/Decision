import math
import struct
import threading
import time

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TwistWithCovarianceStamped
from platoon_interfaces.msg import LeaderDriveTelemetry
from platoon_v2v.telemetry_protocol import LeaderTelemetryFrame
from platoon_v2v.telemetry_protocol import LeaderTelemetryStreamParser
import rclpy
from rclpy.node import Node
import serial
from serial import SerialException
from std_msgs.msg import Bool
from std_msgs.msg import Float32


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.input_mode = self.declare_parameter('input_mode', 'direct').value
        self.ackermann_topic = self.declare_parameter(
            'ackermann_topic', '/ackermann_cmd'
        ).value
        self.throttle_topic = self.declare_parameter(
            'throttle_topic', '/throttle_cmd'
        ).value
        self.steer_topic = self.declare_parameter(
            'steer_topic', '/auto_steer_angle'
        ).value
        self.go_topic = self.declare_parameter('go_topic', '/go').value
        self.drive_telemetry_topic = self.declare_parameter(
            'drive_telemetry_topic', '/leader/drive_telemetry'
        ).value
        self.port_name = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baudrate = int(self.declare_parameter('baud', 115200).value)
        self.tx_period_sec = float(
            self.declare_parameter('tx_period_sec', 0.067).value
        )
        self.expect_bridge_debug_counters = bool(
            self.declare_parameter(
                'expect_bridge_debug_counters', True
            ).value
        )

        self.serial_port = None
        self.requested_speed = 0.0
        self.requested_angle = 0.0
        self.stop_override = False
        self.keep_running = True

        self.last_ros_time_ms = None
        self.last_arduino_time_ms = None
        self.ros_loop_interval = 0.0
        self.arduino_loop_interval = 0.0
        self.ros_loop_hz = 0.0
        self.arduino_loop_hz = 0.0
        self.steer_filtered_angle = 0.0
        self.steer_target_pwm = 0.0
        self.speed_filtered = 0.0
        self.speed_target_pwm = 0.0
        self.auto_speed = 0.0
        self.auto_angle = 0.0
        self.tx_packet_count = 0
        self.rx_status_packet_count = 0
        self.rx_raw_byte_count = 0
        self.rx_sync_drop_count = 0
        self.rx_packet_crc_fail_count = 0
        self.rx_unpack_error_count = 0
        self.rx_leader_telemetry_count = 0
        self.latest_leader_telemetry_frame = None
        self.last_warned_crc_fail_count = 0
        self.last_bridge_warning_time = 0.0
        self.telemetry_parser = LeaderTelemetryStreamParser()

        self.twist_msgs_pub = self.create_publisher(
            TwistWithCovarianceStamped, '/encoder/twist', 10
        )
        self.drive_telemetry_pub = self.create_publisher(
            LeaderDriveTelemetry, self.drive_telemetry_topic, 10
        )

        self.configure_subscriptions()
        self.open_serial_port()

        self.tx_timer = self.create_timer(
            self.tx_period_sec, self.send_serial_data
        )
        self.status_timer = self.create_timer(0.25, self.print_status)
        self.serial_thread = threading.Thread(
            target=self.read_serial_loop, daemon=True
        )
        self.serial_thread.start()

    def configure_subscriptions(self):
        input_mode = str(self.input_mode).strip().lower()
        if input_mode == 'ackermann':
            self.create_subscription(
                AckermannDrive,
                self.ackermann_topic,
                self.ackermann_callback,
                10,
            )
            self.get_logger().info(
                f'Using ackermann input from {self.ackermann_topic}'
            )
        else:
            if input_mode != 'direct':
                self.get_logger().warn(
                    'Unknown input_mode '
                    f"'{self.input_mode}', falling back to 'direct'."
                )
            self.create_subscription(
                Float32, self.throttle_topic, self.throttle_callback, 10
            )
            self.create_subscription(
                Float32, self.steer_topic, self.steer_callback, 10
            )
            self.get_logger().info(
                'Using direct inputs '
                f'throttle={self.throttle_topic}, steer={self.steer_topic}'
            )

        self.create_subscription(
            Bool, self.go_topic, self.stop_override_callback, 10
        )

    def open_serial_port(self):
        try:
            self.serial_port = serial.Serial(
                self.port_name,
                self.baudrate,
                timeout=0.1,
                write_timeout=0.1,
            )
            try:
                self.serial_port.exclusive = True
            except (AttributeError, ValueError, SerialException):
                pass

            # Mega 2560 opens with an auto-reset pulse. Give the sketch time
            # to boot, then drop any boot-time garbage before parsing packets.
            time.sleep(1.0)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.get_logger().info(
                f'Serial port opened: {self.port_name}@{self.baudrate}'
            )
        except Exception as exc:
            self.serial_port = None
            self.get_logger().warn(f'Failed to open serial port: {exc}')

    def reconnect_serial(self):
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        except Exception:
            pass
        self.serial_port = None
        time.sleep(0.5)
        self.open_serial_port()

    def ackermann_callback(self, msg: AckermannDrive):
        self.requested_angle = math.degrees(msg.steering_angle)
        self.requested_speed = float(msg.speed)

    def throttle_callback(self, msg: Float32):
        self.requested_speed = float(msg.data)

    def steer_callback(self, msg: Float32):
        self.requested_angle = float(msg.data)

    def stop_override_callback(self, msg: Bool):
        self.stop_override = bool(msg.data)

    def current_speed_command(self):
        return 0.0 if self.stop_override else self.requested_speed

    def bridge_cmd_ok_count(self):
        if not self.expect_bridge_debug_counters:
            return None
        if self.latest_leader_telemetry_frame is not None:
            return int(self.latest_leader_telemetry_frame.bridge_rx_ok_count)
        return max(0, int(round(self.auto_speed)))

    def bridge_crc_fail_count(self):
        if not self.expect_bridge_debug_counters:
            return None
        if self.latest_leader_telemetry_frame is not None:
            return int(self.latest_leader_telemetry_frame.bridge_crc_fail_count)
        return max(0, int(round(self.auto_angle)))

    def convert_to_nav_msgs(self, speed, angle):
        twist_stamped = TwistWithCovarianceStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'encoder'
        twist_stamped.twist.twist.linear.x = speed
        twist_stamped.twist.twist.angular.z = speed * math.tan(
            math.radians(angle)
        ) / 0.72
        twist_stamped.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
        ]
        self.twist_msgs_pub.publish(twist_stamped)

    def publish_drive_telemetry(self, frame: LeaderTelemetryFrame):
        msg = LeaderDriveTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'leader/base_link'
        msg.seq = int(frame.seq)
        msg.boot_id = int(frame.boot_id)
        msg.drive_mode = int(frame.drive_mode)
        msg.source = int(frame.source)
        msg.rc_valid = bool(frame.rc_valid)
        msg.rc_failsafe = bool(frame.rc_failsafe)
        msg.auto_valid = bool(frame.auto_valid)
        msg.auto_failsafe = bool(frame.auto_failsafe)
        msg.stop_required = bool(frame.stop_required)
        msg.rc_throttle_us = int(frame.rc_throttle_us)
        msg.rc_steer_us = int(frame.rc_steer_us)
        msg.rc_mode_us = int(frame.rc_mode_us)
        msg.throttle_norm = float(frame.throttle_norm)
        msg.throttle_cmd_pwm = float(frame.throttle_cmd_pwm)
        msg.throttle_output_pwm = float(frame.throttle_output_pwm)
        msg.steering_target_adc = float(frame.steering_target_adc)
        msg.steering_current_adc = float(frame.steering_current_adc)
        msg.steering_angle_rad = float(frame.steering_angle_rad)
        msg.speed_estimate_mps = float(frame.speed_estimate_mps)
        msg.auto_speed_cmd_mps = float(frame.auto_speed_cmd_mps)
        msg.auto_steering_cmd_rad = float(frame.auto_steering_cmd_rad)
        msg.arduino_time_ms = int(frame.arduino_time_ms)
        msg.bridge_rx_ok_count = int(frame.bridge_rx_ok_count)
        msg.bridge_crc_fail_count = int(frame.bridge_crc_fail_count)
        self.drive_telemetry_pub.publish(msg)

    def handle_leader_telemetry_bytes(self, data: bytes):
        for frame in self.telemetry_parser.feed(data):
            self.rx_leader_telemetry_count += 1
            self.latest_leader_telemetry_frame = frame
            self.publish_drive_telemetry(frame)

    def send_serial_data(self):
        if self.serial_port is None:
            self.open_serial_port()
            if self.serial_port is None:
                return

        try:
            speed = self.current_speed_command()
            angle = self.requested_angle
            header = b'\xAA\x55'
            payload = struct.pack('<ff', speed, angle)

            crc = 0
            for byte in payload:
                crc ^= byte

            packet = header + payload + bytes([crc])
            self.serial_port.write(packet)
            self.tx_packet_count += 1
        except Exception as exc:
            self.get_logger().warn(f'Serial write failed: {exc}')
            self.reconnect_serial()

    def maybe_warn_bridge_state(self):
        if not self.expect_bridge_debug_counters:
            return

        now = time.monotonic()
        if now - self.last_bridge_warning_time < 1.0:
            return

        cmd_ok = self.bridge_cmd_ok_count()
        crc_fail = self.bridge_crc_fail_count()
        if cmd_ok is None or crc_fail is None:
            return

        if (
            self.tx_packet_count >= 5
            and self.rx_raw_byte_count == 0
        ):
            self.get_logger().warn(
                'No bytes are arriving from Arduino on the serial bridge. '
                'Check whether another process is opening/resetting the port, '
                'whether the flashed board on this port is the expected Mega, '
                'and whether USB serial is stable.'
            )
            self.last_bridge_warning_time = now
            return

        if (
            self.rx_raw_byte_count > 0
            and self.rx_status_packet_count == 0
            and self.rx_leader_telemetry_count == 0
        ):
            self.get_logger().warn(
                'Raw bytes are arriving from Arduino, but no valid status '
                'packet has been decoded yet. Check packet sync/CRC and '
                'whether text output is mixed into the binary stream.'
            )
            self.last_bridge_warning_time = now
            return

        if (
            self.tx_packet_count >= 5
            and self.rx_status_packet_count >= 5
            and cmd_ok == 0
        ):
            self.get_logger().warn(
                'Arduino status packets are arriving, but bridge CMD_OK is '
                'still 0. The firmware is not accepting PC->Arduino command '
                'packets. Check the flashed sketch, serial port ownership, '
                'and command packet parsing.'
            )
            self.last_bridge_warning_time = now
            return

        if crc_fail > self.last_warned_crc_fail_count:
            self.get_logger().warn(
                'Arduino is reporting bridge CRC failures. Check baud rate, '
                'header/packet framing, and whether another process is '
                f'writing to {self.port_name}.'
            )
            self.last_warned_crc_fail_count = crc_fail
            self.last_bridge_warning_time = now

    def print_status(self):
        lines = [
            f"\n{'━' * 60}",
            '[LOOP]  | '
            f'ARDUINO: {self.arduino_loop_interval:>5.1f} ms '
            f'({self.arduino_loop_hz:>4.1f} hz) | '
            f'ROS2: {self.ros_loop_interval:>5.1f} ms '
            f'({self.ros_loop_hz:>4.1f} hz)',
            '[CMD]   | '
            f'SPEED: {self.current_speed_command():>6.2f} m/s | '
            f'ANGLE: {self.requested_angle:>6.2f} deg',
            '[STATE] | '
            f'SPEED: {self.speed_filtered:>6.2f} m/s | '
            f'ANGLE: {self.steer_filtered_angle:>6.2f} deg',
            '[PWM]   | '
            f'DRIVE_TGT: {self.speed_target_pwm:>6.1f} | '
            f'STEER_OUT: {self.steer_target_pwm:>6.1f}',
            '[AUTO]  | '
            f'SPEED: {self.auto_speed:>6.2f} | '
            f'ANGLE: {self.auto_angle:>6.2f}',
            '[V2V]   | '
            f'TELEMETRY_RX: {self.rx_leader_telemetry_count:>6} | '
            f'SYNC_DROP: {self.telemetry_parser.sync_drop_count:>6} | '
            f'CRC: {self.telemetry_parser.crc_fail_count:>6}',
        ]

        if self.expect_bridge_debug_counters:
            cmd_ok = self.bridge_cmd_ok_count()
            crc_fail = self.bridge_crc_fail_count()
            lines.append(
                '[BRIDGE]| '
                f'CMD_OK: {cmd_ok:>6} | '
                f'CRC_FAIL: {crc_fail:>6} | '
                f'TX: {self.tx_packet_count:>6} | '
                f'RX: {self.rx_status_packet_count:>6}'
            )
            lines.append(
                '[RXDBG] | '
                f'BYTES: {self.rx_raw_byte_count:>6} | '
                f'SYNC_DROP: {self.rx_sync_drop_count:>6} | '
                f'CRC: {self.rx_packet_crc_fail_count:>6} | '
                f'UNPACK: {self.rx_unpack_error_count:>6}'
            )

        lines.append(f"{'━' * 60}\n")
        self.get_logger().info('\n'.join(lines))
        self.maybe_warn_bridge_state()

    def read_serial_loop(self):
        header = b'\xAA\x55'
        packet_size = 31
        buffer = bytearray()

        while self.keep_running and rclpy.ok():
            if self.serial_port is None:
                time.sleep(0.1)
                continue

            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    self.rx_raw_byte_count += len(data)
                    self.handle_leader_telemetry_bytes(data)
                    buffer.extend(data)

                while len(buffer) >= packet_size:
                    header_index = buffer.find(header)
                    if header_index == -1:
                        self.rx_sync_drop_count += len(buffer)
                        buffer.clear()
                        break
                    if header_index > 0:
                        self.rx_sync_drop_count += header_index
                        del buffer[:header_index]

                    if len(buffer) < packet_size:
                        break

                    payload = buffer[2:packet_size - 1]
                    crc_received = buffer[packet_size - 1]
                    crc_calculated = 0
                    for byte in payload:
                        crc_calculated ^= byte

                    if crc_received != crc_calculated:
                        self.rx_packet_crc_fail_count += 1
                        del buffer[:1]
                        continue

                    try:
                        unpacked = struct.unpack('<ff ff ff L', payload)
                    except struct.error:
                        self.rx_unpack_error_count += 1
                        del buffer[:1]
                        continue

                    (
                        steer_filtered_angle,
                        steer_target_pwm,
                        speed_filtered,
                        speed_target_pwm,
                        auto_speed,
                        auto_angle,
                        last_time_ms,
                    ) = unpacked

                    now_ros_ms = time.time() * 1000.0
                    if self.last_ros_time_ms is None:
                        self.ros_loop_interval = 0.0
                    else:
                        self.ros_loop_interval = (
                            now_ros_ms - self.last_ros_time_ms
                        )
                    self.last_ros_time_ms = now_ros_ms

                    if self.last_arduino_time_ms is None:
                        self.arduino_loop_interval = 0.0
                    else:
                        self.arduino_loop_interval = (
                            last_time_ms - self.last_arduino_time_ms
                        )
                    self.last_arduino_time_ms = last_time_ms

                    self.ros_loop_hz = (
                        1000.0 / self.ros_loop_interval
                        if self.ros_loop_interval > 0.0
                        else 0.0
                    )
                    self.arduino_loop_hz = (
                        1000.0 / self.arduino_loop_interval
                        if self.arduino_loop_interval > 0.0
                        else 0.0
                    )

                    self.steer_filtered_angle = steer_filtered_angle
                    self.steer_target_pwm = steer_target_pwm
                    self.speed_filtered = speed_filtered
                    self.speed_target_pwm = speed_target_pwm
                    self.auto_speed = auto_speed
                    self.auto_angle = auto_angle
                    self.rx_status_packet_count += 1
                    self.convert_to_nav_msgs(
                        speed_filtered, steer_filtered_angle
                    )

                    del buffer[:packet_size]

            except (SerialException, OSError) as exc:
                self.get_logger().warn(f'Serial read failed: {exc}')
                self.reconnect_serial()
            except Exception as exc:
                self.get_logger().warn(f'Unexpected serial error: {exc}')
                time.sleep(0.1)

    def destroy_node(self):
        self.keep_running = False
        if hasattr(self, 'serial_thread') and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=0.5)
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down serial bridge.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
