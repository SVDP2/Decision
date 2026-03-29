import math
import struct
import threading
import time

import rclpy
import serial
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
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
        self.port_name = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baudrate = int(self.declare_parameter('baud', 115200).value)
        self.tx_period_sec = float(
            self.declare_parameter('tx_period_sec', 0.067).value
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

        self.twist_msgs_pub = self.create_publisher(
            TwistWithCovarianceStamped, '/encoder/twist', 10
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
                    "Unknown input_mode "
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
                self.port_name, self.baudrate, timeout=0.1
            )
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
        except Exception as exc:
            self.get_logger().warn(f'Serial write failed: {exc}')
            self.reconnect_serial()

    def print_status(self):
        self.get_logger().info(
            f"\n{'━' * 60}\n"
            '[LOOP]  | '
            f'ARDUINO: {self.arduino_loop_interval:>5.1f} ms '
            f'({self.arduino_loop_hz:>4.1f} hz) | '
            f'ROS2: {self.ros_loop_interval:>5.1f} ms '
            f'({self.ros_loop_hz:>4.1f} hz)\n'
            '[CMD]   | '
            f'SPEED: {self.current_speed_command():>6.2f} m/s | '
            f'ANGLE: {self.requested_angle:>6.2f} deg\n'
            '[STATE] | '
            f'SPEED: {self.speed_filtered:>6.2f} m/s | '
            f'ANGLE: {self.steer_filtered_angle:>6.2f} deg\n'
            '[AUTO]  | '
            f'SPEED: {self.auto_speed:>6.2f} m/s | '
            f'ANGLE: {self.auto_angle:>6.2f} deg\n'
            f"{'━' * 60}\n"
        )

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
                    buffer.extend(
                        self.serial_port.read(self.serial_port.in_waiting)
                    )

                while len(buffer) >= packet_size:
                    header_index = buffer.find(header)
                    if header_index == -1:
                        buffer.clear()
                        break
                    if header_index > 0:
                        del buffer[:header_index]

                    if len(buffer) < packet_size:
                        break

                    payload = buffer[2 : packet_size - 1]
                    crc_received = buffer[packet_size - 1]
                    crc_calculated = 0
                    for byte in payload:
                        crc_calculated ^= byte

                    if crc_received != crc_calculated:
                        del buffer[:packet_size]
                        continue

                    unpacked = struct.unpack('<ff ff ff L', payload)
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
