import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String

from auto_drive.command_mux_logic import CommandMuxCore
from auto_drive.command_mux_logic import PlannerCommand
from auto_drive.command_mux_logic import parse_mission_state


class CommandMuxNode(Node):
    def __init__(self):
        super().__init__('command_mux_node')

        self.mission_state_topic = self.declare_parameter(
            'mission_state_topic', '/mission_state'
        ).value
        self.drive_context_topic = self.declare_parameter(
            'drive_context_topic', '/mission_context'
        ).value
        self.highway_steer_topic = self.declare_parameter(
            'highway_steer_topic', '/highway/auto_steer_angle'
        ).value
        self.highway_throttle_topic = self.declare_parameter(
            'highway_throttle_topic', '/highway/throttle_from_planning'
        ).value
        self.city_steer_topic = self.declare_parameter(
            'city_steer_topic', '/city/auto_steer_angle'
        ).value
        self.city_throttle_topic = self.declare_parameter(
            'city_throttle_topic', '/city/throttle_from_planning'
        ).value
        self.complex_steer_topic = self.declare_parameter(
            'complex_steer_topic', '/complex/auto_steer_angle'
        ).value
        self.complex_throttle_topic = self.declare_parameter(
            'complex_throttle_topic', '/complex/throttle_from_planning'
        ).value
        self.output_steer_topic = self.declare_parameter(
            'output_steer_topic', '/auto_steer_angle'
        ).value
        self.output_throttle_topic = self.declare_parameter(
            'output_throttle_topic', '/throttle_from_planning'
        ).value
        self.active_source_topic = self.declare_parameter(
            'active_source_topic', '/planning_command_source'
        ).value
        self.command_timeout_sec = float(
            self.declare_parameter('command_timeout_sec', 0.5).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 20.0).value
        )
        self.default_mission = self.declare_parameter(
            'default_mission', 'highway'
        ).value

        self.core = CommandMuxCore(
            command_timeout_sec=self.command_timeout_sec
        )
        self.mission_state = parse_mission_state(self.default_mission)
        self.highway_steer = None
        self.highway_steer_stamp_sec = None
        self.highway_throttle = None
        self.highway_throttle_stamp_sec = None
        self.city_steer = None
        self.city_steer_stamp_sec = None
        self.city_throttle = None
        self.city_throttle_stamp_sec = None
        self.complex_steer = None
        self.complex_steer_stamp_sec = None
        self.complex_throttle = None
        self.complex_throttle_stamp_sec = None
        self.last_logged_reason = None

        self.create_subscription(
            String, self.mission_state_topic, self.mission_state_callback, 10
        )
        self.create_subscription(
            String, self.drive_context_topic, self.drive_context_callback, 10
        )
        self.create_subscription(
            Float32, self.highway_steer_topic, self.highway_steer_callback, 10
        )
        self.create_subscription(
            Float32,
            self.highway_throttle_topic,
            self.highway_throttle_callback,
            10,
        )
        self.create_subscription(
            Float32, self.city_steer_topic, self.city_steer_callback, 10
        )
        self.create_subscription(
            Float32,
            self.city_throttle_topic,
            self.city_throttle_callback,
            10,
        )
        self.create_subscription(
            Float32, self.complex_steer_topic, self.complex_steer_callback, 10
        )
        self.create_subscription(
            Float32,
            self.complex_throttle_topic,
            self.complex_throttle_callback,
            10,
        )

        self.steer_pub = self.create_publisher(
            Float32, self.output_steer_topic, 10
        )
        self.throttle_pub = self.create_publisher(
            Float32, self.output_throttle_topic, 10
        )
        self.source_pub = self.create_publisher(
            String, self.active_source_topic, 10
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1e-3), self.timer_callback
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def mission_state_callback(self, msg: String):
        self.mission_state = parse_mission_state(msg.data)

    def drive_context_callback(self, msg: String):
        # Fallback for running planning without mission_supervisor.
        self.mission_state = parse_mission_state(msg.data)

    def highway_steer_callback(self, msg: Float32):
        self.highway_steer = float(msg.data)
        self.highway_steer_stamp_sec = self.now_sec()

    def highway_throttle_callback(self, msg: Float32):
        self.highway_throttle = float(msg.data)
        self.highway_throttle_stamp_sec = self.now_sec()

    def city_steer_callback(self, msg: Float32):
        self.city_steer = float(msg.data)
        self.city_steer_stamp_sec = self.now_sec()

    def city_throttle_callback(self, msg: Float32):
        self.city_throttle = float(msg.data)
        self.city_throttle_stamp_sec = self.now_sec()

    def complex_steer_callback(self, msg: Float32):
        self.complex_steer = float(msg.data)
        self.complex_steer_stamp_sec = self.now_sec()

    def complex_throttle_callback(self, msg: Float32):
        self.complex_throttle = float(msg.data)
        self.complex_throttle_stamp_sec = self.now_sec()

    def timer_callback(self):
        now_sec = self.now_sec()
        result = self.core.select(
            self.mission_state,
            self.make_highway_command(),
            self.make_city_command(),
            self.make_complex_command(),
            now_sec,
        )

        steer_msg = Float32()
        steer_msg.data = result.steer
        self.steer_pub.publish(steer_msg)

        throttle_msg = Float32()
        throttle_msg.data = result.throttle
        self.throttle_pub.publish(throttle_msg)

        source_msg = String()
        source_msg.data = result.source if result.valid else result.reason
        self.source_pub.publish(source_msg)

        log_key = (result.source, result.reason, self.mission_state)
        if log_key != self.last_logged_reason:
            self.get_logger().info(
                'Command mux -> '
                f'source={result.source} reason={result.reason} '
                f'mission={self.mission_state}'
            )
            self.last_logged_reason = log_key

    def make_highway_command(self):
        if (
            self.highway_steer is None
            or self.highway_throttle is None
            or self.highway_steer_stamp_sec is None
            or self.highway_throttle_stamp_sec is None
        ):
            return None
        return PlannerCommand(
            self.highway_steer,
            self.highway_throttle,
            self.highway_steer_stamp_sec,
            self.highway_throttle_stamp_sec,
        )

    def make_city_command(self):
        if (
            self.city_steer is None
            or self.city_throttle is None
            or self.city_steer_stamp_sec is None
            or self.city_throttle_stamp_sec is None
        ):
            return None
        return PlannerCommand(
            self.city_steer,
            self.city_throttle,
            self.city_steer_stamp_sec,
            self.city_throttle_stamp_sec,
        )

    def make_complex_command(self):
        if (
            self.complex_steer is None
            or self.complex_throttle is None
            or self.complex_steer_stamp_sec is None
            or self.complex_throttle_stamp_sec is None
        ):
            return None
        return PlannerCommand(
            self.complex_steer,
            self.complex_throttle,
            self.complex_steer_stamp_sec,
            self.complex_throttle_stamp_sec,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
