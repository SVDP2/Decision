import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32


class MissionSupervisorNode(Node):
    def __init__(self):
        super().__init__('mission_supervisor_node')

        self.throttle_from_planning_topic = self.declare_parameter(
            'throttle_from_planning_topic', '/throttle_from_planning'
        ).value
        self.throttle_cmd_topic = self.declare_parameter(
            'throttle_cmd_topic', '/throttle_cmd'
        ).value
        self.safety_stop_topic = self.declare_parameter(
            'safety_stop_topic', '/safety_stop'
        ).value
        self.manual_stop_topic = self.declare_parameter(
            'manual_stop_topic', '/manual_stop'
        ).value
        self.command_timeout_sec = float(
            self.declare_parameter('command_timeout_sec', 0.5).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 20.0).value
        )

        self.latest_planning_throttle = 0.0
        self.latest_planning_stamp = None
        self.safety_stop = False
        self.manual_stop = False

        self.create_subscription(
            Float32,
            self.throttle_from_planning_topic,
            self.throttle_callback,
            10,
        )
        self.create_subscription(
            Bool, self.safety_stop_topic, self.safety_stop_callback, 10
        )
        self.create_subscription(
            Bool, self.manual_stop_topic, self.manual_stop_callback, 10
        )

        self.throttle_cmd_pub = self.create_publisher(
            Float32, self.throttle_cmd_topic, 10
        )
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1e-3), self.timer_callback
        )

    def throttle_callback(self, msg: Float32):
        self.latest_planning_throttle = float(msg.data)
        self.latest_planning_stamp = self.get_clock().now()

    def safety_stop_callback(self, msg: Bool):
        self.safety_stop = msg.data

    def manual_stop_callback(self, msg: Bool):
        self.manual_stop = msg.data

    def timer_callback(self):
        command = 0.0

        if (
            self.latest_planning_stamp is not None
            and (
                self.get_clock().now() - self.latest_planning_stamp
            ).nanoseconds
            <= int(self.command_timeout_sec * 1e9)
            and not self.safety_stop
            and not self.manual_stop
        ):
            command = self.latest_planning_throttle

        cmd_msg = Float32()
        cmd_msg.data = float(command)
        self.throttle_cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
