import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String

from auto_drive.traffic_signal_gate_logic import TrafficSignalGateCore


class TrafficSignalGateNode(Node):
    def __init__(self):
        super().__init__('traffic_signal_gate_node')

        self.signal_present_topic = self.declare_parameter(
            'signal_present_topic', '/traffic_signal/present'
        ).value
        self.signal_red_topic = self.declare_parameter(
            'signal_red_topic', '/traffic_signal/red'
        ).value
        self.signal_green_topic = self.declare_parameter(
            'signal_green_topic', '/traffic_signal/green'
        ).value
        self.traffic_stop_topic = self.declare_parameter(
            'traffic_stop_topic', '/traffic_stop'
        ).value
        self.status_topic = self.declare_parameter(
            'status_topic', '/traffic_signal_gate/status'
        ).value
        self.signal_timeout_sec = float(
            self.declare_parameter('signal_timeout_sec', 5.0).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 20.0).value
        )

        self.core = TrafficSignalGateCore(
            signal_timeout_sec=self.signal_timeout_sec
        )
        self.last_logged_state = None

        self.create_subscription(
            Bool,
            self.signal_present_topic,
            self.signal_present_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.signal_red_topic,
            self.signal_red_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.signal_green_topic,
            self.signal_green_callback,
            10,
        )

        self.traffic_stop_pub = self.create_publisher(
            Bool, self.traffic_stop_topic, 10
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1e-3), self.timer_callback
        )

        self.get_logger().info(
            'Traffic signal gate ready: only a fresh red observation '
            'asserts the traffic stop request.'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def signal_present_callback(self, msg: Bool):
        self.core.set_signal_present(msg.data, self.now_sec())

    def signal_red_callback(self, msg: Bool):
        self.core.set_signal_red(msg.data, self.now_sec())

    def signal_green_callback(self, msg: Bool):
        self.core.set_signal_green(msg.data, self.now_sec())

    def timer_callback(self):
        decision = self.core.evaluate(self.now_sec())

        stop_msg = Bool()
        stop_msg.data = decision.stop_required
        self.traffic_stop_pub.publish(stop_msg)

        status_msg = String()
        status_msg.data = decision.state
        self.status_pub.publish(status_msg)

        if decision.state != self.last_logged_state:
            self.get_logger().info(
                'Traffic signal gate -> '
                f'state={decision.state} stop={decision.stop_required}'
            )
            self.last_logged_state = decision.state


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignalGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
