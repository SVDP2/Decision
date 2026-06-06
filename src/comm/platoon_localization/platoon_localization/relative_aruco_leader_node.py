from __future__ import annotations

from platoon_interfaces.msg import RelativeLeaderState
import rclpy

from platoon_localization.relative_odom_leader_node import RelativeOdomLeaderNode


class RelativeArucoLeaderNode(RelativeOdomLeaderNode):
    """Build follower-frame leader state from fused leader_base odometry."""

    def __init__(self) -> None:
        super().__init__(
            node_name="relative_aruco_leader_node",
            default_odom_topic="/follower/localization/leader_base/odom",
            default_source=RelativeLeaderState.SOURCE_V2V_ARUCO,
            status_prefix="ARUCO",
            legacy_odom_parameter="aruco_odom_topic",
            default_expected_odom_frame="leader/base_link",
            default_target_frame="leader/base_link",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeArucoLeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
