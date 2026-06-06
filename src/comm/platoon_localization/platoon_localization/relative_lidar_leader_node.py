from __future__ import annotations

from platoon_interfaces.msg import RelativeLeaderState
import rclpy

from platoon_localization.relative_odom_leader_node import RelativeOdomLeaderNode


class RelativeLidarLeaderNode(RelativeOdomLeaderNode):
    """Build follower-frame leader state from LiDAR wheel-fitting odometry."""

    def __init__(self) -> None:
        super().__init__(
            node_name="relative_lidar_leader_node",
            default_odom_topic="/follower/localization/lidar_wheels/leader_base_detection",
            default_source=RelativeLeaderState.SOURCE_LIDAR_WHEELS,
            status_prefix="LIDAR",
            default_expected_odom_frame="follower/base_link",
            default_expected_odom_child_frame="leader/base_link_lidar",
            default_target_frame="leader/base_link",
            default_odom_pose_is_base_to_target=True,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeLidarLeaderNode()
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
