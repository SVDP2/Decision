import math

from nav_msgs.msg import Odometry
from platoon_interfaces.msg import RelativeLeaderState
import pytest

from platoon_localization.relative_odom_leader_node import fill_relative_pose_from_odom


def _odom(
    *,
    frame_id="leader/leader_rear",
    child_frame_id="follower/base_link",
    x=-1.0,
    y=-0.2,
    z=0.0,
    yaw=0.0,
):
    msg = Odometry()
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
    msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
    msg.pose.covariance[0] = 0.11
    msg.pose.covariance[7] = 0.22
    return msg


def test_relative_source_constants_are_available():
    assert RelativeLeaderState.SOURCE_LIDAR_WHEELS == 4
    assert RelativeLeaderState.SOURCE_FUSED == 5


def test_relative_state_exposes_target_frame():
    state = RelativeLeaderState()
    state.target_frame = "leader/base_link"
    assert state.target_frame == "leader/base_link"


def test_fill_relative_pose_inverts_leader_rear_to_follower_odom():
    state = RelativeLeaderState()

    valid, status = fill_relative_pose_from_odom(
        state,
        _odom(),
        "leader/leader_rear",
        "follower/base_link",
        True,
        "LIDAR",
    )

    assert valid
    assert status == "OK"
    assert state.longitudinal_distance_m == pytest.approx(1.0)
    assert state.lateral_offset_m == pytest.approx(0.2)
    assert state.range_m == pytest.approx(math.hypot(1.0, 0.2))
    assert state.relative_pose.covariance[0] == pytest.approx(0.11)
    assert state.relative_pose.covariance[7] == pytest.approx(0.22)


def test_fill_relative_pose_rejects_unexpected_lidar_frame():
    state = RelativeLeaderState()

    valid, status = fill_relative_pose_from_odom(
        state,
        _odom(frame_id="leader/base_link"),
        "leader/leader_rear",
        "follower/base_link",
        True,
        "LIDAR",
    )

    assert not valid
    assert status == "LIDAR_ODOM_FRAME_MISMATCH"



def test_fill_relative_pose_uses_base_to_target_odom_directly():
    state = RelativeLeaderState()

    valid, status = fill_relative_pose_from_odom(
        state,
        _odom(
            frame_id="follower/base_link",
            child_frame_id="leader/base_link_lidar",
            x=1.4,
            y=-0.15,
            yaw=0.25,
        ),
        "follower/base_link",
        "leader/base_link_lidar",
        True,
        "LIDAR",
        True,
    )

    assert valid
    assert status == "OK"
    assert state.longitudinal_distance_m == pytest.approx(1.4)
    assert state.lateral_offset_m == pytest.approx(-0.15)
    assert state.range_m == pytest.approx(math.hypot(1.4, -0.15))
    assert state.relative_pose.pose.orientation.z == pytest.approx(math.sin(0.125))
    assert state.relative_pose.pose.orientation.w == pytest.approx(math.cos(0.125))
