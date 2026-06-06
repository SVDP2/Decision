from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import pytest

from platoon_localization.utm_offset_gps_odom_node import UtmOffsetGpsOdomNode


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, msg, *args, **kwargs):
        self.warnings.append((msg, kwargs))


def _fix(status):
    msg = NavSatFix()
    msg.status.status = status
    msg.latitude = 37.0
    msg.longitude = 127.0
    msg.altitude = 10.0
    return msg


def _node_for_fix_callback():
    node = UtmOffsetGpsOdomNode.__new__(UtmOffsetGpsOdomNode)
    logger = _Logger()
    gps_pub = _Publisher()
    odom = Odometry()

    node.min_fix_status = 2
    node.utm_zone = 52
    node.origin_easting_m = 0.0
    node.origin_northing_m = 0.0
    node.origin_altitude_m = 0.0
    node.gps_frame = 'follower/follower_gps'
    node.gps_pub = gps_pub
    node.base_pub = None
    node.tf_broadcaster = None
    node.get_logger = lambda: logger
    node._make_odom = lambda *args, **_kwargs: odom
    return node, gps_pub, logger, odom


@pytest.mark.parametrize('status', [0, 1])
def test_fix_callback_rejects_fix_status_below_rtk_fixed(status):
    node, gps_pub, logger, _odom = _node_for_fix_callback()

    node._fix_callback(_fix(status))

    assert gps_pub.messages == []
    assert logger.warnings
    assert logger.warnings[0][1]['throttle_duration_sec'] == pytest.approx(1.0)


def test_fix_callback_accepts_rtk_fixed_status():
    node, gps_pub, _logger, odom = _node_for_fix_callback()

    node._fix_callback(_fix(2))

    assert gps_pub.messages == [odom]


def test_fix_callback_publishes_position_only_base_odom_without_heading():
    node = UtmOffsetGpsOdomNode.__new__(UtmOffsetGpsOdomNode)
    logger = _Logger()
    base_pub = _Publisher()

    node.min_fix_status = 2
    node.utm_zone = 52
    node.origin_easting_m = 0.0
    node.origin_northing_m = 0.0
    node.origin_altitude_m = 0.0
    node.gps_to_base_x_m = 0.1625
    node.gps_to_base_y_m = 0.0
    node.gps_to_base_z_m = 0.135
    node.use_fixed_base_z = True
    node.fixed_base_z_m = 0.0
    node.gps_frame = 'follower/follower_gps'
    node.base_frame = 'follower/base_link'
    node.map_frame = 'map'
    node.gps_pub = None
    node.base_pub = base_pub
    node.tf_broadcaster = None
    node.latest_velocity = None
    node.latest_heading = None
    node.fallback_heading = 0.0
    node.heading_valid = False
    node.publish_base_without_heading = True
    node.min_position_variance_m2 = 1.0e-4
    node.heading_yaw_variance_rad2 = 0.01
    node.unknown_heading_yaw_variance_rad2 = 9.0
    node.get_logger = lambda: logger

    node._fix_callback(_fix(2))

    assert len(base_pub.messages) == 1
    base_odom = base_pub.messages[0]
    assert base_odom.child_frame_id == 'follower/base_link'
    assert base_odom.pose.covariance[35] == pytest.approx(9.0)
    assert logger.warnings
    assert 'unknown heading covariance' in logger.warnings[0][0]


def test_make_odom_marks_valid_heading_covariance():
    node = UtmOffsetGpsOdomNode.__new__(UtmOffsetGpsOdomNode)
    node.map_frame = 'map'
    node.heading_yaw_variance_rad2 = 0.02
    node.unknown_heading_yaw_variance_rad2 = 9.0
    node.min_position_variance_m2 = 1.0e-4
    node.latest_velocity = None

    odom = node._make_odom(_fix(2), 'follower/base_link', 1.0, 2.0, 0.0, 0.5, yaw_valid=True)

    assert odom.pose.covariance[35] == pytest.approx(0.02)
