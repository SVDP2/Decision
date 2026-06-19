from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    args = [
        DeclareLaunchArgument("log_level", default_value=TextSubstitution(text="INFO")),
        DeclareLaunchArgument("device_serial_string", default_value=""),
        DeclareLaunchArgument("frame_id", default_value="leader/leader_gps"),
        DeclareLaunchArgument("publish_ubx_debug", default_value="false"),
        DeclareLaunchArgument("ntrip_enabled", default_value="true"),
        DeclareLaunchArgument("ntrip_host", default_value="gnss.eseoul.go.kr"),
        DeclareLaunchArgument("ntrip_port", default_value="2101"),
        DeclareLaunchArgument("ntrip_mountpoint", default_value="SONP-RTCM32"),
        DeclareLaunchArgument("ntrip_username", default_value="seoul"),
        DeclareLaunchArgument("ntrip_password", default_value="seoul"),
        DeclareLaunchArgument("ntrip_version", default_value="Ntrip/2.0"),
        DeclareLaunchArgument("ntrip_use_https", default_value="false"),
        DeclareLaunchArgument("ntrip_gga_interval_s", default_value="5.0"),
        DeclareLaunchArgument("ntrip_rtcm_timeout_s", default_value="10.0"),
    ]

    params = {
        "leader_mode": True,
        "publish_ubx_debug": ParameterValue(
            LaunchConfiguration("publish_ubx_debug"), value_type=bool
        ),
        "DEVICE_FAMILY": "F9P",
        "UBX_CONFIG_FILE": PathJoinSubstitution(
            [FindPackageShare("ublox_dgnss"), "config", "f9p_leader_ubx_config.toml"]
        ),
        "DEVICE_SERIAL_STRING": LaunchConfiguration("device_serial_string"),
        "FRAME_ID": LaunchConfiguration("frame_id"),
        "ESF_MEAS_INPUT_ENABLED": False,
        "RTCM_INPUT_ENABLED": False,
        "fail_on_cfg_nak": True,
        "nav_cov_timeout_sec": 0.5,
        "use_hpposllh_accuracy_fallback": True,
        "ntrip_enabled": ParameterValue(
            LaunchConfiguration("ntrip_enabled"), value_type=bool
        ),
        "ntrip_host": LaunchConfiguration("ntrip_host"),
        "ntrip_port": ParameterValue(LaunchConfiguration("ntrip_port"), value_type=int),
        "ntrip_mountpoint": LaunchConfiguration("ntrip_mountpoint"),
        "ntrip_username": LaunchConfiguration("ntrip_username"),
        "ntrip_password": LaunchConfiguration("ntrip_password"),
        "ntrip_version": LaunchConfiguration("ntrip_version"),
        "ntrip_use_https": ParameterValue(
            LaunchConfiguration("ntrip_use_https"), value_type=bool
        ),
        "ntrip_gga_interval_s": ParameterValue(
            LaunchConfiguration("ntrip_gga_interval_s"), value_type=float
        ),
        "ntrip_rtcm_timeout_s": ParameterValue(
            LaunchConfiguration("ntrip_rtcm_timeout_s"), value_type=float
        ),
        "CFG_NAVHPG_DGNSSMODE": 3,
        "CFG_RATE_MEAS": 67,
        "CFG_RATE_NAV": 1,
        "CFG_USBINPROT_UBX": True,
        "CFG_USBINPROT_NMEA": False,
        "CFG_USBINPROT_RTCM3X": True,
        "CFG_USBOUTPROT_UBX": True,
        "CFG_USBOUTPROT_NMEA": False,
        "CFG_USBOUTPROT_RTCM3X": False,
        "CFG_MSGOUT_UBX_MON_COMMS_USB": 0,
        "CFG_MSGOUT_UBX_NAV_CLOCK_USB": 0,
        "CFG_MSGOUT_UBX_NAV_COV_USB": 1,
        "CFG_MSGOUT_UBX_NAV_DOP_USB": 0,
        "CFG_MSGOUT_UBX_NAV_EOE_USB": 0,
        "CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB": 0,
        "CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB": 1,
        "CFG_MSGOUT_UBX_NAV_ODO_USB": 0,
        "CFG_MSGOUT_UBX_NAV_ORB_USB": 0,
        "CFG_MSGOUT_UBX_NAV_POSECEF_USB": 0,
        "CFG_MSGOUT_UBX_NAV_POSLLH_USB": 0,
        "CFG_MSGOUT_UBX_NAV_PVT_USB": 1,
        "CFG_MSGOUT_UBX_NAV_RELPOSNED_USB": 0,
        "CFG_MSGOUT_UBX_NAV_SAT_USB": 1,
        "CFG_MSGOUT_UBX_NAV_SIG_USB": 1,
        "CFG_MSGOUT_UBX_NAV_STATUS_USB": 0,
        "CFG_MSGOUT_UBX_NAV_SVIN_USB": 0,
        "CFG_MSGOUT_UBX_NAV_TIMEUTC_USB": 0,
        "CFG_MSGOUT_UBX_NAV_VELECEF_USB": 0,
        "CFG_MSGOUT_UBX_NAV_VELNED_USB": 0,
        "CFG_MSGOUT_UBX_RXM_MEASX_USB": 0,
        "CFG_MSGOUT_UBX_RXM_RAWX_USB": 0,
        "CFG_MSGOUT_UBX_RXM_RTCM_USB": 1,
        "CFG_MSGOUT_UBX_RXM_SFRBX_USB": 0,
        "CFG_MSGOUT_RTCM_3X_TYPE1005_USB": 0,
        "CFG_MSGOUT_RTCM_3X_TYPE1077_USB": 0,
        "CFG_MSGOUT_RTCM_3X_TYPE1087_USB": 0,
        "CFG_MSGOUT_RTCM_3X_TYPE1097_USB": 0,
        "CFG_MSGOUT_RTCM_3X_TYPE1127_USB": 0,
        "CFG_MSGOUT_RTCM_3X_TYPE1230_USB": 0,
        "CFG_MSGOUT_RTCM_3X_TYPE4072_0_USB": 0,
        "CFG_SIGNAL_GPS_ENA": True,
        "CFG_SIGNAL_GPS_L1CA_ENA": True,
        "CFG_SIGNAL_GPS_L2C_ENA": True,
        "CFG_SIGNAL_SBAS_ENA": False,
        "CFG_SIGNAL_SBAS_L1CA_ENA": False,
        "CFG_SIGNAL_GAL_ENA": True,
        "CFG_SIGNAL_GAL_E1_ENA": True,
        "CFG_SIGNAL_GAL_E5B_ENA": True,
        "CFG_SIGNAL_BDS_ENA": True,
        "CFG_SIGNAL_BDS_B1_ENA": True,
        "CFG_SIGNAL_BDS_B2_ENA": True,
        "CFG_SIGNAL_GLO_ENA": True,
        "CFG_SIGNAL_GLO_L1_ENA": True,
        "CFG_SIGNAL_GLO_L2_ENA": True,
        "CFG_SIGNAL_QZSS_ENA": True,
        "CFG_SIGNAL_QZSS_L1CA_ENA": True,
        "CFG_SIGNAL_QZSS_L1S_ENA": False,
        "CFG_SIGNAL_QZSS_L2C_ENA": True,
    }

    driver = Node(
        package="ublox_dgnss_node",
        executable="ublox_dgnss_node",
        name="f9p_leader",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[params],
    )

    return LaunchDescription(args + [driver])
