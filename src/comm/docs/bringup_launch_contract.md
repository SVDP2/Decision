# Leader/Follower Bringup Launch Contract

이 문서는 현재 canonical launch ownership을 고정한다. LiDAR deep learning, AEB,
VLM, 자율주행 판단/decision stack은 각 담당 파트가 별도로 켠다.

전체 block diagram은 [`architecture_block_diagram.html`](architecture_block_diagram.html)을 본다.

## Ownership Rules

- Static TF는 `platoon_localization`이 소유한다.
- Follower canonical static TF는 `follower_static_tf.launch.py` 하나다.
- `relative_localization_eskf`는 static TF를 publish하지 않는다.
- Relative localization은 ArUco/IMU/LiDAR를 한 launch에서 같이 켠다.
- `/platoon/relative_leader/state`는 항상 publisher 1개만 허용한다.
- GPS odom은 현재 bringup에서 map TF authority가 아니라 candidate odom source로 취급한다.

## Canonical Leader Runtime

```bash
ros2 launch platoon_bringup leader_bringup.launch.py
```

Includes by default:

- `platoon_localization leader_static_tf.launch.py`
- `platoon_v2v leader_v2v.launch.py`
- `serial_bridge serial_bridge_node`

Optional GPS candidate odom:

```bash
ros2 launch platoon_bringup leader_bringup.launch.py enable_leader_rtk_gps_odom:=true
```

The wrapper passes `publish_base_tf:=false` to `leader_gps_odom.launch.py` so GPS
odom does not silently become map TF owner.

## Canonical Follower Runtime

```bash
ros2 launch platoon_bringup follower_bringup.launch.py
```

Includes by default:

- `platoon_localization follower_static_tf.launch.py`
- `platoon_v2v follower_v2v.launch.py`
- `platoon_localization relative_localization.launch.py`
- `xycar_motor_native xycar_drive.launch.py`
- `xycar_longitudinal_controller signed_sync_longitudinal_controller.launch.py`

Optional GPS candidate odom:

```bash
ros2 launch platoon_bringup follower_bringup.launch.py enable_follower_rtk_gps_odom:=true
```

The wrapper passes `publish_base_tf:=false` to `follower_gps_odom.launch.py`.

## Relative Localization

Canonical wrapper:

```bash
ros2 launch platoon_localization relative_localization.launch.py
```

This includes `relative_localization_eskf relative_localization.launch.py`,
`follower_lidar_localization leader_wheel_fitting.launch.py`, and bridges
`/follower/localization/leader_base/odom` to `/platoon/relative_leader/state`.
It does not publish follower static TF; launch `follower_static_tf.launch.py` separately
when running this wrapper outside `platoon_bringup`. LiDAR driver and wheel fitting
are enabled by default. There is no separate
`indoor_lidar_relative_localization.launch.py` anymore.

Main outputs:

- `/follower/localization/leader_base/odom`: fused relative odom, frame `leader/base_link`, child `follower/base_link`
- dynamic TF `leader/base_link -> follower/base_link`
- `/platoon/relative_leader/state`: controller input

## Removed Launches

Removed because ownership is now explicit:

- `platoon_localization leader_vehicle_model_viz.launch.py`
- `platoon_localization indoor_lidar_relative_localization.launch.py`
- `platoon_localization outdoor_relative_sensing.launch.py`
- `platoon_v2v fake_v2v_test.launch.py`

`outdoor_relative_localization.launch.py` remains as a GPS-only primitive while
map-frame fusion ownership is being decided, but it is not used by the canonical
bringup wrappers.

## Verification Checklist

```bash
ros2 topic info /platoon/relative_leader/state -v
ros2 topic info /leader/vehicle_model/markers -v
ros2 topic info /follower/vehicle_model/markers -v
ros2 topic echo /follower/localization/leader_base/odom --once
ros2 run tf2_ros tf2_echo leader/base_link follower/base_link
ros2 run tf2_ros tf2_echo follower/base_link follower/lidar
```

Expected:

- `/platoon/relative_leader/state` has one publisher.
- Leader and follower marker topics each have one publisher.
- Follower static TF includes `follower/base_link -> follower/usb_cam` and `follower/usb_cam -> follower/lidar`.
- GPS odom nodes do not publish map TF when launched through `platoon_bringup`.
