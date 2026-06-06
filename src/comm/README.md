# Platoon Communication and Bringup Packages

이 repository는 leader/follower platooning에서 공유하는 ROS 2 message,
V2V adapter, relative-state bridge, static TF/vehicle marker launch, 그리고
canonical bringup wrapper를 담는다.

Architecture diagram: [`docs/architecture_block_diagram.html`](docs/architecture_block_diagram.html).

LiDAR deep learning, AEB, VLM, autonomous decision stack은 이 bringup에서
켜지 않는다. 각 담당 파트가 별도로 실행한다.

## Packages

- `platoon_interfaces`: shared V2V/control messages.
- `platoon_v2v`: leader telemetry adapter, follower receiver, heartbeat/link stats.
- `platoon_localization`: static TF, GPS odom adapter, relative-state bridge primitives.
- `platoon_bringup`: leader/follower runtime을 한 번에 묶는 canonical launch package.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/follower_ws
colcon build --packages-up-to platoon_bringup platoon_localization platoon_v2v
source install/setup.bash
```

## Canonical Bringup

Leader:

```bash
ros2 launch platoon_bringup leader_bringup.launch.py
```

Follower:

```bash
ros2 launch platoon_bringup follower_bringup.launch.py
```

Follower bringup starts canonical static TF, V2V receiver, relative localization
with ArUco/IMU/LiDAR enabled, and motor/controller launch by default.

GPS odom is intentionally off in the bringup wrappers until map-frame authority
is finalized. To publish GPS odom candidate topics without owning map TF:

```bash
ros2 launch platoon_bringup leader_bringup.launch.py enable_leader_rtk_gps_odom:=true
ros2 launch platoon_bringup follower_bringup.launch.py enable_follower_rtk_gps_odom:=true
```

## Primitive Launches

Use these for focused debug or partial bringup:

```bash
ros2 launch platoon_localization leader_static_tf.launch.py
ros2 launch platoon_localization follower_static_tf.launch.py
ros2 launch platoon_localization relative_localization.launch.py
ros2 launch platoon_localization leader_gps_odom.launch.py publish_base_tf:=false
ros2 launch platoon_localization follower_gps_odom.launch.py publish_base_tf:=false
ros2 launch platoon_v2v leader_v2v.launch.py
ros2 launch platoon_v2v follower_v2v.launch.py
```

Deleted stale wrappers: `leader_vehicle_model_viz.launch.py`,
`indoor_lidar_relative_localization.launch.py`, `outdoor_relative_sensing.launch.py`,
and `fake_v2v_test.launch.py`.

## Verification

```bash
ros2 topic info /platoon/relative_leader/state -v
ros2 topic echo /v2v/link_stats --once
ros2 topic echo /follower/localization/leader_base/odom --once
ros2 run tf2_ros tf2_echo leader/base_link follower/base_link
ros2 run tf2_ros tf2_echo follower/base_link follower/lidar
```

Expected ownership:

- `platoon_localization/follower_static_tf.launch.py` owns follower static TF.
- `relative_localization_eskf` does not publish static TF.
- `/platoon/relative_leader/state` has one publisher.
- GPS odom nodes are measurement/candidate producers unless a future map-fusion owner explicitly takes TF authority.
