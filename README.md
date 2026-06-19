# GP_Decision
- gps 구동 명령어

```
ros2 launch um980_driver um980_20hz.launch.py

ros2 launch ublox_gps ublox_f9p_launch.py
ros2 run fix2nmea fix2nmea
ros2 launch ntrip_client ntrip_client_launch.py
```

- 리더 자율주행 serial_bridge true/false  (complex용 RVIZ)
```
ros2 launch auto_drive bringup_single_f9p.launch.py \
    csv_file_path:=/home/kai/dc_ws/GP_Decision/config/path_csv/full_course_3_30.csv \
    use_rviz:=true \
    rviz_config:=/home/kai/dc_ws/GP_Decision/src/leader/auto_drive/config/complex.rviz \
    publish_velodyne_tf:=true \
    velodyne_frame:=velodyne \
    use_serial_bridge:=true
```

/home/kai/dc_ws/GP_Decision/config/path_csv/rosbag2_2026_03_14-16_23_49.csv

ros2 launch auto_drive bringup_single_f9p.launch.py \
    csv_file_path:=/home/kai/dc_ws/GP_Decision/config/path_csv/rosbag2_2026_03_29-17_44_57.csv \
    use_rviz:=true \
    rviz_config:=/home/kai/dc_ws/GP_Decision/src/leader/auto_drive/config/complex.rviz \
    publish_velodyne_tf:=true \
    velodyne_frame:=velodyne \
    use_serial_bridge:=true

- f9p to utm_csv: bag파일로 CSV 만들기
```
python3 f9p_to_csv.py
ros2 launch gps_to_utm tf_gps_csv.launch.py
```



-for home
```
ros2 launch auto_drive bringup_single_f9p.launch.py \
    csv_file_path:=/home/yoo/GP_Decision/config/path_csv/full_course_3_30.csv \
    use_rviz:=true \
    rviz_config:=./src/leader/auto_drive/config/complex.rviz \
    publish_velodyne_tf:=true \
    velodyne_frame:=velodyne \
    use_serial_bridge:=true
```

- gps 모드 전환
ros2 topic pub /mission_context std_msgs/msg/String "{data: complex}" --once

- 신호등 연동

VLM의 `/drive_context` JSON과 mission 문자열 토픽을 분리하기 위해 DECISION은
`/mission_context`를 사용한다. VLM이 발행하는 다음 Bool 토픽은
`traffic_signal_gate_node`가 구독한다.

```
/traffic_signal/present
/traffic_signal/red
/traffic_signal/green
```

`mission_zones.yaml`에서 정지선 접근 구간의 CSV index와 활성 여부를 관리한다.
traffic zone 내부에서는 fresh한 red만 `/traffic_stop`을 활성화하고
green·미검출·unknown·stale은 주행한다.

- highway / city / complex 미션 전환

기본 미션은 highway다. `city_zone`의 CSV index 100에 진입하면 city,
`complex_start`의 CSV index 1300에 진입하면 complex로 전환한다.

```
HIGHWAY -> /highway/* -> nominal throttle 0.38
CITY    -> /city/*    -> nominal throttle 0.28
COMPLEX -> /complex/*
```

상태 및 실제 속도 확인:

```
ros2 topic echo /mission_state
ros2 topic echo /planning_command_source
ros2 topic echo /throttle_cmd
ros2 topic echo /encoder/twist
```

현장 주행 전 `city_zone.csv_index: 100`이 실제 전환 위치인지 반드시 확인한다.

- 라이다 실행 명령어
cd LiDAR_ws/SVCD_perception
conda activate pcdet_ros2
python inference_server.py

cd LiDAR_ws/SVCD_perception
si
ros2 launch lidar_processor liadr_all.launch.py
