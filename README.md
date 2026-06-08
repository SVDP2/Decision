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
ros2 topic pub /drive_context std_msgs/msg/String "{data: complex}" --once

- 라이다 실행 명령어
cd LiDAR_ws/SVCD_perception
conda activate pcdet_ros2
python inference_server.py

cd LiDAR_ws/SVCD_perception
si
ros2 launch lidar_processor liadr_all.launch.py