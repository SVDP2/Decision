# How to run

## Journey to start

#### gps open source setting

```bash
sudo apt update
sudo apt install ros-humble-rtcm-msgs ros-humble-nmea-msgs
sudo apt install ros-humble-tf-transformations
sudo apt install ros-humble-ackermann-msgs ros-humble-nav2-bringup
```

#### RTK_GPS_NTRIP
https://github.com/olvdhrm/RTK_GPS_NTRIP

#### ntrip
https://github.com/SGroe/ntrip_client_ros2

---
* GPS 백파일에서 /f9r/fix 정보를 이용하여 utm 좌표값이 담긴 csv 파일 생성

ros2 run gps_to_utm f9r_to_csv
