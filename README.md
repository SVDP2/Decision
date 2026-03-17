# GP_Decision


- gps 구동 명령어

```
ros2 launch ublox_gps ublox_f9p_launch.py
ros2 run fix2nmea fix2nmea
ros2 launch ntrip_client ntrip_client_launch.py
```

- f9p to utm_csv
```
python3 f9p_to_csv.py
ros2 launch gps_to_utm tf_gps_csv.launch.py
```

# 시나리오 for Leader

- **Highway**
  - 장애물 없이 고속도로 본선을 일정 속도로 주행하는 시나리오를 가정. Leader는 GPS path를 따라 횡/종방향 제어 안정성에 집중하고, Follower는 Leader의 경로와 속도 변화를 안정적으로 추종하며 차량 간 간격이 과도하게 벌어지거나 좁혀지지 않는지 검증
  - GPS path, 횡/종방향 제어, leader-follower relative pose, V2V 또는 follower tracking 안정성 확인
    - Sensor Requirements: GPS/RTK, VLM context, leader-follower relative pose

- **City**
  - 도심 저속 주행 중 교차로와 횡단보도가 포함된 시나리오를 가정. Leader는 GPS path를 따라 차로를 유지하며 진행하다가, 횡단보도 앞 보행자 또는 교차로 신호에 따라 우선 정지하고 통과 가능해진 뒤 재출발한다. Follower는 Leader의 저속 주행, 정지, 재출발 흐름을 안정적으로 추종하며 차간거리와 경로 오차가 과도하게 커지지 않는지 검증
  - GPS path, traffic light/sign 또는 intersection context, pedestrian / vehicle detection & tracking, 횡단보도/정지선 인지, 정지 후 재출발 판단, leader-follower relative pose, follower tracking 안정성 확인
    - Sensor Requirements: GPS/RTK, 신호등/표지판/횡단보도/보행자 인지를 위한 전방 camera, 근거리 vehicle/pedestrian 확인을 위한 LiDAR, VLM context, leader-follower relative pose

- **Complex**
  - Leader는 GPS path를 rrt 타겟으로 사용해 콘 사이를 주행한다. GPS path는 전방 목표점을 만드는 용도이고 실제 주행에 사용되는 local 경로는 LiDAR로 생성한 cone 맵 위에서 RRT 브랜치를 고른 뒤 사용한다. **GPS가 장거리 목표를 주고, LiDAR을 이용하여 근거리 장애물/cone 맵을 만들어 local waypoint**를 생성하며, Follower는 Leader가 선택한 진행 흐름을 안정적으로 추종하는 것을 검증
  - LiDAR Cone Detection, cone map generation, GPS path 기반 장거리 target point 생성, local free-space / obstacle map, RRT 또는 local branch selection, local waypoint 생성 및 추종 제어, vehicle localization, leader-follower relative pose 또는 distance estimation, follower tracking 안정성 확인, VLM Complex transition
    - Sensor Requirements: GPS/RTK, cone 및 근거리 obstacle 인지를 위한 LiDAR, leader-follower relative pose