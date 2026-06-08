# UM980 ROS2 드라이버

UM980 GNSS 수신기를 시리얼로 읽고, ROS2 파이프라인에서 바로 쓰는 최소 토픽과 사람이 읽기 쉬운 상태 토픽을 발행하는 드라이버입니다.

## 공개 토픽

기본 공개 토픽은 네 개입니다.

| 토픽 | 타입 | 목적 |
| --- | --- | --- |
| `/f9p/fix` | `sensor_msgs/msg/NavSatFix` | 표준 GNSS 위치. 위도, 경도, WGS84 ellipsoid 고도, covariance를 담습니다. |
| `/f9p/fix_velocity` | `geometry_msgs/msg/TwistWithCovarianceStamped` | BESTNAV 기반 ENU 속도와 속도 covariance를 담습니다. |
| `/f9p/status` | `um980_msgs/msg/Status` | 운영자가 바로 읽는 추상화된 GNSS/RTK/NTRIP/RTCM 상태입니다. |
| `/f9p/status_verbose` | `um980_msgs/msg/StatusVerbose` | 원본 sentence와 native 세부값을 보존하는 저수준 디버그 상태입니다. |

아래 토픽은 더 이상 발행하지 않습니다.

```text
/f9p/twist
/f9p/receiver_status
/f9p/nmea_sentence
/f9p/bestnav
/f9p/pvtsln
/f9p/rtk_status
/f9p/rtcm_status
```

원본 NMEA/Unicore/RTCM 정보는 개별 토픽으로 흩뿌리지 않고 `/f9p/status_verbose`에 모읍니다.

## 상태 토픽 설계

`/f9p/status`는 사람이 빠르게 판단할 수 있도록 추상화된 값만 담습니다.

주요 필드는 다음 의미를 가집니다.

| 필드 | 의미 |
| --- | --- |
| `health_level` | `OK`, `WARN`, `ERROR` 중 하나입니다. |
| `health_message` | 사람이 읽는 한 줄 요약입니다. |
| `fix_mode` | `RTK_FIXED`, `RTK_FLOAT`, `DGNSS`, `GNSS`, `NO_GNSS`, `UNKNOWN` 중 하나입니다. |
| `solution_status` | 수신기 native solution 상태입니다. 예: `SOL_COMPUTED`, `NO_FIX`. |
| `position_source` | 위치 계산에 사용한 소스입니다. 현재는 `BESTNAV`, `GGA`, `UNKNOWN`입니다. |
| `accuracy_source` | 정확도/covariance 소스입니다. `BESTNAV`, `PVTSLN`, `GST`, `UNKNOWN`입니다. |
| `satellites_visible` | 수신기가 보고 있거나 추적 중인 위성 수입니다. 가능하면 GSV visible 값을 우선합니다. |
| `satellites_used` | 실제 위치 계산에 사용된 위성 수입니다. 운영 판단에서는 이 값이 더 중요합니다. |
| `hdop` | 수평 위성 배치 품질입니다. 작을수록 좋습니다. |
| `ntrip_*`, `rtcm_*` | 보정 데이터 연결, 지연, 기준국, 메시지 수신 상태입니다. |

`position_type` 같은 UM980 native 세부 상태는 `/f9p/status`에서 제외하고 `/f9p/status_verbose.native_position_type`으로 보냅니다.

## Verbose 상태

`/f9p/status_verbose`는 추상화하지 않은 저수준 디버그 정보입니다.

포함 정보:

- 타입별 최신 raw sentence: GGA, RMC, GSA, GSV, GST, BESTNAV, PVTSLN, RTKSTATUS, RTCMSTATUS
- 최신 RTCM3 frame hex
- 최근 raw line 버퍼
- 마지막 parse error line, 마지막 unknown line
- sentence별 수신 rate
- native position/velocity type
- tracked/used/visible/SNR 같은 위성 디버그 값

`status_verbose_period_s` 파라미터로 발행 주기를 조정합니다. 기본값은 `1.0`초입니다.

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `port` | `/dev/gps` | UM980 시리얼 포트입니다. |
| `baud` | `0` | `0`이면 자동 감지합니다. |
| `frame_id` | `gps` | 발행 메시지 header frame입니다. |
| `publish_period_s` | `0.05` 또는 `0.02` | `/f9p/fix`, `/f9p/fix_velocity` 발행 주기입니다. |
| `status_verbose_period_s` | `1.0` | `/f9p/status_verbose` 발행 주기입니다. |
| `base_station_name` | `""` | 사람이 읽는 기준국 이름입니다. launch argument로 넣을 수 있습니다. |
| `ntrip_enabled` | `true` | 내장 NTRIP 클라이언트 사용 여부입니다. |
| `ntrip_private_yaml` | `""` | NTRIP 접속 정보 YAML 경로입니다. |

## udev 시리얼 고정

INS 장비는 USB 포트 순서에 따라 `/dev/ttyUSB0`, `/dev/ttyUSB1` 번호가 바뀔 수 있으므로 udev symlink를 고정해서 씁니다. 드라이버와 launch 파일은 변동 가능한 `/dev/ttyUSB*` 대신 아래 이름을 기준으로 동작하게 둡니다.

정책:

```text
IMU -> /dev/imu
GPS -> /dev/gps
```

사용자 udev 규칙은 아래 경로에 저장합니다.

```text
/etc/udev/rules.d/99-ins-serial.rules
```

아래 블록을 그대로 복붙하면 규칙 생성, 재로딩, 재스캔까지 한 번에 수행합니다.

```bash
sudo tee /etc/udev/rules.d/99-ins-serial.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="imu", GROUP="dialout", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="gps", GROUP="dialout", MODE="0660", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

의미:

- EBIMU 계열 CP210x 장치: `10c4:ea60`, serial `0001` -> `/dev/imu`
- UM980 GPS 쪽 CH340 USB-serial: `1a86:7523` -> `/dev/gps`
- `GROUP="dialout"`, `MODE="0660"`으로 dialout 그룹 사용자에게 접근 권한을 줍니다.
- `ENV{ID_MM_DEVICE_IGNORE}="1"`로 ModemManager가 GNSS/IMU 시리얼을 잡아먹지 않게 합니다.

상태 확인:

```bash
ls -l /dev/imu /dev/gps
readlink -f /dev/imu /dev/gps
udevadm info -q property -n /dev/gps | grep -E 'DEVNAME|DEVLINKS|ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_USB_DRIVER|ID_MM_DEVICE_IGNORE'
```

GPS가 정상적으로 잡히면 보통 아래처럼 보입니다. `/dev/ttyUSB0` 번호는 환경에 따라 달라질 수 있지만 `/dev/gps` symlink가 있으면 됩니다.

```text
/dev/gps -> /dev/ttyUSB0
ID_VENDOR_ID=1a86
ID_MODEL_ID=7523
ID_USB_DRIVER=ch341
ID_MM_DEVICE_IGNORE=1
```

`/dev/imu`는 IMU 장치가 연결되어 있을 때 생성됩니다. GPS만 연결한 상태라면 `/dev/gps`만 보여도 정상입니다.

## 실행 예시

20 Hz 설정:

```bash
ros2 launch um980_driver um980_20hz.launch.py \
  base_station_name:=SONP \
  ntrip_private_yaml:=/path/to/ntrip_private.yaml
```

50 Hz 설정:

```bash
ros2 launch um980_driver um980_50hz.launch.py \
  base_station_name:=SONP \
  ntrip_private_yaml:=/path/to/ntrip_private.yaml
```

상태 확인:

```bash
ros2 topic echo --once /f9p/status
ros2 topic echo --once /f9p/status_verbose
ros2 interface show um980_msgs/msg/Status
ros2 interface show um980_msgs/msg/StatusVerbose
```


## 빌드와 테스트

```bash
colcon build --packages-select um980_msgs um980_driver
pytest src/INS/um980_driver/test
```
