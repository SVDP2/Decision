# Outdoor RTK Platooning Foundation

This document is the implementation contract for outdoor leader/follower RTK
platooning. It records the current repo ownership, frame policy, launch
ownership, and first bringup checks.

Status as of 2026-05-25:

- Outdoor RTK fixed operation was field-checked with `NavSatFix.status.status == 2`.
- Leader and follower can be visualized together in the shared DDS graph.
- The next development step is not a full ESKF. The next step is a relative
  state selector that arbitrates GPS and ArUco candidates while preserving the
  existing controller interface.

## Workspace ownership

Authoritative follower runtime paths:

- `xytron-svdp:~/follower_ws/src/comm`
- `xytron-svdp:~/follower_ws/src/control`
- `xytron-svdp:~/follower_ws/src/localization`

Authoritative leader runtime paths:

- `leader-svdp:~/dc_ws/GP_Decision`
- `leader-svdp:~/dc_ws/svdp_comm_min`
- `leader-svdp:~/joljak` for Arduino/serial only

Excluded paths:

- `follower_control` is not used.
- `leader_ws` and `joljak/src/comm` are not outdoor platooning authorities.

Do not pull, copy, or overwrite remote dirty workspaces until their local diffs
have been saved or explicitly accepted. The leader `svdp_comm_min` tree is a
deployment copy, not a git authority; sync it from the local `Communication`
subset or document every manual copy.

## Current Git and deployment state

Local repo authority:

- `Communication`: `main`, synced to `origin/main`.
- `Localization`: `main`, synced to `origin/main` locally. The follower vehicle
  may have extra local commits on top of origin; preserve them.
- `Control`: `main`, synced to `origin/main`.
- `Decision`: local branch `ysm`, currently has local commits and unrelated
  dirty config changes. Do not mix platooning documentation commits with those
  changes.

Vehicle deployment notes:

- Follower runtime uses `~/follower_ws/src/comm`, `~/follower_ws/src/control`,
  and `~/follower_ws/src/localization`.
- Leader communication runtime uses
  `~/dc_ws/GP_Decision/src/comm`.
- Leader GPS/NTRIP runtime lives under
  `~/dc_ws/GP_Decision/src/gps/RTK_GPS_NTRIP`; this tree is not currently a
  clean Git authority on the leader vehicle, so any manual changes there must be
  recorded in this document or mirrored into the local `Decision` repo.

## Outdoor frame contract

Outdoor uses `map` as a local UTM-offset meter frame. Raw UTM coordinates are
only an internal conversion result and must not be published as TF positions.

The canonical map origin is loaded from
`platoon_localization/config/outdoor_utm_map.yaml`:

- `utm_zone`
- `origin_easting_m`
- `origin_northing_m`
- `origin_altitude_m`
- `origin_source_csv`

The default origin is the first numeric row of the active leader route CSV.
Leader and follower must load the same origin artifact before runtime. GPS fixes
whose computed UTM zone does not match `utm_zone` are rejected.

Allowed outdoor TF ownership:

- `map -> leader/base_link`
- `map -> follower/base_link`
- static vehicle/sensor transforms, including `leader/base_link -> leader/leader_rear`
  and `follower/base_link -> follower/follower_gps`

Forbidden in outdoor autonomous mode:

- a dynamic `leader/leader_rear -> follower/base_link` TF while
  `map -> follower/base_link` is active.

Indoor relative localization can keep its existing relative TF behavior. Outdoor
ArUco/ESKF observation must set `relative_localization_node.publish_tf=false`.

Outdoor relative TF policy:

- Final relative TF is owned by the final relative-state publisher only.
- GPS candidate nodes and ArUco candidate nodes must not publish the final
  `follower/base_link -> leader/base_link` or
  `follower/base_link -> leader/leader_rear` TF.
- During P1 selector development, the selector owns this TF.
- During P2 full ESKF development, the ESKF/fusion node owns this TF.

## Topic contract

GPS raw frames:

- leader ublox `NavSatFix.header.frame_id`: `leader/leader_gps`
- follower ublox `NavSatFix.header.frame_id`: `follower/follower_gps`

Derived odometry:

- `/leader/localization/gps/odom`: `nav_msgs/Odometry`, `map -> leader/base_link`
- `/v2v/leader/odom`: same odometry forwarded over V2V
- `/follower/localization/gps/odom`: `map -> follower/follower_gps`
- `/follower/localization/global/odom`: `map -> follower/base_link`

Canonical relative control input:

- `/platoon/relative_leader/state`

Candidate relative state inputs for selector/fusion:

- `/platoon/relative_leader/gps_state`: `RelativeLeaderState`, GPS-derived
  candidate from leader/follower map odometry.
- `/platoon/relative_leader/aruco_state`: `RelativeLeaderState`, ArUco/IMU ESKF
  candidate from follower camera and IMU.

Final relative state:

- `/platoon/relative_leader/state`: `RelativeLeaderState`, the only topic that
  controllers consume.

Do not launch more than one publisher to `/platoon/relative_leader/state`.
Before the selector exists, the simple RTK demo may let `relative_gps_leader_node`
publish this final topic directly. After the selector exists, GPS and ArUco
relative nodes must publish only their candidate topics.

Reference path:

- `/v2v/leader/reference_path`: `nav_msgs/Path`, `header.frame_id=map`
- `/follower/lateral/reference_path`: `nav_msgs/Path`, `header.frame_id=map`

P0 treats path relay as an interface/debug signal. Follower lateral path
tracking is a later `xycar_lateral_controller` task.

RTCM/NTRIP topics:

- Both vehicles use `ROS_DOMAIN_ID=42` so leader RViz and V2V can see the shared
  graph.
- Leader NTRIP publishes `/leader/ntrip_client/rtcm`.
- Follower NTRIP publishes `/follower/ntrip_client/rtcm`.
- Leader ublox subscribes only to `/leader/ntrip_client/rtcm`.
- Follower ublox subscribes only to `/follower/ntrip_client/rtcm`.
- The bare `/ntrip_client/rtcm` topic is forbidden while both vehicles share
  domain 42.

## Roadmap

### P0: RTK-fixed autonomous demo

Purpose: prove the full leader-autonomous/follower-following loop with minimal
sensor fusion.

Assumptions:

- Leader and follower RTK remain fixed for the test window.
- `NavSatFix.status.status == 2` is the expected GPS quality state.
- ArUco may run as observation/debug, but GPS relative state is the control
  source.

Required behavior:

- Leader drives autonomously from its own waypoint/path stack.
- Leader publishes V2V motion, safety, heartbeat, odometry, and reference path.
- Follower receives leader path and motion/safety state.
- Follower keeps distance using `/platoon/relative_leader/state`.
- Emergency stop or leader stop propagates through V2V and causes follower stop.

P0 final-state publisher:

- `relative_gps_leader_node` may publish `/platoon/relative_leader/state`
  directly.

Exit criteria:

- Leader starts autonomous forward motion.
- Follower moves only while leader motion is valid and safe.
- Follower stops on leader emergency stop, V2V timeout, GPS odom timeout, or
  safety timeout.
- RViz shows both vehicles in `map`.

### P1: GPS/ArUco relative-state selector

Purpose: tolerate NTRIP/RTK dropouts without changing controller interfaces.

Architecture:

```text
GPS relative candidate   -> /platoon/relative_leader/gps_state
ArUco relative candidate -> /platoon/relative_leader/aruco_state
GPS fix/covariance       -> selector
ArUco diagnostics/cov    -> selector
                              |
                              v
                    /platoon/relative_leader/state
                    relative TF for RViz/control
```

Selector inputs:

- `/platoon/relative_leader/gps_state`
- `/platoon/relative_leader/aruco_state`
- leader raw GPS fix, default `/f9p/fix`
- follower raw GPS fix, default `/ublox_gps_node/fix`
- ArUco diagnostics, default diagnostics entry `aruco_detector`

Selector output:

- `/platoon/relative_leader/state`
- optional debug topic `/platoon/relative_leader/selector_debug`
- optional final relative TF from `follower/base_link` to `leader/base_link`
  or `leader/leader_rear`.

Default source policy:

- If both raw GPS fixes have `status.status == 2` and GPS covariance is low,
  prefer GPS.
- If either GPS fix is not fixed, timed out, or has high covariance, reduce GPS
  confidence.
- If ArUco odom is fresh, valid, and diagnostics report `pose_published=true`,
  allow ArUco as a candidate.
- Treat `known_markers >= 3` as high confidence, `known_markers == 2` as medium
  confidence, and `known_markers == 1` as low confidence.
- If ArUco reports a rejection reason such as `motion_position_gate`,
  `reprojection_gate`, `feasible_box_gate`, or `view_angle_gate`, do not use the
  rejected sample.
- If GPS and ArUco are both valid and close, blend or smooth between them.
- If both are valid but disagree strongly, prefer GPS only when both GPS fixes
  are RTK fixed; otherwise prefer ArUco if its diagnostics are high confidence.
- If both are invalid, publish `valid=false` and stop publishing fresh final TF
  after a short hold timeout.

P1 must not implement leader dead reckoning beyond a short hold timeout. Leader
dead reckoning belongs to P2.

### P2: Full relative ESKF/fusion

Purpose: replace selector heuristics with a single relative-state estimator.

Inputs:

- GPS relative position and covariance.
- ArUco/IMU relative pose, diagnostics, and covariance.
- Follower IMU yaw rate.
- Optional LiDAR leader detection around the ArUco/GPS prior.
- Optional leader V2V motion for prediction.

State candidates:

- relative x/y/yaw
- relative velocity
- yaw-rate or gyro bias terms if needed

Design decisions deferred to P2:

- leader dead-reckoning model
- process noise during leader stop, turn, and low-speed motion
- covariance schedule for RTK fixed, float, and no-fix GPS
- ArUco outlier recovery after long dropout
- LiDAR association around the leader body

P2 must keep the external controller contract unchanged:
controllers still consume only `/platoon/relative_leader/state`.

## Bringup order

Leader:

```bash
cd ~/dc_ws/GP_Decision
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch ublox_gps ublox_f9p_launch.py
ros2 launch ntrip_client ntrip_client_launch.py
ros2 launch auto_drive bringup_single_f9p.launch.py
ros2 launch platoon_localization leader_static_tf.launch.py
ros2 launch platoon_localization leader_gps_odom.launch.py
ros2 launch platoon_v2v leader_v2v.launch.py enable_reference_path:=true
```

Follower:

```bash
cd ~/follower_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch ublox_gps ublox_gps_node-launch.py
ros2 launch ntrip_client ntrip_client_launch.py
ros2 launch platoon_localization follower_static_tf.launch.py
ros2 launch platoon_localization follower_gps_odom.launch.py
ros2 launch platoon_v2v follower_v2v.launch.py enable_reference_path:=true
ros2 launch platoon_localization outdoor_relative_localization.launch.py
```

P1 selector bringup will replace the last line with candidate publishers plus
the selector launch once implemented.

Optional outdoor sensing observation:

```bash
# Removed: outdoor_relative_sensing.launch.py
```

This wrapper forces the relative ESKF TF output off.

## Acceptance checks

```bash
ros2 topic echo /f9p/fix --once
ros2 topic echo /ublox_gps_node/fix --once
ros2 topic echo /leader/localization/gps/odom --once
ros2 topic echo /follower/localization/global/odom --once
ros2 topic echo /platoon/relative_leader/state --once
ros2 topic info /platoon/relative_leader/state
ros2 topic info /leader/ntrip_client/rtcm -v
ros2 topic info /follower/ntrip_client/rtcm -v
ros2 run tf2_ros tf2_echo map leader/base_link
ros2 run tf2_ros tf2_echo map follower/base_link
```

Pass conditions:

- raw GPS frame IDs are not the unqualified `gps` frame.
- all derived outdoor odometry has `header.frame_id=map`.
- leader odom child frame is `leader/base_link`.
- follower global odom child frame is `follower/base_link`.
- `relative_gps_leader_node` does not report `ODOM_FRAME_MISMATCH`,
  `LEADER_CHILD_FRAME_MISMATCH`, or `FOLLOWER_CHILD_FRAME_MISMATCH`.
- `/platoon/relative_leader/state` has one publisher in P0 outdoor.
- `/leader/ntrip_client/rtcm` and `/follower/ntrip_client/rtcm` each have one
  publisher and one subscriber when NTRIP and ublox are running.
- `/ntrip_client/rtcm` is absent or has zero active endpoints during shared
  domain outdoor operation.
- `follower/base_link` has exactly one active TF parent.
- signed sync stops on heartbeat, safety, odom timeout, and leader stop.

Closed-loop autonomous beyond P0 is blocked until leader/follower GPS lever arms
and `leader/base_link -> leader/leader_rear` have been measured on the vehicles.
