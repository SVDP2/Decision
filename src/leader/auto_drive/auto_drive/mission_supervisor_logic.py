from dataclasses import dataclass
from enum import Enum


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class MissionState(str, Enum):
    HIGHWAY = 'HIGHWAY'
    CITY = 'CITY'
    COMPLEX = 'COMPLEX'


class SafetyStatus(str, Enum):
    SAFE_OK = 'SAFE_OK'
    STOP_MANUAL = 'STOP_MANUAL'
    STOP_GENERIC = 'STOP_GENERIC'
    STOP_TRAFFIC = 'STOP_TRAFFIC'
    STOP_ROI = 'STOP_ROI'


class ActiveAlgorithm(str, Enum):
    HIGHWAY_GPS_TRACKING = 'HIGHWAY_GPS_TRACKING'
    CITY_GPS_TRACKING = 'CITY_GPS_TRACKING'
    COMPLEX_GPS_TARGETING = 'COMPLEX_GPS_TARGETING'
    SAFETY_HOLD = 'SAFETY_HOLD'


@dataclass(frozen=True)
class MissionPolicy:
    throttle_scale: float
    throttle_limit: float


@dataclass(frozen=True)
class SupervisorSnapshot:
    mission_state: MissionState
    requested_mission_state: MissionState
    previous_mission_state: MissionState
    safety_status: SafetyStatus
    safety_active: bool
    active_algorithm: ActiveAlgorithm
    output_throttle: float
    applied_planning_throttle: float
    planning_timed_out: bool


class MissionSupervisorCore:
    """Mission/safety arbitration logic without ROS dependencies."""

    def __init__(
        self,
        command_timeout_sec=0.5,
        release_hysteresis_sec=0.5,
        default_mission='highway',
        highway_policy=None,
        city_policy=None,
        complex_policy=None,
    ):
        self.command_timeout_sec = max(float(command_timeout_sec), 0.0)
        self.release_hysteresis_sec = max(float(release_hysteresis_sec), 0.0)

        self.mission_policies = {
            MissionState.HIGHWAY: highway_policy
            or MissionPolicy(throttle_scale=1.0, throttle_limit=0.7),
            MissionState.CITY: city_policy
            or MissionPolicy(throttle_scale=0.75, throttle_limit=0.5),
            MissionState.COMPLEX: complex_policy
            or MissionPolicy(throttle_scale=0.6, throttle_limit=0.4),
        }

        default_state = (
            self.parse_context(default_mission) or MissionState.HIGHWAY
        )
        self.requested_mission_state = default_state
        self.mission_state = default_state
        self.previous_mission_state = default_state

        self.safety_status = SafetyStatus.SAFE_OK
        self.safety_active = False

        self.latest_planning_throttle = 0.0
        self.latest_planning_stamp_sec = None
        self.release_timer_start_sec = None

        self.manual_stop = False
        self.generic_safety_stop = False
        self.traffic_stop = False
        self.intersection = False
        self.roi_warning = False

    def parse_context(self, context):
        if context is None:
            return None

        normalized = str(context).strip().lower()
        if not normalized:
            return None

        if any(
            token in normalized
            for token in ('complex', 'cone', 'rrt', 'unstructured')
        ):
            return MissionState.COMPLEX
        if any(
            token in normalized
            for token in (
                'city',
                'urban',
                'intersection',
                'crosswalk',
                'traffic',
            )
        ):
            return MissionState.CITY
        if any(
            token in normalized
            for token in ('highway', 'main road', 'freeway', 'expressway')
        ):
            return MissionState.HIGHWAY
        return None

    def set_drive_context(self, context):
        mission_state = self.parse_context(context)
        if mission_state is not None:
            self.requested_mission_state = mission_state
        return mission_state

    def set_planning_throttle(self, throttle, now_sec):
        self.latest_planning_throttle = float(throttle)
        self.latest_planning_stamp_sec = float(now_sec)

    def set_manual_stop(self, active):
        self.manual_stop = bool(active)

    def set_generic_safety_stop(self, active):
        self.generic_safety_stop = bool(active)

    def set_traffic_stop(self, active):
        self.traffic_stop = bool(active)

    def set_intersection(self, active):
        self.intersection = bool(active)

    def set_roi_warning(self, active):
        self.roi_warning = bool(active)

    def evaluate_safety_status(self):
        if self.manual_stop:
            return SafetyStatus.STOP_MANUAL
        if self.generic_safety_stop:
            return SafetyStatus.STOP_GENERIC
        if self.traffic_stop and self.intersection:
            return SafetyStatus.STOP_TRAFFIC
        if self.roi_warning:
            return SafetyStatus.STOP_ROI
        return SafetyStatus.SAFE_OK

    def active_algorithm(self):
        if self.safety_active:
            return ActiveAlgorithm.SAFETY_HOLD
        if self.mission_state == MissionState.CITY:
            return ActiveAlgorithm.CITY_GPS_TRACKING
        if self.mission_state == MissionState.COMPLEX:
            return ActiveAlgorithm.COMPLEX_GPS_TARGETING
        return ActiveAlgorithm.HIGHWAY_GPS_TRACKING

    def applied_planning_throttle(self, now_sec):
        if self.latest_planning_stamp_sec is None:
            return 0.0, True

        age_sec = float(now_sec) - self.latest_planning_stamp_sec
        if age_sec > self.command_timeout_sec:
            return 0.0, True

        return max(0.0, float(self.latest_planning_throttle)), False

    def update(self, now_sec):
        now_sec = float(now_sec)
        current_safety_status = self.evaluate_safety_status()
        is_stop_signal_active = current_safety_status != SafetyStatus.SAFE_OK

        if is_stop_signal_active:
            if not self.safety_active:
                self.safety_active = True
                self.previous_mission_state = self.mission_state
            self.safety_status = current_safety_status
            self.release_timer_start_sec = None
        elif self.safety_active:
            if self.release_timer_start_sec is None:
                self.release_timer_start_sec = now_sec
            elif (
                now_sec - self.release_timer_start_sec
                >= self.release_hysteresis_sec
            ):
                self.safety_active = False
                self.safety_status = SafetyStatus.SAFE_OK
                self.release_timer_start_sec = None

        if not self.safety_active:
            self.mission_state = self.requested_mission_state

        planning_throttle, planning_timed_out = self.applied_planning_throttle(
            now_sec
        )
        policy = self.mission_policies[self.mission_state]
        mission_throttle = clamp(
            planning_throttle * policy.throttle_scale,
            0.0,
            policy.throttle_limit,
        )
        output_throttle = 0.0 if self.safety_active else mission_throttle

        return SupervisorSnapshot(
            mission_state=self.mission_state,
            requested_mission_state=self.requested_mission_state,
            previous_mission_state=self.previous_mission_state,
            safety_status=self.safety_status,
            safety_active=self.safety_active,
            active_algorithm=self.active_algorithm(),
            output_throttle=output_throttle,
            applied_planning_throttle=planning_throttle,
            planning_timed_out=planning_timed_out,
        )
