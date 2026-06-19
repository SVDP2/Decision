from dataclasses import dataclass


@dataclass(frozen=True)
class TrafficSignalDecision:
    stop_required: bool
    state: str
    fresh: bool


class TrafficSignalGateCore:
    """Fail-safe traffic-signal state aggregation without ROS dependencies."""

    def __init__(self, signal_timeout_sec=5.0):
        self.signal_timeout_sec = max(float(signal_timeout_sec), 0.0)
        self.signal_present = None
        self.signal_red = None
        self.signal_green = None
        self.present_stamp_sec = None
        self.red_stamp_sec = None
        self.green_stamp_sec = None

    def set_signal_present(self, active, now_sec):
        self.signal_present = bool(active)
        self.present_stamp_sec = float(now_sec)

    def set_signal_red(self, active, now_sec):
        self.signal_red = bool(active)
        self.red_stamp_sec = float(now_sec)

    def set_signal_green(self, active, now_sec):
        self.signal_green = bool(active)
        self.green_stamp_sec = float(now_sec)

    def evaluate(self, now_sec):
        now_sec = float(now_sec)

        # A fresh red observation always wins, including contradictory inputs.
        if self.signal_red and self._is_stamp_fresh(
            self.red_stamp_sec, now_sec
        ):
            state = 'CONFLICT' if self.signal_green else 'RED'
            return TrafficSignalDecision(True, state, True)

        if not self._all_inputs_fresh(now_sec):
            return TrafficSignalDecision(True, 'STALE_OR_MISSING', False)

        if not self.signal_present:
            return TrafficSignalDecision(True, 'NOT_DETECTED', True)

        if self.signal_green and not self.signal_red:
            return TrafficSignalDecision(False, 'GREEN', True)

        return TrafficSignalDecision(True, 'UNKNOWN', True)

    def _all_inputs_fresh(self, now_sec):
        return all(
            self._is_stamp_fresh(stamp, now_sec)
            for stamp in (
                self.present_stamp_sec,
                self.red_stamp_sec,
                self.green_stamp_sec,
            )
        )

    def _is_stamp_fresh(self, stamp_sec, now_sec):
        if stamp_sec is None:
            return False
        age_sec = float(now_sec) - float(stamp_sec)
        return 0.0 <= age_sec <= self.signal_timeout_sec
