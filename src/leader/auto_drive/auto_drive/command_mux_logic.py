from dataclasses import dataclass


@dataclass(frozen=True)
class PlannerCommand:
    steer: float
    throttle: float
    steer_stamp_sec: float
    throttle_stamp_sec: float


@dataclass(frozen=True)
class MuxResult:
    source: str
    steer: float
    throttle: float
    valid: bool
    reason: str


def parse_mission_state(text):
    normalized = str(text or '').strip().lower()
    if any(
        token in normalized
        for token in ('complex', 'cone', 'rrt', 'unstructured')
    ):
        return 'complex'
    if any(
        token in normalized
        for token in ('city', 'urban', 'intersection', 'crosswalk', 'traffic')
    ):
        return 'city'
    if any(token in normalized for token in ('highway', 'main road', 'freeway')):
        return 'highway'
    return 'highway'


class CommandMuxCore:
    def __init__(self, command_timeout_sec=0.5):
        self.command_timeout_sec = max(float(command_timeout_sec), 0.0)

    def select(self, mission_state, highway_command, complex_command, now_sec):
        mission = parse_mission_state(mission_state)
        if mission == 'complex':
            return self._select_source('complex', complex_command, now_sec)
        return self._select_source('highway', highway_command, now_sec)

    def _select_source(self, source, command, now_sec):
        if command is None:
            return MuxResult(source, 0.0, 0.0, False, f'{source}_missing')

        steer_age = float(now_sec) - float(command.steer_stamp_sec)
        throttle_age = float(now_sec) - float(command.throttle_stamp_sec)
        if (
            steer_age > self.command_timeout_sec
            or throttle_age > self.command_timeout_sec
        ):
            return MuxResult(source, 0.0, 0.0, False, f'{source}_stale')

        return MuxResult(
            source,
            float(command.steer),
            max(0.0, float(command.throttle)),
            True,
            'ok',
        )
