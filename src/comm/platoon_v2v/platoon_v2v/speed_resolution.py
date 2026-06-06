import math
from typing import Optional


def signed_speed_from_telemetry(
    speed_estimate_mps: float,
    throttle_norm: float,
    rc_throttle_us: Optional[int] = None,
    *,
    speed_deadband_mps: float = 0.02,
    throttle_deadband: float = 0.05,
    throttle_speed_gain_mps: float = 0.50,
    rc_throttle_neutral_us: int = 1500,
    rc_throttle_deadband_us: int = 80,
    rc_throttle_full_scale_us: int = 400,
) -> float:
    """Resolve signed leader speed when encoder estimate loses reverse sign."""
    speed = float(speed_estimate_mps)
    if not math.isfinite(speed):
        speed = 0.0

    throttle = _signed_throttle(
        throttle_norm,
        rc_throttle_us,
        throttle_deadband=throttle_deadband,
        rc_throttle_neutral_us=rc_throttle_neutral_us,
        rc_throttle_deadband_us=rc_throttle_deadband_us,
        rc_throttle_full_scale_us=rc_throttle_full_scale_us,
    )
    if throttle == 0.0:
        return speed

    if abs(speed) > speed_deadband_mps:
        return math.copysign(abs(speed), throttle)

    return throttle * throttle_speed_gain_mps


def _signed_throttle(
    throttle_norm: float,
    rc_throttle_us: Optional[int],
    *,
    throttle_deadband: float,
    rc_throttle_neutral_us: int,
    rc_throttle_deadband_us: int,
    rc_throttle_full_scale_us: int,
) -> float:
    throttle = float(throttle_norm)
    if math.isfinite(throttle) and abs(throttle) > throttle_deadband:
        return max(-1.0, min(1.0, throttle))

    if rc_throttle_us is None:
        return 0.0
    delta_us = int(rc_throttle_us) - int(rc_throttle_neutral_us)
    if abs(delta_us) <= int(rc_throttle_deadband_us):
        return 0.0
    scale = max(1, int(rc_throttle_full_scale_us))
    return max(-1.0, min(1.0, delta_us / scale))
