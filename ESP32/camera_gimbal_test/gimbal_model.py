"""Host-side reference for the camera gimbal limits and sweep order."""

from typing import List, Tuple

PAN_MIN_ANGLE = 0
PAN_MAX_ANGLE = 180
TILT_MIN_ANGLE = 90
TILT_MAX_ANGLE = 162


def clamp_pan_angle(angle: int) -> int:
    return max(PAN_MIN_ANGLE, min(PAN_MAX_ANGLE, angle))


def clamp_tilt_angle(angle: int) -> int:
    return max(TILT_MIN_ANGLE, min(TILT_MAX_ANGLE, angle))


def build_sweep_sequence(pan_center: int, tilt_center: int) -> List[Tuple[int, int]]:
    pan_center = clamp_pan_angle(pan_center)
    tilt_center = clamp_tilt_angle(tilt_center)
    return [
        (PAN_MIN_ANGLE, tilt_center),
        (PAN_MAX_ANGLE, tilt_center),
        (pan_center, tilt_center),
        (pan_center, TILT_MIN_ANGLE),
        (pan_center, TILT_MAX_ANGLE),
        (pan_center, tilt_center),
    ]
