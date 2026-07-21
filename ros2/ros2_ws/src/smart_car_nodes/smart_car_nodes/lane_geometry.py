import math
from typing import Iterable, List, Sequence, Tuple


LaneTuple = Tuple[float, float, float]


def transform_lane_coords(lanes: Iterable[Sequence[float]], model_dims, image_dims) -> List[LaneTuple]:
    model_w, model_h = model_dims
    image_w, image_h = image_dims
    if model_w <= 0 or model_h <= 0:
        return [(float(k), float(b), float(conf)) for k, b, conf in lanes]

    scale_x = image_w / model_w
    scale_y = image_h / model_h
    if abs(scale_y) < 1e-6:
        return []

    return [
        (float(k) * (scale_x / scale_y), float(b) * scale_x, float(conf))
        for k, b, conf in lanes
    ]


def compute_steering_command(lanes: Iterable[Sequence[float]], image_width: int, image_height: int) -> float:
    lane_params = sorted(
        [(float(k), float(b), float(conf)) for k, b, conf in lanes],
        key=lambda item: item[2],
        reverse=True,
    )[:2]
    if not lane_params:
        return 0.0

    left_line = None
    right_line = None
    for k, b, _ in lane_params:
        if k > 0:
            left_line = (k, b)
        elif k < 0:
            right_line = (k, b)

    if left_line and right_line:
        k1, b1 = left_line
        k2, b2 = right_line
        if abs(k1 - k2) < 1e-6:
            return 0.0
        inter_y = (b2 - b1) / (k1 - k2)
        inter_x = k1 * inter_y + b1
        base_x = image_width / 2.0
        bottom_y = float(image_height)
        dx = inter_x - base_x
        dy = inter_y - bottom_y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return 0.0
        cos_theta = max(min((-dy) / norm, 1.0), -1.0)
        angle = math.acos(cos_theta)
        return -angle if dx < 0 else angle

    k, _, _ = lane_params[0]
    cos_theta = 1.0 / math.sqrt(k * k + 1.0)
    angle = math.acos(max(min(cos_theta, 1.0), -1.0))
    return angle if k < 0 else -angle


def compute_lateral_steering(
    lanes: Iterable[Sequence[float]],
    image_width: int,
    image_height: int,
    lookahead_ratio: float = 0.75,
    target_x_ratio: float = 0.50,
    deadband_px: float = 10.0,
    lateral_gain: float = 1.6,
):
    lane_params = sorted(
        [(float(k), float(b), float(conf)) for k, b, conf in lanes],
        key=lambda item: item[2],
        reverse=True,
    )[:2]
    left_line = next(((k, b) for k, b, _ in lane_params if k > 0), None)
    right_line = next(((k, b) for k, b, _ in lane_params if k < 0), None)
    target_x = float(image_width) * float(target_x_ratio)
    if left_line is None or right_line is None:
        return 0.0, None, None, target_x
    y = float(image_height) * float(lookahead_ratio)
    lane_center_x = ((left_line[0] * y + left_line[1]) + (right_line[0] * y + right_line[1])) / 2.0
    lateral_error_px = lane_center_x - target_x
    if abs(lateral_error_px) <= float(deadband_px):
        return 0.0, lateral_error_px, lane_center_x, target_x
    steering = (lateral_error_px / float(image_width)) * float(lateral_gain)
    return steering, lateral_error_px, lane_center_x, target_x


def combine_steering(heading_steering: float, lateral_steering: float) -> float:
    return float(heading_steering) + float(lateral_steering)


def compute_turn_control(
    steering_rad: float,
    turn_trigger_deg: float = 15.0,
    turn_gain: float = 1.5,
    min_turn_strength: float = 0.25,
    max_turn_strength: float = 1.0,
):
    magnitude = abs(float(steering_rad))
    trigger = math.radians(float(turn_trigger_deg))
    if magnitude <= trigger:
        return False, 0.0
    strength = float(min_turn_strength) + (magnitude - trigger) * float(turn_gain)
    return True, min(strength, float(max_turn_strength))
