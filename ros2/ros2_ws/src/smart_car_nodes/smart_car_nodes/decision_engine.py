import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

from smart_car_nodes.action_sequences import ActionRunner, build_legacy_command
from smart_car_nodes.protocol import CommandFrame


@dataclass(frozen=True)
class Detection:
    label: str
    score: float
    box: Tuple[int, int, int, int]

    @property
    def center(self):
        x1, y1, x2, y2 = self.box
        return ((x1 + x2) // 2, (y1 + y2) // 2)


@dataclass
class DecisionConfig:
    straight_speed: int = 26
    turn_speed: int = 20
    turn_trigger_deg: float = 15.0
    min_turn_strength: float = 0.25
    turn_gain: float = 1.5
    max_turn_strength: float = 1.0
    sign_score_threshold: float = 0.30
    turn_region_x_min: int = 420
    turn_region_x_max: int = 950
    turn_region_y_min: int = 270
    first_left_turn_count: int = 3
    human_score_threshold: float = 0.30
    human_region_x_min: int = 350
    human_region_x_max: int = 750
    human_region_y_min: int = 300
    human_clear_frames: int = 2
    park_score_threshold: float = 0.80
    park_region_x_min: int = 900
    park_region_y_min: int = 230
    park_right_turn_gate_enabled: bool = True
    min_right_turns_before_park: int = 2
    back_trigger_y: int = 300
    back_rearm_y: int = 240
    max_turnarounds: int = 2


@dataclass(frozen=True)
class DecisionOutput:
    command: CommandFrame
    status: str
    action: str
    pedestrian_blocked: bool
    turn_count: int
    right_turn_count: int


class DecisionEngine:
    def __init__(self, config: Optional[DecisionConfig] = None):
        self.config = config or DecisionConfig()
        self.enabled = False
        self.runner = ActionRunner()
        self.latest_lane = build_legacy_command("stop")
        self.latest_lane_action = "stop"
        self.pedestrian_blocked = False
        self.pedestrian_clear_frames = 0
        self.turn_count = 0
        self.right_turn_count = 0
        self.turnaround_count = 0
        self.back_marker_armed = True
        self.park_available = True
        self._latched_labels = set()

    @property
    def active_action(self):
        return self.runner.action_name

    def set_enabled(self, enabled: bool, now: float) -> None:
        self.enabled = bool(enabled)
        if self.enabled:
            self.runner.start("start", now)
        else:
            self.runner.cancel()

    def cancel_action(self) -> None:
        self.runner.cancel()

    def update_lane(self, filtered_steering: float) -> None:
        magnitude = abs(float(filtered_steering))
        trigger = math.radians(self.config.turn_trigger_deg)
        if magnitude <= trigger:
            self.latest_lane_action = "advance"
            self.latest_lane = build_legacy_command("advance", self.config.straight_speed)
            return
        strength = self.config.min_turn_strength + (magnitude - trigger) * self.config.turn_gain
        strength = min(max(strength, self.config.min_turn_strength), self.config.max_turn_strength)
        if filtered_steering > 0:
            self.latest_lane_action = "turn_right"
            self.latest_lane = build_legacy_command("turn_right", self.config.turn_speed, strength)
        else:
            self.latest_lane_action = "turn_left"
            self.latest_lane = build_legacy_command("turn_left", self.config.turn_speed, strength)

    def _human_detected(self, detections: Sequence[Detection]) -> bool:
        cfg = self.config
        for detection in detections:
            x, y = detection.center
            if (
                detection.label == "human"
                and detection.score >= cfg.human_score_threshold
                and cfg.human_region_x_min <= x <= cfg.human_region_x_max
                and y >= cfg.human_region_y_min
            ):
                return True
        return False

    def _update_pedestrian(self, detected: bool, now: float) -> None:
        if detected:
            self.pedestrian_clear_frames = 0
            if not self.pedestrian_blocked:
                self.pedestrian_blocked = True
                self.runner.pause(now)
            return
        if not self.pedestrian_blocked:
            return
        self.pedestrian_clear_frames += 1
        if self.pedestrian_clear_frames < self.config.human_clear_frames:
            return
        self.pedestrian_blocked = False
        self.pedestrian_clear_frames = 0
        if self.runner.active:
            self.runner.resume(now)
        elif self.enabled:
            self.runner.start("start", now)

    def update_signs(self, detections: Iterable[Detection], now: float) -> None:
        detections = sorted(list(detections), key=lambda item: item.score, reverse=True)
        self._update_pedestrian(self._human_detected(detections), now)
        if self.pedestrian_blocked or not self.enabled or self.runner.active:
            return

        trigger_labels = set()
        for detection in detections:
            x, y = detection.center
            if detection.score < self.config.sign_score_threshold:
                continue
            if detection.label in ("left", "right") and (
                self.config.turn_region_x_min < x < self.config.turn_region_x_max
                and y >= self.config.turn_region_y_min
            ):
                trigger_labels.add("turn")
                if "turn" not in self._latched_labels:
                    if self.turn_count < self.config.first_left_turn_count:
                        self.turn_count += 1
                        self.runner.start("left_turn", now)
                    else:
                        self.right_turn_count += 1
                        self.runner.start("right_turn", now)
                    break
            if detection.label == "park" and (
                detection.score >= self.config.park_score_threshold
                and x > self.config.park_region_x_min
                and y >= self.config.park_region_y_min
            ):
                trigger_labels.add("park")
                gate_open = (
                    not self.config.park_right_turn_gate_enabled
                    or self.right_turn_count >= self.config.min_right_turns_before_park
                )
                if self.park_available and gate_open and "park" not in self._latched_labels:
                    self.park_available = False
                    self.runner.start("park", now)
                    break
            if detection.label == "back" and self.config.turn_region_x_min < x < self.config.turn_region_x_max:
                trigger_labels.add("back")
                if not self.back_marker_armed and y <= self.config.back_rearm_y:
                    self.back_marker_armed = True
                if (
                    self.back_marker_armed
                    and y >= self.config.back_trigger_y
                    and self.turnaround_count < self.config.max_turnarounds
                    and "back" not in self._latched_labels
                ):
                    self.turnaround_count += 1
                    self.back_marker_armed = False
                    self.runner.start("turnaround_entry", now)
                    break
        self._latched_labels.intersection_update(trigger_labels)
        self._latched_labels.update(trigger_labels)

    def _output(self, command: CommandFrame, status: str, action: str) -> DecisionOutput:
        return DecisionOutput(
            command=command,
            status=status,
            action=action,
            pedestrian_blocked=self.pedestrian_blocked,
            turn_count=self.turn_count,
            right_turn_count=self.right_turn_count,
        )

    def tick(self, now: float) -> DecisionOutput:
        if not self.enabled:
            return self._output(build_legacy_command("stop"), "decision_disabled", "stop")
        if self.pedestrian_blocked:
            return self._output(build_legacy_command("stop"), "pedestrian_blocked", "stop")
        if self.runner.active:
            action_name = self.runner.action_name
            step = self.runner.current(now)
            if step is not None:
                return self._output(step.command, f"sequence:{action_name}", step.name)
            self.runner.cancel()
            if action_name != "start":
                self.runner.start("start", now)
                step = self.runner.current(now)
                return self._output(step.command, "sequence:start", step.name)
        return self._output(self.latest_lane, f"lane:{self.latest_lane_action}", self.latest_lane_action)
