from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

from smart_car_nodes.protocol import CommandFrame, command_from_parts


LEGACY_MOTOR_RATINGS = (0.96, 0.96, 0.8, 0.8)
LEGACY_SERVO = (93, 162)


@dataclass(frozen=True)
class ActionStep:
    name: str
    command: CommandFrame
    duration: float


def _rated(values: Sequence[int]) -> List[int]:
    return [int(value * rating) for value, rating in zip(values, LEGACY_MOTOR_RATINGS)]


def build_legacy_command(action: str, speed: int = 0, degree: float = 0.0) -> CommandFrame:
    speed = int(speed)
    if action == "advance":
        motor = [-speed, -speed, speed, speed]
    elif action == "back":
        motor = [speed, speed, -speed, -speed]
    elif action == "turn_left":
        motor = [-speed, -speed, int(speed * (1.0 + degree)), int(speed * (1.0 + degree))]
    elif action == "turn_right":
        motor = [-int(speed * (1.0 + degree)), -int(speed * (1.0 + degree)), speed, speed]
    elif action == "shift_left":
        motor = [speed, -speed, -speed, speed]
    elif action == "shift_right":
        motor = [-speed, speed, speed, -speed]
    elif action == "spin_clockwise":
        motor = [-speed, -speed, -speed, -speed]
    elif action == "spin_anticlockwise":
        motor = [speed, speed, speed, speed]
    elif action == "stop":
        motor = [0, 0, 0, 0]
    else:
        raise ValueError(f"unknown legacy action: {action}")
    return command_from_parts(_rated(motor), servo=LEGACY_SERVO, source=f"decision:{action}")


def _step(name: str, speed: int, duration: float, degree: float = 0.0) -> ActionStep:
    return ActionStep(name, build_legacy_command(name, speed, degree), float(duration))


ACTION_SPECS: Dict[str, Tuple[Tuple[str, int, float, float], ...]] = {
    "start": (
        ("advance", 35, 0.2, 0.0),
        ("advance", 25, 0.1, 0.0),
    ),
    "left_turn": (
        ("advance", 28, 1.3, 0.0),
        ("spin_anticlockwise", 32, 0.65, 0.0),
        ("stop", 0, 0.2, 0.0),
    ),
    "right_turn": (
        ("advance", 27, 1.3, 0.0),
        ("spin_clockwise", 32, 0.6, 0.0),
        ("stop", 0, 0.2, 0.0),
    ),
    "turnaround_entry": (
        ("advance", 32, 1.0, 0.0),
        ("stop", 0, 1.5, 0.0),
        ("advance", 32, 0.5, 0.0),
        ("turn_left", 15, 1.6, 3.0),
        ("stop", 0, 0.0, 0.0),
    ),
    "turnaround_finish": (
        ("advance", 30, 1.0, 0.0),
        ("stop", 0, 15.0, 0.0),
    ),
    "park": (
        ("advance", 32, 1.1, 0.0),
        ("stop", 0, 1.0, 0.0),
        ("shift_right", 40, 1.5, 0.0),
        ("stop", 0, 2.0, 0.0),
        ("shift_left", 40, 1.5, 0.0),
        ("advance", 30, 0.5, 0.0),
    ),
    "stop_sign": (("stop", 0, 2.0, 0.0),),
}


def build_action_sequence(name: str) -> List[ActionStep]:
    try:
        specs = ACTION_SPECS[name]
    except KeyError as exc:
        raise ValueError(f"unknown action sequence: {name}") from exc
    return [_step(action, speed, duration, degree) for action, speed, duration, degree in specs]


class ActionRunner:
    def __init__(self):
        self.action_name = None
        self.steps: List[ActionStep] = []
        self.step_index = 0
        self.step_started_at = 0.0
        self.paused_at = None
        self.zero_step_emitted = False

    @property
    def active(self) -> bool:
        return self.action_name is not None

    def start(self, action_name: str, now: float) -> None:
        self.action_name = action_name
        self.steps = build_action_sequence(action_name)
        self.step_index = 0
        self.step_started_at = float(now)
        self.paused_at = None
        self.zero_step_emitted = False

    def cancel(self) -> None:
        self.action_name = None
        self.steps = []
        self.step_index = 0
        self.paused_at = None
        self.zero_step_emitted = False

    def pause(self, now: float) -> None:
        if self.active and self.paused_at is None:
            self.paused_at = float(now)

    def resume(self, now: float) -> None:
        if self.paused_at is not None:
            self.step_started_at += float(now) - self.paused_at
            self.paused_at = None

    def current(self, now: float):
        if not self.active:
            return None
        effective_now = self.paused_at if self.paused_at is not None else float(now)
        while self.step_index < len(self.steps):
            step = self.steps[self.step_index]
            if step.duration <= 0.0:
                if self.zero_step_emitted:
                    self.step_index += 1
                    self.zero_step_emitted = False
                    if self.step_index >= len(self.steps):
                        return None
                    continue
                self.zero_step_emitted = True
                return step
            if effective_now - self.step_started_at < step.duration:
                return step
            self.step_started_at += step.duration
            self.step_index += 1
            self.zero_step_emitted = False
        return None
