from dataclasses import dataclass
from typing import Callable, Dict, Optional

from smart_car_nodes.protocol import CommandFrame, command_from_parts


MotorBuilder = Callable[[int, float], list]


@dataclass(frozen=True)
class ManualAction:
    name: str
    build_motor: MotorBuilder
    degree: float = 0.1


def _forward(speed, degree):
    return [-speed, -speed, speed, speed]


def _back(speed, degree):
    return [speed, speed, -speed, -speed]


def _turn_left(speed, degree):
    outer = int(speed * (1 + degree))
    return [-speed, -speed, outer, outer]


def _turn_right(speed, degree):
    outer = int(speed * (1 + degree))
    return [-outer, -outer, speed, speed]


def _shift_left(speed, degree):
    return [speed, -speed, -speed, speed]


def _shift_right(speed, degree):
    return [-speed, speed, speed, -speed]


def _left_oblique(speed, degree):
    return [0, -speed, 0, speed]


def _right_oblique(speed, degree):
    return [-speed, 0, speed, 0]


def _spin_clockwise(speed, degree):
    return [-speed, -speed, -speed, -speed]


def _spin_anticlockwise(speed, degree):
    return [speed, speed, speed, speed]


def _stop(speed, degree):
    return [0, 0, 0, 0]


MANUAL_ACTIONS: Dict[str, ManualAction] = {
    "w": ManualAction("forward", _forward),
    "s": ManualAction("back", _back),
    "a": ManualAction("turn_left", _turn_left),
    "d": ManualAction("turn_right", _turn_right),
    "j": ManualAction("shift_left", _shift_left),
    "left": ManualAction("shift_left", _shift_left),
    "l": ManualAction("shift_right", _shift_right),
    "right": ManualAction("shift_right", _shift_right),
    "u": ManualAction("left_oblique", _left_oblique),
    "p": ManualAction("right_oblique", _right_oblique),
    "q": ManualAction("spin_anticlockwise", _spin_anticlockwise),
    "e": ManualAction("spin_clockwise", _spin_clockwise),
    "space": ManualAction("stop", _stop),
}


def action_from_key(key: str) -> Optional[ManualAction]:
    return MANUAL_ACTIONS.get(key)


def build_manual_command(key: str, speed: int, servo=None, degree=None) -> Optional[CommandFrame]:
    action = action_from_key(key)
    if action is None:
        return None
    use_degree = action.degree if degree is None else float(degree)
    motor = action.build_motor(int(speed), use_degree)
    return command_from_parts(motor, servo=servo, source=f"manual:{action.name}")


def apply_motion_gate(command: CommandFrame, enabled: bool) -> CommandFrame:
    if enabled:
        return command
    return command_from_parts([0, 0, 0, 0], servo=command.servo, source=command.source)


@dataclass
class MotionGate:
    enabled: bool = True

    def set_enabled(self, enabled: bool):
        self.enabled = bool(enabled)

    def apply(self, command: CommandFrame) -> CommandFrame:
        return apply_motion_gate(command, enabled=self.enabled)
