import struct
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence


CHECK_VALUE = -12345
DEFAULT_SERVO = [93, 162]
FRAME_VALUE_COUNT = 7
FRAME_BYTE_COUNT = FRAME_VALUE_COUNT * 2
INT16_MIN = -32768
INT16_MAX = 32767


def _as_int16_list(values: Iterable[int], name: str, expected_len: int) -> List[int]:
    result = [int(value) for value in values]
    if len(result) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values, got {len(result)}")
    for value in result:
        if value < INT16_MIN or value > INT16_MAX:
            raise ValueError(f"{name} value {value} is outside int16 range")
    return result


@dataclass(frozen=True)
class CommandFrame:
    motor: Sequence[int]
    servo: Sequence[int] = field(default_factory=lambda: list(DEFAULT_SERVO))
    check_value: int = CHECK_VALUE
    source: str = ""

    def __post_init__(self):
        object.__setattr__(self, "motor", _as_int16_list(self.motor, "motor", 4))
        object.__setattr__(self, "servo", _as_int16_list(self.servo, "servo", 2))
        check_value = int(self.check_value)
        if check_value < INT16_MIN or check_value > INT16_MAX:
            raise ValueError(f"check_value {check_value} is outside int16 range")
        object.__setattr__(self, "check_value", check_value)

    @property
    def values(self) -> List[int]:
        return list(self.motor) + list(self.servo) + [self.check_value]


def command_from_parts(motor, servo=None, source="") -> CommandFrame:
    return CommandFrame(
        motor=motor,
        servo=list(DEFAULT_SERVO) if servo is None else servo,
        check_value=CHECK_VALUE,
        source=source,
    )


def pack_command(command: CommandFrame) -> bytes:
    return struct.pack("<7h", *command.values)


def unpack_command(payload: bytes) -> CommandFrame:
    if len(payload) != FRAME_BYTE_COUNT:
        raise ValueError(f"payload must be {FRAME_BYTE_COUNT} bytes, got {len(payload)}")
    values = list(struct.unpack("<7h", payload))
    return CommandFrame(motor=values[:4], servo=values[4:6], check_value=values[6])
