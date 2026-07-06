#!/usr/bin/env python3
import argparse
import csv
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


WHEEL_COUNT = 4


def default_log_path() -> Path:
    return Path(__file__).resolve().parents[1] / "motor_encoder_test" / "log.txt"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def sign_of(value: float) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


@dataclass
class MotorCalibration:
    samples: Dict[Tuple[int, int], List[float]] = field(default_factory=dict)

    def add_sample(self, motor: int, pwm: int, wheel_rpm: float) -> None:
        self.samples.setdefault((motor, pwm), []).append(wheel_rpm)

    def avg_rpm_for(self, motor: int, pwm: int) -> float:
        values = self.samples[(motor, pwm)]
        return sum(values) / len(values)

    def available_pwms(self, motor: int, direction: int) -> List[int]:
        return sorted(
            pwm
            for sample_motor, pwm in self.samples
            if sample_motor == motor and sign_of(pwm) == direction
        )

    def reference_for(self, motor: int, pwm: float) -> Tuple[float, float]:
        direction = sign_of(pwm)
        if direction == 0:
            return 0.0, 0.0

        same_direction = self.available_pwms(motor, direction)
        if same_direction:
            reference_pwm = max(same_direction, key=lambda value: abs(value))
            return float(reference_pwm), self.avg_rpm_for(motor, reference_pwm)

        opposite_direction = self.available_pwms(motor, -direction)
        if opposite_direction:
            reference_pwm = max(opposite_direction, key=lambda value: abs(value))
            reference_rpm = abs(self.avg_rpm_for(motor, reference_pwm)) * direction
            return float(abs(reference_pwm) * direction), reference_rpm

        raise KeyError(f"No calibration sample for M{motor}")

    def steady_rpm_for(self, motor: int, pwm: float) -> float:
        if abs(pwm) < 0.001:
            return 0.0
        reference_pwm, reference_rpm = self.reference_for(motor, pwm)
        if abs(reference_pwm) < 0.001:
            return 0.0
        return reference_rpm * (pwm / reference_pwm)


def parse_motor_log(path: Path) -> MotorCalibration:
    calibration = MotorCalibration()
    active_motor: Optional[int] = None
    active_pwm: Optional[int] = None
    command_re = re.compile(r"^M([1-4]) pwm=(-?\d+) driver=(-?\d+)")
    csv_re = re.compile(
        r"^M([1-4]),(-?\d+),(-?\d+),(-?\d+),(-?[0-9.]+),(-?[0-9.]+)"
    )

    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        command_match = command_re.match(line.strip())
        if command_match:
            active_motor = int(command_match.group(1))
            active_pwm = int(command_match.group(2))
            continue

        csv_match = csv_re.match(line.strip())
        if not csv_match:
            continue

        motor = int(csv_match.group(1))
        pwm = int(csv_match.group(2))
        delta = int(csv_match.group(4))
        wheel_rpm = float(csv_match.group(6))

        if active_motor != motor or active_pwm != pwm:
            continue
        if pwm == 0 or abs(delta) <= 3:
            continue
        calibration.add_sample(motor, pwm, wheel_rpm)

    if not calibration.samples:
        raise ValueError(f"No motor calibration samples found in {path}")

    return calibration


@dataclass
class SimulationConfig:
    max_wheel_rpm: float = 70.0
    control_period_ms: int = 50
    transition_pwm: float = 32.0
    transition_ms: int = 100
    pid_handoff_pwm: float = 18.0
    pid_kp: float = 0.22
    pid_ki: float = 0.10
    integral_limit: float = 90.0
    plant_time_constant_s: float = 0.6


@dataclass
class WheelState:
    target_rpm: float = 0.0
    measured_rpm: float = 0.0
    actual_rpm: float = 0.0
    integral: float = 0.0
    pwm: int = 0
    transition_active: bool = False
    transition_start_ms: int = 0


@dataclass
class SimulationResult:
    rows: List[Dict[str, float]]
    command: Sequence[int]

    def rows_for(self, motor: int) -> List[Dict[str, float]]:
        return [row for row in self.rows if int(row["motor"]) == motor]

    def summary(self) -> Dict[str, Dict[str, float]]:
        output: Dict[str, Dict[str, float]] = {}
        for motor in range(1, WHEEL_COUNT + 1):
            rows = self.rows_for(motor)
            if not rows:
                continue
            row_1s = min(rows, key=lambda row: abs(row["time_s"] - 1.0))
            end = rows[-1]
            output[f"M{motor}"] = {
                "target_rpm": end["target_rpm"],
                "pwm_at_1s": row_1s["pwm"],
                "end_pwm": end["pwm"],
                "max_pwm": max(row["pwm"] for row in rows),
                "max_abs_pwm": max(abs(row["pwm"]) for row in rows),
                "end_actual_rpm": end["actual_rpm"],
                "end_measured_rpm": end["measured_rpm"],
                "end_integral": end["integral"],
            }
        return output


def round_to_int(value: float) -> int:
    return int(round(value))


def reset_wheel_pid(wheel: WheelState) -> None:
    wheel.integral = 0.0
    wheel.measured_rpm = 0.0
    wheel.pwm = 0
    wheel.transition_active = False


def begin_transition(wheel: WheelState, now_ms: int) -> None:
    wheel.integral = 0.0
    wheel.transition_active = True
    wheel.transition_start_ms = now_ms


def prepare_pid_handoff(wheel: WheelState, config: SimulationConfig) -> None:
    target = wheel.target_rpm
    feed_forward = target * 100.0 / config.max_wheel_rpm
    error = target - wheel.measured_rpm
    handoff_command = config.pid_handoff_pwm if target > 0.0 else -config.pid_handoff_pwm

    if abs(feed_forward) > config.pid_handoff_pwm:
        handoff_command = feed_forward

    if abs(config.pid_ki) < 0.000001:
        wheel.integral = 0.0
        return

    wheel.integral = (handoff_command - feed_forward - config.pid_kp * error) / config.pid_ki
    wheel.integral = clamp(wheel.integral, -config.integral_limit, config.integral_limit)


def set_wheel_target(wheel: WheelState, percent: float, now_ms: int, config: SimulationConfig) -> None:
    percent = clamp(percent, -100.0, 100.0)
    wheel.target_rpm = percent * config.max_wheel_rpm / 100.0

    if abs(percent) < 0.1:
        reset_wheel_pid(wheel)
        return

    begin_transition(wheel, now_ms)


def step_motor_plant(
    calibration: MotorCalibration,
    wheel: WheelState,
    motor: int,
    dt_s: float,
    config: SimulationConfig,
) -> None:
    steady_rpm = calibration.steady_rpm_for(motor, wheel.pwm)
    alpha = dt_s / (config.plant_time_constant_s + dt_s)
    wheel.actual_rpm += (steady_rpm - wheel.actual_rpm) * alpha


def step_controller(wheel: WheelState, now_ms: int, dt_s: float, config: SimulationConfig) -> None:
    target = wheel.target_rpm
    if abs(target) < 0.5:
        reset_wheel_pid(wheel)
        return

    instant_rpm = abs(wheel.actual_rpm) if target > 0.0 else -abs(wheel.actual_rpm)
    wheel.measured_rpm = 0.65 * wheel.measured_rpm + 0.35 * instant_rpm

    if wheel.transition_active:
        if now_ms - wheel.transition_start_ms < config.transition_ms:
            wheel.pwm = round_to_int(config.transition_pwm) * (1 if target > 0.0 else -1)
            return
        wheel.transition_active = False
        prepare_pid_handoff(wheel, config)

    error = target - wheel.measured_rpm
    feed_forward = target * 100.0 / config.max_wheel_rpm
    next_integral = clamp(
        wheel.integral + error * dt_s,
        -config.integral_limit,
        config.integral_limit,
    )
    raw_command = feed_forward + config.pid_kp * error + config.pid_ki * next_integral
    command = clamp(raw_command, -100.0, 100.0)

    saturated_high = raw_command > 100.0
    saturated_low = raw_command < -100.0
    error_pulls_back = (saturated_high and error < 0.0) or (saturated_low and error > 0.0)
    if (not saturated_high and not saturated_low) or error_pulls_back:
        wheel.integral = next_integral

    wheel.pwm = round_to_int(command)


def run_simulation(
    calibration: MotorCalibration,
    command: Sequence[int],
    seconds: float,
    config: SimulationConfig,
) -> SimulationResult:
    if len(command) != WHEEL_COUNT:
        raise ValueError("command must contain 4 motor values")

    dt_s = config.control_period_ms / 1000.0
    steps = max(1, int(math.ceil(seconds / dt_s)))
    wheels = [WheelState() for _ in range(WHEEL_COUNT)]
    rows: List[Dict[str, float]] = []

    for index, value in enumerate(command):
        set_wheel_target(wheels[index], float(value), 0, config)

    for step in range(steps + 1):
        now_ms = round_to_int(step * config.control_period_ms)
        time_s = now_ms / 1000.0

        for index, wheel in enumerate(wheels, start=1):
            step_motor_plant(calibration, wheel, index, dt_s, config)
            step_controller(wheel, now_ms, dt_s, config)
            rows.append(
                {
                    "time_s": time_s,
                    "motor": float(index),
                    "target_rpm": wheel.target_rpm,
                    "actual_rpm": wheel.actual_rpm,
                    "measured_rpm": wheel.measured_rpm,
                    "integral": wheel.integral,
                    "pwm": float(wheel.pwm),
                }
            )

    return SimulationResult(rows=rows, command=command)


def command_from_args(args: argparse.Namespace) -> List[int]:
    if args.command:
        return args.command

    speed = int(args.speed)
    motions = {
        "forward": [-speed, -speed, speed, speed],
        "back": [speed, speed, -speed, -speed],
        "spin_cw": [-speed, -speed, -speed, -speed],
        "spin_ccw": [speed, speed, speed, speed],
        "all": [speed, speed, speed, speed],
    }
    return motions[args.motion]


def print_calibration(calibration: MotorCalibration) -> None:
    print("Calibration averages from log:")
    for motor in range(1, WHEEL_COUNT + 1):
        parts = []
        for direction_pwm in (-45, 45):
            try:
                parts.append(f"{direction_pwm:+d}: {calibration.avg_rpm_for(motor, direction_pwm):.2f} rpm")
            except KeyError:
                pass
        print(f"  M{motor}: " + (", ".join(parts) if parts else "no samples"))


def print_summary(result: SimulationResult) -> None:
    print("\nSimulation summary:")
    print("motor target_rpm pwm@1s end_pwm max_abs_pwm end_actual end_measured integral trend")
    for motor, item in result.summary().items():
        trend = "up" if abs(item["end_pwm"]) > abs(item["pwm_at_1s"]) + 5 else "stable"
        print(
            f"{motor:>5} "
            f"{item['target_rpm']:>10.2f} "
            f"{item['pwm_at_1s']:>6.0f} "
            f"{item['end_pwm']:>7.0f} "
            f"{item['max_abs_pwm']:>11.0f} "
            f"{item['end_actual_rpm']:>10.2f} "
            f"{item['end_measured_rpm']:>12.2f} "
            f"{item['end_integral']:>8.2f} "
            f"{trend}"
        )


def write_csv(path: Path, rows: Iterable[Dict[str, float]]) -> None:
    fieldnames = ["time_s", "motor", "target_rpm", "actual_rpm", "measured_rpm", "integral", "pwm"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulate the ESP32 GMR motor PID loop on PC.")
    parser.add_argument(
        "--log",
        type=Path,
        default=default_log_path(),
        help="motor_encoder_test log path",
    )
    parser.add_argument("--seconds", type=float, default=8.0, help="simulation duration")
    parser.add_argument("--motion", choices=["forward", "back", "spin_cw", "spin_ccw", "all"], default="forward")
    parser.add_argument("--speed", type=int, default=14, help="Atlas motor command magnitude")
    parser.add_argument("--command", type=int, nargs=4, metavar=("M1", "M2", "M3", "M4"))
    parser.add_argument("--max-wheel-rpm", type=float, default=70.0)
    parser.add_argument("--kp", type=float, default=0.22)
    parser.add_argument("--ki", type=float, default=0.10)
    parser.add_argument("--integral-limit", type=float, default=90.0)
    parser.add_argument("--transition-pwm", type=float, default=32.0)
    parser.add_argument("--transition-ms", type=int, default=100)
    parser.add_argument("--handoff-pwm", type=float, default=18.0)
    parser.add_argument("--plant-tau", type=float, default=0.6, help="motor response time constant in seconds")
    parser.add_argument("--csv", type=Path, help="write detailed per-step data to CSV")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    calibration = parse_motor_log(args.log)
    config = SimulationConfig(
        max_wheel_rpm=args.max_wheel_rpm,
        transition_pwm=args.transition_pwm,
        transition_ms=args.transition_ms,
        pid_handoff_pwm=args.handoff_pwm,
        pid_kp=args.kp,
        pid_ki=args.ki,
        integral_limit=args.integral_limit,
        plant_time_constant_s=args.plant_tau,
    )
    command = command_from_args(args)

    print_calibration(calibration)
    print(
        "\nConfig: "
        f"command={command}, max_wheel_rpm={config.max_wheel_rpm}, "
        f"kp={config.pid_kp}, ki={config.pid_ki}, integral_limit={config.integral_limit}"
    )
    result = run_simulation(calibration, command, args.seconds, config)
    print_summary(result)

    if args.csv:
        write_csv(args.csv, result.rows)
        print(f"\nCSV written to {args.csv}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
