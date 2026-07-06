#!/usr/bin/env python3
import argparse
import struct
import subprocess
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Install it on Atlas with: pip3 install pyserial"
    ) from exc


ESP32_NAME = "1a86_USB_Serial"
CHECK_VAL = -12345
DEFAULT_SERVO = (90, 65)

PORT_CODE_FINDER = r"""
#!/bin/bash
for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && exit
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && exit
        echo "/dev/$devname - $ID_SERIAL"
    )
done
"""


def clamp(value, low=-100, high=100):
    return max(low, min(high, int(value)))


def official_udev_ports():
    try:
        output = subprocess.check_output(
            PORT_CODE_FINDER, shell=True, executable="/bin/bash"
        ).decode(errors="replace")
    except Exception:
        return []

    ports = []
    for line in output.splitlines():
        if " - " not in line:
            continue
        device, serial_name = line.split(" - ", 1)
        ports.append((device, serial_name))
    return ports


def find_esp32_port():
    for device, serial_name in official_udev_ports():
        if serial_name == ESP32_NAME or ESP32_NAME in serial_name:
            return device

    for port in list_ports.comports():
        text = " ".join(
            item for item in [port.device, port.description, port.hwid] if item
        )
        if ESP32_NAME in text or "CH340" in text or "USB Serial" in text:
            return port.device

    return None


def print_ports():
    print("udev ports:")
    for device, serial_name in official_udev_ports():
        print(f"  {device} - {serial_name}")

    print("pyserial ports:")
    for port in list_ports.comports():
        print(f"  {port.device} - {port.description} - {port.hwid}")


def motor_state(mode, speed):
    speed = clamp(speed)
    states = {
        "stop": (0, 0, 0, 0),
        "forward": (-speed, -speed, speed, speed),
        "back": (speed, speed, -speed, -speed),
        "spin_cw": (-speed, -speed, -speed, -speed),
        "spin_ccw": (speed, speed, speed, speed),
        "shift_left": (speed, -speed, -speed, speed),
        "shift_right": (-speed, speed, speed, -speed),
        "m1_pos": (speed, 0, 0, 0),
        "m1_neg": (-speed, 0, 0, 0),
        "m2_pos": (0, speed, 0, 0),
        "m2_neg": (0, -speed, 0, 0),
        "m3_pos": (0, 0, speed, 0),
        "m3_neg": (0, 0, -speed, 0),
        "m4_pos": (0, 0, 0, speed),
        "m4_neg": (0, 0, 0, -speed),
    }
    return states[mode]


def build_packet(mode, speed, servo):
    motors = [clamp(value) for value in motor_state(mode, speed)]
    state = motors + [int(servo[0]), int(servo[1]), CHECK_VAL]
    return state, struct.pack("<7h", *state)


def send_command(ser, mode, speed, servo, timeout_note=True):
    state, packet = build_packet(mode, speed, servo)
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    reply = ser.readline().decode(errors="replace").strip()
    if not reply and timeout_note:
        reply = "TIMEOUT"
    print(f"{mode:>11} {state} -> {reply}")
    return reply


def run_sequence(ser, speed, duration, servo):
    steps = [
        ("stop", 0.3),
        ("forward", duration),
        ("stop", 0.3),
        ("back", duration),
        ("stop", 0.3),
        ("spin_cw", duration),
        ("stop", 0.3),
        ("spin_ccw", duration),
        ("stop", 0.3),
    ]
    for mode, hold_time in steps:
        send_command(ser, mode, speed, servo)
        time.sleep(hold_time)
    send_command(ser, "stop", speed, servo)


def run_interactive(ser, speed, servo):
    names = [
        "stop",
        "forward",
        "back",
        "spin_cw",
        "spin_ccw",
        "shift_left",
        "shift_right",
        "m1_pos",
        "m1_neg",
        "m2_pos",
        "m2_neg",
        "m3_pos",
        "m3_neg",
        "m4_pos",
        "m4_neg",
    ]
    print("Commands:", ", ".join(names), "quit")
    while True:
        try:
            text = input("motion> ").strip()
        except (KeyboardInterrupt, EOFError):
            print()
            break
        if text in {"quit", "exit", "q"}:
            break
        if text not in names:
            print("unknown command")
            continue
        send_command(ser, text, speed, servo)
    send_command(ser, "stop", speed, servo)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Minimal Atlas-to-ESP32 motion test for the GMR TT motor car."
    )
    parser.add_argument("--port", help="ESP32 serial port, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--speed", type=int, default=25, help="motor command percent")
    parser.add_argument("--duration", type=float, default=0.8, help="seconds per move")
    parser.add_argument(
        "--mode",
        default="sequence",
        choices=[
            "sequence",
            "interactive",
            "stop",
            "forward",
            "back",
            "spin_cw",
            "spin_ccw",
            "shift_left",
            "shift_right",
            "m1_pos",
            "m1_neg",
            "m2_pos",
            "m2_neg",
            "m3_pos",
            "m3_neg",
            "m4_pos",
            "m4_neg",
        ],
    )
    parser.add_argument("--servo1", type=int, default=DEFAULT_SERVO[0])
    parser.add_argument("--servo2", type=int, default=DEFAULT_SERVO[1])
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument(
        "--no-reset-delay",
        action="store_true",
        help="skip the 2 second wait after opening the ESP32 serial port",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.list_ports:
        print_ports()
        return 0

    port = args.port or find_esp32_port()
    if not port:
        print("Could not find ESP32 serial port. Try --list-ports or --port /dev/ttyUSB0.")
        return 2

    servo = (args.servo1, args.servo2)
    print(f"Opening {port} at {args.baud} baud")
    with serial.Serial(port, args.baud, timeout=1.0) as ser:
        if not args.no_reset_delay:
            time.sleep(2.0)

        if args.mode == "sequence":
            run_sequence(ser, args.speed, args.duration, servo)
        elif args.mode == "interactive":
            run_interactive(ser, args.speed, servo)
        else:
            send_command(ser, args.mode, args.speed, servo)
            if args.mode != "stop":
                time.sleep(args.duration)
                send_command(ser, "stop", args.speed, servo)

    return 0


if __name__ == "__main__":
    sys.exit(main())
