from dataclasses import dataclass
from typing import Optional

from smart_car_nodes.protocol import CommandFrame, pack_command
from smart_car_nodes.encoder_pid_telemetry import TelemetryStreamParser


@dataclass
class SerialConfig:
    port: str = ""
    baudrate: int = 115200
    esp32_name: str = "1a86_USB_Serial"
    timeout: float = 1.0


def find_serial_port(esp32_name: str = "1a86_USB_Serial") -> Optional[str]:
    try:
        from serial.tools import list_ports
    except Exception:
        return None

    for port in list_ports.comports():
        text = " ".join(
            str(value)
            for value in [
                getattr(port, "device", ""),
                getattr(port, "description", ""),
                getattr(port, "hwid", ""),
                getattr(port, "serial_number", ""),
            ]
            if value
        )
        if esp32_name in text:
            return port.device
    return None


class Esp32SerialClient:
    def __init__(self, config: SerialConfig):
        self.config = config
        self._serial = None
        self.port = ""
        self._telemetry_parser = TelemetryStreamParser()
        self._telemetry_frames = []

    def connect(self):
        if self._serial is not None:
            return
        try:
            import serial
        except Exception as exc:
            raise RuntimeError("pyserial is required to talk to ESP32") from exc

        port = self.config.port or find_serial_port(self.config.esp32_name)
        if not port:
            raise RuntimeError(f"cannot find ESP32 serial port matching {self.config.esp32_name}")
        self._serial = serial.Serial(port, self.config.baudrate, timeout=self.config.timeout)
        self.port = port
        self._telemetry_parser = TelemetryStreamParser()

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def send(self, command: CommandFrame) -> str:
        self.connect()
        self._serial.write(pack_command(command))
        deadline = __import__("time").time() + max(float(self.config.timeout), 0.1)
        while __import__("time").time() < deadline:
            waiting = getattr(self._serial, "in_waiting", 0)
            chunk = self._serial.read(waiting or 1)
            frames, lines = self._telemetry_parser.feed(chunk)
            self._telemetry_frames.extend(frames)
            if lines:
                return lines[-1]
        return "TIMEOUT"

    def drain_telemetry(self):
        if self._serial is not None:
            waiting = getattr(self._serial, "in_waiting", 0)
            if waiting:
                frames, _ = self._telemetry_parser.feed(self._serial.read(waiting))
                self._telemetry_frames.extend(frames)
        frames = self._telemetry_frames
        self._telemetry_frames = []
        return frames

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
