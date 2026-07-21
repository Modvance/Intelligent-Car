"""Encoder PID telemetry codec shared by the ESP32 serial consumers.

ESP32 keeps the legacy 14-byte command input unchanged. This module parses its
separate framed output stream while preserving ``SUCC``/``FAIL`` ACK lines.
"""

import binascii
import struct


FRAME_MAGIC = b"\xA5\x5A"
FRAME_VERSION = 1
WHEEL_COUNT = 4
RPM_SCALE = 100
TERM_SCALE = 100
HEADER_SIZE = 4
CRC_SIZE = 2
WHEEL_FORMAT = "<hhhhhhhhB"
WHEEL_SIZE = struct.calcsize(WHEEL_FORMAT)
PAYLOAD_HEADER_FORMAT = "<IHH"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_HEADER_FORMAT) + WHEEL_COUNT * WHEEL_SIZE
MAX_PAYLOAD_SIZE = 128
FLAG_SATURATED = 0x01
FLAG_START_PULSE = 0x02


def _clamp_i16(value):
    return max(-32768, min(32767, int(round(value))))


def _crc16(data):
    return binascii.crc_hqx(data, 0xFFFF)


def _to_scaled(value, scale):
    return _clamp_i16(float(value) * scale)


def _from_scaled(value, scale):
    return float(value) / scale


def encode_telemetry_frame(snapshot):
    """Encode a telemetry snapshot for tests and host-side protocol checks."""
    wheels = snapshot.get("wheels", [])
    if len(wheels) != WHEEL_COUNT:
        raise ValueError("telemetry snapshot must contain four wheels")

    payload = bytearray(
        struct.pack(
            PAYLOAD_HEADER_FORMAT,
            int(snapshot.get("sequence", 0)) & 0xFFFFFFFF,
            int(snapshot.get("period_ms", 0)) & 0xFFFF,
            int(snapshot.get("controller_flags", 0)) & 0xFFFF,
        )
    )
    for wheel in wheels:
        flags = 0
        if wheel.get("saturated"):
            flags |= FLAG_SATURATED
        if wheel.get("start_pulse"):
            flags |= FLAG_START_PULSE
        payload.extend(
            struct.pack(
                WHEEL_FORMAT,
                _to_scaled(wheel.get("target_rpm", 0.0), RPM_SCALE),
                _to_scaled(wheel.get("measured_rpm", 0.0), RPM_SCALE),
                _clamp_i16(wheel.get("tick_delta", 0)),
                _clamp_i16(wheel.get("pwm", 0)),
                _to_scaled(wheel.get("error", 0.0), RPM_SCALE),
                _to_scaled(wheel.get("p_term", 0.0), TERM_SCALE),
                _to_scaled(wheel.get("i_term", 0.0), TERM_SCALE),
                _to_scaled(wheel.get("d_term", 0.0), TERM_SCALE),
                flags,
            )
        )

    body = bytes((FRAME_VERSION, len(payload))) + bytes(payload)
    return FRAME_MAGIC + body + struct.pack("<H", _crc16(body))


def decode_telemetry_payload(payload):
    """Decode the fixed-size payload emitted by the PID firmware."""
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError("unexpected telemetry payload size")

    sequence, period_ms, controller_flags = struct.unpack_from(PAYLOAD_HEADER_FORMAT, payload, 0)
    wheels = []
    offset = struct.calcsize(PAYLOAD_HEADER_FORMAT)
    for index in range(WHEEL_COUNT):
        values = struct.unpack_from(WHEEL_FORMAT, payload, offset)
        offset += WHEEL_SIZE
        target, measured, ticks, pwm, error, p_term, i_term, d_term, flags = values
        wheels.append(
            {
                "name": "M{}".format(index + 1),
                "target_rpm": _from_scaled(target, RPM_SCALE),
                "measured_rpm": _from_scaled(measured, RPM_SCALE),
                "tick_delta": int(ticks),
                "pwm": int(pwm),
                "error": _from_scaled(error, RPM_SCALE),
                "p_term": _from_scaled(p_term, TERM_SCALE),
                "i_term": _from_scaled(i_term, TERM_SCALE),
                "d_term": _from_scaled(d_term, TERM_SCALE),
                "saturated": bool(flags & FLAG_SATURATED),
                "start_pulse": bool(flags & FLAG_START_PULSE),
                "flags": int(flags),
            }
        )
    return {
        "sequence": int(sequence),
        "period_ms": int(period_ms),
        "controller_flags": int(controller_flags),
        "wheels": wheels,
    }


class TelemetryStreamParser:
    """Incrementally split mixed ACK text and framed ESP32 telemetry."""

    def __init__(self):
        self._buffer = bytearray()
        self.invalid_frames = 0

    def feed(self, data):
        if data:
            self._buffer.extend(data)

        frames = []
        lines = []
        while self._buffer:
            if self._buffer.startswith(FRAME_MAGIC):
                if len(self._buffer) < HEADER_SIZE:
                    break
                version = self._buffer[2]
                payload_size = self._buffer[3]
                frame_size = HEADER_SIZE + payload_size + CRC_SIZE
                if version != FRAME_VERSION or payload_size > MAX_PAYLOAD_SIZE:
                    self.invalid_frames += 1
                    del self._buffer[0]
                    continue
                if len(self._buffer) < frame_size:
                    break
                body = bytes(self._buffer[2:HEADER_SIZE + payload_size])
                received_crc = struct.unpack_from("<H", self._buffer, HEADER_SIZE + payload_size)[0]
                del self._buffer[:frame_size]
                if _crc16(body) != received_crc:
                    self.invalid_frames += 1
                    continue
                try:
                    frames.append(decode_telemetry_payload(body[2:]))
                except ValueError:
                    self.invalid_frames += 1
                continue

            magic_index = self._buffer.find(FRAME_MAGIC)
            newline_index = self._buffer.find(b"\n")
            if newline_index >= 0 and (magic_index < 0 or newline_index < magic_index):
                raw_line = bytes(self._buffer[:newline_index]).rstrip(b"\r")
                del self._buffer[:newline_index + 1]
                try:
                    line = raw_line.decode("ascii").strip()
                except UnicodeDecodeError:
                    line = ""
                if line in ("SUCC", "FAIL"):
                    lines.append(line)
                continue

            if magic_index > 0:
                del self._buffer[:magic_index]
                continue
            if magic_index == 0:
                continue
            if len(self._buffer) > 256:
                keep = self._buffer[-1:] if self._buffer[-1:] == FRAME_MAGIC[:1] else b""
                self._buffer[:] = keep
            break

        return frames, lines
