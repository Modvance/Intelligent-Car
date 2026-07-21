#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import json
import os
import socket
import tempfile
import threading
import time
from pathlib import Path

from filelock import FileLock


DEFAULT_EVENT_LIMIT = 80
DISABLE_STATE_PUBLISH_ENV = "OLD_CAR_DISABLE_STATE_PUBLISH"
STATE_PUBLISH_INTERVAL_ENV = "OLD_CAR_MONITOR_STATE_INTERVAL"
TELEMETRY_HOST_ENV = "OLD_CAR_MONITOR_TELEMETRY_HOST"
TELEMETRY_PORT_ENV = "OLD_CAR_MONITOR_TELEMETRY_PORT"
_last_state_publish = 0.0
_telemetry_sender = None


def default_monitor_dir():
    return Path(os.path.expanduser("~")) / "temp" / "old_car_monitor"


def now_ts():
    return time.time()


def to_jsonable(value):
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return to_jsonable(value.tolist())
    if hasattr(value, "item"):
        return to_jsonable(value.item())
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def default_state():
    return {
        "updated_at": now_ts(),
        "system": {
            "running": False,
            "mode": "unknown",
            "pid": os.getpid(),
            "motion_enabled": True,
            "decision_enabled": True,
        },
        "camera": {
            "status": "unknown",
            "width": 0,
            "height": 0,
            "fps": 0,
            "memory_name": "",
        },
        "controller": {
            "state": [0, 0, 0, 0, 93, 162, -12345],
            "speed": 0,
            "servo_angle": [93, 162],
            "last_action": "",
            "last_result": "",
            "last_modify_time": 0,
            "serial_port": "",
            "latency_ms": 0,
        },
        "motor_control": {
            "mode": "encoder_pid",
            "drive_mode": "closed_loop_encoder_pid",
            "enabled": False,
            "fresh": False,
            "age_ms": None,
            "data_source": "esp32_encoder",
            "period_ms": 0,
            "wheels": [],
            "history": [],
            "updated_at": 0,
        },
        "scene": {
            "name": "",
            "status": "idle",
            "manual": {},
            "lane": {},
            "detections": [],
        },
        "scenes": {},
        "decision": {
            "lane": {
                "source": "LF_Lanenet",
                "status": "idle",
                "action": "",
                "updated_at": 0,
            },
            "sign": {
                "source": "Helper",
                "status": "idle",
                "action": "",
                "updated_at": 0,
            },
        },
        "events": [],
    }


def deep_merge(base, patch):
    output = copy.deepcopy(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(output.get(key), dict):
            output[key] = deep_merge(output[key], value)
        else:
            output[key] = copy.deepcopy(value)
    return output


class MemoryMonitorBus:
    """Thread-safe in-memory state store used by the monitor process."""

    def __init__(self, event_limit=DEFAULT_EVENT_LIMIT):
        self.event_limit = event_limit
        self._state = default_state()
        self._lock = threading.RLock()

    def read(self):
        with self._lock:
            return copy.deepcopy(self._state)

    def update(self, patch, event=None):
        with self._lock:
            self._state = deep_merge(self._state, to_jsonable(patch))
            self._state["updated_at"] = now_ts()
            if event is not None:
                next_event = to_jsonable(event)
                next_event.setdefault("time", now_ts())
                self._state.setdefault("events", []).append(next_event)
                self._state["events"] = self._state["events"][-self.event_limit:]
            return copy.deepcopy(self._state)


class TelemetrySender:
    """Best-effort, non-blocking UDP sender for monitor-only telemetry."""

    def __init__(self, host, port):
        self.endpoint = (host, int(port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def send(self, payload):
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.socket.sendto(encoded, self.endpoint)


def _telemetry_sender_from_environment():
    global _telemetry_sender
    port_text = os.environ.get(TELEMETRY_PORT_ENV, "").strip()
    if not port_text:
        return None
    try:
        port = int(port_text)
        if not 1 <= port <= 65535:
            return None
    except ValueError:
        return None

    host = os.environ.get(TELEMETRY_HOST_ENV, "127.0.0.1").strip() or "127.0.0.1"
    endpoint = (host, port)
    if _telemetry_sender is None or _telemetry_sender.endpoint != endpoint:
        if _telemetry_sender is not None:
            try:
                _telemetry_sender.socket.close()
            except OSError:
                pass
        _telemetry_sender = TelemetrySender(host, port)
    return _telemetry_sender


def _publish_telemetry(patch, event):
    sender = _telemetry_sender_from_environment()
    if sender is None:
        return False
    payload = {
        "patch": to_jsonable(patch),
        "event": to_jsonable(event) if event is not None else None,
    }
    try:
        sender.send(payload)
        return True
    except (OSError, ValueError, TypeError):
        return False


class MonitorBus:
    def __init__(self, base_dir=None, event_limit=DEFAULT_EVENT_LIMIT):
        self.base_dir = Path(base_dir) if base_dir is not None else default_monitor_dir()
        self.event_limit = event_limit
        self.state_path = self.base_dir / "state.json"
        self.lock_path = self.base_dir / "state.lock"

    def read(self):
        if not self.state_path.exists():
            return default_state()
        try:
            with self.state_path.open("r", encoding="utf-8") as handle:
                state = json.load(handle)
        except (OSError, json.JSONDecodeError):
            return default_state()
        return deep_merge(default_state(), state)

    def update(self, patch, event=None):
        self.base_dir.mkdir(parents=True, exist_ok=True)
        with FileLock(str(self.lock_path)):
            state = self.read()
            state = deep_merge(state, to_jsonable(patch))
            state["updated_at"] = now_ts()

            if event is not None:
                next_event = to_jsonable(event)
                next_event.setdefault("time", now_ts())
                state.setdefault("events", []).append(next_event)
                state["events"] = state["events"][-self.event_limit:]

            self.write(state)
            return state

    def write(self, state):
        self.base_dir.mkdir(parents=True, exist_ok=True)
        fd, temp_name = tempfile.mkstemp(prefix="state.", suffix=".json", dir=str(self.base_dir))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(to_jsonable(state), handle, ensure_ascii=False, sort_keys=True)
            os.replace(temp_name, self.state_path)
        finally:
            if os.path.exists(temp_name):
                os.unlink(temp_name)


def publish_state(patch, event=None, force=False):
    global _last_state_publish
    if os.environ.get(DISABLE_STATE_PUBLISH_ENV) == "1":
        return None
    if event is None and not force:
        try:
            interval = max(0.0, float(os.environ.get(STATE_PUBLISH_INTERVAL_ENV, "0")))
        except ValueError:
            interval = 0.0
        now = time.monotonic()
        if interval > 0.0 and now - _last_state_publish < interval:
            return None
        _last_state_publish = now
    if os.environ.get(TELEMETRY_PORT_ENV):
        _publish_telemetry(patch, event)
        return None
    try:
        return MonitorBus().update(patch, event=event)
    except Exception:
        return None


def publish_event(event_type, message, data=None):
    return publish_state({}, event={"type": event_type, "message": message, "data": data or {}})


def publish_system(mode=None, running=None, camera_info=None, memory_name=None, motion_enabled=None, decision_enabled=None):
    patch = {"system": {"pid": os.getpid()}}
    if mode is not None:
        patch["system"]["mode"] = mode
    if running is not None:
        patch["system"]["running"] = running
    if motion_enabled is not None:
        patch["system"]["motion_enabled"] = bool(motion_enabled)
    if decision_enabled is not None:
        patch["system"]["decision_enabled"] = bool(decision_enabled)
    if camera_info is not None:
        patch["camera"] = {
            "status": "running" if running else "stopped",
            "width": camera_info.get("width", 0),
            "height": camera_info.get("height", 0),
            "fps": camera_info.get("fps", 0),
            "memory_name": memory_name or "",
        }
    return publish_state(patch, force=True)


def publish_controller(action_name, state, result, speed, servo_angle, serial_port="", started_at=None, ended_at=None, motor_control=None):
    latency_ms = 0
    if started_at is not None and ended_at is not None:
        latency_ms = int((ended_at - started_at) * 1000)
    patch = {
        "controller": {
            "state": state,
            "speed": speed,
            "servo_angle": servo_angle,
            "last_action": action_name or "",
            "last_result": result,
            "last_modify_time": ended_at or now_ts(),
            "serial_port": serial_port,
            "latency_ms": latency_ms,
        }
    }
    if motor_control is not None:
        patch["motor_control"] = motor_control
    return publish_state(
        patch,
        event={"type": "controller", "message": f"{action_name or 'command'} -> {result}"},
    )


def publish_scene(scene_name, status=None, data=None):
    scene_patch = {"name": scene_name}
    if status is not None:
        scene_patch["status"] = status
    if data:
        scene_patch = deep_merge(scene_patch, to_jsonable(data))
    force = status not in (None, "running")
    return publish_state({"scene": scene_patch, "scenes": {scene_name: scene_patch}}, force=force)


def publish_decision(channel, source, status=None, action=None, data=None):
    decision_patch = {
        "source": source,
        "updated_at": now_ts(),
    }
    if status is not None:
        decision_patch["status"] = status
    if action is not None:
        decision_patch["action"] = action
    if data:
        decision_patch = deep_merge(decision_patch, to_jsonable(data))
    force = status not in (None, "running") or bool(action)
    return publish_state({"decision": {channel: decision_patch}}, force=force)
