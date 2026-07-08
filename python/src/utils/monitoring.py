#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import json
import os
import tempfile
import time
from pathlib import Path


DEFAULT_EVENT_LIMIT = 80


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
        },
        "camera": {
            "status": "unknown",
            "width": 0,
            "height": 0,
            "fps": 0,
            "memory_name": "",
        },
        "controller": {
            "state": [0, 0, 0, 0, 90, 65, -12345],
            "speed": 0,
            "servo_angle": [90, 65],
            "last_action": "",
            "last_result": "",
            "last_modify_time": 0,
            "serial_port": "",
            "latency_ms": 0,
        },
        "scene": {
            "name": "",
            "status": "idle",
            "manual": {},
            "lane": {},
            "detections": [],
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


class MonitorBus:
    def __init__(self, base_dir=None, event_limit=DEFAULT_EVENT_LIMIT):
        self.base_dir = Path(base_dir) if base_dir is not None else default_monitor_dir()
        self.event_limit = event_limit
        self.state_path = self.base_dir / "state.json"

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


def publish_state(patch, event=None):
    try:
        return MonitorBus().update(patch, event=event)
    except Exception:
        return None


def publish_event(event_type, message, data=None):
    return publish_state({}, event={"type": event_type, "message": message, "data": data or {}})


def publish_system(mode=None, running=None, camera_info=None, memory_name=None, motion_enabled=None):
    patch = {"system": {"pid": os.getpid()}}
    if mode is not None:
        patch["system"]["mode"] = mode
    if running is not None:
        patch["system"]["running"] = running
    if motion_enabled is not None:
        patch["system"]["motion_enabled"] = bool(motion_enabled)
    if camera_info is not None:
        patch["camera"] = {
            "status": "running" if running else "stopped",
            "width": camera_info.get("width", 0),
            "height": camera_info.get("height", 0),
            "fps": camera_info.get("fps", 0),
            "memory_name": memory_name or "",
        }
    return publish_state(patch)


def publish_controller(action_name, state, result, speed, servo_angle, serial_port="", started_at=None, ended_at=None):
    latency_ms = 0
    if started_at is not None and ended_at is not None:
        latency_ms = int((ended_at - started_at) * 1000)
    return publish_state(
        {
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
        },
        event={"type": "controller", "message": f"{action_name or 'command'} -> {result}"},
    )


def publish_scene(scene_name, status=None, data=None):
    scene_patch = {"name": scene_name}
    if status is not None:
        scene_patch["status"] = status
    if data:
        scene_patch = deep_merge(scene_patch, to_jsonable(data))
    return publish_state({"scene": scene_patch})
