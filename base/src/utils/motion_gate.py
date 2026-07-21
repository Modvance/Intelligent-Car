#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import os
from pathlib import Path


GATE_DIR_ENV = "OLD_CAR_MOTION_GATE_DIR"


def _gate_dir():
    custom_dir = os.environ.get(GATE_DIR_ENV)
    if custom_dir:
        return Path(custom_dir)
    return Path(os.path.expanduser("~")) / "temp" / "old_car_motion_gate"


def _gate_file():
    return _gate_dir() / "state.json"


def _resume_file():
    return _gate_dir() / "resume.json"


def _sign_action_file():
    return _gate_dir() / "sign_action.json"


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(data), encoding="utf-8")
    os.replace(str(temp), str(path))


def set_motion_enabled(enabled):
    _write_json(_gate_file(), {"enabled": bool(enabled)})


def is_motion_enabled(default=True):
    try:
        data = json.loads(_gate_file().read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return bool(default)
    return bool(data.get("enabled", default))


def gate_motor_state(state, enabled=None, pedestrian_blocked=False):
    gated_state = list(state)
    if enabled is None:
        enabled = is_motion_enabled(default=True)
    if enabled and not pedestrian_blocked:
        return gated_state
    if len(gated_state) >= 4:
        gated_state[:4] = [0, 0, 0, 0]
    return gated_state


def request_motion_resume():
    """Ask LaneNet to run its one-time Start() pulse after a completed sign action."""
    _write_json(_resume_file(), {"pending": True})


def consume_motion_resume():
    """Return a pending resume request once, then clear it."""
    target = _resume_file()
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return False
    if not bool(data.get("pending", False)):
        return False
    _write_json(target, {"pending": False})
    return True


def set_sign_action_active(active):
    """Mark whether a sign action currently owns the motor controller."""
    _write_json(_sign_action_file(), {"active": bool(active)})


def is_sign_action_active(default=False):
    try:
        data = json.loads(_sign_action_file().read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return bool(default)
    return bool(data.get("active", default))
