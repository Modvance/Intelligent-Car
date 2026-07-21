#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import os
from pathlib import Path


GATE_DIR_ENV = "OLD_CAR_DECISION_GATE_DIR"


def _gate_dir():
    custom_dir = os.environ.get(GATE_DIR_ENV)
    if custom_dir:
        return Path(custom_dir)
    return Path(os.path.expanduser("~")) / "temp" / "old_car_decision_gate"


def _gate_file():
    return _gate_dir() / "state.json"


def set_decision_enabled(enabled):
    gate_dir = _gate_dir()
    gate_dir.mkdir(parents=True, exist_ok=True)
    target = _gate_file()
    temp = target.with_suffix(".tmp")
    temp.write_text(json.dumps({"enabled": bool(enabled)}), encoding="utf-8")
    os.replace(str(temp), str(target))


def is_decision_enabled(default=True):
    try:
        data = json.loads(_gate_file().read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return bool(default)
    return bool(data.get("enabled", default))
