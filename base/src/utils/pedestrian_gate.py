#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cross-process pedestrian stop gate and its pure decision helpers."""

import json
import os
from pathlib import Path


GATE_DIR_ENV = "OLD_CAR_PEDESTRIAN_GATE_DIR"
DEFAULT_CLEAR_FRAMES = 2


def _gate_dir():
    custom_dir = os.environ.get(GATE_DIR_ENV)
    if custom_dir:
        return Path(custom_dir)
    return Path(os.path.expanduser("~")) / "temp" / "old_car_pedestrian_gate"


def _state_file():
    return _gate_dir() / "state.json"


def _resume_file():
    return _gate_dir() / "resume.json"


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(data), encoding="utf-8")
    os.replace(str(temp), str(path))


def set_pedestrian_blocked(blocked):
    """Set whether a pedestrian currently blocks vehicle motion."""
    _write_json(_state_file(), {"blocked": bool(blocked)})


def is_pedestrian_blocked(default=False):
    try:
        data = json.loads(_state_file().read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return bool(default)
    return bool(data.get("blocked", default))


def request_pedestrian_resume():
    """Ask LaneNet to take control again after the pedestrian leaves."""
    _write_json(_resume_file(), {"pending": True})


def clear_pedestrian_resume():
    """Discard any stale resume request during system startup or shutdown."""
    _write_json(_resume_file(), {"pending": False})


def consume_pedestrian_resume():
    """Consume one pending resume request."""
    target = _resume_file()
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return False
    if not bool(data.get("pending", False)):
        return False
    _write_json(target, {"pending": False})
    return True


def is_human_in_stop_region(
    bboxes,
    *,
    label="human",
    score_threshold=0.30,
    x_min=420,
    x_max=950,
    y_min=270,
):
    """Return whether the highest-score human is inside the forward region."""
    candidates = []
    for bbox in bboxes:
        if len(bbox) < 6:
            continue
        x1, y1, x2, y2, cate, score = bbox[:6]
        if cate != label or float(score) < float(score_threshold):
            continue
        center_x = (float(x1) + float(x2)) / 2.0
        center_y = (float(y1) + float(y2)) / 2.0
        if x_min < center_x < x_max and center_y >= y_min:
            candidates.append((float(score), bbox))
    if not candidates:
        return False, None
    return True, max(candidates, key=lambda item: item[0])[1]


def update_pedestrian_state(blocked, clear_frames, detected, clear_threshold=DEFAULT_CLEAR_FRAMES):
    """Advance the pedestrian state machine and report entry/exit edges."""
    if detected:
        return True, 0, not blocked, False
    if not blocked:
        return False, 0, False, False
    next_clear_frames = int(clear_frames) + 1
    if next_clear_frames >= int(clear_threshold):
        return False, 0, False, True
    return True, next_clear_frames, False, False


def should_defer_sign_decisions(pedestrian_blocked):
    """Do not start a sign action while pedestrian protection owns the stop."""
    return bool(pedestrian_blocked)


def should_resume_after_pedestrian(pedestrian_blocked, sign_action_active):
    """Resume only when both pedestrian and sign-action gates are clear."""
    return not bool(pedestrian_blocked) and not bool(sign_action_active)
