#!/usr/bin/env python3
import re
import threading
import time
from pathlib import Path


_NUMBERED_JPG = re.compile(r"^(\d{5})\.jpg$")


def next_capture_index(save_dir):
    save_path = Path(save_dir)
    if not save_path.exists():
        return 1

    indices = []
    for path in save_path.iterdir():
        match = _NUMBERED_JPG.match(path.name)
        if match:
            indices.append(int(match.group(1)))
    return max(indices, default=0) + 1


def _default_image_writer(path, frame):
    import cv2

    return cv2.imwrite(str(path), frame)


def _copy_frame(frame):
    copy_func = getattr(frame, "copy", None)
    if callable(copy_func):
        return copy_func()
    return frame


class TimedImageCapture:
    def __init__(self, save_dir, interval=0.5, enabled=False, image_writer=None, threaded=False):
        self.save_dir = Path(save_dir)
        self.interval = max(0.0, float(interval))
        self.enabled = bool(enabled)
        self.image_writer = image_writer or _default_image_writer
        self.threaded = bool(threaded)
        self.frame_index = next_capture_index(self.save_dir)
        self.last_capture_time = None

    def toggle(self):
        self.enabled = not self.enabled
        self.last_capture_time = None
        return self.enabled

    def set_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled != self.enabled:
            self.last_capture_time = None
        self.enabled = enabled

    def maybe_capture(self, frame, now=None):
        if not self.enabled:
            return None

        now = time.time() if now is None else float(now)
        if self.last_capture_time is not None and now - self.last_capture_time < self.interval:
            return None

        self.save_dir.mkdir(parents=True, exist_ok=True)
        path = self.save_dir / f"{self.frame_index:05d}.jpg"
        frame_snapshot = _copy_frame(frame)

        if self.threaded:
            thread = threading.Thread(target=self.image_writer, args=(path, frame_snapshot), daemon=True)
            thread.start()
        else:
            self.image_writer(path, frame_snapshot)

        self.frame_index += 1
        self.last_capture_time = now
        return path
