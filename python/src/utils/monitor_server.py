#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import math
import signal
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from multiprocessing import shared_memory
from urllib.parse import urlparse

import numpy as np

from src.utils.monitoring import MonitorBus


BOUNDARY = "frame"


def camera_shape(camera_info):
    return (
        int(camera_info.get("height", 720)),
        int(camera_info.get("width", 1280)),
        3,
    )


def make_index_html():
    return """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Old Car Monitor</title>
  <style>
    :root { color-scheme: dark; font-family: Arial, sans-serif; }
    body { margin: 0; background: #101214; color: #e8edf2; }
    header { height: 48px; display: flex; align-items: center; justify-content: space-between; padding: 0 16px; background: #1b2026; border-bottom: 1px solid #303842; }
    main { display: grid; grid-template-columns: minmax(0, 1.5fr) minmax(360px, 0.8fr); gap: 12px; padding: 12px; }
    section { background: #171b20; border: 1px solid #2b333d; border-radius: 6px; padding: 12px; }
    h2 { margin: 0 0 10px; font-size: 16px; }
    img { width: 100%; background: #000; border-radius: 4px; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
    .item { background: #11161b; border: 1px solid #28313b; border-radius: 4px; padding: 8px; min-height: 46px; }
    .label { color: #93a4b7; font-size: 12px; }
    .value { font-size: 16px; margin-top: 4px; overflow-wrap: anywhere; }
    pre { white-space: pre-wrap; overflow-wrap: anywhere; margin: 0; font-size: 12px; color: #d7e1eb; }
    .events { max-height: 220px; overflow: auto; }
    @media (max-width: 900px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <header>
    <strong>Car Monitor</strong>
    <span id="heartbeat">connecting...</span>
  </header>
  <main>
    <section>
      <h2>Camera</h2>
      <img src="/stream.mjpg" alt="camera stream">
    </section>
    <section>
      <h2>Controller</h2>
      <div class="grid">
        <div class="item"><div class="label">Mode</div><div class="value" id="mode">-</div></div>
        <div class="item"><div class="label">Motion</div><div class="value" id="motion">-</div></div>
        <div class="item"><div class="label">Result</div><div class="value" id="result">-</div></div>
        <div class="item"><div class="label">Action</div><div class="value" id="action">-</div></div>
        <div class="item"><div class="label">Speed</div><div class="value" id="speed">-</div></div>
        <div class="item"><div class="label">Motors</div><div class="value" id="motors">-</div></div>
        <div class="item"><div class="label">Servo</div><div class="value" id="servo">-</div></div>
      </div>
      <h2 style="margin-top:14px;">AI</h2>
      <pre id="ai">-</pre>
      <h2 style="margin-top:14px;">Events</h2>
      <pre class="events" id="events">-</pre>
    </section>
  </main>
  <script>
    async function refresh() {
      try {
        const res = await fetch('/api/state', {cache: 'no-store'});
        const state = await res.json();
        const controller = state.controller || {};
        const system = state.system || {};
        const scene = state.scene || {};
        const motors = (controller.state || []).slice(0, 4);
        document.getElementById('heartbeat').textContent = new Date().toLocaleTimeString();
        document.getElementById('mode').textContent = system.mode || '-';
        document.getElementById('motion').textContent = system.motion_enabled ? 'enabled' : 'locked';
        document.getElementById('result').textContent = controller.last_result || '-';
        document.getElementById('action').textContent = controller.last_action || '-';
        document.getElementById('speed').textContent = controller.speed ?? '-';
        document.getElementById('motors').textContent = JSON.stringify(motors);
        document.getElementById('servo').textContent = JSON.stringify(controller.servo_angle || []);
        document.getElementById('ai').textContent = JSON.stringify(scene, null, 2);
        document.getElementById('events').textContent = (state.events || []).slice(-12).map(e => {
          const t = e.time ? new Date(e.time * 1000).toLocaleTimeString() : '';
          return `${t} ${e.type || ''}: ${e.message || ''}`;
        }).join('\\n');
      } catch (err) {
        document.getElementById('heartbeat').textContent = 'offline';
      }
    }
    refresh();
    setInterval(refresh, 500);
  </script>
</body>
</html>
"""


def import_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise RuntimeError("opencv-python is required for monitor video stream") from exc
    return cv2


def normalize_request_path(raw_path):
    parsed = urlparse(raw_path)
    path = parsed.path or "/"
    if path != "/" and path.endswith("/"):
        path = path.rstrip("/")
    return path or "/"


def is_index_path(path):
    return path in {"/", "/index.html", "/monitor"}


def _safe_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _safe_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _clamp(value, low, high):
    return max(low, min(high, value))


def overlay_detections(cv2, frame, detections):
    height, width = frame.shape[:2]
    for item in detections or []:
        box = item.get("box", [])
        if len(box) != 4:
            continue
        x1, y1, x2, y2 = [_safe_int(value) for value in box]
        x1 = _clamp(x1, 0, width - 1)
        x2 = _clamp(x2, 0, width - 1)
        y1 = _clamp(y1, 0, height - 1)
        y2 = _clamp(y2, 0, height - 1)
        if x2 <= x1 or y2 <= y1:
            continue

        color = (0, 220, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cate = str(item.get("cate", "object"))
        score = _safe_float(item.get("score", 0.0))
        label = f"{cate} {score:.2f}"
        cv2.putText(frame, label, (x1, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return frame


def overlay_lane(cv2, frame, lane):
    lane = lane or {}
    height, width = frame.shape[:2]
    for item in lane.get("lanes", []) or []:
        k = _safe_float(item.get("k", 0.0))
        b = _safe_float(item.get("b", 0.0))
        confidence = _safe_float(item.get("confidence", 0.0))
        y1, y2 = 0, height - 1
        x1 = int(k * y1 + b)
        x2 = int(k * y2 + b)
        ok, pt1, pt2 = cv2.clipLine((0, 0, width, height), (x1, y1), (x2, y2))
        if not ok:
            continue
        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
        label_pos = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)
        cv2.putText(frame, f"{confidence:.0f}", label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    steering = _safe_float(lane.get("filtered_steering", lane.get("steering_command", 0.0)))
    arrow_len = max(60, min(width, height) // 5)
    start = (width // 2, height - 24)
    end = (
        int(start[0] + arrow_len * math.sin(steering)),
        int(start[1] - arrow_len * math.cos(steering)),
    )
    cv2.arrowedLine(frame, start, end, (0, 0, 255), 5, line_type=cv2.LINE_AA, tipLength=0.35)
    cv2.putText(frame, f"steer {steering:.2f}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    return frame


def overlay_monitor_results(cv2, frame, state, enabled=True):
    if not enabled:
        return frame
    scene = state.get("scene", {}) if isinstance(state, dict) else {}
    overlay_detections(cv2, frame, scene.get("detections", []))
    overlay_lane(cv2, frame, scene.get("lane", {}))
    return frame


def make_handler(memory_name, camera_info, bus):
    shape = camera_shape(camera_info)

    class MonitorHandler(BaseHTTPRequestHandler):
        server_version = "OldCarMonitor/1.0"

        def log_message(self, fmt, *args):
            sys.stderr.write("%s - - [%s] %s\n" % (self.address_string(), self.log_date_time_string(), fmt % args))

        def send_bytes(self, content, content_type, status=HTTPStatus.OK):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            path = normalize_request_path(self.path)
            if is_index_path(path):
                self.send_bytes(make_index_html().encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/api/state":
                self.send_bytes(json.dumps(bus.read(), ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")
            elif path == "/health":
                self.send_bytes(b"ok\n", "text/plain; charset=utf-8")
            elif path == "/snapshot.jpg":
                self.handle_snapshot()
            elif path == "/stream.mjpg":
                self.handle_stream()
            else:
                self.send_bytes(make_index_html().encode("utf-8"), "text/html; charset=utf-8")

        def read_frame(self):
            shm = shared_memory.SharedMemory(name=memory_name)
            try:
                frame = np.ndarray(shape, dtype=np.uint8, buffer=shm.buf)
                return frame.copy()
            finally:
                shm.close()

        def encode_frame(self):
            cv2 = import_cv2()
            frame = self.read_frame()
            frame = overlay_monitor_results(cv2, frame, bus.read(), enabled=camera_info.get("overlay_results", True))
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
            if not ok:
                raise RuntimeError("failed to encode frame")
            return encoded.tobytes()

        def handle_snapshot(self):
            try:
                jpeg = self.encode_frame()
            except Exception as exc:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, str(exc))
                return
            self.send_bytes(jpeg, "image/jpeg")

        def handle_stream(self):
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

            while True:
                try:
                    jpeg = self.encode_frame()
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode("ascii"))
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.12)
                except (BrokenPipeError, ConnectionResetError):
                    break
                except Exception:
                    time.sleep(0.5)

    return MonitorHandler


def run_monitor_server(memory_name, camera_info, host="127.0.0.1", port=8080, stop_sign=None):
    bus = MonitorBus()
    server = ThreadingHTTPServer((host, port), make_handler(memory_name, camera_info, bus))

    def shutdown(_signum=None, _frame=None):
        server.shutdown()

    def watch_stop():
        while stop_sign is not None and not stop_sign.value:
            time.sleep(0.2)
        if stop_sign is not None:
            server.shutdown()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    if stop_sign is not None:
        threading.Thread(target=watch_stop, daemon=True).start()

    print(f"Monitor serving on http://{host}:{port}")
    print(f"SSH tunnel: ssh -L {port}:127.0.0.1:{port} root@CAR_IP")
    try:
        server.serve_forever()
    finally:
        server.server_close()
