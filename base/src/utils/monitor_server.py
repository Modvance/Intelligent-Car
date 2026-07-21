#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import math
import signal
import socket
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from multiprocessing import shared_memory
from urllib.parse import urlparse

import numpy as np

from src.utils.monitoring import MemoryMonitorBus


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
    h2:not(:first-child) { margin-top: 14px; }
    img { width: 100%; background: #000; border-radius: 4px; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
    .item { background: #11161b; border: 1px solid #28313b; border-radius: 4px; padding: 8px; min-height: 46px; }
    .label { color: #93a4b7; font-size: 12px; }
    .value { font-size: 16px; margin-top: 4px; overflow-wrap: anywhere; }
    .actions { display: flex; gap: 8px; flex-wrap: wrap; margin: 10px 0 12px; }
    button { background: #243242; color: #e8edf2; border: 1px solid #3b4a5b; border-radius: 4px; padding: 7px 10px; cursor: pointer; }
    button:active { transform: translateY(1px); }
    pre { white-space: pre-wrap; overflow-wrap: anywhere; margin: 0; font-size: 12px; color: #d7e1eb; }
    details { border-top: 1px solid #2b333d; margin-top: 14px; padding-top: 10px; }
    summary { color: #93a4b7; cursor: pointer; font-size: 13px; }
    .events { max-height: 132px; overflow: auto; }
    .full { grid-column: 1 / -1; }
    .wheel-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; margin-top: 8px; }
    .wheel-item { background: #11161b; border: 1px solid #28313b; border-radius: 4px; padding: 8px; min-height: 92px; }
    .wheel-name { color: #7fc5ff; font-size: 13px; margin-bottom: 5px; }
    .wheel-values { color: #d7e1eb; font-size: 12px; line-height: 1.55; }
    .chart-bar { display: flex; align-items: center; justify-content: space-between; gap: 8px; margin: 8px 0; }
    select { background: #11161b; color: #e8edf2; border: 1px solid #3b4a5b; border-radius: 4px; padding: 5px 7px; }
    canvas { width: 100%; height: auto; display: block; background: #11161b; border: 1px solid #28313b; border-radius: 4px; }
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
      <h2>System</h2>
      <div class="grid">
        <div class="item"><div class="label">Mode</div><div class="value" id="mode">-</div></div>
        <div class="item"><div class="label">Decision</div><div class="value" id="gate-decision">-</div></div>
        <div class="item"><div class="label">Motion</div><div class="value" id="gate-motion">-</div></div>
        <div class="item"><div class="label">Result</div><div class="value" id="result">-</div></div>
        <div class="item"><div class="label">Last Command</div><div class="value" id="action">-</div></div>
        <div class="item"><div class="label">Speed</div><div class="value" id="speed">-</div></div>
      </div>

      <h2>Motor Control</h2>
      <div class="grid">
        <div class="item full"><div class="label">Mode</div><div class="value" id="motor-mode">Encoder PID</div></div>
        <div class="item full"><div class="label">PID / Data Source</div><div class="value" id="pid-gains">ESP32 encoder telemetry</div></div>
      </div>
      <div class="wheel-grid" id="motor-wheels"></div>
      <div class="chart-bar">
        <div class="label">Measured wheel trend</div>
        <select id="pid-wheel" aria-label="PID chart wheel">
          <option value="0">M1</option>
          <option value="1">M2</option>
          <option value="2">M3</option>
          <option value="3">M4</option>
        </select>
      </div>
      <canvas id="pid-chart" width="560" height="180"></canvas>

      <h2>Lane Detection</h2>
      <div class="grid">
        <div class="item"><div class="label">Lane Action</div><div class="value" id="lane-action">-</div></div>
        <div class="item"><div class="label">Status</div><div class="value" id="lane-status">-</div></div>
        <div class="item"><div class="label">Steering / Turn</div><div class="value" id="lane-steering">-</div></div>
        <div class="item"><div class="label">Lines</div><div class="value" id="lane-lines">-</div></div>
        <div class="item full"><div class="label">Lateral</div><div class="value" id="lane-lateral">-</div></div>
        <div class="item full"><div class="label">Inference</div><div class="value" id="lane-inference">-</div></div>
      </div>

      <h2>Sign Detection</h2>
      <div class="grid">
        <div class="item"><div class="label">Sign Action</div><div class="value" id="sign-action">-</div></div>
        <div class="item"><div class="label">Status</div><div class="value" id="sign-status">-</div></div>
        <div class="item full"><div class="label">Detected</div><div class="value" id="sign-detected">-</div></div>
        <div class="item"><div class="label">Best Score</div><div class="value" id="sign-score">-</div></div>
        <div class="item"><div class="label">Center</div><div class="value" id="sign-center">-</div></div>
      </div>

      <h2>Controls</h2>
      <div class="actions">
        <button type="button" id="toggle-decision">Decision (d)</button>
        <button type="button" id="start-car">Start (g)</button>
        <button type="button" id="disable-motion">Stop (space)</button>
      </div>

      <h2>Events</h2>
      <pre class="events" id="events">-</pre>

      <details>
        <summary>Debug Raw State</summary>
        <pre id="debug-state">-</pre>
      </details>
    </section>
  </main>
  <script>
    const formatNumber = (value, digits = 2) =>
      typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '-';

    const getCenterFromBox = (item) => {
      const box = item && Array.isArray(item.box) ? item.box : [];
      if (box.length !== 4) return null;
      return [
        Math.round((Number(box[0]) + Number(box[2])) / 2),
        Math.round((Number(box[1]) + Number(box[3])) / 2),
      ];
    };

    const bestDetection = (detections) => {
      if (!Array.isArray(detections) || detections.length === 0) return null;
      return detections.reduce((best, item) => Number(item.score || 0) > Number(best.score || 0) ? item : best, detections[0]);
    };

    let selectedPidWheel = 0;

    const formatSigned = (value, digits = 1) => {
      if (typeof value !== 'number' || !Number.isFinite(value)) return '-';
      return `${value >= 0 ? '+' : ''}${value.toFixed(digits)}`;
    };

    function renderMotorWheels(motorControl) {
      const host = document.getElementById('motor-wheels');
      const wheels = motorControl && Array.isArray(motorControl.wheels) ? motorControl.wheels : [];
      host.replaceChildren();
      if (!motorControl || !motorControl.enabled || wheels.length !== 4) {
        const item = document.createElement('div');
        item.className = 'wheel-item';
        item.style.gridColumn = '1 / -1';
        item.textContent = 'Unavailable';
        host.appendChild(item);
        return;
      }
      wheels.forEach((wheel, index) => {
        const item = document.createElement('div');
        item.className = 'wheel-item';
        const name = wheel.name || `M${index + 1}`;
        item.innerHTML = `<div class="wheel-name">${name}</div><div class="wheel-values">PWM ${formatSigned(wheel.pwm, 0)} | ticks ${formatSigned(wheel.tick_delta, 0)}<br>target ${formatSigned(wheel.target_rpm, 1)} rpm<br>measured ${formatSigned(wheel.measured_rpm, 1)} rpm<br>error ${formatSigned(wheel.error, 1)} rpm</div>`;
        host.appendChild(item);
      });
    }

    function drawPidChart(history, selectedWheel) {
      const canvas = document.getElementById('pid-chart');
      const context = canvas.getContext('2d');
      const width = canvas.width;
      const height = canvas.height;
      const padding = {left: 38, right: 10, top: 18, bottom: 22};
      context.clearRect(0, 0, width, height);
      context.fillStyle = '#11161b';
      context.fillRect(0, 0, width, height);

      const points = (Array.isArray(history) ? history : []).map(point => {
        const wheel = Array.isArray(point.wheels) ? point.wheels[selectedWheel] : null;
        return wheel || null;
      }).filter(Boolean);
      if (!points.length) {
        context.fillStyle = '#93a4b7';
        context.font = '13px Arial';
        context.fillText('Waiting for ESP32 encoder telemetry', padding.left, height / 2);
        return;
      }

      const values = points.flatMap(point => [point.target_rpm, point.measured_rpm, point.pwm])
        .filter(Number.isFinite);
      const maxValue = Math.max(20, ...values.map(value => Math.abs(value)));
      const plotWidth = width - padding.left - padding.right;
      const plotHeight = height - padding.top - padding.bottom;
      const centerY = padding.top + plotHeight / 2;
      const yFor = value => centerY - (Number(value) / maxValue) * (plotHeight / 2 - 4);
      const xFor = index => padding.left + (points.length <= 1 ? 0 : index * plotWidth / (points.length - 1));

      context.strokeStyle = '#28313b';
      context.lineWidth = 1;
      for (let row = 0; row <= 4; row += 1) {
        const y = padding.top + row * plotHeight / 4;
        context.beginPath();
        context.moveTo(padding.left, y);
        context.lineTo(width - padding.right, y);
        context.stroke();
      }
      context.fillStyle = '#93a4b7';
      context.font = '11px Arial';
      context.fillText(`${maxValue.toFixed(0)}`, 4, padding.top + 4);
      context.fillText('0', 21, centerY + 4);
      context.fillText(`${-maxValue.toFixed(0)}`, 4, height - padding.bottom + 4);

      const drawSeries = (key, color) => {
        context.strokeStyle = color;
        context.lineWidth = 2;
        context.beginPath();
        points.forEach((point, index) => {
          const x = xFor(index);
          const y = yFor(point[key]);
          if (index === 0) context.moveTo(x, y);
          else context.lineTo(x, y);
        });
        context.stroke();
      };
      drawSeries('target_rpm', '#6bd98c');
      drawSeries('measured_rpm', '#7fc5ff');
      drawSeries('pwm', '#f4bc62');
      context.fillStyle = '#6bd98c';
      context.fillText('target', padding.left, 12);
      context.fillStyle = '#7fc5ff';
      context.fillText('measured', padding.left + 60, 12);
      context.fillStyle = '#f4bc62';
      context.fillText('output', padding.left + 145, 12);
    }

    async function refresh() {
      try {
        const res = await fetch('/api/state', {cache: 'no-store'});
        const state = await res.json();
        const controller = state.controller || {};
        const system = state.system || {};
        const decisions = state.decision || {};
        const scenes = state.scenes || {};
        const laneScene = scenes.LF_Lanenet || {};
        const helperScene = scenes.Helper || {};
        const laneDecision = decisions.lane || {};
        const signDecision = decisions.sign || {};
        const lane = laneScene.lane || {};
        const motorControl = state.motor_control || {};
        const detections = helperScene.detections || [];
        const best = bestDetection(detections);
        const bestCenter = signDecision.center || getCenterFromBox(best);
        const detectedText = detections.length
          ? detections.map(item => `${item.cate || '-'} ${formatNumber(Number(item.score || 0), 2)}`).join(', ')
          : '-';

        document.getElementById('heartbeat').textContent = new Date().toLocaleTimeString();
        document.getElementById('mode').textContent = system.mode || '-';
        document.getElementById('gate-decision').textContent = system.decision_enabled ? 'enabled' : 'locked';
        document.getElementById('gate-motion').textContent = system.motion_enabled ? 'enabled' : 'locked';
        document.getElementById('result').textContent = controller.last_result || '-';
        document.getElementById('action').textContent = controller.last_action || '-';
        document.getElementById('speed').textContent = controller.speed ?? '-';
        document.getElementById('motor-mode').textContent = motorControl.enabled
          ? 'Encoder PID'
          : 'Unavailable';
        document.getElementById('pid-gains').textContent = motorControl.enabled
          ? `${motorControl.period_ms || '-'} ms | ESP32 encoder telemetry | ${motorControl.fresh ? 'live' : 'stale'} ${motorControl.age_ms ?? '-'} ms`
          : 'No ESP32 encoder telemetry';
        renderMotorWheels(motorControl);
        drawPidChart(motorControl.history, selectedPidWheel);

        document.getElementById('lane-action').textContent = laneDecision.action || lane.intended_action || lane.would_action || '-';
        document.getElementById('lane-status').textContent = laneDecision.status || laneScene.status || '-';
        const rawSteering = laneDecision.steering_command ?? lane.steering_command;
        const headingSteering = laneDecision.heading_steering ?? lane.heading_steering;
        const lateralSteering = laneDecision.lateral_steering ?? lane.lateral_steering;
        const filteredSteering = laneDecision.filtered_steering ?? lane.filtered_steering;
        const turnStrength = laneDecision.turn_strength ?? lane.turn_strength;
        document.getElementById('lane-steering').textContent =
          `heading ${formatNumber(headingSteering)} | lateral ${formatNumber(lateralSteering)} | total ${formatNumber(rawSteering)} | filtered ${formatNumber(filteredSteering)} | strength ${formatNumber(turnStrength)}`;
        document.getElementById('lane-lines').textContent = lane.kept_count ?? (Array.isArray(lane.lanes) ? lane.lanes.length : '-');
        const lateralError = laneDecision.lateral_error_px ?? lane.lateral_error_px;
        const laneCenterX = laneDecision.lane_center_x ?? lane.lane_center_x;
        const laneTargetX = laneDecision.lane_target_x ?? lane.lane_target_x;
        document.getElementById('lane-lateral').textContent =
          `error ${formatNumber(lateralError, 1)} px | center ${formatNumber(laneCenterX, 1)} -> target ${formatNumber(laneTargetX, 1)}`;
        document.getElementById('lane-inference').textContent =
          typeof lane.inference_time === 'number' ? `${lane.inference_time.toFixed(1)} ms` : '-';

        document.getElementById('sign-action').textContent = signDecision.action || helperScene.helper?.would_trigger || helperScene.helper?.trigger || '-';
        document.getElementById('sign-status').textContent = signDecision.status || helperScene.status || '-';
        document.getElementById('sign-detected').textContent = detectedText;
        document.getElementById('sign-score').textContent = formatNumber(signDecision.score ?? best?.score);
        document.getElementById('sign-center').textContent = bestCenter ? `(${bestCenter[0]}, ${bestCenter[1]})` : '-';

        document.getElementById('events').textContent = (state.events || []).slice(-6).map(e => {
          const t = e.time ? new Date(e.time * 1000).toLocaleTimeString() : '';
          return `${t} ${e.type || ''}: ${e.message || ''}`;
        }).join('\\n');

        document.getElementById('debug-state').textContent = JSON.stringify({
          system,
          controller,
          motor_control: motorControl,
          decision: decisions,
          scenes,
        }, null, 2);
      } catch (err) {
        document.getElementById('heartbeat').textContent = 'offline';
      }
    }
    async function postControl(path) {
      await fetch(path, {method: 'POST', cache: 'no-store'});
      await refresh();
    }
    document.getElementById('toggle-decision').addEventListener('click', () => postControl('/api/toggle-decision'));
    document.getElementById('start-car').addEventListener('click', () => postControl('/api/start-car'));
    document.getElementById('disable-motion').addEventListener('click', () => postControl('/api/disable-motion'));
    document.getElementById('pid-wheel').addEventListener('change', event => {
      selectedPidWheel = Number(event.target.value) || 0;
      refresh();
    });
    document.addEventListener('keydown', (event) => {
      if (event.repeat) return;
      const key = event.key.toLowerCase();
      if (key === 'd') {
        event.preventDefault();
        postControl('/api/toggle-decision');
      } else if (key === 'g') {
        event.preventDefault();
        postControl('/api/start-car');
      } else if (key === ' ') {
        event.preventDefault();
        postControl('/api/disable-motion');
      }
    });
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


def apply_gate_command(command):
    from src.utils.decision_gate import is_decision_enabled, set_decision_enabled
    from src.utils.motion_gate import set_motion_enabled
    from src.utils.monitoring import publish_event, publish_system

    if command == "toggle-decision":
        enabled = not is_decision_enabled(default=False)
        set_decision_enabled(enabled)
        publish_system(decision_enabled=enabled)
        publish_event("decision_gate", f'Decision {"enabled" if enabled else "disabled"}')
        return {"ok": True, "decision_enabled": enabled}

    if command == "start-car":
        set_decision_enabled(True)
        set_motion_enabled(True)
        publish_system(decision_enabled=True, motion_enabled=True)
        publish_event("car", "Car started: decision and motion enabled")
        return {"ok": True, "decision_enabled": True, "motion_enabled": True}

    if command == "disable-motion":
        set_motion_enabled(False)
        publish_system(motion_enabled=False)
        publish_event("motion_gate", "Motion disabled")
        return {"ok": True, "motion_enabled": False}

    return {"ok": False, "error": f"unknown command: {command}"}


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
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        cv2.line(frame, (center_x, y1), (center_x, y2), color, 1)
        cv2.line(frame, (x1, center_y), (x2, center_y), color, 1)
        cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
        cate = str(item.get("cate", "object"))
        score = _safe_float(item.get("score", 0.0))
        label = f"{cate} {score:.2f}"
        center_label = f"x={center_x} y={center_y}"
        cv2.putText(frame, label, (x1, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        cv2.putText(
            frame,
            center_label,
            (min(width - 110, center_x + 6), max(16, center_y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2,
        )
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
    if not isinstance(state, dict):
        return frame
    scene = state.get("scene", {}) or {}
    scenes = state.get("scenes", {}) or {}
    helper_scene = scenes.get("Helper", {}) or scene
    lane_scene = scenes.get("LF_Lanenet", {}) or scene
    overlay_detections(cv2, frame, helper_scene.get("detections", scene.get("detections", [])))
    overlay_lane(cv2, frame, lane_scene.get("lane", scene.get("lane", {})))
    return frame


def run_telemetry_receiver(host, port, bus, stop_event):
    """Receive latest-only monitor patches without touching the filesystem."""
    telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    telemetry_socket.bind((host, int(port)))
    telemetry_socket.settimeout(0.2)
    try:
        while not stop_event.is_set():
            try:
                packet, _ = telemetry_socket.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                payload = json.loads(packet.decode("utf-8"))
                patch = payload.get("patch", {})
                event = payload.get("event")
                if isinstance(patch, dict):
                    bus.update(patch, event=event if isinstance(event, dict) else None)
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
                continue
    finally:
        telemetry_socket.close()


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

        def do_POST(self):
            path = normalize_request_path(self.path)
            if not path.startswith("/api/"):
                self.send_bytes(
                    json.dumps({"ok": False, "error": "not found"}, ensure_ascii=False).encode("utf-8"),
                    "application/json; charset=utf-8",
                    status=HTTPStatus.NOT_FOUND,
                )
                return

            result = apply_gate_command(path[len("/api/"):])
            status = HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
            self.send_bytes(json.dumps(result, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8", status=status)

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


def run_monitor_server(
    memory_name,
    camera_info,
    host="127.0.0.1",
    port=8080,
    stop_sign=None,
    telemetry_host="127.0.0.1",
    telemetry_port=None,
    initial_state=None,
):
    bus = MemoryMonitorBus()
    if initial_state:
        bus.update(initial_state)
    server = ThreadingHTTPServer((host, port), make_handler(memory_name, camera_info, bus))
    telemetry_stop = threading.Event()
    telemetry_thread = None
    if telemetry_port is not None:
        telemetry_thread = threading.Thread(
            target=run_telemetry_receiver,
            args=(telemetry_host, telemetry_port, bus, telemetry_stop),
            daemon=True,
        )
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
    if telemetry_thread is not None:
        telemetry_thread.start()
    print(f"Monitor serving on http://{host}:{port}")
    print(f"SSH tunnel: ssh -L {port}:127.0.0.1:{port} root@CAR_IP")
    try:
        server.serve_forever()
    finally:
        telemetry_stop.set()
        if telemetry_thread is not None:
            telemetry_thread.join(timeout=1)
        server.server_close()
