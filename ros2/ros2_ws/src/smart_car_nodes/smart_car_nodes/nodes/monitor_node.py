import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import SetBool

from smart_car_msgs.msg import CarState, DecisionState, LaneResult, MotorTelemetry, SignDetectionArray
from smart_car_nodes.legacy_monitor_state import LegacyMonitorState
from smart_car_nodes.legacy_monitor_ui import make_index_html, overlay_monitor_results


class MonitorStore:
    def __init__(self):
        self.state = LegacyMonitorState(mode="ros2")
        self.lock = threading.Lock()
        self.image = None
        self.last_image_at = 0.0
        self.image_period = 0.125

    def update_car(self, msg):
        self.state.update_car(
            motor=list(msg.motor),
            servo=list(msg.servo),
            result=msg.result,
            action=msg.action,
            serial_port=msg.serial_port,
            motion_enabled=bool(msg.motion_enabled),
            latency_ms=float(msg.latency_ms),
        )

    def update_lane(self, msg):
        self.state.update_lane(
            steering=float(msg.steering_command),
            filtered=float(msg.filtered_steering),
            inference_ms=float(msg.inference_time_ms),
            lanes=[(line.k, line.b, line.confidence) for line in msg.lanes],
            heading=float(msg.heading_steering),
            lateral=float(msg.lateral_steering),
            lateral_error=float(msg.lateral_error_px),
            lane_center=float(msg.lane_center_x),
            lane_target=float(msg.lane_target_x),
            turn_strength=float(msg.turn_strength),
            intended_action=msg.intended_action,
        )

    def update_signs(self, msg):
        self.state.update_signs(
            [(item.label, item.score, list(item.box)) for item in msg.detections]
        )

    def update_decision(self, msg):
        self.state.update_decision(
            enabled=bool(msg.enabled),
            pedestrian_blocked=bool(msg.pedestrian_blocked),
            action_active=bool(msg.action_active),
            status=msg.status,
            action=msg.action,
            turn_count=int(msg.turn_count),
            right_turn_count=int(msg.right_turn_count),
            turnaround_count=int(msg.turnaround_count),
        )

    def update_motor_control(self, snapshot):
        self.state.update_motor_control(snapshot)

    def update_image(self, msg):
        now = time.time()
        with self.lock:
            if now - self.last_image_at < self.image_period:
                return
            self.last_image_at = now
        try:
            import cv2
            import numpy as np

            frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return
            height, width = frame.shape[:2]
            self.state.update_camera(width, height, 0)
            frame = overlay_monitor_results(cv2, frame, self.state.snapshot())
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
            image = encoded.tobytes() if ok else bytes(msg.data)
        except Exception:
            image = bytes(msg.data)
        with self.lock:
            self.image = image

    def snapshot(self):
        with self.lock:
            image = self.image
        state = self.state.snapshot()
        return state, image


def _path(raw_path):
    path = urlparse(raw_path).path or "/"
    return path.rstrip("/") or "/"


def make_handler(store, control_callback):
    class Handler(BaseHTTPRequestHandler):
        server_version = "CarROS2Monitor/1.0"

        def log_message(self, fmt, *args):
            return

        def send_bytes(self, content, content_type, status=HTTPStatus.OK):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            path = _path(self.path)
            if path in ("/", "/index.html", "/monitor"):
                self.send_bytes(make_index_html().encode("utf-8"), "text/html; charset=utf-8")
                return
            if path == "/api/state":
                state, _ = store.snapshot()
                self.send_bytes(json.dumps(state, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")
                return
            if path == "/health":
                self.send_bytes(b"ok\n", "text/plain; charset=utf-8")
                return
            if path in ("/snapshot.jpg", "/stream.mjpg"):
                if path == "/snapshot.jpg":
                    _, image = store.snapshot()
                    if image is None:
                        self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "waiting for camera")
                    else:
                        self.send_bytes(image, "image/jpeg")
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last_image = None
                while True:
                    _, image = store.snapshot()
                    if image is None or image == last_image:
                        time.sleep(0.05)
                        continue
                    last_image = image
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(image)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(image)
                        self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        break
                return
            self.send_bytes(make_index_html().encode("utf-8"), "text/html; charset=utf-8")

        def do_POST(self):
            path = _path(self.path)
            if not path.startswith("/api/"):
                self.send_bytes(b'{"ok":false,"error":"not found"}', "application/json", HTTPStatus.NOT_FOUND)
                return
            result = control_callback(path[len("/api/") :])
            status = HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
            self.send_bytes(json.dumps(result, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8", status)

    return Handler


class MonitorNode(Node):
    def __init__(self):
        super().__init__("monitor_node")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8080)
        self.store = MonitorStore()
        self.decision_client = self.create_client(SetBool, "/decision_gate/set_enabled")
        self.motion_client = self.create_client(SetBool, "/motion_gate/set_enabled")
        self.create_subscription(CarState, "/car/state", self.store.update_car, 10)
        self.create_subscription(MotorTelemetry, "/car/motor_telemetry", self.store.update_motor_telemetry, 10)
        self.create_subscription(DecisionState, "/decision/state", self.store.update_decision, 10)
        self.create_subscription(LaneResult, "/perception/lane", self.store.update_lane, 10)
        self.create_subscription(SignDetectionArray, "/perception/signs", self.store.update_signs, 10)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self.store.update_image, qos_profile_sensor_data)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.server = ThreadingHTTPServer((host, port), make_handler(self.store, self.handle_control))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f"Monitor serving on http://{host}:{port}/")

    def _set_gate(self, client, enabled, name):
        if not client.service_is_ready():
            self.store.state.add_event(name, "service unavailable")
            return False
        request = SetBool.Request()
        request.data = bool(enabled)
        client.call_async(request)
        self.store.state.add_event(name, "enabled" if enabled else "disabled")
        return True

    def handle_control(self, command):
        state, _ = self.store.snapshot()
        system = state.get("system", {})
        if command == "toggle-decision":
            enabled = not bool(system.get("decision_enabled", False))
            return {"ok": self._set_gate(self.decision_client, enabled, "decision_gate"), "decision_enabled": enabled}
        if command == "start-car":
            motion_ok = self._set_gate(self.motion_client, True, "motion_gate")
            decision_ok = self._set_gate(self.decision_client, True, "decision_gate")
            return {"ok": motion_ok and decision_ok, "motion_enabled": True, "decision_enabled": True}
        if command == "disable-motion":
            return {"ok": self._set_gate(self.motion_client, False, "motion_gate"), "motion_enabled": False}
        return {"ok": False, "error": f"unknown command: {command}"}

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
