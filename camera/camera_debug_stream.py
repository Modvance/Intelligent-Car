#!/usr/bin/env python3
import argparse
import signal
import sys
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Sequence


BOUNDARY = "frame"


@dataclass
class CameraConfig:
    camera: int = 0
    width: int = 1280
    height: int = 720
    fps: int = 30
    jpeg_quality: int = 80
    host: str = "127.0.0.1"
    port: int = 8080
    flip: int = -1
    crosshair: bool = False
    timestamp: bool = False


class FrameStore:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._frame_id = 0
        self.error: Optional[str] = None
        self.running = True

    def update(self, jpeg: bytes) -> None:
        with self._condition:
            self._jpeg = jpeg
            self._frame_id += 1
            self._condition.notify_all()

    def set_error(self, message: str) -> None:
        with self._condition:
            self.error = message
            self._condition.notify_all()

    def stop(self) -> None:
        with self._condition:
            self.running = False
            self._condition.notify_all()

    def latest(self) -> Optional[bytes]:
        with self._condition:
            return self._jpeg

    def wait_for_next(self, last_frame_id: int, timeout: float = 2.0) -> tuple[int, Optional[bytes]]:
        with self._condition:
            deadline = time.monotonic() + timeout
            while self.running and self._frame_id == last_frame_id and self.error is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            return self._frame_id, self._jpeg


def make_index_html(config: CameraConfig) -> str:
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Camera Debug Stream</title>
  <style>
    body {{
      margin: 0;
      background: #111;
      color: #eee;
      font-family: Arial, sans-serif;
    }}
    header {{
      padding: 12px 16px;
      background: #202020;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      flex-wrap: wrap;
    }}
    main {{
      display: grid;
      place-items: center;
      padding: 12px;
    }}
    img {{
      max-width: 100%;
      max-height: calc(100vh - 92px);
      background: #000;
    }}
    a {{
      color: #8cc8ff;
      margin-left: 12px;
    }}
  </style>
</head>
<body>
  <header>
    <div>Camera Debug Stream - camera {config.camera}, {config.width}x{config.height}@{config.fps}</div>
    <nav>
      <a href="/snapshot.jpg" target="_blank">snapshot.jpg</a>
      <a href="/health" target="_blank">health</a>
    </nav>
  </header>
  <main>
    <img src="/stream.mjpg" alt="camera stream">
  </main>
</body>
</html>
"""


def make_tunnel_hint(ssh_target: str, port: int) -> str:
    return (
        f"SSH tunnel from your PC:\n"
        f"  ssh -L {port}:127.0.0.1:{port} {ssh_target}\n\n"
        f"Then open on your PC:\n"
        f"  http://127.0.0.1:{port}\n"
    )


def import_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise SystemExit("opencv-python is required. Install with: python3 -m pip install opencv-python") from exc
    return cv2


def draw_crosshair(cv2, frame) -> None:
    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2
    color = (0, 255, 0)
    cv2.line(frame, (center_x, 0), (center_x, height - 1), color, 1)
    cv2.line(frame, (0, center_y), (width - 1, center_y), color, 1)
    cv2.circle(frame, (center_x, center_y), 8, color, 1)


def apply_overlays(cv2, frame, config: CameraConfig):
    if config.flip in (0, 1, 2):
        flip_code = 0 if config.flip == 0 else 1 if config.flip == 1 else -1
        frame = cv2.flip(frame, flip_code)

    if config.crosshair:
        draw_crosshair(cv2, frame)

    if config.timestamp:
        text = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(frame, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

    return frame


def camera_loop(config: CameraConfig, store: FrameStore) -> None:
    cv2 = import_cv2()
    cap = cv2.VideoCapture()
    cap.open(config.camera, apiPreference=cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    cap.set(cv2.CAP_PROP_FPS, config.fps)

    if not cap.isOpened():
        store.set_error(f"Failed to open camera {config.camera}")
        return

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(config.jpeg_quality)]
    frame_interval = 1.0 / max(config.fps, 1)

    try:
        while store.running:
            started = time.monotonic()
            ok, frame = cap.read()
            if not ok or frame is None:
                store.set_error("Failed to read frame from camera")
                time.sleep(0.2)
                continue

            frame = apply_overlays(cv2, frame, config)
            ok, encoded = cv2.imencode(".jpg", frame, encode_params)
            if ok:
                store.update(encoded.tobytes())

            elapsed = time.monotonic() - started
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
    finally:
        cap.release()


def make_handler(config: CameraConfig, store: FrameStore):
    class CameraDebugHandler(BaseHTTPRequestHandler):
        server_version = "CameraDebugStream/1.0"

        def log_message(self, fmt: str, *args) -> None:
            sys.stderr.write("%s - - [%s] %s\n" % (self.address_string(), self.log_date_time_string(), fmt % args))

        def send_bytes(self, content: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self) -> None:
            if self.path in {"/", "/index.html"}:
                self.send_bytes(make_index_html(config).encode("utf-8"), "text/html; charset=utf-8")
            elif self.path == "/snapshot.jpg":
                self.handle_snapshot()
            elif self.path == "/stream.mjpg":
                self.handle_stream()
            elif self.path == "/health":
                self.handle_health()
            else:
                self.send_error(HTTPStatus.NOT_FOUND, "not found")

        def handle_health(self) -> None:
            latest = store.latest()
            status = "ok" if latest is not None and store.error is None else "waiting"
            if store.error:
                status = f"error: {store.error}"
            self.send_bytes((status + "\n").encode("utf-8"), "text/plain; charset=utf-8")

        def handle_snapshot(self) -> None:
            latest = store.latest()
            if latest is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, store.error or "waiting for first frame")
                return
            self.send_bytes(latest, "image/jpeg")

        def handle_stream(self) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

            frame_id = -1
            while store.running:
                frame_id, jpeg = store.wait_for_next(frame_id)
                if jpeg is None:
                    if store.error:
                        break
                    continue

                try:
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode("ascii"))
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break

    return CameraDebugHandler


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve a USB camera as an MJPEG stream for SSH camera adjustment.")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality, 1-100")
    parser.add_argument("--host", default="127.0.0.1", help="bind address; keep 127.0.0.1 when using SSH tunnel")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--flip", type=int, choices=[-1, 0, 1, 2], default=-1, help="-1 none, 0 vertical, 1 horizontal, 2 both")
    parser.add_argument("--crosshair", action="store_true", help="draw center lines")
    parser.add_argument("--timestamp", action="store_true", help="draw timestamp")
    parser.add_argument("--ssh-target", default="root@CAR_IP", help="printed in SSH tunnel hint")
    return parser


def config_from_args(args: argparse.Namespace) -> CameraConfig:
    return CameraConfig(
        camera=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps,
        jpeg_quality=max(1, min(100, args.quality)),
        host=args.host,
        port=args.port,
        flip=args.flip,
        crosshair=args.crosshair,
        timestamp=args.timestamp,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    config = config_from_args(args)
    store = FrameStore()

    capture_thread = threading.Thread(target=camera_loop, args=(config, store), daemon=True)
    capture_thread.start()

    server = ThreadingHTTPServer((config.host, config.port), make_handler(config, store))

    def shutdown(_signum=None, _frame=None):
        store.stop()
        server.shutdown()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"Camera stream serving on http://{config.host}:{config.port}")
    print(make_tunnel_hint(args.ssh_target, config.port))
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    finally:
        store.stop()
        server.server_close()
        capture_thread.join(timeout=2.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
