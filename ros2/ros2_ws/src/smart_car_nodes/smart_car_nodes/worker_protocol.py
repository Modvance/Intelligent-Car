import json
import struct
import subprocess
import sys
import threading
from pathlib import Path


HEADER = struct.Struct("<I")


def encode_packet(payload: bytes) -> bytes:
    return HEADER.pack(len(payload)) + payload


def _read_exact(stream, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            raise EOFError(f"expected {size} bytes, received {size - remaining}")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_packet(stream):
    header = stream.read(HEADER.size)
    if header == b"":
        return None
    if len(header) != HEADER.size:
        raise EOFError("truncated packet header")
    (size,) = HEADER.unpack(header)
    return _read_exact(stream, size)


class InferenceWorkerClient:
    def __init__(self, kind: str, python_executable: str, model_path: str):
        worker = Path(__file__).with_name("inference_worker.py")
        executable = python_executable or sys.executable
        self.process = subprocess.Popen(
            [executable, str(worker), "--kind", kind, "--model", model_path],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            bufsize=0,
        )
        self.lock = threading.Lock()

    def infer(self, jpeg_bytes: bytes):
        with self.lock:
            if self.process.poll() is not None:
                raise RuntimeError(f"inference worker exited with code {self.process.returncode}")
            self.process.stdin.write(encode_packet(jpeg_bytes))
            self.process.stdin.flush()
            payload = read_packet(self.process.stdout)
            if payload is None:
                raise EOFError("inference worker closed stdout")
        result = json.loads(payload.decode("utf-8"))
        if "error" in result:
            raise RuntimeError(result["error"])
        return result

    def close(self):
        if self.process.poll() is not None:
            return
        try:
            self.process.stdin.close()
            self.process.wait(timeout=2.0)
        except (BrokenPipeError, subprocess.TimeoutExpired):
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
