import argparse
import json
import sys
import traceback
from pathlib import Path


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from smart_car_nodes.worker_protocol import encode_packet, read_packet


def load_model(kind, model_path):
    if kind == "lane":
        from smart_car_nodes.models.quick_lf import LFModel

        return LFModel(model_path)
    if kind == "sign":
        from smart_car_nodes.models.yolov5 import YoloV5

        return YoloV5(model_path)
    raise ValueError(f"unknown worker kind: {kind}")


def infer(model, kind, jpeg_bytes):
    import cv2
    import numpy as np

    frame = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("failed to decode JPEG frame")
    if kind == "lane":
        lanes, inference_ms = model.pred(frame)
        return {
            "lanes": [[float(k), float(b), float(conf)] for k, b, conf in lanes],
            "inference_time_ms": float(inference_ms),
            "model_width": int(model.model_width),
            "model_height": int(model.model_height),
            "image_width": int(frame.shape[1]),
            "image_height": int(frame.shape[0]),
        }
    detections = model.infer(frame)
    return {
        "detections": [
            [int(x1), int(y1), int(x2), int(y2), str(label), float(score)]
            for x1, y1, x2, y2, label, score in detections
        ]
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kind", choices=("lane", "sign"), required=True)
    parser.add_argument("--model", required=True)
    args = parser.parse_args()

    protocol_out = sys.stdout.buffer
    sys.stdout = sys.stderr
    model = load_model(args.kind, args.model)
    while True:
        payload = read_packet(sys.stdin.buffer)
        if payload is None:
            break
        try:
            result = infer(model, args.kind, payload)
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            result = {"error": str(exc)}
        encoded = json.dumps(result, ensure_ascii=True).encode("utf-8")
        protocol_out.write(encode_packet(encoded))
        protocol_out.flush()


if __name__ == "__main__":
    main()
