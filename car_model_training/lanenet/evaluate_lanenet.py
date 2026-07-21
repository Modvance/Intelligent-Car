import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from tqdm import tqdm

LANENET_ROOT = Path(__file__).resolve().parent
if str(LANENET_ROOT) not in sys.path:
    sys.path.insert(0, str(LANENET_ROOT))

from dataset.dataset_utils import TUSIMPLE


def build_model(model_module_name):
    if model_module_name == "model2":
        from Lanenet.model2 import Lanenet
    elif model_module_name == "model3":
        from Lanenet.model3 import Lanenet
    else:
        raise ValueError("Unsupported model module: {}".format(model_module_name))
    return Lanenet(2, 4)


def select_device(device_name):
    if device_name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device_name)


def safe_divide(numerator, denominator):
    return float(numerator / denominator) if denominator else 0.0


class SegmentationMetric:
    def __init__(self):
        self.tp = 0
        self.fp = 0
        self.tn = 0
        self.fn = 0
        self.samples = 0
        self.inference_time_ms = 0.0

    def update(self, pred_mask, target_mask, inference_time_ms=0.0):
        pred = np.asarray(pred_mask).astype(bool)
        target = np.asarray(target_mask).astype(bool)
        if pred.shape != target.shape:
            raise ValueError("shape mismatch: pred {} vs target {}".format(pred.shape, target.shape))

        self.tp += int(np.logical_and(pred, target).sum())
        self.fp += int(np.logical_and(pred, np.logical_not(target)).sum())
        self.tn += int(np.logical_and(np.logical_not(pred), np.logical_not(target)).sum())
        self.fn += int(np.logical_and(np.logical_not(pred), target).sum())
        self.samples += 1
        self.inference_time_ms += float(inference_time_ms)

    def compute(self):
        total = self.tp + self.fp + self.tn + self.fn
        precision = safe_divide(self.tp, self.tp + self.fp)
        recall = safe_divide(self.tp, self.tp + self.fn)
        f1 = safe_divide(2 * precision * recall, precision + recall)
        foreground_iou = safe_divide(self.tp, self.tp + self.fp + self.fn)
        background_iou = safe_divide(self.tn, self.tn + self.fp + self.fn)

        return {
            "samples": self.samples,
            "pixels": total,
            "tp": self.tp,
            "fp": self.fp,
            "tn": self.tn,
            "fn": self.fn,
            "pixel_accuracy": safe_divide(self.tp + self.tn, total),
            "foreground_precision": precision,
            "foreground_recall": recall,
            "foreground_f1": f1,
            "foreground_iou": foreground_iou,
            "background_iou": background_iou,
            "mean_iou": (foreground_iou + background_iou) / 2,
            "avg_inference_time_ms": safe_divide(self.inference_time_ms, self.samples),
        }


def tensor_to_bgr_image(image_tensor):
    image = image_tensor.detach().cpu().numpy()
    image = np.transpose(image, (1, 2, 0))
    image = np.clip((image + 1.0) * 127.5, 0, 255).astype(np.uint8)
    return image


def make_overlay(image_bgr, pred_mask, target_mask):
    pred = np.asarray(pred_mask).astype(bool)
    target = np.asarray(target_mask).astype(bool)
    color = np.zeros_like(image_bgr, dtype=np.uint8)

    color[target] = (0, 255, 0)
    color[pred] = (0, 0, 255)
    color[np.logical_and(pred, target)] = (0, 255, 255)

    return cv2.addWeighted(image_bgr, 0.65, color, 0.35, 0)


def write_metrics(output_dir, metrics):
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "metrics.json"
    csv_path = output_dir / "metrics.csv"

    json_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(metrics.keys()))
        writer.writeheader()
        writer.writerow(metrics)

    return json_path, csv_path


def load_model(weights_path, model_module, device):
    weights_path = Path(weights_path)
    if not weights_path.exists():
        raise FileNotFoundError("weights not found: {}".format(weights_path))
    if weights_path.stat().st_size == 0:
        raise ValueError("weights file is empty: {}".format(weights_path))

    model = build_model(model_module).to(device)
    state_dict = torch.load(str(weights_path), map_location=device)
    model.load_state_dict(state_dict, strict=False)
    model.eval()
    return model


def split_to_flag(split):
    return "valid" if split == "val" else split


def evaluate(args):
    device = select_device(args.device)
    output_dir = Path(args.output_dir)
    vis_dir = output_dir / "vis"
    if args.save_vis:
        vis_dir.mkdir(parents=True, exist_ok=True)

    dataset = TUSIMPLE(root=str(args.txt_root), flag=split_to_flag(args.split))
    data_loader = torch.utils.data.DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
    )
    model = load_model(args.weights, args.model_module, device)

    metric = SegmentationMetric()
    saved_vis = 0
    sample_index = 0

    with torch.no_grad():
        for batch in tqdm(data_loader, desc="evaluate {}".format(args.split)):
            images = batch[0].to(device)
            targets = batch[1].cpu().numpy().astype(np.uint8)

            if device.type == "cuda":
                torch.cuda.synchronize()
            start_time = time.perf_counter()
            binary_logits, _ = model(images)
            if device.type == "cuda":
                torch.cuda.synchronize()
            elapsed_ms = (time.perf_counter() - start_time) * 1000.0

            preds = torch.argmax(binary_logits, dim=1).cpu().numpy().astype(np.uint8)
            batch_time_ms = elapsed_ms / max(len(preds), 1)

            for item_idx in range(len(preds)):
                metric.update(preds[item_idx], targets[item_idx], inference_time_ms=batch_time_ms)

                if args.save_vis and saved_vis < args.vis_count:
                    image = tensor_to_bgr_image(batch[0][item_idx])
                    overlay = make_overlay(image, preds[item_idx], targets[item_idx])
                    source_path = Path(dataset.img_pathes[sample_index][0])
                    out_name = "{:05d}_{}.jpg".format(sample_index, source_path.stem)
                    cv2.imwrite(str(vis_dir / out_name), overlay)
                    saved_vis += 1
                sample_index += 1

    metrics = metric.compute()
    metrics.update(
        {
            "weights": str(args.weights),
            "txt_root": str(args.txt_root),
            "split": args.split,
            "model_module": args.model_module,
            "device": str(device),
            "batch_size": args.batch_size,
        }
    )

    json_path, csv_path = write_metrics(output_dir, metrics)
    print("LaneNet evaluation complete")
    print("metrics_json: {}".format(json_path))
    print("metrics_csv: {}".format(csv_path))
    if args.save_vis:
        print("visualizations: {}".format(vis_dir))
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    return metrics


def parse_args():
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Evaluate LaneNet binary lane segmentation.")
    parser.add_argument("--weights", required=True, help="Path to .model weights.")
    parser.add_argument(
        "--txt-root",
        default=str(script_dir / "Datasets" / "Tusimple" / "txt_for_local"),
        type=Path,
        help="Directory containing train.txt, val.txt and test.txt.",
    )
    parser.add_argument("--split", choices=("train", "val", "test"), default="test")
    parser.add_argument("--model-module", choices=("model2", "model3"), default="model3")
    parser.add_argument("--device", default="auto", help="auto, cpu, cuda, cuda:0, ...")
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--output-dir", default=str(script_dir / "runs" / "lanenet_eval"), type=Path)
    parser.add_argument("--save-vis", action="store_true", help="Save overlay visualizations.")
    parser.add_argument("--vis-count", type=int, default=50)
    return parser.parse_args()


def main():
    args = parse_args()
    evaluate(args)


if __name__ == "__main__":
    main()
