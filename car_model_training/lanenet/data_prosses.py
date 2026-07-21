import argparse
import json
import os
import random
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm


def line_fitting(p1, p2, h_samples, img_width):
    """Fit one LabelMe line segment and sample x coordinates at fixed y rows."""
    x1, y1 = p1
    x2, y2 = p2
    min_y, max_y = sorted([y1, y2])

    if x2 == x1:
        return [int(x1) if min_y <= y <= max_y and 0 <= x1 < img_width else -2 for y in h_samples]

    k = (y2 - y1) / (x2 - x1)
    b = y1 - k * x1

    lane = []
    for y in h_samples:
        if y < min_y or y > max_y:
            lane.append(-2)
            continue
        x = (y - b) / k
        lane.append(int(x) if 0 <= x < img_width else -2)
    return lane


def line_to_polygon(p1, p2, width=16):
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    length = np.sqrt(dx**2 + dy**2)
    if length == 0:
        return np.array([[p1, p1, p1, p1]], dtype=np.int32)

    nx = -dy / length
    ny = dx / length
    offset_x = (width / 2) * nx
    offset_y = (width / 2) * ny

    p1_left = (int(x1 + offset_x), int(y1 + offset_y))
    p1_right = (int(x1 - offset_x), int(y1 - offset_y))
    p2_left = (int(x2 + offset_x), int(y2 + offset_y))
    p2_right = (int(x2 - offset_x), int(y2 - offset_y))
    return np.array([[p1_left, p2_left, p2_right, p1_right]], dtype=np.int32)


def _shape_sort_key(shape):
    label = str(shape.get("label", ""))
    try:
        return 0, int(label)
    except ValueError:
        return 1, label


def _choose_existing_dir(root, preferred, fallbacks):
    preferred_path = root / preferred
    if preferred_path.exists():
        return preferred_path
    for name in fallbacks:
        candidate = root / name
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        "Could not find directory under {}. Tried: {}".format(
            root, ", ".join([preferred] + list(fallbacks))
        )
    )


def _resolve_image_path(json_path, image_dir, label_data):
    candidates = []
    image_path = label_data.get("imagePath")
    if image_path:
        raw = Path(image_path)
        if raw.is_absolute():
            candidates.append(raw)
        else:
            candidates.append((json_path.parent / raw).resolve())
            candidates.append((image_dir / raw.name).resolve())

    for suffix in (".jpg", ".jpeg", ".png", ".bmp"):
        candidates.append((image_dir / "{}{}".format(json_path.stem, suffix)).resolve())

    for candidate in candidates:
        if candidate.exists():
            return candidate

    raise FileNotFoundError("No image found for label {}".format(json_path))


def _h_samples_for_height(img_height):
    start = int(round(img_height * 160 / 720))
    return list(range(max(0, start), img_height, 10))


def _split_lines(lines, train_ratio, val_ratio, test_ratio, seed):
    if train_ratio <= 0 or val_ratio < 0 or test_ratio < 0:
        raise ValueError("train_ratio must be positive, val_ratio/test_ratio must be non-negative")
    if train_ratio + val_ratio + test_ratio <= 0:
        raise ValueError("At least one split ratio must be positive")

    shuffled = list(lines)
    random.Random(seed).shuffle(shuffled)

    total = len(shuffled)
    if total == 0:
        return [], [], []

    normalized = train_ratio + val_ratio + test_ratio
    train_count = int(total * train_ratio / normalized)
    val_count = int(total * val_ratio / normalized)

    # Keep every sample by assigning rounding remainder to the test split.
    train_lines = shuffled[:train_count]
    val_lines = shuffled[train_count : train_count + val_count]
    test_lines = shuffled[train_count + val_count :]

    if total >= 3 and val_ratio > 0 and len(val_lines) == 0:
        val_lines.append(train_lines.pop() if train_lines else test_lines.pop())
    if total >= 3 and test_ratio > 0 and len(test_lines) == 0:
        test_lines.append(train_lines.pop() if train_lines else val_lines.pop())

    return train_lines, val_lines, test_lines


def convert_to_tusimple_format(
    input_dirs,
    output_label_path,
    img_width=None,
    img_height=None,
    image_dir_name="images",
    label_dir_name="jsons",
    mask_width=16,
    train_ratio=0.8,
    val_ratio=0.1,
    test_ratio=0.1,
    seed=2026,
    debug=False,
):
    """Convert LabelMe lane-line annotations into the LaneNet/TuSimple layout."""
    input_dirs = [Path(item) for item in input_dirs]
    output_label_path = Path(output_label_path)
    dataset_root = output_label_path.parent

    training_dir = dataset_root / "training"
    gt_image_dir = training_dir / "gt_image"
    gt_binary_dir = training_dir / "gt_binary_image"
    gt_instance_dir = training_dir / "gt_instance_image"
    txt_dir = dataset_root / "txt_for_local"

    for directory in (gt_image_dir, gt_binary_dir, gt_instance_dir, txt_dir):
        directory.mkdir(parents=True, exist_ok=True)

    label_records = []
    image_info_lines = []
    processed_count = 0

    for input_dir in input_dirs:
        input_dir = input_dir.resolve()
        image_dir = _choose_existing_dir(input_dir, image_dir_name, ("images", "image"))
        label_dir = _choose_existing_dir(input_dir, label_dir_name, ("jsons", "labels", "label"))

        json_files = sorted(label_dir.glob("*.json"))
        if not json_files:
            raise FileNotFoundError("No LabelMe json files found in {}".format(label_dir))

        for json_path in tqdm(json_files, desc="process {}".format(input_dir.name)):
            with json_path.open("r", encoding="utf-8") as file:
                label_data = json.load(file)

            image_path = _resolve_image_path(json_path, image_dir, label_data)
            src_image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if src_image is None:
                raise FileNotFoundError("OpenCV could not read {}".format(image_path))

            source_height, source_width = src_image.shape[:2]
            width = int(img_width or label_data.get("imageWidth") or source_width)
            height = int(img_height or label_data.get("imageHeight") or source_height)
            h_samples = _h_samples_for_height(height)

            lanes = []
            shapes = sorted(label_data.get("shapes", []), key=_shape_sort_key)
            for shape in shapes:
                if shape.get("shape_type") != "line" or len(shape.get("points", [])) != 2:
                    continue
                p1, p2 = shape["points"]
                lanes.append(line_fitting(p1, p2, h_samples, width))

            while len(lanes) < 4:
                lanes.append([-2] * len(h_samples))

            output_index = processed_count + 1
            output_name = "{:04d}.png".format(output_index)
            image_output_path = gt_image_dir / output_name
            binary_output_path = gt_binary_dir / output_name
            instance_output_path = gt_instance_dir / output_name

            output_image = src_image
            if source_width != width or source_height != height:
                output_image = cv2.resize(src_image, (width, height), interpolation=cv2.INTER_LINEAR)

            binary_mask = np.zeros((height, width), dtype=np.uint8)
            instance_mask = np.zeros((height, width), dtype=np.uint8)

            for lane_idx, lane in enumerate(lanes):
                lane_points = [(x, y) for x, y in zip(lane, h_samples) if x != -2]
                if len(lane_points) < 2:
                    continue
                for point_idx in range(len(lane_points) - 1):
                    poly = line_to_polygon(lane_points[point_idx], lane_points[point_idx + 1], width=mask_width)
                    cv2.fillPoly(binary_mask, poly, color=255)
                    cv2.fillPoly(instance_mask, poly, color=lane_idx * 50 + 20)

            cv2.imwrite(str(image_output_path), output_image)
            cv2.imwrite(str(binary_output_path), binary_mask)
            cv2.imwrite(str(instance_output_path), instance_mask)

            label_records.append(
                {
                    "raw_file": str(image_path),
                    "lanes": lanes[:4],
                    "h_samples": h_samples,
                }
            )
            image_info_lines.append(
                "\t".join(
                    [
                        str(image_output_path.resolve()),
                        str(binary_output_path.resolve()),
                        str(instance_output_path.resolve()),
                    ]
                )
            )

            processed_count += 1
            if debug:
                break

        if debug and processed_count:
            break

    output_label_path.parent.mkdir(parents=True, exist_ok=True)
    with output_label_path.open("w", encoding="utf-8") as file:
        for record in label_records:
            file.write(json.dumps(record, ensure_ascii=False) + "\n")

    train_lines, val_lines, test_lines = _split_lines(
        image_info_lines,
        train_ratio=train_ratio,
        val_ratio=val_ratio,
        test_ratio=test_ratio,
        seed=seed,
    )
    split_map = {
        "train.txt": train_lines,
        "val.txt": val_lines,
        "test.txt": test_lines,
    }
    for filename, lines in split_map.items():
        with (txt_dir / filename).open("w", encoding="utf-8") as file:
            for line in lines:
                file.write(line + "\n")

    summary = {
        "total": processed_count,
        "train": len(train_lines),
        "val": len(val_lines),
        "test": len(test_lines),
        "output_root": str(dataset_root.resolve()),
        "label_file": str(output_label_path.resolve()),
        "txt_dir": str(txt_dir.resolve()),
    }
    print("LaneNet dataset prepared: {}".format(summary))
    return summary


def _default_workspace_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "datasets_final").exists():
            return parent
    return Path(__file__).resolve().parents[2]


def parse_args():
    workspace_root = _default_workspace_root()
    default_input_root = workspace_root / "datasets_final" / "lane"
    default_output_root = Path(__file__).resolve().parent / "Datasets" / "Tusimple"

    parser = argparse.ArgumentParser(description="Convert LabelMe lane annotations for LaneNet training.")
    parser.add_argument(
        "--input-root",
        action="append",
        dest="input_roots",
        default=None,
        help="Dataset root containing images/ and jsons/. Can be repeated.",
    )
    parser.add_argument(
        "--output-root",
        default=str(default_output_root),
        help="Output root for training/, txt_for_local/, and labels_tusimple.json.",
    )
    parser.add_argument("--output-label-name", default="labels_tusimple.json")
    parser.add_argument("--image-dir-name", default="images")
    parser.add_argument("--label-dir-name", default="jsons")
    parser.add_argument("--img-width", type=int, default=1280)
    parser.add_argument("--img-height", type=int, default=720)
    parser.add_argument("--mask-width", type=int, default=16)
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--val-ratio", type=float, default=0.1)
    parser.add_argument("--test-ratio", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--debug", action="store_true", help="Only process one sample for visual checking.")

    args = parser.parse_args()
    if args.input_roots is None:
        args.input_roots = [str(default_input_root)]
    return args


def main():
    args = parse_args()
    output_root = Path(args.output_root)
    output_label_path = output_root / args.output_label_name
    convert_to_tusimple_format(
        input_dirs=args.input_roots,
        output_label_path=output_label_path,
        img_width=args.img_width,
        img_height=args.img_height,
        image_dir_name=args.image_dir_name,
        label_dir_name=args.label_dir_name,
        mask_width=args.mask_width,
        train_ratio=args.train_ratio,
        val_ratio=args.val_ratio,
        test_ratio=args.test_ratio,
        seed=args.seed,
        debug=args.debug,
    )


if __name__ == "__main__":
    main()
