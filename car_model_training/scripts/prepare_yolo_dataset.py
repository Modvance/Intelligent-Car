import argparse
import random
import re
from collections import defaultdict
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
FILENAME_CLASS_RE = re.compile(r"^yolo_(?P<class_name>.+)_\d+$")


def read_classes(dataset_root):
    classes_path = dataset_root / "classes.txt"
    if not classes_path.exists():
        raise FileNotFoundError("Missing classes.txt: {}".format(classes_path))
    classes = [line.strip() for line in classes_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not classes:
        raise ValueError("classes.txt is empty")
    return classes


def read_label_class_ids(label_path):
    class_ids = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        class_ids.append(int(stripped.split()[0]))
    return class_ids


def class_group_for_image(image_path, label_path, classes):
    match = FILENAME_CLASS_RE.match(image_path.stem)
    if match:
        class_name = match.group("class_name")
        if class_name in classes:
            return class_name

    class_ids = read_label_class_ids(label_path)
    if class_ids:
        class_id = class_ids[0]
        if 0 <= class_id < len(classes):
            return classes[class_id]
    return "__unknown__"


def collect_samples(dataset_root, classes):
    image_dir = dataset_root / "images"
    label_dir = dataset_root / "labels"
    if not image_dir.exists():
        raise FileNotFoundError("Missing image directory: {}".format(image_dir))
    if not label_dir.exists():
        raise FileNotFoundError("Missing label directory: {}".format(label_dir))

    samples_by_class = defaultdict(list)
    image_paths = sorted(path for path in image_dir.iterdir() if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES)
    if not image_paths:
        raise FileNotFoundError("No images found in {}".format(image_dir))

    for image_path in image_paths:
        label_path = label_dir / "{}.txt".format(image_path.stem)
        if not label_path.exists():
            raise FileNotFoundError("Missing label for image: {}".format(image_path.name))

        class_ids = read_label_class_ids(label_path)
        invalid_ids = [class_id for class_id in class_ids if class_id < 0 or class_id >= len(classes)]
        if invalid_ids:
            raise ValueError("{} has invalid class ids: {}".format(label_path, invalid_ids))

        class_name = class_group_for_image(image_path, label_path, classes)
        samples_by_class[class_name].append(image_path)

    return samples_by_class


def split_group(samples, train_ratio, val_ratio, seed):
    shuffled = list(samples)
    random.Random(seed).shuffle(shuffled)
    total = len(shuffled)
    if total == 0:
        return [], [], []

    train_count = int(total * train_ratio)
    val_count = int(total * val_ratio)

    if total >= 3 and train_ratio > 0 and train_count == 0:
        train_count = 1
    if total >= 3 and val_ratio > 0 and val_count == 0:
        val_count = 1
    if train_count + val_count >= total and total >= 3:
        train_count = max(1, total - 2)
        val_count = 1

    train = shuffled[:train_count]
    val = shuffled[train_count : train_count + val_count]
    test = shuffled[train_count + val_count :]
    return train, val, test


def write_split_file(dataset_root, split_name, image_paths):
    split_path = dataset_root / "{}.txt".format(split_name)
    relative_lines = ["./images/{}".format(path.name) for path in sorted(image_paths)]
    split_path.write_text("\n".join(relative_lines) + ("\n" if relative_lines else ""), encoding="utf-8")
    return split_path


def write_data_yaml(dataset_root, classes):
    names = "[" + ", ".join("'{}'".format(name) for name in classes) + "]"
    data_yaml = "\n".join(
        [
            "path: {}".format(dataset_root.resolve().as_posix()),
            "train: train.txt",
            "val: val.txt",
            "test: test.txt",
            "nc: {}".format(len(classes)),
            "names: {}".format(names),
            "",
        ]
    )
    data_yaml_path = dataset_root / "data.yaml"
    data_yaml_path.write_text(data_yaml, encoding="utf-8")
    return data_yaml_path


def prepare_yolo_dataset(dataset_root, train_ratio=0.8, val_ratio=0.1, seed=2026):
    dataset_root = Path(dataset_root).resolve()
    if train_ratio <= 0 or val_ratio < 0 or train_ratio + val_ratio >= 1:
        raise ValueError("Require train_ratio > 0, val_ratio >= 0, and train_ratio + val_ratio < 1")

    classes = read_classes(dataset_root)
    samples_by_class = collect_samples(dataset_root, classes)

    train_paths = []
    val_paths = []
    test_paths = []
    for class_index, class_name in enumerate(sorted(samples_by_class.keys())):
        train, val, test = split_group(
            samples_by_class[class_name],
            train_ratio=train_ratio,
            val_ratio=val_ratio,
            seed=seed + class_index,
        )
        train_paths.extend(train)
        val_paths.extend(val)
        test_paths.extend(test)

    write_split_file(dataset_root, "train", train_paths)
    write_split_file(dataset_root, "val", val_paths)
    write_split_file(dataset_root, "test", test_paths)
    data_yaml_path = write_data_yaml(dataset_root, classes)

    summary = {
        "total": len(train_paths) + len(val_paths) + len(test_paths),
        "train": len(train_paths),
        "val": len(val_paths),
        "test": len(test_paths),
        "classes": classes,
        "data_yaml": str(data_yaml_path),
    }
    print("YOLO dataset prepared: {}".format(summary))
    return summary


def parse_args():
    workspace_root = next(
        (
            parent
            for parent in Path(__file__).resolve().parents
            if (parent / "datasets_final").exists()
        ),
        Path(__file__).resolve().parents[2],
    )
    parser = argparse.ArgumentParser(description="Prepare YOLOv5 train/val/test files and data.yaml.")
    parser.add_argument(
        "--dataset-root",
        default=str(workspace_root / "datasets_final" / "yolo"),
        help="YOLO dataset root containing images/, labels/, and classes.txt.",
    )
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--val-ratio", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=2026)
    return parser.parse_args()


def main():
    args = parse_args()
    prepare_yolo_dataset(
        dataset_root=args.dataset_root,
        train_ratio=args.train_ratio,
        val_ratio=args.val_ratio,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
