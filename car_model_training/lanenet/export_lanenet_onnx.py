import argparse
import sys
from pathlib import Path

import torch


LANENET_ROOT = Path(__file__).resolve().parent
if str(LANENET_ROOT) not in sys.path:
    sys.path.insert(0, str(LANENET_ROOT))


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


def normalize_state_dict(state_dict):
    if isinstance(state_dict, dict) and "state_dict" in state_dict:
        state_dict = state_dict["state_dict"]
    if not isinstance(state_dict, dict):
        raise TypeError("weights must be a PyTorch state_dict or a dict containing state_dict")

    normalized = {}
    for key, value in state_dict.items():
        normalized[key[7:] if key.startswith("module.") else key] = value
    return normalized


def load_lanenet(weights_path, model_module, device):
    weights_path = Path(weights_path)
    if not weights_path.exists():
        raise FileNotFoundError("weights not found: {}".format(weights_path))
    if weights_path.stat().st_size == 0:
        raise ValueError("weights file is empty: {}".format(weights_path))

    model = build_model(model_module).to(device)
    state_dict = normalize_state_dict(torch.load(str(weights_path), map_location=device))
    missing, unexpected = model.load_state_dict(state_dict, strict=False)
    if missing:
        print("warning: missing keys: {}".format(", ".join(missing)))
    if unexpected:
        print("warning: unexpected keys: {}".format(", ".join(unexpected)))
    model.eval()
    return model


def ensure_onnx_available():
    try:
        import onnx  # noqa: F401
    except ImportError as exc:
        raise SystemExit("onnx is required for export. Install it with: pip install onnx") from exc


def export_onnx(args):
    ensure_onnx_available()
    device = select_device(args.device)
    output_path = Path(args.output) if args.output else Path(args.weights).with_suffix(".onnx")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    model = load_lanenet(args.weights, args.model_module, device)
    dummy_input = torch.randn(
        args.batch_size,
        3,
        args.img_height,
        args.img_width,
        dtype=torch.float32,
        device=device,
    )

    with torch.no_grad():
        model(dummy_input)

    torch.onnx.export(
        model,
        dummy_input,
        str(output_path),
        export_params=True,
        opset_version=args.opset,
        do_constant_folding=True,
        input_names=[args.input_name],
        output_names=["binary_logits", "instance_embedding"],
        dynamic_axes=None,
    )

    if not args.skip_check:
        import onnx

        onnx_model = onnx.load(str(output_path))
        onnx.checker.check_model(onnx_model)

    print("exported LaneNet ONNX: {}".format(output_path))
    print(
        "input_shape: {}:{},{},{},{}".format(
            args.input_name,
            args.batch_size,
            3,
            args.img_height,
            args.img_width,
        )
    )
    print("outputs: binary_logits, instance_embedding")
    return output_path


def parse_args():
    parser = argparse.ArgumentParser(description="Export LaneNet .model weights to static ONNX for ATC.")
    parser.add_argument("--weights", required=True, help="Path to trained .model weights.")
    parser.add_argument(
        "--output",
        default=None,
        help="Output .onnx path. Defaults to the weights path with .onnx suffix.",
    )
    parser.add_argument("--model-module", choices=("model2", "model3"), default="model3")
    parser.add_argument("--device", default="auto", help="auto, cpu, cuda, cuda:0, ...")
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--img-height", type=int, default=256)
    parser.add_argument("--img-width", type=int, default=512)
    parser.add_argument("--input-name", default="images")
    parser.add_argument("--opset", type=int, default=11)
    parser.add_argument("--skip-check", action="store_true", help="Skip onnx.checker.check_model.")
    return parser.parse_args()


def main():
    args = parse_args()
    export_onnx(args)


if __name__ == "__main__":
    main()
