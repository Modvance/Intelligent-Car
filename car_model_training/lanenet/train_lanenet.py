"""Train or fine-tune LaneNet from prepared TuSimple-style text manifests."""

import argparse
import time
from pathlib import Path

import torch
from tqdm import tqdm

from dataset.dataset_utils import TUSIMPLE, TUSIMPLE_AUG


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


def parse_args():
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Train LaneNet with prepared TuSimple-style data.")
    parser.add_argument(
        "--txt-root",
        default=str(script_dir / "Datasets" / "Tusimple" / "txt_for_local"),
        help="Directory containing train.txt, val.txt and test.txt.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(script_dir / "runs" / "lanenet_output"),
        help="Directory for saved .model checkpoints.",
    )
    parser.add_argument("--pretrained", default="", help="Optional .model checkpoint to continue training from.")
    parser.add_argument("--model-module", choices=("model2", "model3"), default="model3")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--learning-rate", type=float, default=5e-4)
    parser.add_argument("--weight-decay", type=float, default=2e-4)
    parser.add_argument("--step-size", type=int, default=5)
    parser.add_argument("--gamma", type=float, default=0.1)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--save-period", type=int, default=5)
    parser.add_argument("--device", default="auto", help="auto, cpu, cuda, cuda:0, ...")
    parser.add_argument("--aug", action="store_true", help="Use horizontal-flip augmentation dataset wrapper.")
    parser.add_argument("--run-name", default="lanenet_custom")
    return parser.parse_args()


def train(args):
    txt_root = Path(args.txt_root)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    dataset_class = TUSIMPLE_AUG if args.aug else TUSIMPLE
    train_set = dataset_class(root=str(txt_root), flag="train")
    valid_set = dataset_class(root=str(txt_root), flag="valid")
    test_set = dataset_class(root=str(txt_root), flag="test")

    print("train_set length {}".format(len(train_set)))
    print("valid_set length {}".format(len(valid_set)))
    print("test_set length {}".format(len(test_set)))

    data_loader_train = torch.utils.data.DataLoader(
        train_set,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
    )

    device = select_device(args.device)
    print("device: {}".format(device))

    model = build_model(args.model_module).to(device)
    if args.pretrained:
        pretrained_path = Path(args.pretrained)
        state_dict = torch.load(str(pretrained_path), map_location=device)
        model.load_state_dict(state_dict, strict=False)
        print("loaded pretrained weights: {}".format(pretrained_path))

    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.Adam(params, lr=args.learning_rate, weight_decay=args.weight_decay)
    lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=args.step_size, gamma=args.gamma)
    from Lanenet.cluster_loss3 import cluster_loss

    criterion = cluster_loss()

    loss_history = []
    for epoch in tqdm(range(args.epochs), desc="epochs"):
        model.train()
        start_time = time.time()

        for iteration, batch in enumerate(data_loader_train):
            input_image = batch[0].to(device)
            binary_labels = batch[1].to(device)
            instance_labels = batch[2].to(device)

            binary_final_logits, instance_embedding = model(input_image)
            binary_loss, instance_loss = criterion(
                binary_logits=binary_final_logits,
                binary_labels=binary_labels,
                instance_logits=instance_embedding,
                instance_labels=instance_labels,
                delta_v=0.5,
                delta_d=3,
            )
            loss = binary_loss + instance_loss

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            loss_history.append(loss.item())

            if iteration % 20 == 0:
                print(
                    "epoch[{}] iter[{}] loss: [{:.6f}, {:.6f}]".format(
                        epoch,
                        iteration,
                        binary_loss.item(),
                        instance_loss.item(),
                    )
                )

        lr_scheduler.step()
        print("Finish epoch[{}], time elapsed[{:.2f}s]".format(epoch, time.time() - start_time))

        if args.save_period > 0 and epoch % args.save_period == 0:
            checkpoint_path = output_dir / "{}_epoch_{}_batch_{}.model".format(
                args.run_name,
                epoch,
                args.batch_size,
            )
            torch.save(model.state_dict(), str(checkpoint_path))
            print("saved {}".format(checkpoint_path))

    final_path = output_dir / "{}_final_batch_{}.model".format(args.run_name, args.batch_size)
    torch.save(model.state_dict(), str(final_path))
    print("saved final model: {}".format(final_path))
    return final_path


def main():
    args = parse_args()
    train(args)


if __name__ == "__main__":
    main()
