from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class PackagedModelPaths:
    model_path: str
    default_filename: str

    def resolve_model(self) -> Path:
        if self.model_path:
            model = Path(self.model_path).expanduser().resolve()
        else:
            try:
                from ament_index_python.packages import get_package_share_directory

                model = Path(get_package_share_directory("smart_car_nodes")) / "weights" / self.default_filename
            except ImportError:
                model = Path(__file__).resolve().parents[1] / "weights" / self.default_filename
        if not model.exists():
            raise FileNotFoundError(f"model file does not exist: {model}")
        return model


class PackagedLaneAdapter:
    def __init__(self, paths: PackagedModelPaths):
        from smart_car_nodes.models.quick_lf import LFModel

        self.model = LFModel(str(paths.resolve_model()))

    def pred(self, frame):
        return self.model.pred(frame)


class PackagedSignAdapter:
    def __init__(self, paths: PackagedModelPaths):
        from smart_car_nodes.models.yolov5 import YoloV5

        self.model = YoloV5(str(paths.resolve_model()))

    @property
    def names(self):
        return getattr(self.model, "names", [])

    def infer(self, frame):
        return self.model.infer(frame)
