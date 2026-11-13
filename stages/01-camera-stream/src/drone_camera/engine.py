"""YOLO11 detection + Ultralytics ByteTrack for ONE camera. Qt independent.

torch/ultralytics are imported lazily so the camera application still starts
(and keeps its previous performance) when AI is unused or not installed. One
engine owns one YOLO instance, hence one ByteTrack state: never share an engine
between cameras (Ultralytics' documented rule for multiple streams).

Security note: a ``.pt`` file is a pickle. Only load model files you trust.
"""

from dataclasses import dataclass
import os
from pathlib import Path

import numpy as np

from .detections import RawTrack

TRACKER_CONFIG = "bytetrack.yaml"   # Explicit: Ultralytics' default tracker is BoT-SORT.
EXPECTED_NAMES = {0: "drone"}


class ModelError(Exception):
    """Message is shown to the user; it never contains stream addresses."""


@dataclass(frozen=True)
class ModelInfo:
    path: str
    names: dict
    device: str
    device_note: str
    ultralytics_version: str
    torch_version: str

    @property
    def class_warning(self) -> str:
        if self.names == EXPECTED_NAMES:
            return ""
        return f"모델 클래스가 {{0: 'drone'}}이 아닙니다: {self.names}"


def configure_environment(config_dir: Path | None = None) -> None:
    """Call before the first import of ultralytics: no network, no auto-install,
    no analytics, and settings kept in an app folder instead of the user's global one."""
    os.environ["YOLO_OFFLINE"] = "1"
    os.environ["YOLO_AUTOINSTALL"] = "False"
    os.environ["YOLO_VERBOSE"] = "False"
    if config_dir is not None and "YOLO_CONFIG_DIR" not in os.environ:
        config_dir.mkdir(parents=True, exist_ok=True)
        os.environ["YOLO_CONFIG_DIR"] = str(config_dir)


def _import_runtime(config_dir: Path | None = None):
    configure_environment(config_dir)
    try:
        import torch
        import ultralytics
        from ultralytics import YOLO
    except ImportError:
        raise ModelError("AI 런타임(PyTorch/Ultralytics)을 불러올 수 없습니다. 설치를 확인하세요.") from None
    except Exception as error:  # Broken native libraries raise OSError and others.
        raise ModelError(f"AI 런타임 초기화에 실패했습니다 ({type(error).__name__}).") from None
    try:
        ultralytics.settings.update({"sync": False})
    except Exception:
        pass
    return torch, ultralytics, YOLO


def resolve_device(preference: str, torch) -> tuple[str, str]:
    """Return (device, note). Never raises: unsupported accelerators fall back to CPU.

    Auto prefers CUDA, otherwise CPU. Apple MPS is available on request only: for the
    small yolo11n model it measured slower than CPU on Apple Silicon (see docs).
    """
    if preference in ("auto", "cuda"):
        if torch.cuda.is_available():
            return "cuda:0", ""
        if preference == "cuda":
            return "cpu", "CUDA를 사용할 수 없어 CPU로 실행합니다."
        return "cpu", ""
    if preference == "mps":
        backend = getattr(torch.backends, "mps", None)
        if backend is not None and backend.is_available():
            return "mps", ""
        return "cpu", "Apple GPU(MPS)를 사용할 수 없어 CPU로 실행합니다."
    return "cpu", ""


def check_model_file(path: str) -> Path:
    if not path or not path.strip():
        raise ModelError("AI 모델 파일이 선택되지 않았습니다.")
    model = Path(path).expanduser()
    if not model.exists():
        raise ModelError(f"모델 파일을 찾을 수 없습니다. 이동하거나 삭제되었는지 확인하세요: {model.name}")
    if not model.is_file():
        raise ModelError("선택한 경로가 파일이 아닙니다.")
    if model.suffix.lower() != ".pt":
        raise ModelError("PyTorch 모델 파일(.pt)을 선택하세요.")
    try:
        size = model.stat().st_size
    except OSError:
        raise ModelError("모델 파일 정보를 읽을 수 없습니다.") from None
    if size == 0:
        raise ModelError("모델 파일이 비어 있습니다.")
    if not os.access(model, os.R_OK):
        raise ModelError("모델 파일을 읽을 권한이 없습니다.")
    return model


class YoloTrackerEngine:
    def __init__(self, model_path: str, device_preference: str = "auto",
                 config_dir: Path | None = None):
        self.model_path = model_path
        self.device_preference = device_preference
        self.config_dir = config_dir
        self.info: ModelInfo | None = None
        self._model = None
        self._device = "cpu"
        self._classes = None

    def load(self) -> ModelInfo:
        path = check_model_file(self.model_path)
        torch, ultralytics, YOLO = _import_runtime(self.config_dir)
        try:
            model = YOLO(str(path))
        except Exception as error:
            raise ModelError("모델 파일이 손상되었거나 Ultralytics가 지원하지 않는 형식입니다 "
                             f"({type(error).__name__}).") from None
        if getattr(model, "task", None) != "detect":
            raise ModelError("객체 탐지(detect) 모델이 아닙니다.")
        names = {int(k): str(v) for k, v in dict(model.names).items()}
        device, note = resolve_device(self.device_preference, torch)
        self._model = model
        self._classes = [k for k, v in names.items() if v.lower() == "drone"] or None
        self._device = device
        try:
            self._warm_up()
        except Exception:
            if device == "cpu":
                self._model = None
                raise ModelError("모델 추론 초기화에 실패했습니다.") from None
            # Accelerator failed to initialise: keep the program usable on CPU.
            self._device = "cpu"
            note = f"{device} 초기화에 실패하여 CPU로 실행합니다."
            self._model = YOLO(str(path))
            try:
                self._warm_up()
            except Exception:
                self._model = None
                raise ModelError("모델 추론 초기화에 실패했습니다.") from None
        self.info = ModelInfo(str(path), names, self._device, note,
                              str(ultralytics.__version__), str(torch.__version__))
        return self.info

    def _warm_up(self):
        blank = np.zeros((64, 64, 3), np.uint8)
        self._model.predict(blank, imgsz=64, device=self._device, verbose=False)
        self.reset()

    def track(self, image: np.ndarray, conf: float, iou: float, imgsz: int) -> list[RawTrack]:
        if self._model is None:
            raise ModelError("모델이 로드되지 않았습니다.")
        result = self._model.track(
            image, persist=True, tracker=TRACKER_CONFIG, conf=conf, iou=iou, imgsz=imgsz,
            device=self._device, classes=self._classes, verbose=False)[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = boxes.xyxy.cpu().numpy()
        scores = boxes.conf.cpu().numpy()
        # boxes.id is None when no detection could be associated to a track.
        ids = boxes.id.cpu().numpy().astype(int) if boxes.is_track else [None] * len(xyxy)
        return [RawTrack(None if i is None else int(i), float(b[0]), float(b[1]), float(b[2]),
                         float(b[3]), float(s)) for b, s, i in zip(xyxy, scores, ids)]

    def reset(self) -> None:
        """Forget all tracks (new connection, AI toggled, parameters replaced)."""
        predictor = getattr(self._model, "predictor", None)
        trackers = getattr(predictor, "trackers", None)
        if not trackers:
            return
        try:
            for tracker in trackers:
                tracker.reset()
        except Exception:
            self._model.predictor = None  # Re-registered with fresh trackers on next call.


def runtime_report(config_dir: Path | None = None) -> dict:
    """Prove the AI runtime works without a trained model file.

    Runs a randomly initialised yolo11n (no weights) and Ultralytics' ByteTrack on
    synthetic data, covering torch, torchvision NMS, bundled config files and lap.
    Used by the frozen-app self test. It says nothing about detection quality.
    """
    report = {"ok": False}
    try:
        torch, ultralytics, YOLO = _import_runtime(config_dir)
        report.update(torch=str(torch.__version__), ultralytics=str(ultralytics.__version__),
                      cuda=bool(torch.cuda.is_available()),
                      mps=bool(getattr(torch.backends, "mps", None) and torch.backends.mps.is_available()))
        from ultralytics.trackers import BYTETracker
        from ultralytics.utils import IterableSimpleNamespace, YAML
        from ultralytics.utils import ROOT
        args = IterableSimpleNamespace(**YAML.load(ROOT / "cfg" / "trackers" / TRACKER_CONFIG))
        tracker = BYTETracker(args)

        class _Detections:
            """Minimal stand-in for Ultralytics Boxes: supports len() and mask/index selection."""
            def __init__(self, xywh, conf=None, cls=None):
                self.xywh = np.asarray(xywh, np.float32).reshape(-1, 4)
                self.conf = np.full(len(self.xywh), 0.9, np.float32) if conf is None else conf
                self.cls = np.zeros(len(self.xywh), np.float32) if cls is None else cls

            def __len__(self):
                return len(self.xywh)

            def __getitem__(self, selection):
                return _Detections(self.xywh[selection], self.conf[selection], self.cls[selection])

        ids = [tracker.update(_Detections([[50 + 4 * f, 60, 20, 10], [200, 90 + 3 * f, 24, 12]]))[:, 4].tolist()
               for f in range(4)]
        report["bytetrack_ids"] = ids[-1]
        if len(set(ids[-1])) != 2 or ids[0] != ids[-1]:
            raise RuntimeError("ByteTrack did not keep two stable IDs.")
        model = YOLO("yolo11n.yaml")  # Random weights from the bundled config; no download.
        model.track(np.zeros((96, 96, 3), np.uint8), persist=True, tracker=TRACKER_CONFIG,
                    imgsz=96, device="cpu", conf=0.01, verbose=False)
        report["ok"] = True
    except ModelError as error:
        report["error"] = str(error)
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"[:300]
    return report
