"""Phase 03 camera intrinsic/extrinsic calibration, lens undistortion and persistence.

Important safety rules:
* When no calibration file is loaded, the system is UNCALIBRATED and must never
  invent or output 3D coordinates as if they were real measurements.
* Synthetic calibrations created for software testing or simulation carry
  ``calibration_type="synthetic"`` so the GUI and logs always distinguish them
  from real field calibrations (``calibration_type="field"``).
"""

from dataclasses import asdict, dataclass, replace
import json
import math
import os
from pathlib import Path
import tempfile

import cv2
import numpy as np

CALIBRATION_FILE_NAME = "stereo_calibration.json"
CALIBRATION_TYPES = ("field", "synthetic")


class CalibrationError(ValueError):
    pass


def _is_finite_number(value) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


@dataclass(frozen=True)
class CameraIntrinsics:
    camera_id: str
    image_size: tuple[int, int]  # (width, height) in pixels
    camera_matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    dist_coeffs: tuple[float, ...]  # (k1, k2, p1, p2[, k3[, k4, k5, k6]])
    rms_error: float = 0.0

    def __post_init__(self):
        if self.camera_id not in ("camera-01", "camera-02"):
            raise CalibrationError("Camera ID must be camera-01 or camera-02.")
        if (not isinstance(self.image_size, (tuple, list)) or len(self.image_size) != 2
                or any(type(v) is not int or v < 16 or v > 7680 for v in self.image_size)):
            raise CalibrationError("Image size must be (width, height) integers between 16 and 7680.")
        object.__setattr__(self, "image_size", (int(self.image_size[0]), int(self.image_size[1])))

        try:
            mat = tuple(tuple(float(v) for v in row) for row in self.camera_matrix)
        except (TypeError, ValueError):
            raise CalibrationError("Camera matrix must be a 3x3 numeric matrix.") from None
        if len(mat) != 3 or any(len(row) != 3 for row in mat):
            raise CalibrationError("Camera matrix must have shape 3x3.")
        if any(not math.isfinite(v) for row in mat for v in row):
            raise CalibrationError("Camera matrix entries must be finite numbers.")
        if mat[0][0] <= 0 or mat[1][1] <= 0:
            raise CalibrationError("Focal lengths fx and fy must be positive.")
        if abs(mat[0][1]) > 1e-2 or abs(mat[1][0]) > 1e-2 or abs(mat[2][0]) > 1e-4 or abs(mat[2][1]) > 1e-4 or abs(mat[2][2] - 1.0) > 1e-4:
            raise CalibrationError("Camera matrix must be upper-triangular with bottom row [0, 0, 1].")
        object.__setattr__(self, "camera_matrix", mat)

        try:
            dist = tuple(float(v) for v in self.dist_coeffs)
        except (TypeError, ValueError):
            raise CalibrationError("Distortion coefficients must be numeric.") from None
        if len(dist) not in (4, 5, 8, 12, 14) or any(not math.isfinite(v) for v in dist):
            raise CalibrationError("Distortion coefficients must contain 4, 5, 8, 12, or 14 finite numbers.")
        object.__setattr__(self, "dist_coeffs", dist)

        if not _is_finite_number(self.rms_error) or self.rms_error < 0:
            raise CalibrationError("RMS reprojection error must be a non-negative finite number.")
        object.__setattr__(self, "rms_error", float(self.rms_error))

    @property
    def K(self) -> np.ndarray:
        return np.array(self.camera_matrix, dtype=np.float64)

    @property
    def D(self) -> np.ndarray:
        return np.array(self.dist_coeffs, dtype=np.float64)

    @property
    def fx(self) -> float:
        return self.camera_matrix[0][0]

    @property
    def fy(self) -> float:
        return self.camera_matrix[1][1]

    @property
    def cx(self) -> float:
        return self.camera_matrix[0][2]

    @property
    def cy(self) -> float:
        return self.camera_matrix[1][2]


@dataclass(frozen=True)
class StereoCalibration:
    """Complete stereo calibration transforming Camera 1 coordinates to Camera 2:
    X_cam2 = R @ X_cam1 + T (in meters). Camera 1 is the world origin [I | 0].
    """
    camera_01: CameraIntrinsics
    camera_02: CameraIntrinsics
    rotation: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    translation: tuple[float, float, float]  # (tx, ty, tz) in meters
    baseline_m: float
    calibration_type: str = "field"          # "field" or "synthetic"
    rms_error: float = 0.0
    time_offset_s: float = 0.0               # Added to camera-02 timestamps: t2_sync = t2 + time_offset_s
    max_sync_delta_s: float = 0.10           # Max allowed |t1 - t2_sync| in seconds
    max_reprojection_error_px: float = 15.0  # Gate for valid stereo triangulation
    max_epipolar_error_px: float = 15.0      # Gate for cross-camera epipolar matching
    min_depth_m: float = 0.2
    max_depth_m: float = 200.0
    ambiguity_margin_px: float = 5.0         # Minimum cost separation to disambiguate multi-drone matches
    notes: str = ""

    def __post_init__(self):
        if not isinstance(self.camera_01, CameraIntrinsics) or self.camera_01.camera_id != "camera-01":
            raise CalibrationError("camera_01 must be CameraIntrinsics for camera-01.")
        if not isinstance(self.camera_02, CameraIntrinsics) or self.camera_02.camera_id != "camera-02":
            raise CalibrationError("camera_02 must be CameraIntrinsics for camera-02.")
        if self.calibration_type not in CALIBRATION_TYPES:
            raise CalibrationError(f"calibration_type must be one of {CALIBRATION_TYPES}.")

        try:
            rot = tuple(tuple(float(v) for v in row) for row in self.rotation)
        except (TypeError, ValueError):
            raise CalibrationError("Rotation must be a 3x3 numeric matrix.") from None
        if len(rot) != 3 or any(len(row) != 3 for row in rot) or any(not math.isfinite(v) for row in rot for v in row):
            raise CalibrationError("Rotation must be a 3x3 finite matrix.")
        r_mat = np.array(rot, dtype=np.float64)
        ortho_err = float(np.max(np.abs(r_mat.T @ r_mat - np.eye(3))))
        det_val = float(np.linalg.det(r_mat))
        if ortho_err > 1e-2 or abs(det_val - 1.0) > 1e-2:
            raise CalibrationError("Rotation matrix must be a valid 3D rotation (R^T R = I, det(R) = 1).")
        object.__setattr__(self, "rotation", rot)

        try:
            trans = tuple(float(v) for v in self.translation)
        except (TypeError, ValueError):
            raise CalibrationError("Translation must be a 3-element numeric vector.") from None
        if len(trans) != 3 or any(not math.isfinite(v) for v in trans):
            raise CalibrationError("Translation must contain 3 finite numbers in meters.")
        t_norm = math.sqrt(sum(v * v for v in trans))
        if t_norm <= 1e-6:
            raise CalibrationError("Translation vector norm (baseline) must be positive.")
        object.__setattr__(self, "translation", trans)

        if not _is_finite_number(self.baseline_m) or self.baseline_m <= 1e-4 or self.baseline_m > 1000.0:
            raise CalibrationError("Baseline length must be a positive number in meters (0.0001 to 1000).")
        if abs(t_norm - float(self.baseline_m)) > 1e-3:
            raise CalibrationError(
                f"Translation vector length ({t_norm:.4f} m) does not match baseline_m ({self.baseline_m:.4f} m).")
        object.__setattr__(self, "baseline_m", float(self.baseline_m))

        if not _is_finite_number(self.rms_error) or self.rms_error < 0:
            raise CalibrationError("Stereo RMS error must be non-negative.")
        object.__setattr__(self, "rms_error", float(self.rms_error))

        if not _is_finite_number(self.time_offset_s) or not -10.0 <= self.time_offset_s <= 10.0:
            raise CalibrationError("time_offset_s must be between -10.0 and 10.0 seconds.")
        object.__setattr__(self, "time_offset_s", float(self.time_offset_s))

        if not _is_finite_number(self.max_sync_delta_s) or not 0.001 <= self.max_sync_delta_s <= 5.0:
            raise CalibrationError("max_sync_delta_s must be between 0.001 and 5.0 seconds.")
        object.__setattr__(self, "max_sync_delta_s", float(self.max_sync_delta_s))

        for name, low, high in (
            ("max_reprojection_error_px", 0.5, 200.0),
            ("max_epipolar_error_px", 0.5, 200.0),
            ("min_depth_m", 0.05, 50.0),
            ("max_depth_m", 1.0, 2000.0),
            ("ambiguity_margin_px", 0.0, 100.0),
        ):
            val = getattr(self, name)
            if not _is_finite_number(val) or not low <= val <= high:
                raise CalibrationError(f"{name} must be a finite number from {low} to {high}.")
            object.__setattr__(self, name, float(val))
        if self.min_depth_m >= self.max_depth_m:
            raise CalibrationError("min_depth_m must be smaller than max_depth_m.")
        if not isinstance(self.notes, str) or len(self.notes) > 1000:
            raise CalibrationError("notes must be a string up to 1000 characters.")

    @property
    def is_field_calibrated(self) -> bool:
        return self.calibration_type == "field"

    @property
    def R(self) -> np.ndarray:
        return np.array(self.rotation, dtype=np.float64)

    @property
    def T(self) -> np.ndarray:
        return np.array(self.translation, dtype=np.float64).reshape(3, 1)

    @property
    def P1_normalized(self) -> np.ndarray:
        """3x4 normalized projection matrix [I | 0] for Camera 1."""
        return np.hstack((np.eye(3, dtype=np.float64), np.zeros((3, 1), dtype=np.float64)))

    @property
    def P2_normalized(self) -> np.ndarray:
        """3x4 normalized projection matrix [R | T] for Camera 2."""
        return np.hstack((self.R, self.T))

    def camera_2_center_in_cam1(self) -> np.ndarray:
        """3D position of Camera 2's optical center in Camera 1's coordinate system (meters)."""
        return (-self.R.T @ self.T).reshape(3)

    def fundamental_matrix(self) -> np.ndarray:
        """Fundamental matrix F such that x2^T F x1 = 0 for undistorted pixel coordinates."""
        tx, ty, tz = self.translation
        t_cross = np.array([
            [0.0, -tz, ty],
            [tz, 0.0, -tx],
            [-ty, tx, 0.0],
        ], dtype=np.float64)
        essential = t_cross @ self.R
        k1_inv = np.linalg.inv(self.camera_01.K)
        k2_inv_t = np.linalg.inv(self.camera_02.K).T
        f_mat = k2_inv_t @ essential @ k1_inv
        norm = np.linalg.norm(f_mat)
        return f_mat / norm if norm > 0 else f_mat

    def with_baseline(self, baseline_m: float) -> "StereoCalibration":
        """Return a new StereoCalibration with translation scaled to ``baseline_m`` meters."""
        if not _is_finite_number(baseline_m) or baseline_m <= 1e-4 or baseline_m > 1000.0:
            raise CalibrationError("Baseline length must be between 0.0001 and 1000 meters.")
        current_norm = math.sqrt(sum(v * v for v in self.translation))
        scale = float(baseline_m) / current_norm
        scaled_t = tuple(v * scale for v in self.translation)
        return replace(self, translation=scaled_t, baseline_m=float(baseline_m))

    def with_sync_params(self, time_offset_s: float, max_sync_delta_s: float) -> "StereoCalibration":
        return replace(self, time_offset_s=float(time_offset_s), max_sync_delta_s=float(max_sync_delta_s))

    def to_dict(self) -> dict:
        return {"version": 1, **asdict(self)}

    @classmethod
    def from_dict(cls, data: dict) -> "StereoCalibration":
        if not isinstance(data, dict) or data.get("version") != 1:
            raise CalibrationError("Expected stereo calibration version 1.")
        try:
            cam1_raw = data["camera_01"]
            cam2_raw = data["camera_02"]
            cam1 = CameraIntrinsics(**cam1_raw)
            cam2 = CameraIntrinsics(**cam2_raw)
            kwargs = {
                k: v for k, v in data.items()
                if k not in ("version", "camera_01", "camera_02") and k in cls.__dataclass_fields__
            }
            return cls(camera_01=cam1, camera_02=cam2, **kwargs)
        except CalibrationError:
            raise
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationError(f"Invalid stereo calibration structure: {exc}") from None


# ------------------------------------------------------------------ persistence
def calibration_path_for(config_path: Path) -> Path:
    return config_path.with_name(CALIBRATION_FILE_NAME)


def load_calibration(path: Path) -> StereoCalibration | None:
    """Return None when no calibration file exists (system stays uncalibrated).
    Raise CalibrationError if the file exists but is damaged or invalid.
    """
    if not path.exists():
        return None
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise CalibrationError(f"보정 파일을 읽을 수 없습니다: {exc}") from None
    return StereoCalibration.from_dict(raw)


def save_calibration(path: Path, calibration: StereoCalibration) -> None:
    if not isinstance(calibration, StereoCalibration):
        raise CalibrationError("A valid StereoCalibration instance is required.")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, prefix=".calib-", suffix=".tmp", delete=False
        ) as stream:
            tmp = Path(stream.name)
            json.dump(calibration.to_dict(), stream, indent=2, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)


# ---------------------------------------------------------- OpenCV calibration
def chessboard_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    """3D coordinates of inner corners on a flat Z=0 chessboard in meters."""
    if type(cols) is not int or type(rows) is not int or cols < 3 or rows < 3 or cols > 30 or rows > 30:
        raise CalibrationError("체커보드 내부 코너 수는 가로·세로 각각 3~30 사이의 정수여야 합니다.")
    if not _is_finite_number(square_size_m) or not 0.001 <= square_size_m <= 2.0:
        raise CalibrationError("체커보드 사각형 한 변의 길이는 0.001 m ~ 2.0 m 사이여야 합니다.")
    grid = np.zeros((rows * cols, 3), dtype=np.float32)
    xs, ys = np.meshgrid(np.arange(cols, dtype=np.float32), np.arange(rows, dtype=np.float32))
    grid[:, 0] = xs.reshape(-1) * float(square_size_m)
    grid[:, 1] = ys.reshape(-1) * float(square_size_m)
    return grid


def detect_chessboard_corners(image: np.ndarray, cols: int, rows: int, refine: bool = True) -> np.ndarray | None:
    """Find inner chessboard corners (returns Nx1x2 float32 array or None)."""
    if image is None or not isinstance(image, np.ndarray) or image.ndim not in (2, 3):
        raise CalibrationError("유효한 영상 배열이 필요합니다.")
    if type(cols) is not int or type(rows) is not int or cols < 3 or rows < 3:
        raise CalibrationError("체커보드 내부 코너 수는 3 이상이어야 합니다.")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not found or corners is None:
        return None
    if refine:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return corners.astype(np.float32)


def draw_chessboard_preview(image: np.ndarray, cols: int, rows: int, corners: np.ndarray | None) -> np.ndarray:
    canvas = image.copy()
    if corners is not None:
        cv2.drawChessboardCorners(canvas, (cols, rows), corners, True)
    return canvas


def calibrate_single_camera(
    camera_id: str,
    corners_list: list[np.ndarray],
    image_size: tuple[int, int],
    cols: int,
    rows: int,
    square_size_m: float,
) -> CameraIntrinsics:
    """Compute camera matrix K and lens distortion coefficients from chessboard views."""
    if len(corners_list) < 3:
        raise CalibrationError("카메라 내부 보정을 위해 최소 3장 이상의 체커보드 코너 세트가 필요합니다.")
    obj_pts = chessboard_object_points(cols, rows, square_size_m)
    expected_len = cols * rows
    clean_corners = []
    for corners in corners_list:
        arr = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
        if len(arr) != expected_len:
            raise CalibrationError(f"코너 개수({len(arr)})가 체커보드 규격({expected_len})과 일치하지 않습니다.")
        clean_corners.append(arr)
    object_points = [obj_pts for _ in clean_corners]
    rms, k_mat, dist, _rvecs, _tvecs = cv2.calibrateCamera(
        object_points, clean_corners, (int(image_size[0]), int(image_size[1])), None, None
    )
    k_tuple = tuple(tuple(float(v) for v in row) for row in k_mat)
    d_tuple = tuple(float(v) for v in dist.reshape(-1))
    return CameraIntrinsics(
        camera_id=camera_id,
        image_size=(int(image_size[0]), int(image_size[1])),
        camera_matrix=k_tuple,
        dist_coeffs=d_tuple,
        rms_error=float(rms),
    )


def calibrate_stereo_cameras(
    corners_01: list[np.ndarray],
    corners_02: list[np.ndarray],
    image_size_01: tuple[int, int],
    image_size_02: tuple[int, int],
    cols: int,
    rows: int,
    square_size_m: float,
    baseline_m: float | None = None,
    intrinsics_01: CameraIntrinsics | None = None,
    intrinsics_02: CameraIntrinsics | None = None,
    calibration_type: str = "field",
    time_offset_s: float = 0.0,
    max_sync_delta_s: float = 0.10,
    notes: str = "",
) -> StereoCalibration:
    """Calibrate both cameras' intrinsics and their relative pose (R, T)."""
    if len(corners_01) != len(corners_02) or len(corners_01) < 3:
        raise CalibrationError("스테레오 보정을 위해 두 카메라에서 동시에 촬영된 최소 3쌍 이상의 코너가 필요합니다.")
    intr_1 = intrinsics_01 or calibrate_single_camera(
        "camera-01", corners_01, image_size_01, cols, rows, square_size_m
    )
    intr_2 = intrinsics_02 or calibrate_single_camera(
        "camera-02", corners_02, image_size_02, cols, rows, square_size_m
    )
    obj_pts = chessboard_object_points(cols, rows, square_size_m)
    expected_len = cols * rows
    pts_1 = [np.asarray(c, dtype=np.float32).reshape(expected_len, 1, 2) for c in corners_01]
    pts_2 = [np.asarray(c, dtype=np.float32).reshape(expected_len, 1, 2) for c in corners_02]
    object_points = [obj_pts for _ in pts_1]

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    rms, _k1, _d1, _k2, _d2, r_mat, t_vec, _e_mat, _f_mat = cv2.stereoCalibrate(
        object_points,
        pts_1,
        pts_2,
        intr_1.K,
        intr_1.D,
        intr_2.K,
        intr_2.D,
        (int(image_size_01[0]), int(image_size_01[1])),
        criteria=criteria,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    t_flat = tuple(float(v) for v in t_vec.reshape(3))
    measured_baseline = math.sqrt(sum(v * v for v in t_flat))
    if measured_baseline <= 1e-6:
        raise CalibrationError("추정된 두 카메라 간 기준선 길이가 0에 가깝습니다. 카메라 배치를 확인하세요.")
    rot_tuple = tuple(tuple(float(v) for v in row) for row in r_mat)
    calib = StereoCalibration(
        camera_01=intr_1,
        camera_02=intr_2,
        rotation=rot_tuple,
        translation=t_flat,
        baseline_m=measured_baseline,
        calibration_type=calibration_type,
        rms_error=float(rms),
        time_offset_s=time_offset_s,
        max_sync_delta_s=max_sync_delta_s,
        notes=notes,
    )
    if baseline_m is not None:
        calib = calib.with_baseline(baseline_m)
    return calib


# ------------------------------------------- lens undistortion & projection
def undistort_frame(image: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    """Remove lens distortion from a full image using the calibrated intrinsics."""
    return cv2.undistort(image, intrinsics.K, intrinsics.D)


def undistort_points_normalized(points_px: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    """Convert distorted pixel coordinates (Nx2) to normalized pinhole coordinates (Nx2)."""
    pts = np.asarray(points_px, dtype=np.float64).reshape(-1, 1, 2)
    undist = cv2.undistortPoints(pts, intrinsics.K, intrinsics.D)
    return undist.reshape(-1, 2)


def undistort_point_px(u: float, v: float, intrinsics: CameraIntrinsics) -> tuple[float, float]:
    """Convert one distorted pixel (u, v) to ideal undistorted pixel coordinates."""
    pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
    undist = cv2.undistortPoints(pts, intrinsics.K, intrinsics.D, P=intrinsics.K)
    u_out, v_out = undist.reshape(2)
    return float(u_out), float(v_out)


def project_points_3d(
    points_3d: np.ndarray,
    intrinsics: CameraIntrinsics,
    rotation: np.ndarray | None = None,
    translation: np.ndarray | None = None,
) -> np.ndarray:
    """Project 3D points (Nx3 in Camera 1 frame) into image pixels (Nx2) including lens distortion."""
    pts = np.asarray(points_3d, dtype=np.float64).reshape(-1, 1, 3)
    r_mat = np.eye(3, dtype=np.float64) if rotation is None else np.asarray(rotation, dtype=np.float64)
    rvec, _ = cv2.Rodrigues(r_mat)
    tvec = np.zeros((3, 1), dtype=np.float64) if translation is None else np.asarray(translation, dtype=np.float64).reshape(3, 1)
    img_pts, _ = cv2.projectPoints(pts, rvec, tvec, intrinsics.K, intrinsics.D)
    return img_pts.reshape(-1, 2)


# ----------------------------------------- synthetic calibration & test helpers
def create_synthetic_calibration(
    image_size: tuple[int, int] = (1280, 720),
    focal_px: float = 900.0,
    baseline_m: float = 2.0,
    convergence_deg: float = 0.0,
    dist_coeffs_01: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0),
    dist_coeffs_02: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0),
    time_offset_s: float = 0.0,
    max_sync_delta_s: float = 0.10,
    notes: str = "합성 스테레오 보정 (실험 전 소프트웨어 검증용)",
) -> StereoCalibration:
    """Create a known stereo calibration rig with ``calibration_type='synthetic'``."""
    width, height = int(image_size[0]), int(image_size[1])
    cx, cy = width / 2.0, height / 2.0
    k_tuple = (
        (float(focal_px), 0.0, cx),
        (0.0, float(focal_px), cy),
        (0.0, 0.0, 1.0),
    )
    intr_1 = CameraIntrinsics("camera-01", (width, height), k_tuple, dist_coeffs_01, rms_error=0.0)
    intr_2 = CameraIntrinsics("camera-02", (width, height), k_tuple, dist_coeffs_02, rms_error=0.0)

    # Camera 2 is located at (+baseline_m, 0, 0) in Camera 1 frame, rotated by convergence_deg around Y.
    yaw_rad = math.radians(float(convergence_deg))
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    # R transforms vectors from cam1 to cam2
    r_mat = np.array([
        [cos_y, 0.0, sin_y],
        [0.0, 1.0, 0.0],
        [-sin_y, 0.0, cos_y],
    ], dtype=np.float64)
    cam2_pos_in_cam1 = np.array([float(baseline_m), 0.0, 0.0], dtype=np.float64)
    t_vec = -r_mat @ cam2_pos_in_cam1
    return StereoCalibration(
        camera_01=intr_1,
        camera_02=intr_2,
        rotation=tuple(tuple(float(v) for v in row) for row in r_mat),
        translation=tuple(float(v) for v in t_vec),
        baseline_m=float(baseline_m),
        calibration_type="synthetic",
        rms_error=0.0,
        time_offset_s=time_offset_s,
        max_sync_delta_s=max_sync_delta_s,
        notes=notes,
    )


def project_chessboard_corners_pair(
    calibration: StereoCalibration,
    cols: int,
    rows: int,
    square_size_m: float,
    board_rvec: tuple[float, float, float],
    board_tvec_m: tuple[float, float, float],
) -> tuple[np.ndarray, np.ndarray]:
    """Project a 3D chessboard pose into Camera 1 and Camera 2 pixel coordinates (Nx1x2 float32)."""
    local_pts = chessboard_object_points(cols, rows, square_size_m).astype(np.float64)
    r_board, _ = cv2.Rodrigues(np.array(board_rvec, dtype=np.float64))
    t_board = np.array(board_tvec_m, dtype=np.float64).reshape(1, 3)
    world_pts = (local_pts @ r_board.T) + t_board

    pts_cam1 = project_points_3d(world_pts, calibration.camera_01)
    pts_cam2 = project_points_3d(world_pts, calibration.camera_02, calibration.R, calibration.T)
    return pts_cam1.astype(np.float32).reshape(-1, 1, 2), pts_cam2.astype(np.float32).reshape(-1, 1, 2)


def render_synthetic_chessboard_image(
    intrinsics: CameraIntrinsics,
    cols: int,
    rows: int,
    square_size_m: float,
    board_rvec: tuple[float, float, float],
    board_tvec_m: tuple[float, float, float],
    cam_rotation: np.ndarray | None = None,
    cam_translation: np.ndarray | None = None,
) -> np.ndarray:
    """Render a synthetic image of a 3D chessboard so cv2.findChessboardCorners can be tested."""
    width, height = intrinsics.image_size
    image = np.full((height, width, 3), 200, dtype=np.uint8)
    r_board, _ = cv2.Rodrigues(np.array(board_rvec, dtype=np.float64))
    t_board = np.array(board_tvec_m, dtype=np.float64).reshape(1, 3)

    # A chessboard with (cols, rows) inner corners has (cols + 1, rows + 1) squares.
    num_sq_x, num_sq_y = cols + 1, rows + 1
    # White border around the board
    border_pts = np.array([
        [-1.0, -1.0, 0.0],
        [num_sq_x, -1.0, 0.0],
        [num_sq_x, num_sq_y, 0.0],
        [-1.0, num_sq_y, 0.0],
    ], dtype=np.float64) * float(square_size_m)
    world_border = (border_pts @ r_board.T) + t_board
    proj_border = project_points_3d(world_border, intrinsics, cam_rotation, cam_translation)
    cv2.fillConvexPoly(image, np.round(proj_border).astype(np.int32), (255, 255, 255), lineType=cv2.LINE_AA)

    for r in range(num_sq_y):
        for c in range(num_sq_x):
            local_quad = np.array([
                [c - 1, r - 1, 0.0],
                [c, r - 1, 0.0],
                [c, r, 0.0],
                [c - 1, r, 0.0],
            ], dtype=np.float64) * float(square_size_m)
            world_quad = (local_quad @ r_board.T) + t_board
            proj_quad = project_points_3d(world_quad, intrinsics, cam_rotation, cam_translation)
            color = (20, 20, 20) if (r + c) % 2 == 0 else (245, 245, 245)
            cv2.fillConvexPoly(image, np.round(proj_quad).astype(np.int32), color, lineType=cv2.LINE_AA)
    return image
