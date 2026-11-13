"""Synthetic two-drone scenes built from real labelled drone crops.

Needs a folder with ``*.jpg`` images and same-named YOLO ``*.txt`` labels (path in
``DRONE_YOLO_SAMPLES``). Nothing from the dataset is copied into the repository.
Results on these scenes only prove the pipeline works; they are NOT a measure of
real-flight detection quality (the crops come from the training set).
"""

from pathlib import Path

import cv2
import numpy as np


def load_crops(folder: Path, count: int = 4, margin: float = 0.25):
    crops = []
    for image_path in sorted(folder.glob("*.jpg")):
        label_path = image_path.with_suffix(".txt")
        if not label_path.is_file():
            continue
        parts = label_path.read_text().split()
        if len(parts) < 5:
            continue
        image = cv2.imread(str(image_path))
        height, width = image.shape[:2]
        cx, cy, w, h = (float(v) for v in parts[1:5])
        x1 = max(0, int((cx - w / 2 * (1 + margin)) * width))
        x2 = min(width, int((cx + w / 2 * (1 + margin)) * width))
        y1 = max(0, int((cy - h / 2 * (1 + margin)) * height))
        y2 = min(height, int((cy + h / 2 * (1 + margin)) * height))
        crop = image[y1:y2, x1:x2]
        if crop.shape[0] >= 8 and crop.shape[1] >= 8:
            crops.append(crop)
        if len(crops) == count:
            break
    return crops


def background(width, height):
    sky = np.zeros((height, width, 3), np.uint8)
    for row in range(height):
        sky[row, :] = (235 - row * 40 // height, 200 - row * 30 // height, 150)
    return sky


def paste(scene, crop, center, target_width):
    """Paste a crop with a feathered elliptical mask so no rectangle edge remains."""
    scale = target_width / crop.shape[1]
    resized = cv2.resize(crop, (target_width, max(8, int(crop.shape[0] * scale))))
    h, w = resized.shape[:2]
    mask = np.zeros((h, w), np.float32)
    cv2.ellipse(mask, (w // 2, h // 2), (w // 2 - 1, h // 2 - 1), 0, 0, 360, 1.0, -1)
    mask = cv2.GaussianBlur(mask, (0, 0), 2)[..., None]
    x = int(center[0] - w / 2)
    y = int(center[1] - h / 2)
    x1, y1 = max(0, x), max(0, y)
    x2, y2 = min(scene.shape[1], x + w), min(scene.shape[0], y + h)
    if x2 <= x1 or y2 <= y1:
        return
    patch = resized[y1 - y:y2 - y, x1 - x:x2 - x]
    alpha = mask[y1 - y:y2 - y, x1 - x:x2 - x]
    region = scene[y1:y2, x1:x2].astype(np.float32)
    scene[y1:y2, x1:x2] = (patch * alpha + region * (1 - alpha)).astype(np.uint8)


def render_frames(crops, frames=60, size=(1280, 720), hidden=None, width=150, overlap=True):
    """Two drones on opposite horizontal paths. ``overlap=True`` makes them cross at the same
    height (heavy overlap, IDs may legitimately switch); ``overlap=False`` keeps them apart.
    ``hidden`` = {drone index: range of frames} removes a drone for those frames."""
    w, h = size
    base = background(w, h)
    hidden = hidden or {}
    out = []
    for f in range(frames):
        t = f / (frames - 1)
        scene = base.copy()
        if overlap:
            paths = [((0.2 + 0.6 * t) * w, (0.30 + 0.30 * t) * h),   # drone 0: left -> right, downward
                     ((0.8 - 0.6 * t) * w, (0.62 - 0.30 * t) * h)]   # drone 1: right -> left, upward
        else:
            paths = [((0.2 + 0.6 * t) * w, (0.25 + 0.08 * t) * h),
                     ((0.8 - 0.6 * t) * w, (0.72 - 0.08 * t) * h)]
        for index, center in enumerate(paths):
            if f in hidden.get(index, ()):
                continue
            paste(scene, crops[index % len(crops)], center, width)
        out.append((scene, paths))
    return out
