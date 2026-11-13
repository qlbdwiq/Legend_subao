"""Bounding-circle geometry for detected drones. Pure Python: no Qt, torch or numpy.

The circle is the circumscribed circle of the detector's bounding box (radius =
half the box diagonal) enlarged by an experimental ``safety_factor``. All values
are image pixels. The factor compensates for detector error (for example
propellers outside the box); it does NOT guarantee flight safety.
"""

from dataclasses import dataclass
import math

DEFAULT_SAFETY_FACTOR = 1.2
MIN_SAFETY_FACTOR = 1.0
MAX_SAFETY_FACTOR = 3.0


class GeometryError(ValueError):
    pass


@dataclass(frozen=True)
class BoundingCircle:
    cx: float
    cy: float
    base_radius: float  # Half of the box diagonal, in pixels.
    safe_radius: float  # base_radius * safety_factor, in pixels.
    safety_factor: float


def validate_safety_factor(value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
            or not MIN_SAFETY_FACTOR <= value <= MAX_SAFETY_FACTOR:
        raise GeometryError(
            f"Safety factor must be a finite number from {MIN_SAFETY_FACTOR} to {MAX_SAFETY_FACTOR}.")
    return float(value)


def bounding_circle(x1: float, y1: float, x2: float, y2: float,
                    safety_factor: float = DEFAULT_SAFETY_FACTOR) -> BoundingCircle:
    """Circle around an (x1, y1, x2, y2) pixel box with ``x2 > x1`` and ``y2 > y1``."""
    factor = validate_safety_factor(safety_factor)
    values = (x1, y1, x2, y2)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise GeometryError("Box coordinates must be finite numbers.")
    width = x2 - x1
    height = y2 - y1
    if width <= 0 or height <= 0:
        raise GeometryError("Box must have positive width and height.")
    radius = math.hypot(width, height) / 2
    return BoundingCircle((x1 + x2) / 2, (y1 + y2) / 2, radius, radius * factor, factor)
