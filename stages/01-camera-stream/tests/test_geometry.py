import math

import pytest

from drone_camera.geometry import GeometryError, bounding_circle


def test_center_and_radius_from_box():
    circle = bounding_circle(10, 20, 40, 60)
    assert (circle.cx, circle.cy) == (25, 40)
    assert circle.base_radius == pytest.approx(25.0)           # sqrt(30^2 + 40^2) / 2
    assert circle.safe_radius == pytest.approx(30.0)           # default factor 1.2
    assert circle.safety_factor == 1.2


@pytest.mark.parametrize("factor", [1.0, 1.5, 2.0, 3.0])
def test_safety_factor_scales_radius_only(factor):
    base = bounding_circle(0, 0, 6, 8, 1.0)
    circle = bounding_circle(0, 0, 6, 8, factor)
    assert circle.base_radius == base.base_radius == 5.0
    assert circle.safe_radius == pytest.approx(5.0 * factor)
    assert (circle.cx, circle.cy) == (3, 4)


def test_float_and_partially_off_screen_boxes():
    circle = bounding_circle(-5.5, -2.5, 4.5, 7.5)
    assert (circle.cx, circle.cy) == (-0.5, 2.5)
    assert circle.base_radius == pytest.approx(math.hypot(10, 10) / 2)


@pytest.mark.parametrize("box", [(5, 5, 5, 9), (5, 5, 9, 5), (9, 5, 5, 9), (5, 9, 9, 5),
                                 (math.nan, 0, 1, 1), (0, 0, math.inf, 1), (0, 0, 1, None),
                                 (True, 0, 5, 5)])
def test_invalid_boxes_are_rejected(box):
    with pytest.raises(GeometryError):
        bounding_circle(*box)


@pytest.mark.parametrize("factor", [0.99, 3.01, -1, math.nan, math.inf, None, True, "1.2"])
def test_invalid_safety_factors_are_rejected(factor):
    with pytest.raises(GeometryError):
        bounding_circle(0, 0, 10, 10, factor)
