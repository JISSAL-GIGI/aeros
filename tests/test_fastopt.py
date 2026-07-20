"""Warm-start refinement must fly a known-good configuration."""
from aeros.validate import falcon9_expendable
from aeros.fastopt import refine_steering


def test_refine_from_good_start_reaches_orbit():
    params, res = refine_steering(
        falcon9_expendable(20000), 200_000, 28.5,
        (89.36, 27.37, 0.959, -0.076))
    assert res is not None and res.reached_orbit
