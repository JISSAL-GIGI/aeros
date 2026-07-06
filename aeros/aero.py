"""Aerodynamic drag model for slender launch vehicles.

Mach-dependent drag coefficient for a typical slender body-of-revolution
launcher, assembled from the classic transonic drag-rise shape used in
conceptual launch vehicle design (e.g. Fleeman, Sutton & Biblarz).
Ascent drag losses are a small fraction of total delta-v (~50-150 m/s),
so a curve of this fidelity contributes <1% error to payload predictions.
"""

from __future__ import annotations

# (Mach, Cd) knots for a slender launcher, power-on ascent
_CD_TABLE = [
    (0.0, 0.25),
    (0.8, 0.25),
    (1.05, 0.55),
    (1.25, 0.60),
    (2.0, 0.45),
    (3.0, 0.34),
    (5.0, 0.26),
    (8.0, 0.23),
    (25.0, 0.22),
]


def drag_coefficient(mach: float) -> float:
    """Piecewise-linear Cd(M) for a slender launch vehicle."""
    if mach <= _CD_TABLE[0][0]:
        return _CD_TABLE[0][1]
    for (m0, c0), (m1, c1) in zip(_CD_TABLE, _CD_TABLE[1:]):
        if mach <= m1:
            f = (mach - m0) / (m1 - m0)
            return c0 + f * (c1 - c0)
    return _CD_TABLE[-1][1]
