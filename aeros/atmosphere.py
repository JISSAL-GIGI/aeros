"""US Standard Atmosphere 1976.

Implements the seven-layer temperature-gradient model to 86 km geopotential
altitude, with an exponential extrapolation above. Validated against the
published USSA-1976 tables (see tests/test_atmosphere.py).

Reference: NOAA/NASA/USAF, "U.S. Standard Atmosphere, 1976", NASA-TM-X-74335.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Physical constants (USSA-1976 values)
G0 = 9.80665          # m/s^2, standard gravity
R_AIR = 287.053       # J/(kg K), specific gas constant for air
GAMMA = 1.4           # ratio of specific heats
R_EARTH = 6_356_766.0  # m, effective Earth radius used by USSA-1976

# Layer base geopotential altitude [m], base temperature [K], lapse rate [K/m]
_LAYERS = [
    (0.0,     288.15, -0.0065),
    (11_000.0, 216.65,  0.0),
    (20_000.0, 216.65,  0.0010),
    (32_000.0, 228.65,  0.0028),
    (47_000.0, 270.65,  0.0),
    (51_000.0, 270.65, -0.0028),
    (71_000.0, 214.65, -0.0020),
    (86_000.0, 186.87,  0.0),
]

_P0 = 101_325.0  # Pa, sea-level pressure


def _layer_base_pressures():
    """Pre-compute pressure at the base of each layer."""
    pressures = [_P0]
    for i in range(1, len(_LAYERS)):
        h_b, t_b, lam = _LAYERS[i - 1]
        h_top = _LAYERS[i][0]
        p_b = pressures[-1]
        if abs(lam) < 1e-12:
            p = p_b * math.exp(-G0 * (h_top - h_b) / (R_AIR * t_b))
        else:
            t_top = t_b + lam * (h_top - h_b)
            p = p_b * (t_top / t_b) ** (-G0 / (lam * R_AIR))
        pressures.append(p)
    return pressures


_BASE_P = _layer_base_pressures()


@dataclass(frozen=True)
class AtmoState:
    """Atmospheric state at a given altitude."""
    altitude_m: float
    temperature_K: float
    pressure_Pa: float
    density_kg_m3: float
    speed_of_sound_m_s: float


def geopotential(z: float) -> float:
    """Convert geometric altitude [m] to geopotential altitude [m]."""
    return R_EARTH * z / (R_EARTH + z)


def atmosphere(altitude_m: float) -> AtmoState:
    """Atmospheric state at geometric altitude [m] above mean sea level."""
    z = max(altitude_m, 0.0)
    h = geopotential(z)

    if h >= 86_000.0:
        # Exponential decay above the tabulated region; adequate for launch
        # ascent work where q ~ 0 above 80 km.
        t = 186.87
        p = _BASE_P[-1] * math.exp(-G0 * (h - 86_000.0) / (R_AIR * t))
    else:
        idx = 0
        for i in range(len(_LAYERS) - 1, -1, -1):
            if h >= _LAYERS[i][0]:
                idx = i
                break
        h_b, t_b, lam = _LAYERS[idx]
        p_b = _BASE_P[idx]
        t = t_b + lam * (h - h_b)
        if abs(lam) < 1e-12:
            p = p_b * math.exp(-G0 * (h - h_b) / (R_AIR * t_b))
        else:
            p = p_b * (t / t_b) ** (-G0 / (lam * R_AIR))

    rho = p / (R_AIR * t)
    a = math.sqrt(GAMMA * R_AIR * t)
    return AtmoState(z, t, p, rho, a)
