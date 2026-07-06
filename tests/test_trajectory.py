"""Physics invariants of the trajectory integrator."""
import math
import numpy as np
import pytest
from scipy.integrate import solve_ivp

from aeros.trajectory import (MU_EARTH, R_EARTH, _orbital_elements,
                              simulate_ascent)
from aeros.validate import falcon9_expendable


def test_orbital_elements_circular():
    r = R_EARTH + 400_000
    v = math.sqrt(MU_EARTH / r)
    per, apo = _orbital_elements(r, 0.0, v)
    assert per == pytest.approx(400_000, abs=1.0)
    assert apo == pytest.approx(400_000, abs=1.0)


def test_mass_conservation_during_flight():
    """No phantom propellant: total burned mass never exceeds usable load."""
    v = falcon9_expendable(20_000)
    res = simulate_ascent(v, 200_000, 28.5, kick_speed_m_s=89.36,
                          s1_pitch_final_deg=27.37, s2_tan0=0.959,
                          upper_tan_final=-0.076)
    burned = 0.0
    for p in res.phases:
        if len(p.t) >= 2:
            dm = p.m[0] - p.m[-1]
            assert dm >= -1e-6
            burned += max(dm, 0.0)
    usable = sum(s.usable_propellant_kg for s in v.stages)
    assert burned <= usable + 1.0


def test_flies_rated_payload():
    """The simulator must fly a Falcon 9-class payload to orbit with a
    physically plausible max-q. (Full validation: python -m aeros.validate)"""
    v = falcon9_expendable(20_000)
    res = simulate_ascent(v, 200_000, 28.5, kick_speed_m_s=89.36,
                          s1_pitch_final_deg=27.37, s2_tan0=0.959,
                          upper_tan_final=-0.076)
    assert res.reached_orbit
    assert res.max_q_Pa < 60_000
