"""Mass bookkeeping and rocket-equation invariants."""
import math
import pytest
from aeros.validate import falcon9_expendable
from aeros.engines import G0


def test_glow_is_sum_of_parts():
    v = falcon9_expendable(10_000)
    parts = sum(s.wet_mass_kg for s in v.stages) + v.fairing_mass_kg + 10_000
    assert v.glow_kg == pytest.approx(parts)


def test_ideal_dv_matches_hand_calculation():
    v = falcon9_expendable(22_800)
    s1, s2 = v.stages
    above1 = s2.wet_mass_kg + v.fairing_mass_kg + 22_800
    m0 = s1.wet_mass_kg + above1
    m1 = m0 - s1.usable_propellant_kg
    dv1 = s1.engine.isp_vac_s * G0 * math.log(m0 / m1)
    above2 = v.fairing_mass_kg + 22_800
    m0 = s2.wet_mass_kg + above2
    m1 = m0 - s2.usable_propellant_kg
    dv2 = s2.engine.isp_vac_s * G0 * math.log(m0 / m1)
    assert v.ideal_delta_v() == pytest.approx(dv1 + dv2, rel=1e-9)
