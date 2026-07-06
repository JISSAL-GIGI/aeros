"""Structural buildup must reproduce flown stage dry masses within 15%."""
import pytest
from aeros.structures import size_stage_structure
from aeros.materials import PROPELLANTS, MATERIALS
from aeros.engines import get_engine

CASES = [
    ("Falcon 9 S1", 411_000, "Merlin 1D", 9, 3.7, "Al-Li-2195", True, 22_200),
    ("Falcon 9 S2", 107_500, "Merlin 1D Vacuum", 1, 3.7, "Al-Li-2195", False, 4_000),
    ("Electron S1", 9_250, "Rutherford", 9, 1.2, "CFRP-IM7", True, 950),
    ("Electron S2", 2_150, "Rutherford Vacuum", 1, 1.2, "CFRP-IM7", False, 250),
]


@pytest.mark.parametrize("name,prop,eng,n,dia,mat,booster,actual", CASES)
def test_flown_stage_masses(name, prop, eng, n, dia, mat, booster, actual):
    s = size_stage_structure(prop, PROPELLANTS["LOX/RP-1"], get_engine(eng),
                             n, dia, MATERIALS[mat], is_booster=booster)
    assert s.dry_mass_kg == pytest.approx(actual, rel=0.15), name
