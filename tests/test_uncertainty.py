"""UQ module invariants."""
import numpy as np
from aeros.validate import falcon9_expendable
from aeros.uncertainty import (UncertaintyModel, _perturb_vehicle,
                               orbit_probability)
from aeros import aero as aero_mod


def test_perturbation_reproducible_and_bounded():
    v = falcon9_expendable(20000)
    u = UncertaintyModel(seed=3)
    rng1 = np.random.default_rng(3)
    rng2 = np.random.default_rng(3)
    v1, f1 = _perturb_vehicle(v, u, rng1)
    v2, f2 = _perturb_vehicle(v, u, rng2)
    assert f1 == f2
    assert v1.stages[0].dry_mass_kg == v2.stages[0].dry_mass_kg
    # propellant load must be untouched; dry mass within plausible band
    assert v1.stages[0].propellant_kg == v.stages[0].propellant_kg
    assert 0.6 * v.stages[0].dry_mass_kg < v1.stages[0].dry_mass_kg \
        < 1.4 * v.stages[0].dry_mass_kg


def test_orbit_probability_with_known_steering():
    r = orbit_probability(
        falcon9_expendable(20000), 200_000, 28.5, n_samples=12,
        steering_params=(89.36, 27.37, 0.959, -0.076))
    assert 0.0 <= r["p_orbit"] <= 1.0
    assert r["p_orbit"] >= 0.5     # 20 t on a 22.8 t-rated vehicle
    # the drag patch must be restored after the run
    assert abs(aero_mod.drag_coefficient(1.05) - 0.55) < 1e-9
