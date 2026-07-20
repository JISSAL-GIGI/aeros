"""Uncertainty-aware performance analysis and chance-constrained design.

Deterministic conceptual design hides the fact that every input carries
uncertainty: stage dry masses are estimates (+/-10-15% at concept level),
engine Isp varies unit-to-unit, drag models carry error, and residual
propellant is stochastic. This module propagates those uncertainties
through the flight-validated trajectory simulator by Monte Carlo:

  * `orbit_probability`    -- P(orbit | payload) for one vehicle
  * `payload_confidence`   -- payload capability at a stated confidence
  * `chance_constrained_design` -- resize a design until it reaches orbit
                                   with (e.g.) 95% probability

Sampling model (independent, applied multiplicatively):
  dry mass   ~ N(1, sigma_dry)     default sigma 7%  (AIAA concept-level MGA)
  Isp        ~ N(1, sigma_isp)     default sigma 0.7%
  drag       ~ N(1, sigma_cd)      default sigma 10%
  residuals  ~ U(0.3%, 1.0%) of propellant

Steering is optimised once on the nominal vehicle and then held fixed for
all samples -- a real vehicle's guidance adapts in flight, so holding the
pitch program fixed is slightly conservative, which is the safe direction
for a design tool.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from .vehicle import Stage, Vehicle
from . import aero as aero_mod
from .trajectory import optimize_steering, simulate_ascent


@dataclass(frozen=True)
class UncertaintyModel:
    sigma_dry: float = 0.07
    sigma_isp: float = 0.007
    sigma_cd: float = 0.10
    residual_lo: float = 0.003
    residual_hi: float = 0.010
    seed: int = 7


def _perturb_vehicle(vehicle: Vehicle, u: UncertaintyModel,
                     rng: np.random.Generator) -> tuple[Vehicle, float]:
    """Return a perturbed copy of the vehicle and a drag multiplier."""
    stages = []
    for s in vehicle.stages:
        f_dry = max(0.5, rng.normal(1.0, u.sigma_dry))
        f_isp = max(0.9, rng.normal(1.0, u.sigma_isp))
        resid = rng.uniform(u.residual_lo, u.residual_hi)
        eng = replace(s.engine,
                      isp_vac_s=s.engine.isp_vac_s * f_isp,
                      thrust_vac_N=s.engine.thrust_vac_N * f_isp)
        stages.append(Stage(s.name, s.dry_mass_kg * f_dry, s.propellant_kg,
                            eng, s.n_engines, s.diameter_m,
                            unusable_prop_fraction=resid))
    v = Vehicle(vehicle.name, stages, vehicle.fairing_mass_kg,
                vehicle.fairing_jettison_alt_m, vehicle.payload_kg)
    f_cd = max(0.5, rng.normal(1.0, u.sigma_cd))
    return v, f_cd


class _ScaledDrag:
    """Context manager that scales the global drag model."""
    def __init__(self, factor: float):
        self.factor = factor
        self._orig = aero_mod.drag_coefficient

    def __enter__(self):
        orig = self._orig
        f = self.factor
        # patch both the aero module and the trajectory module's reference
        from . import trajectory as tj
        self._tj_orig = tj.drag_coefficient
        aero_mod.drag_coefficient = lambda m: orig(m) * f
        tj.drag_coefficient = aero_mod.drag_coefficient
        return self

    def __exit__(self, *exc):
        from . import trajectory as tj
        aero_mod.drag_coefficient = self._orig
        tj.drag_coefficient = self._tj_orig
        return False


def orbit_probability(
    vehicle: Vehicle,
    target_altitude_m: float,
    launch_latitude_deg: float,
    n_samples: int = 100,
    model: UncertaintyModel = UncertaintyModel(),
    steering_params: tuple | None = None,
) -> dict:
    """Monte Carlo probability that the vehicle reaches orbit.

    Steering is optimised on the nominal vehicle (or supplied) and held
    fixed across samples. Returns probability and perigee statistics.
    """
    if steering_params is None:
        steering_params, _ = optimize_steering(
            vehicle, target_altitude_m, launch_latitude_deg)
    ks, pf, t0u, uptf = steering_params

    rng = np.random.default_rng(model.seed)
    successes = 0
    perigees = []
    for _ in range(n_samples):
        v, f_cd = _perturb_vehicle(vehicle, model, rng)
        try:
            with _ScaledDrag(f_cd):
                res = simulate_ascent(
                    v, target_altitude_m, launch_latitude_deg,
                    kick_speed_m_s=ks, s1_pitch_final_deg=pf,
                    s2_tan0=t0u, upper_tan_final=uptf)
            ok = res.reached_orbit
            per = res.perigee_m
        except Exception:
            ok, per = False, -1e6
        successes += ok
        if per < 1e11:
            perigees.append(per)

    perigees = np.array(perigees)
    return {
        "p_orbit": successes / n_samples,
        "n_samples": n_samples,
        "perigee_km_p5": float(np.percentile(perigees, 5) / 1000)
        if len(perigees) else None,
        "perigee_km_p50": float(np.percentile(perigees, 50) / 1000)
        if len(perigees) else None,
        "steering_params": tuple(float(x) for x in steering_params),
    }


def payload_confidence(
    vehicle_factory,
    target_altitude_m: float,
    launch_latitude_deg: float,
    payloads_kg: list[float],
    confidence: float = 0.95,
    n_samples: int = 60,
    model: UncertaintyModel = UncertaintyModel(),
    verbose: bool = False,
) -> dict:
    """P(orbit) across a payload sweep; interpolate payload at confidence.

    Returns the curve and the largest payload whose orbit probability
    meets the requested confidence.
    """
    curve = []
    for pl in payloads_kg:
        v = vehicle_factory(pl)
        r = orbit_probability(v, target_altitude_m, launch_latitude_deg,
                              n_samples=n_samples, model=model)
        curve.append({"payload_kg": pl, **r})
        if verbose:
            print(f"  payload {pl/1000:7.2f} t -> P(orbit) = {r['p_orbit']:.2f}")

    # largest payload meeting the confidence requirement (linear interp)
    pl_conf = None
    for a, b in zip(curve, curve[1:]):
        pa, pb = a["p_orbit"], b["p_orbit"]
        if pa >= confidence >= pb:
            f = (pa - confidence) / max(pa - pb, 1e-9)
            pl_conf = a["payload_kg"] + f * (b["payload_kg"] - a["payload_kg"])
            break
    if pl_conf is None and curve and curve[0]["p_orbit"] >= confidence:
        pl_conf = curve[-1]["payload_kg"] if curve[-1]["p_orbit"] >= confidence \
            else curve[0]["payload_kg"]

    return {"curve": curve, "confidence": confidence,
            "payload_at_confidence_kg": pl_conf}


def chance_constrained_design(
    mission,
    confidence: float = 0.95,
    n_samples: int = 60,
    model: UncertaintyModel = UncertaintyModel(),
    max_margin_iters: int = 4,
    verbose: bool = True,
):
    """Design a vehicle that reaches orbit with at least `confidence`
    probability under input uncertainty.

    Wraps the deterministic designer; if the Monte Carlo check fails,
    the delta-v budget is inflated 4% per iteration and the vehicle
    resized until the chance constraint holds.
    """
    from .design import (DesignDecision, design_vehicle, dv_required,
                         size_two_stage)

    result = design_vehicle(mission, verbose=verbose)
    booster = result.vehicle.stages[0].engine
    upper = result.vehicle.stages[1].engine
    dia = result.vehicle.stages[0].diameter_m
    split = None
    for d in result.decisions:
        if d.subject == "staging":
            split = float(d.choice.split("%")[0]) / 100.0
    dv = dv_required(mission.target_altitude_m, mission.launch_latitude_deg)

    for it in range(max_margin_iters + 1):
        mc = orbit_probability(result.vehicle, mission.target_altitude_m,
                               mission.launch_latitude_deg,
                               n_samples=n_samples, model=model)
        if verbose:
            print(f"[UQ] iteration {it}: P(orbit) = {mc['p_orbit']:.2f} "
                  f"(target {confidence:.2f})")
        if mc["p_orbit"] >= confidence:
            result.decisions.append(DesignDecision(
                "chance constraint",
                f"P(orbit) = {mc['p_orbit']:.2f} >= {confidence:.2f} "
                f"({n_samples} Monte Carlo samples)",
                "dry mass +/-7%, Isp +/-0.7%, drag +/-10%, residuals "
                "0.3-1.0% propagated through the flight-validated simulator; "
                "steering held fixed (conservative)"))
            result.uq = mc
            return result
        if split is None or it == max_margin_iters:
            break
        dv *= 1.04
        v2 = size_two_stage(mission.payload_kg, dv, split, booster, upper, dia)
        if v2 is None:
            break
        from .trajectory import optimize_steering as _opt
        params, res = _opt(v2, mission.target_altitude_m,
                           mission.launch_latitude_deg)
        result.vehicle, result.ascent = v2, res
        result.verified = res is not None and res.reached_orbit
        result.decisions.append(DesignDecision(
            "margin iteration (UQ)", f"delta-v budget +4% (iter {it + 1})",
            "Monte Carlo orbit probability below required confidence; "
            "vehicle resized with additional performance margin"))

    result.uq = mc
    return result
