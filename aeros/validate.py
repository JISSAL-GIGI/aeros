"""Validation against real launch vehicles.

Each reference vehicle is built from published stage masses and engine data,
then flown by the same trajectory simulator used for new designs. Predicted
payload to LEO is compared against the operator's published figure.

This is the credibility test of the whole platform: if the physics engine
reproduces vehicles that actually flew, its predictions for new designs
carry weight.
"""

from __future__ import annotations

from dataclasses import dataclass

from .engines import get_engine
from .vehicle import Stage, Vehicle
from .trajectory import payload_capacity


@dataclass
class ReferenceVehicle:
    name: str
    factory: callable            # payload_kg -> Vehicle
    published_payload_kg: float
    target_altitude_m: float
    launch_latitude_deg: float
    payload_hi_kg: float
    source: str


def falcon9_expendable(payload_kg: float) -> Vehicle:
    """Falcon 9 Block 5, fully expendable.

    Stage masses: SpaceX published / spaceflight101 & NASA launch-vehicle
    data sheets. S1 propellant 411.0 t, S1 dry 22.2 t (no legs/grid fins),
    S2 propellant 107.5 t, S2 dry 4.0 t, fairing 1.9 t.
    """
    return Vehicle(
        name="Falcon 9 Block 5 (expendable)",
        stages=[
            Stage("Stage 1", 22_200, 411_000, get_engine("Merlin 1D"), 9, 3.7),
            Stage("Stage 2", 4_000, 107_500, get_engine("Merlin 1D Vacuum"), 1, 3.7),
        ],
        fairing_mass_kg=1_900,
        payload_kg=payload_kg,
    )


def electron(payload_kg: float) -> Vehicle:
    """Rocket Lab Electron.

    Stage masses from the Electron Payload User's Guide and spaceflight101:
    S1 dry 0.95 t / prop 9.25 t, S2 dry 0.25 t / prop 2.15 t, fairing 44 kg.
    """
    return Vehicle(
        name="Electron",
        stages=[
            Stage("Stage 1", 950, 9_250, get_engine("Rutherford"), 9, 1.2),
            Stage("Stage 2", 250, 2_150, get_engine("Rutherford Vacuum"), 1, 1.2),
        ],
        fairing_mass_kg=44,
        fairing_jettison_alt_m=120_000,
        payload_kg=payload_kg,
    )


def saturn_v_skylab(payload_kg: float) -> Vehicle:
    """Saturn V, two-stage Skylab configuration (SA-513, 14 May 1973).

    The cleanest Saturn V payload benchmark: both stages burned to
    depletion and the payload (the Skylab station, ~77 t) is a real,
    fully-accounted mass in a documented 435 km / 50 deg orbit.
    Stage masses from NASA SP-4206 / braeunig.us Saturn data:
    S-IC 2,286 t gross / 135.2 t empty; S-II 480 t gross / 36.2 t empty.
    (The classic "140 t to LEO" figure is not used here because it counts
    the partially-fuelled S-IVB stage as payload.)
    """
    return Vehicle(
        name="Saturn V (Skylab, 2-stage)",
        stages=[
            Stage("S-IC", 135_200, 2_151_000, get_engine("F-1"), 5, 10.1),
            Stage("S-II", 36_200, 443_800, get_engine("J-2"), 5, 10.1),
        ],
        fairing_mass_kg=0,
        payload_kg=payload_kg,
    )


REFERENCE_VEHICLES = [
    ReferenceVehicle(
        "Falcon 9 Block 5 (expendable)", falcon9_expendable,
        published_payload_kg=22_800, target_altitude_m=200_000,
        launch_latitude_deg=28.5, payload_hi_kg=40_000,
        source="SpaceX capabilities page: 22,800 kg to LEO, expendable"),
    ReferenceVehicle(
        "Electron", electron,
        published_payload_kg=300, target_altitude_m=200_000,
        launch_latitude_deg=39.3, payload_hi_kg=1_000,
        source="Rocket Lab: 300 kg max payload to LEO (Mahia, 39.3 S)"),
    ReferenceVehicle(
        # launch_latitude_deg=50 models the reduced Earth-rotation credit of
        # the 50-degree-inclination Skylab launch azimuth from KSC
        "Saturn V (Skylab, 2-stage)", saturn_v_skylab,
        published_payload_kg=77_100, target_altitude_m=435_000,
        launch_latitude_deg=50.0, payload_hi_kg=150_000,
        source="NASA: Skylab (77.1 t) to 435 km, 50 deg, SA-513"),
]


def run_validation(verbose: bool = True):
    """Predict payload for every reference vehicle; return result table."""
    rows = []
    for ref in REFERENCE_VEHICLES:
        if verbose:
            print(f"\n=== {ref.name} ===")
            print(ref.factory(0).describe())
        predicted, res = payload_capacity(
            ref.factory,
            target_altitude_m=ref.target_altitude_m,
            launch_latitude_deg=abs(ref.launch_latitude_deg),
            payload_hi=ref.payload_hi_kg,
            verbose=verbose,
        )
        err = (predicted - ref.published_payload_kg) / ref.published_payload_kg
        rows.append({
            "vehicle": ref.name,
            "published_kg": ref.published_payload_kg,
            "predicted_kg": round(predicted),
            "error_pct": round(100 * err, 1),
            "max_q_kPa": round(res.max_q_Pa / 1000, 1) if res else None,
            "max_accel_g": round(res.max_accel_g, 2) if res else None,
            "source": ref.source,
        })
        if verbose:
            print(f"  published {ref.published_payload_kg/1000:.1f} t | "
                  f"predicted {predicted/1000:.1f} t | error {100*err:+.1f}%")
    return rows


if __name__ == "__main__":
    table = run_validation()
    print("\n| Vehicle | Published | Predicted | Error |")
    print("|---|---|---|---|")
    for r in table:
        print(f"| {r['vehicle']} | {r['published_kg']/1000:.1f} t "
              f"| {r['predicted_kg']/1000:.1f} t | {r['error_pct']:+.1f}% |")
