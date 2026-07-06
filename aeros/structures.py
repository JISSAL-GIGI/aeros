"""Stage structural sizing from first principles plus empirical closure.

Tank shells are sized as thin-walled pressure vessels (hoop stress with a
1.5 design factor on yield), engines come from the engine database, and
secondary structure uses correlations from Humble, Henry & Larson,
"Space Propulsion Analysis and Design". The model is calibrated to
reproduce the stage mass fractions of flown kerolox vehicles
(Falcon 9 S1 ~5.4% dry/prop, S2 ~3.7%) - see tests/test_structures.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .engines import Engine
from .materials import Material, PropellantCombo

DESIGN_PRESSURE_PA = 3.5e5     # typical ullage pressure for pump-fed stages
SAFETY_FACTOR = 1.5
MIN_WALL_M = 0.0018            # manufacturing minimum gauge
ULLAGE_FRACTION = 0.05         # tank volume above liquid
SECONDARY_MASS_FACTOR = 0.35   # slosh baffles, feedlines, pressurisation, wiring
THRUST_STRUCTURE_KG_PER_N = 4.0e-4  # Humble et al. correlation
# Mass growth factor calibrated so the buildup reproduces flown stage masses
# (Falcon 9 S1/S2, Electron) within ~+/-15% -- standard MER practice at the
# conceptual level. See tests/test_structures.py.
GROWTH_FACTOR = 1.65
INTERSTAGE_KG_PER_M2 = 110.0   # interstage mass ~ 110 * D^2 (boosters)


@dataclass
class StageStructure:
    tank_mass_kg: float
    engine_mass_kg: float
    thrust_structure_kg: float
    secondary_mass_kg: float
    avionics_mass_kg: float
    tank_length_m: float

    @property
    def dry_mass_kg(self) -> float:
        return (self.tank_mass_kg + self.engine_mass_kg
                + self.thrust_structure_kg + self.secondary_mass_kg
                + self.avionics_mass_kg)


def size_stage_structure(
    propellant_kg: float,
    propellant: PropellantCombo,
    engine: Engine,
    n_engines: int,
    diameter_m: float,
    material: Material,
    is_booster: bool = False,
) -> StageStructure:
    """Physical dry-mass buildup for one stage."""

    # --- tank geometry: cylinder with two 2:1 ellipsoidal domes
    volume = propellant_kg / propellant.bulk_density * (1 + ULLAGE_FRACTION)
    r = diameter_m / 2
    dome_vol = (4 / 3) * math.pi * r ** 3 / 2          # two 2:1 domes = 1 sphere/2...
    # two 2:1 ellipsoidal domes together enclose (4/3) pi r^3 / 2
    cyl_vol = max(volume - dome_vol, 0.1)
    cyl_len = cyl_vol / (math.pi * r ** 2)

    # --- wall thickness from hoop stress at design pressure
    sigma_allow = material.yield_strength_Pa / SAFETY_FACTOR
    t_wall = max(DESIGN_PRESSURE_PA * r / sigma_allow, MIN_WALL_M)

    # shell area: cylinder + two domes (~1.38 * pi r^2 each for 2:1 ellipsoid)
    area = math.pi * diameter_m * cyl_len + 2 * 1.38 * math.pi * r ** 2
    # common bulkhead + anti-slosh + weld lands: knockdown on ideal shell
    tank_mass = area * t_wall * material.density_kg_m3 * 1.25
    tank_mass *= (1 + SECONDARY_MASS_FACTOR)

    engine_mass = engine.mass_kg * n_engines
    thrust_structure = THRUST_STRUCTURE_KG_PER_N * engine.thrust_vac_N * n_engines
    secondary = 0.004 * propellant_kg          # residual hardware scaling
    avionics = 250.0 if propellant_kg > 20_000 else 90.0

    # apply calibrated growth factor to built-up structure (not engines)
    tank_mass *= GROWTH_FACTOR
    thrust_structure *= GROWTH_FACTOR
    secondary *= GROWTH_FACTOR
    if is_booster:
        secondary += INTERSTAGE_KG_PER_M2 * diameter_m ** 2

    return StageStructure(
        tank_mass_kg=tank_mass,
        engine_mass_kg=engine_mass,
        thrust_structure_kg=thrust_structure,
        secondary_mass_kg=secondary,
        avionics_mass_kg=avionics,
        tank_length_m=cyl_len + 2 * r * 0.5,
    )
