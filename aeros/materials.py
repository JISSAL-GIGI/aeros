"""Aerospace structural material properties.

Room-temperature design values from MMPDS/MIL-HDBK-5 class sources and
manufacturer datasheets. Used for tank wall sizing and mass estimation.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Material:
    name: str
    density_kg_m3: float
    yield_strength_Pa: float
    ultimate_strength_Pa: float
    max_service_temp_K: float
    weldable: bool
    notes: str = ""


MATERIALS: dict[str, Material] = {
    "Al-2219-T87": Material(
        "Al-2219-T87", 2840, 393e6, 476e6, 477, True,
        "Workhorse cryo-tank alloy (Saturn V, Shuttle ET pre-lightweight)."),
    "Al-Li-2195": Material(
        "Al-Li-2195", 2700, 545e6, 593e6, 450, True,
        "Al-Li alloy of the Shuttle super-lightweight tank and Falcon 9 tanks."),
    "SS-301-CH": Material(
        "SS-301-CH", 7880, 965e6, 1275e6, 800, True,
        "Cold-worked stainless; Atlas/Centaur balloon tanks, Starship-class steel."),
    "Ti-6Al-4V": Material(
        "Ti-6Al-4V", 4430, 880e6, 950e6, 670, False,
        "High-pressure vessels, hot structures."),
    "CFRP-IM7": Material(
        "CFRP-IM7", 1600, 600e6, 800e6, 400, False,
        "Quasi-isotropic laminate allowables (conservative); fairings, interstages."),
}


def get_material(name: str) -> Material:
    if name not in MATERIALS:
        raise KeyError(f"Unknown material '{name}'. Available: {sorted(MATERIALS)}")
    return MATERIALS[name]


# Propellant densities [kg/m^3] and typical O/F mass mixture ratios
@dataclass(frozen=True)
class PropellantCombo:
    name: str
    oxidizer_density: float
    fuel_density: float
    mixture_ratio: float  # oxidizer/fuel by mass

    @property
    def bulk_density(self) -> float:
        """Average density of the loaded propellant mass."""
        mr = self.mixture_ratio
        return (1 + mr) / (mr / self.oxidizer_density + 1 / self.fuel_density)


PROPELLANTS: dict[str, PropellantCombo] = {
    "LOX/RP-1": PropellantCombo("LOX/RP-1", 1141, 810, 2.36),
    "LOX/LH2": PropellantCombo("LOX/LH2", 1141, 71, 5.5),
    "LOX/CH4": PropellantCombo("LOX/CH4", 1141, 423, 3.6),
}
