"""Launch vehicle cost estimation (TRANSCOST-class CERs).

First-unit production cost-estimating relationships in the spirit of
Koelle's TRANSCOST handbook: cost scales with stage dry mass and engine
mass through power laws, expressed in work-years (WYr) and converted to
dollars. Coefficients below are of published TRANSCOST class but rounded;
they are suitable for RELATIVE architecture trades (the intended use),
not absolute price quotes. All constants are in one place so users can
recalibrate against their own data.

References: D.E. Koelle, "Handbook of Cost Engineering for Space
Transportation Systems (TRANSCOST)", TCS-TR-190; NASA cost symposium
materials on launch vehicle CERs.
"""

from __future__ import annotations

from dataclasses import dataclass

from .vehicle import Vehicle

WYR_TO_USD = 370_000.0        # one work-year, ~2022 aerospace rate

# First-unit production CERs: cost_WYr = a * (mass_kg)^b
STAGE_CER_A, STAGE_CER_B = 1.265, 0.59       # expendable stage, dry mass
ENGINE_CER_A, ENGINE_CER_B = 1.9, 0.535      # pump-fed liquid engine
FAIRING_CER_A, FAIRING_CER_B = 2.0, 0.59     # fairing/adapter structure
LEARNING_FACTOR = 0.90        # 90% learning curve exponent basis
INTEGRATION_FRACTION = 0.20   # vehicle assembly, integration & test
PROPELLANT_USD_PER_KG = 1.0   # bulk LOX/RP-1-class propellant


@dataclass
class CostBreakdown:
    stages_usd: list[float]
    engines_usd: list[float]
    fairing_usd: float
    integration_usd: float
    propellant_usd: float

    @property
    def total_usd(self) -> float:
        return (sum(self.stages_usd) + sum(self.engines_usd)
                + self.fairing_usd + self.integration_usd
                + self.propellant_usd)

    def summary(self) -> str:
        lines = [f"First-unit production cost estimate: "
                 f"${self.total_usd/1e6:.1f} M"]
        for i, (s, e) in enumerate(zip(self.stages_usd, self.engines_usd)):
            lines.append(f"  Stage {i+1} structure ${s/1e6:6.1f} M | "
                         f"engines ${e/1e6:6.1f} M")
        lines.append(f"  Fairing ${self.fairing_usd/1e6:.1f} M | "
                     f"integration ${self.integration_usd/1e6:.1f} M | "
                     f"propellant ${self.propellant_usd/1e6:.2f} M")
        return "\n".join(lines)


def first_unit_cost(vehicle: Vehicle,
                    business_factor: float = 1.0) -> CostBreakdown:
    """First-unit production cost of an expendable vehicle.

    `business_factor` scales hardware CERs for organisational practice:
    1.0 = traditional prime contractor (TRANSCOST baseline);
    0.3-0.5 = vertically-integrated commercial practice (Koelle's f8-class
    correction; SpaceX-era actuals support the low end). Relative
    architecture rankings are insensitive to this factor.
    """
    stages, engines = [], []
    prop_mass = 0.0
    for s in vehicle.stages:
        struct_dry = max(s.dry_mass_kg - s.engine.mass_kg * s.n_engines, 100.0)
        stages.append(STAGE_CER_A * struct_dry ** STAGE_CER_B * WYR_TO_USD
                      * business_factor)
        one_engine = ENGINE_CER_A * s.engine.mass_kg ** ENGINE_CER_B * WYR_TO_USD
        # learning across identical engines in the cluster (90% curve)
        n = s.n_engines
        lot = one_engine * n ** (1 + (-0.152))   # b = ln(0.9)/ln(2)
        engines.append(lot * business_factor)
        prop_mass += s.propellant_kg
    fairing = FAIRING_CER_A * max(vehicle.fairing_mass_kg, 50.0) ** \
        FAIRING_CER_B * WYR_TO_USD * business_factor
    hardware = sum(stages) + sum(engines) + fairing
    return CostBreakdown(
        stages_usd=stages, engines_usd=engines, fairing_usd=fairing,
        integration_usd=INTEGRATION_FRACTION * hardware,
        propellant_usd=prop_mass * PROPELLANT_USD_PER_KG,
    )


def cost_per_kg(vehicle: Vehicle) -> float:
    """First-unit cost divided by payload mass."""
    if vehicle.payload_kg <= 0:
        return float("inf")
    return first_unit_cost(vehicle).total_usd / vehicle.payload_kg
