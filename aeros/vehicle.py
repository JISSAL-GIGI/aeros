"""Vehicle and stage definitions with exact mass bookkeeping."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .engines import Engine, G0


@dataclass
class Stage:
    name: str
    dry_mass_kg: float
    propellant_kg: float
    engine: Engine
    n_engines: int
    diameter_m: float
    # fraction of propellant held back (residuals, landing reserves, FPR)
    unusable_prop_fraction: float = 0.005

    @property
    def usable_propellant_kg(self) -> float:
        return self.propellant_kg * (1.0 - self.unusable_prop_fraction)

    @property
    def wet_mass_kg(self) -> float:
        return self.dry_mass_kg + self.propellant_kg

    @property
    def thrust_vac_N(self) -> float:
        return self.engine.thrust_vac_N * self.n_engines

    @property
    def mass_flow_kg_s(self) -> float:
        return self.engine.mass_flow_kg_s * self.n_engines

    @property
    def burn_time_s(self) -> float:
        return self.usable_propellant_kg / self.mass_flow_kg_s

    def thrust_at(self, ambient_pressure_Pa: float) -> float:
        return self.engine.thrust_at(ambient_pressure_Pa) * self.n_engines


@dataclass
class Vehicle:
    name: str
    stages: list[Stage]                 # index 0 = first stage
    fairing_mass_kg: float = 0.0
    fairing_jettison_alt_m: float = 110_000.0
    payload_kg: float = 0.0

    @property
    def glow_kg(self) -> float:
        """Gross lift-off weight."""
        return (sum(s.wet_mass_kg for s in self.stages)
                + self.fairing_mass_kg + self.payload_kg)

    @property
    def liftoff_twr(self) -> float:
        from .engines import P_SL
        return self.stages[0].thrust_at(P_SL) / (self.glow_kg * G0)

    def mass_above_stage(self, i: int) -> float:
        """Mass carried by stage i (everything except stage i itself)."""
        m = self.payload_kg + sum(s.wet_mass_kg for s in self.stages[i + 1:])
        # Fairing normally rides into the stage-2 burn; count it above stage 0
        # and above stage 1 (jettison is handled by altitude in the simulator;
        # here it is the conservative static budget).
        m += self.fairing_mass_kg
        return m

    def ideal_delta_v(self) -> float:
        """Vacuum Tsiolkovsky delta-v of the full stack [m/s]."""
        dv = 0.0
        for i, s in enumerate(self.stages):
            m_above = self.mass_above_stage(i)
            m0 = s.wet_mass_kg + m_above
            m1 = s.dry_mass_kg + s.propellant_kg - s.usable_propellant_kg + m_above
            dv += s.engine.isp_vac_s * G0 * math.log(m0 / m1)
        return dv

    def describe(self) -> str:
        lines = [f"{self.name}: GLOW {self.glow_kg/1000:.1f} t, "
                 f"lift-off T/W {self.liftoff_twr:.2f}, "
                 f"ideal dv {self.ideal_delta_v():.0f} m/s"]
        for s in self.stages:
            lines.append(
                f"  {s.name}: {s.n_engines}x {s.engine.name}, "
                f"prop {s.propellant_kg/1000:.1f} t, dry {s.dry_mass_kg/1000:.2f} t, "
                f"burn {s.burn_time_s:.0f} s")
        return "\n".join(lines)
