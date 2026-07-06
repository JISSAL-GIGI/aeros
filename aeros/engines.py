"""Rocket engine database.

Every entry is a real, flown engine with published performance figures.
Sources are cited per engine. Thrust varies with ambient pressure via the
exact nozzle relation  F(p) = F_vac - Ae * p,  where the effective exit area
Ae is recovered from the sea-level/vacuum thrust pair.
"""

from __future__ import annotations

from dataclasses import dataclass, field

P_SL = 101_325.0  # Pa
G0 = 9.80665


@dataclass(frozen=True)
class Engine:
    name: str
    propellants: str          # e.g. "LOX/RP-1"
    cycle: str
    thrust_vac_N: float
    thrust_sl_N: float | None  # None for vacuum-only engines
    isp_vac_s: float
    isp_sl_s: float | None
    mass_kg: float
    diameter_m: float
    source: str = ""

    @property
    def exit_area_m2(self) -> float:
        """Effective exit area from the thrust/pressure relation."""
        if self.thrust_sl_N is None:
            return 0.0
        return (self.thrust_vac_N - self.thrust_sl_N) / P_SL

    @property
    def mass_flow_kg_s(self) -> float:
        """Propellant mass flow (constant for a fixed-throttle engine)."""
        return self.thrust_vac_N / (self.isp_vac_s * G0)

    def thrust_at(self, ambient_pressure_Pa: float) -> float:
        """Thrust [N] at ambient pressure, exact nozzle relation."""
        return self.thrust_vac_N - self.exit_area_m2 * ambient_pressure_Pa

    def isp_at(self, ambient_pressure_Pa: float) -> float:
        return self.thrust_at(ambient_pressure_Pa) / (self.mass_flow_kg_s * G0)


ENGINES: dict[str, Engine] = {}


def _add(e: Engine):
    ENGINES[e.name] = e


_add(Engine(
    name="Merlin 1D", propellants="LOX/RP-1", cycle="gas generator",
    thrust_vac_N=914_000, thrust_sl_N=845_000,
    isp_vac_s=311, isp_sl_s=282,
    mass_kg=470, diameter_m=1.0,
    source="SpaceX Falcon User's Guide; en.wikipedia.org/wiki/SpaceX_Merlin",
))

_add(Engine(
    name="Merlin 1D Vacuum", propellants="LOX/RP-1", cycle="gas generator",
    thrust_vac_N=981_000, thrust_sl_N=None,
    isp_vac_s=348, isp_sl_s=None,
    mass_kg=600, diameter_m=3.3,
    source="SpaceX Falcon User's Guide",
))

_add(Engine(
    name="Rutherford", propellants="LOX/RP-1", cycle="electric pump",
    thrust_vac_N=25_800, thrust_sl_N=24_000,
    isp_vac_s=311, isp_sl_s=290,
    mass_kg=35, diameter_m=0.25,
    source="Rocket Lab Electron Payload User's Guide",
))

_add(Engine(
    name="Rutherford Vacuum", propellants="LOX/RP-1", cycle="electric pump",
    thrust_vac_N=25_800, thrust_sl_N=None,
    isp_vac_s=343, isp_sl_s=None,
    mass_kg=35, diameter_m=0.9,
    source="Rocket Lab Electron Payload User's Guide",
))

_add(Engine(
    name="F-1", propellants="LOX/RP-1", cycle="gas generator",
    thrust_vac_N=7_770_000, thrust_sl_N=6_770_000,
    isp_vac_s=304, isp_sl_s=263,
    mass_kg=8_400, diameter_m=3.7,
    source="NASA SP-4206 'Stages to Saturn'; braeunig.us/space/specs/saturn.htm",
))

_add(Engine(
    name="J-2", propellants="LOX/LH2", cycle="gas generator",
    thrust_vac_N=1_033_000, thrust_sl_N=None,
    isp_vac_s=421, isp_sl_s=None,
    mass_kg=1_788, diameter_m=2.0,
    source="NASA SP-4206 'Stages to Saturn'",
))

_add(Engine(
    name="Raptor 2", propellants="LOX/CH4", cycle="full-flow staged combustion",
    thrust_vac_N=2_530_000, thrust_sl_N=2_300_000,
    isp_vac_s=350, isp_sl_s=327,
    mass_kg=1_600, diameter_m=1.3,
    source="SpaceX published figures (2022)",
))

_add(Engine(
    name="Raptor Vacuum", propellants="LOX/CH4", cycle="full-flow staged combustion",
    thrust_vac_N=2_530_000, thrust_sl_N=None,
    isp_vac_s=380, isp_sl_s=None,
    mass_kg=1_800, diameter_m=2.4,
    source="SpaceX published figures (2022)",
))

_add(Engine(
    name="RS-25", propellants="LOX/LH2", cycle="staged combustion",
    thrust_vac_N=2_279_000, thrust_sl_N=1_860_000,
    isp_vac_s=452.3, isp_sl_s=366,
    mass_kg=3_527, diameter_m=2.4,
    source="Aerojet Rocketdyne RS-25 fact sheet",
))

_add(Engine(
    name="RL10C-1", propellants="LOX/LH2", cycle="expander",
    thrust_vac_N=101_800, thrust_sl_N=None,
    isp_vac_s=449.7, isp_sl_s=None,
    mass_kg=190, diameter_m=1.45,
    source="Aerojet Rocketdyne RL10 fact sheet",
))


def get_engine(name: str) -> Engine:
    if name not in ENGINES:
        raise KeyError(f"Unknown engine '{name}'. Available: {sorted(ENGINES)}")
    return ENGINES[name]
