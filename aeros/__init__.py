"""AEROS - Autonomous Engineering & Reasoning system for launch vehicle design.

Physics-validated autonomous design of multistage launch vehicles:
mission in, validated vehicle out - with the same trajectory simulator
proven against Falcon 9, Electron and Saturn V.
"""

__version__ = "0.1.0"

from .engines import ENGINES, get_engine
from .materials import MATERIALS, PROPELLANTS, get_material
from .vehicle import Stage, Vehicle
from .atmosphere import atmosphere
from .trajectory import simulate_ascent, payload_capacity

from .design import MissionSpec, design_vehicle
