"""End-to-end demo: mission in -> verified vehicle out.

Runs in a few minutes on a laptop. Produces design_sheet.png and
design_report.md in ./demo_out/.
"""
from aeros.design import MissionSpec, design_vehicle
from aeros.plots import design_sheet
from aeros.cli import _markdown_report
from pathlib import Path

mission = MissionSpec("5 t to 500 km LEO", payload_kg=5000,
                      target_altitude_m=500_000, launch_latitude_deg=28.5)
result = design_vehicle(mission)
print(result.summary())

out = Path("demo_out"); out.mkdir(exist_ok=True)
design_sheet(result, path=out / "design_sheet.png")
(out / "design_report.md").write_text(_markdown_report(result))
print(f"\nOutputs in {out}/")
