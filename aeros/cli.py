"""Command-line interface.

    aeros design "5000 kg to 500 km" --out results/
    aeros validate
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


def parse_mission(text: str):
    """Parse '5000 kg to 500 km' / '5 t to 500km' style mission strings."""
    m = re.search(r"([\d.]+)\s*(kg|t)\b", text, re.I)
    a = re.search(r"to\s+([\d.]+)\s*km", text, re.I)
    if not m or not a:
        raise ValueError(
            "Mission must look like '5000 kg to 500 km' or '5 t to 500 km'")
    payload = float(m.group(1)) * (1000.0 if m.group(2).lower() == "t" else 1.0)
    alt = float(a.group(1)) * 1000.0
    return payload, alt


def cmd_design(args):
    from .design import MissionSpec, design_vehicle
    from .plots import design_sheet

    payload, alt = parse_mission(args.mission)
    mission = MissionSpec(args.mission, payload_kg=payload,
                          target_altitude_m=alt,
                          launch_latitude_deg=args.latitude)
    result = design_vehicle(mission)
    print()
    print(result.summary())

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    design_sheet(result, path=out / "design_sheet.png")
    (out / "design_report.md").write_text(_markdown_report(result),
                                          encoding="utf-8")
    print(f"\nWrote {out/'design_sheet.png'} and {out/'design_report.md'}")


def _markdown_report(result) -> str:
    v = result.vehicle
    res = result.ascent
    lines = [
        f"# AEROS design report - {result.mission.name}", "",
        f"**Mission**: {result.mission.payload_kg/1000:.2f} t to "
        f"{result.mission.target_altitude_m/1000:.0f} km "
        f"(launch latitude {result.mission.launch_latitude_deg:.1f} deg)", "",
        f"**Verification**: "
        + ("PASSED - the designed vehicle reached orbit in the same 3-DOF "
           "simulator that reproduces Falcon 9, Electron and Saturn V flight "
           "data" if result.verified else "FAILED"), "",
        "## Vehicle", "", "```", v.describe(), "```", "",
        "## Verification flight", "",
        f"- perigee: {res.perigee_m/1000:.0f} km",
        f"- apogee: {res.apogee_m/1000:.0f} km",
        f"- max dynamic pressure: {res.max_q_Pa/1000:.0f} kPa",
        f"- max acceleration: {res.max_accel_g:.1f} g", "",
        "## Decision log", "",
    ]
    for d in result.decisions:
        lines += [f"### {d.subject}", "", f"**{d.choice}**", "",
                  d.rationale, ""]
    lines += ["## Alternatives considered", "",
              "| Booster | Upper | GLOW [t] | dv split | Diameter [m] |",
              "|---|---|---|---|---|"]
    for a in result.alternatives[:10]:
        lines.append(f"| {a['booster']} | {a['upper']} | {a['glow_t']} "
                     f"| {a['dv_split']} | {a['diameter_m']} |")
    return "\n".join(lines) + "\n"


def cmd_validate(args):
    from .validate import run_validation
    rows = run_validation(verbose=True)
    print("\n| Vehicle | Published | Predicted | Error |")
    print("|---|---|---|---|")
    for r in rows:
        print(f"| {r['vehicle']} | {r['published_kg']/1000:.2f} t "
              f"| {r['predicted_kg']/1000:.2f} t | {r['error_pct']:+.1f}% |")
    if args.json:
        Path(args.json).write_text(json.dumps(rows, indent=1))


def main(argv=None):
    p = argparse.ArgumentParser(prog="aeros",
                                description="AEROS launch vehicle design")
    sub = p.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("design", help="design a launch vehicle for a mission")
    d.add_argument("mission", help="'5000 kg to 500 km'")
    d.add_argument("--latitude", type=float, default=28.5)
    d.add_argument("--out", default="aeros_out")
    d.set_defaults(func=cmd_design)
    v = sub.add_parser("validate",
                       help="reproduce Falcon 9 / Electron / Saturn V "
                            "(takes tens of minutes)")
    v.add_argument("--json", default=None)
    v.set_defaults(func=cmd_validate)
    args = p.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
