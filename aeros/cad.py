"""Concept CAD generation for AEROS vehicles.

The first CAD layer is deliberately lightweight: it turns the physics-sized
vehicle into inspectable mesh and parametric geometry files without requiring
a local CAD kernel. That keeps AEROS reproducible on laptops and CI while
establishing the data path from mission -> design decisions -> geometry.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

from .materials import PROPELLANTS
from .vehicle import Vehicle


@dataclass
class Mesh:
    name: str
    vertices: list[tuple[float, float, float]] = field(default_factory=list)
    faces: list[tuple[int, int, int]] = field(default_factory=list)

    def add_vertex(self, p: tuple[float, float, float]) -> int:
        self.vertices.append(p)
        return len(self.vertices) - 1

    def add_face(self, a: int, b: int, c: int):
        self.faces.append((a, b, c))

    def extend(self, other: "Mesh"):
        offset = len(self.vertices)
        self.vertices.extend(other.vertices)
        self.faces.extend((a + offset, b + offset, c + offset)
                          for a, b, c in other.faces)


@dataclass(frozen=True)
class CadPart:
    name: str
    role: str
    mesh: Mesh
    mass_kg: float
    length_m: float
    diameter_m: float
    z_min_m: float
    z_max_m: float
    notes: str = ""


def stage_length_m(stage) -> float:
    """Return the conceptual external stage length used by drawing/CAD."""
    prop = PROPELLANTS[stage.engine.propellants]
    volume = stage.propellant_kg / prop.bulk_density * 1.05
    area = math.pi / 4 * stage.diameter_m ** 2
    return volume / area + 0.6 * stage.diameter_m


def build_vehicle_cad(vehicle: Vehicle, segments: int = 64) -> list[CadPart]:
    """Build conceptual CAD parts for a launch vehicle.

    Geometry is in metres, with z=0 at the first-stage base. The generated
    parts are not fabrication-ready solids; they are decision-traceable concept
    geometry sized from the same masses, diameters, propellant volumes, and
    engine choices used by the physics model.
    """
    if segments < 12:
        raise ValueError("segments must be >= 12")

    parts: list[CadPart] = []
    z = 0.0
    max_diameter = max(s.diameter_m for s in vehicle.stages)

    for i, stage in enumerate(vehicle.stages):
        length = stage_length_m(stage)
        mesh = _frustum_mesh(f"stage_{i + 1}", z, z + length,
                             stage.diameter_m / 2, stage.diameter_m / 2,
                             segments, cap_bottom=True, cap_top=True)
        parts.append(CadPart(
            name=f"stage_{i + 1}_{_slug(stage.engine.name)}",
            role="propulsive_stage",
            mesh=mesh,
            mass_kg=stage.wet_mass_kg,
            length_m=length,
            diameter_m=stage.diameter_m,
            z_min_m=z,
            z_max_m=z + length,
            notes=(f"{stage.n_engines}x {stage.engine.name}; "
                   f"{stage.propellant_kg:.0f} kg propellant"),
        ))

        parts.append(_thrust_interface_part(stage, z, i + 1, segments))
        parts.extend(_engine_bell_parts(stage, z, segments))

        if i < len(vehicle.stages) - 1:
            next_stage = vehicle.stages[i + 1]
            parts.append(_stage_joint_part(stage, next_stage, z + length,
                                           i + 1, segments))
        z += length

    parts.append(_payload_adapter_part(vehicle, z, max_diameter, segments))
    fairing_len = 2.2 * max_diameter
    fairing = _ogive_mesh("payload_fairing", z, z + fairing_len,
                          max_diameter / 2, segments)
    parts.append(CadPart(
        name="payload_fairing",
        role="payload_accommodation",
        mesh=fairing,
        mass_kg=vehicle.fairing_mass_kg,
        length_m=fairing_len,
        diameter_m=max_diameter,
        z_min_m=z,
        z_max_m=z + fairing_len,
        notes=f"{vehicle.payload_kg:.0f} kg payload envelope",
    ))
    return parts


def export_vehicle_cad(
    vehicle: Vehicle,
    out_dir: str | Path,
    *,
    mission=None,
    decisions: Iterable[object] | None = None,
    alternatives: list[dict] | None = None,
    segments: int = 64,
    prefix: str = "vehicle",
) -> dict[str, Path]:
    """Export OBJ, STL, OpenSCAD, and manifest files for a vehicle."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    parts = build_vehicle_cad(vehicle, segments=segments)

    combined = Mesh(prefix)
    for part in parts:
        combined.extend(part.mesh)

    files = {
        "obj": out / f"{prefix}.obj",
        "stl": out / f"{prefix}.stl",
        "scad": out / f"{prefix}.scad",
        "manifest": out / f"{prefix}_manifest.json",
        "review_png": out / f"{prefix}_cad_review.png",
        "review_json": out / f"{prefix}_cad_review.json",
    }
    write_obj(parts, files["obj"])
    write_stl(combined, files["stl"])
    write_scad(vehicle, parts, files["scad"])
    review = inspect_cad_parts(parts)
    render_cad_review(parts, files["review_png"], title=vehicle.name)
    files["manifest"].write_text(
        json.dumps(_manifest(vehicle, parts, mission, decisions, alternatives),
                   indent=2),
        encoding="utf-8",
    )
    files["review_json"].write_text(json.dumps(review, indent=2),
                                    encoding="utf-8")
    return files


def write_obj(parts: list[CadPart], path: str | Path):
    """Write a grouped Wavefront OBJ file."""
    lines = ["# AEROS conceptual CAD export", "o aeros_vehicle"]
    offset = 1
    for part in parts:
        lines.append(f"g {part.name}")
        for x, y, z in part.mesh.vertices:
            lines.append(f"v {x:.6f} {y:.6f} {z:.6f}")
        for a, b, c in part.mesh.faces:
            lines.append(f"f {a + offset} {b + offset} {c + offset}")
        offset += len(part.mesh.vertices)
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_stl(mesh: Mesh, path: str | Path):
    """Write an ASCII STL file."""
    lines = [f"solid {_slug(mesh.name)}"]
    for a, b, c in mesh.faces:
        p0, p1, p2 = mesh.vertices[a], mesh.vertices[b], mesh.vertices[c]
        nx, ny, nz = _normal(p0, p1, p2)
        lines.append(f"  facet normal {nx:.8e} {ny:.8e} {nz:.8e}")
        lines.append("    outer loop")
        for x, y, z in (p0, p1, p2):
            lines.append(f"      vertex {x:.8e} {y:.8e} {z:.8e}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {_slug(mesh.name)}")
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_scad(vehicle: Vehicle, parts: list[CadPart], path: str | Path):
    """Write a parametric OpenSCAD recipe mirroring the generated mesh."""
    lines = [
        "// AEROS conceptual CAD export",
        "// Units: metres",
        "$fn = 96;",
        "",
        "module stage_shell(z0, h, r) {",
        "  translate([0, 0, z0]) cylinder(h=h, r=r);",
        "}",
        "",
        "module bell(x, y, z0, h, r1, r2) {",
        "  translate([x, y, z0]) cylinder(h=h, r1=r1, r2=r2);",
        "}",
        "",
        "union() {",
    ]
    for part in parts:
        if part.role == "propulsive_stage":
            lines.append(f"  // {part.name}: {part.notes}")
            lines.append(
                f"  stage_shell({part.z_min_m:.6f}, {part.length_m:.6f}, "
                f"{part.diameter_m / 2:.6f});")
        elif part.role in {"thrust_interface", "structural_interface",
                           "payload_interface"}:
            lines.append(f"  // {part.name}: {part.notes}")
            lines.append(
                f"  stage_shell({part.z_min_m:.6f}, {part.length_m:.6f}, "
                f"{part.diameter_m / 2:.6f});")
        elif part.role == "engine_bell":
            cx, cy = _part_center_xy(part.mesh)
            z0 = part.z_min_m
            lines.append(f"  // {part.name}")
            lines.append(
                f"  bell({cx:.6f}, {cy:.6f}, {z0:.6f}, {part.length_m:.6f}, "
                f"{part.diameter_m / 2:.6f}, {part.diameter_m * 0.28:.6f});")
        elif part.role == "payload_accommodation":
            lines.append(f"  // {part.name}: {part.notes}")
            lines.append(
                f"  translate([0, 0, {part.z_min_m:.6f}]) "
                f"cylinder(h={part.length_m:.6f}, r1={part.diameter_m / 2:.6f}, "
                "r2=0);")
    lines.append("}")
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def inspect_cad_parts(parts: list[CadPart]) -> dict:
    """Run visual-review oriented checks on generated concept geometry."""
    checks = []
    total_triangles = sum(len(p.mesh.faces) for p in parts)
    total_vertices = sum(len(p.mesh.vertices) for p in parts)

    empty = [p.name for p in parts if not p.mesh.vertices or not p.mesh.faces]
    _add_check(checks, "mesh_presence", not empty,
               "all parts contain vertices and triangles",
               f"empty parts: {', '.join(empty)}")

    degenerate = sum(_degenerate_faces(p.mesh) for p in parts)
    _add_check(checks, "triangle_quality", degenerate == 0,
               "no zero-area triangles detected",
               f"{degenerate} zero-area triangles detected")

    stages = sorted((p for p in parts if p.role == "propulsive_stage"),
                    key=lambda p: p.z_min_m)
    gaps = [
        abs(stages[i].z_max_m - stages[i + 1].z_min_m)
        for i in range(len(stages) - 1)
    ]
    _add_check(checks, "stage_stack_continuity",
               all(g < 1e-6 for g in gaps),
               "propulsive stages touch without axial gaps",
               f"stage axial gaps: {gaps}")

    joints = [p for p in parts if p.role == "structural_interface"]
    thrusts = [p for p in parts if p.role == "thrust_interface"]
    payload_adapters = [p for p in parts if p.role == "payload_interface"]
    _add_check(checks, "interface_markers",
               len(thrusts) == len(stages)
               and len(joints) == max(0, len(stages) - 1)
               and len(payload_adapters) == 1,
               "thrust, stage-joint, and payload-interface markers present",
               "missing one or more interface marker classes")

    engine_checks = _engine_envelope_checks(parts, stages)
    checks.extend(engine_checks)

    fairings = [p for p in parts if p.role == "payload_accommodation"]
    _add_check(checks, "payload_fairing",
               len(fairings) == 1 and fairings[0].z_min_m >= stages[-1].z_max_m,
               "single payload fairing sits above the upper stage",
               "payload fairing missing or not above upper stage")

    status = "pass" if all(c["status"] == "pass" for c in checks) else "fail"
    return {
        "status": status,
        "units": "m, kg",
        "part_count": len(parts),
        "total_vertices": total_vertices,
        "total_triangles": total_triangles,
        "bounds": _bounds_record(parts),
        "checks": checks,
        "parts": [_part_record(p) | {"bounds": _mesh_bounds_record(p.mesh)}
                  for p in parts],
    }


def render_cad_review(parts: list[CadPart], path: str | Path,
                      *, title: str = "AEROS CAD review"):
    """Render side, base, and isometric review views for human inspection."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    colors = {
        "propulsive_stage": "#8FB3FF",
        "engine_bell": "#444444",
        "payload_accommodation": "#D8E78F",
        "thrust_interface": "#E39D4A",
        "structural_interface": "#D65F5F",
        "payload_interface": "#7FC97F",
    }
    stages = sorted((p for p in parts if p.role == "propulsive_stage"),
                    key=lambda p: p.z_min_m)
    engines = [p for p in parts if p.role == "engine_bell"]
    max_r = max(p.diameter_m / 2 for p in parts)
    min_z = min(p.z_min_m for p in parts)
    max_z = max(p.z_max_m for p in parts)

    fig = plt.figure(figsize=(13, 7), constrained_layout=True)
    fig.suptitle(title, fontsize=13, fontweight="bold")

    ax_side = fig.add_subplot(1, 3, 1)
    ax_side.set_title("Side profile and interfaces")
    for p in sorted(parts, key=lambda q: (q.z_min_m, q.role)):
        r = p.diameter_m / 2
        color = colors.get(p.role, "#cccccc")
        if p.role == "payload_accommodation":
            ax_side.add_patch(Polygon(
                [(-r, p.z_min_m), (r, p.z_min_m), (0, p.z_max_m)],
                closed=True, facecolor=color, edgecolor="black", alpha=0.85))
        elif p.role == "engine_bell":
            cx, _ = _part_center_xy(p.mesh)
            ax_side.add_patch(Polygon(
                [(cx - r, p.z_min_m), (cx + r, p.z_min_m),
                 (cx + r * 0.45, p.z_max_m), (cx - r * 0.45, p.z_max_m)],
                closed=True, facecolor=color, edgecolor="black", alpha=0.9))
        else:
            ax_side.add_patch(Rectangle(
                (-r, p.z_min_m), p.diameter_m, p.length_m,
                facecolor=color, edgecolor="black", alpha=0.65))
    ax_side.set_xlim(-max_r * 1.35, max_r * 1.35)
    ax_side.set_ylim(min_z - max_r * 0.2, max_z + max_r * 0.1)
    ax_side.set_aspect("equal", adjustable="box")
    ax_side.set_xlabel("radius [m]")
    ax_side.set_ylabel("vehicle z [m]")
    ax_side.grid(True, alpha=0.25)

    ax_top = fig.add_subplot(1, 3, 2)
    ax_top.set_title("First-stage engine bay")
    if stages:
        body_r = stages[0].diameter_m / 2
        ax_top.add_patch(Circle((0, 0), body_r, fill=False,
                                edgecolor="black", linewidth=1.5))
        first_stage_engines = [
            e for e in engines if abs(e.z_max_m - stages[0].z_min_m) < 1e-6
        ]
        for e in first_stage_engines:
            cx, cy = _part_center_xy(e.mesh)
            ax_top.add_patch(Circle((cx, cy), e.diameter_m / 2,
                                    facecolor=colors["engine_bell"],
                                    edgecolor="black", alpha=0.85))
        ax_top.set_xlim(-body_r * 1.2, body_r * 1.2)
        ax_top.set_ylim(-body_r * 1.2, body_r * 1.2)
    ax_top.set_aspect("equal", adjustable="box")
    ax_top.set_xlabel("x [m]")
    ax_top.set_ylabel("y [m]")
    ax_top.grid(True, alpha=0.25)

    ax_3d = fig.add_subplot(1, 3, 3, projection="3d")
    ax_3d.set_title("Isometric mesh")
    for p in parts:
        verts = p.mesh.vertices
        faces = p.mesh.faces
        stride = max(1, len(faces) // 350)
        polys = [[verts[i] for i in face] for face in faces[::stride]]
        coll = Poly3DCollection(
            polys,
            facecolor=colors.get(p.role, "#cccccc"),
            edgecolor="#333333",
            linewidth=0.15,
            alpha=0.72,
        )
        ax_3d.add_collection3d(coll)
    _set_axes_review_3d(ax_3d, parts)
    ax_3d.view_init(elev=17, azim=-42)
    ax_3d.set_xlabel("x [m]")
    ax_3d.set_ylabel("y [m]")
    ax_3d.set_zlabel("z [m]")

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _manifest(vehicle, parts, mission, decisions, alternatives):
    total_length = max(p.z_max_m for p in parts if p.role != "engine_bell")
    payload = {
        "name": vehicle.name,
        "glow_kg": _num(vehicle.glow_kg),
        "liftoff_twr": _num(vehicle.liftoff_twr),
        "ideal_delta_v_m_s": _num(vehicle.ideal_delta_v()),
        "total_length_m": _num(total_length),
        "max_diameter_m": _num(max(p.diameter_m for p in parts)),
        "cad_fidelity": "concept_mesh",
        "units": "m, kg, s",
        "parts": [_part_record(p) for p in parts],
        "official_need_alignment": [
            {
                "source": "NASA FY26 Civil Space Shortfall Prioritization",
                "need": "SF32 affordable and resilient supply chain; "
                        "SF22 launch cadence; SF25 onboard computing",
                "aeros_step": "machine-readable geometry manifest connects "
                              "sizing decisions to downstream CAD/CAM tools",
            },
            {
                "source": "ESA Technology Strategy",
                "need": "digital workflow, MBSE, modularity, standardisation, "
                        "and Digital Design-2-Produce",
                "aeros_step": "same mission object now emits analysis, CAD, "
                              "and structured design data",
            },
        ],
    }
    if mission is not None:
        payload["mission"] = {
            "name": mission.name,
            "payload_kg": _num(mission.payload_kg),
            "target_altitude_m": _num(mission.target_altitude_m),
            "launch_latitude_deg": _num(mission.launch_latitude_deg),
        }
    if decisions:
        payload["decisions"] = [
            {
                "subject": d.subject,
                "choice": d.choice,
                "rationale": d.rationale,
            }
            for d in decisions
        ]
    if alternatives:
        payload["alternatives_considered"] = [
            {k: _num(v) for k, v in row.items()} for row in alternatives[:10]
        ]
    return payload


def _part_record(part: CadPart):
    return {
        "name": part.name,
        "role": part.role,
        "mass_kg": _num(part.mass_kg),
        "length_m": _num(part.length_m),
        "diameter_m": _num(part.diameter_m),
        "z_min_m": _num(part.z_min_m),
        "z_max_m": _num(part.z_max_m),
        "vertices": len(part.mesh.vertices),
        "triangles": len(part.mesh.faces),
        "notes": part.notes,
    }


def _thrust_interface_part(stage, z_base: float, stage_number: int,
                           segments: int) -> CadPart:
    thickness = max(0.08, stage.diameter_m * 0.035)
    mesh = _frustum_mesh(
        f"stage_{stage_number}_thrust_interface",
        z_base,
        z_base + thickness,
        stage.diameter_m * 0.48,
        stage.diameter_m * 0.48,
        segments,
        cap_bottom=True,
        cap_top=True,
    )
    return CadPart(
        name=f"stage_{stage_number}_thrust_interface",
        role="thrust_interface",
        mesh=mesh,
        mass_kg=stage.dry_mass_kg * 0.04,
        length_m=thickness,
        diameter_m=stage.diameter_m * 0.96,
        z_min_m=z_base,
        z_max_m=z_base + thickness,
        notes="conceptual thrust-transfer plate; mass included in stage dry mass",
    )


def _stage_joint_part(lower_stage, upper_stage, z_joint: float,
                      stage_number: int, segments: int) -> CadPart:
    diameter = max(lower_stage.diameter_m, upper_stage.diameter_m) * 1.025
    thickness = max(0.10, diameter * 0.03)
    r0 = lower_stage.diameter_m / 2 * 1.025
    r1 = upper_stage.diameter_m / 2 * 1.025
    mesh = _frustum_mesh(
        f"stage_{stage_number}_to_{stage_number + 1}_joint",
        z_joint - thickness / 2,
        z_joint + thickness / 2,
        r0,
        r1,
        segments,
        cap_bottom=True,
        cap_top=True,
    )
    return CadPart(
        name=f"stage_{stage_number}_to_{stage_number + 1}_joint",
        role="structural_interface",
        mesh=mesh,
        mass_kg=(lower_stage.dry_mass_kg + upper_stage.dry_mass_kg) * 0.015,
        length_m=thickness,
        diameter_m=diameter,
        z_min_m=z_joint - thickness / 2,
        z_max_m=z_joint + thickness / 2,
        notes="conceptual interstage separation and load-transfer interface",
    )


def _payload_adapter_part(vehicle: Vehicle, z_base: float, diameter: float,
                          segments: int) -> CadPart:
    thickness = max(0.08, diameter * 0.035)
    mesh = _frustum_mesh(
        "payload_adapter",
        z_base,
        z_base + thickness,
        diameter * 0.40,
        diameter * 0.28,
        segments,
        cap_bottom=True,
        cap_top=True,
    )
    return CadPart(
        name="payload_adapter",
        role="payload_interface",
        mesh=mesh,
        mass_kg=max(vehicle.payload_kg * 0.015, 15.0),
        length_m=thickness,
        diameter_m=diameter * 0.80,
        z_min_m=z_base,
        z_max_m=z_base + thickness,
        notes="conceptual payload attach fitting; not a qualified adapter",
    )


def _engine_bell_parts(stage, z_base: float, segments: int) -> list[CadPart]:
    n = stage.n_engines
    body_r = stage.diameter_m / 2
    exit_r = min(max(stage.diameter_m * 0.055,
                     stage.engine.diameter_m * 0.38),
                 body_r * 0.34)
    throat_r = exit_r * 0.45
    length = max(stage.diameter_m * 0.24, stage.engine.diameter_m * 0.65)
    centers = _engine_layout(n, body_r, exit_r)
    parts = []
    for i, (cx, cy) in enumerate(centers):
        mesh = _frustum_mesh(f"engine_{i + 1}", z_base - length, z_base,
                             exit_r, throat_r, segments,
                             center=(cx, cy), cap_bottom=True, cap_top=True)
        parts.append(CadPart(
            name=f"{_slug(stage.engine.name)}_bell_{i + 1}",
            role="engine_bell",
            mesh=mesh,
            mass_kg=stage.engine.mass_kg,
            length_m=length,
            diameter_m=exit_r * 2,
            z_min_m=z_base - length,
            z_max_m=z_base,
            notes=stage.engine.name,
        ))
    return parts


def _engine_layout(n: int, body_r: float,
                   exit_r: float) -> list[tuple[float, float]]:
    if n <= 0:
        return []
    if n == 1:
        return [(0.0, 0.0)]
    if n in {5, 9}:
        return [(0.0, 0.0)] + _ring_points(
            n - 1, _engine_ring_radius(n - 1, body_r, exit_r,
                                       include_center=True))
    return _ring_points(
        n, _engine_ring_radius(n, body_r, exit_r, include_center=False))


def _engine_ring_radius(count: int, body_r: float, exit_r: float,
                        *, include_center: bool) -> float:
    clearance = exit_r * 0.12
    adjacent = (exit_r * 1.06 / math.sin(math.pi / count)
                if count > 2 else exit_r * 1.2)
    center_clearance = 2 * exit_r + clearance if include_center else 0.0
    target = max(adjacent, center_clearance, body_r * 0.52)
    max_allowed = max(0.0, body_r - exit_r * 1.08)
    return min(target, max_allowed)


def _ring_points(count: int, radius: float) -> list[tuple[float, float]]:
    # Start at +Y so the top view reads like the usual engine-bay diagrams.
    return [
        (radius * math.cos(2 * math.pi * i / count + math.pi / 2),
         radius * math.sin(2 * math.pi * i / count + math.pi / 2))
        for i in range(count)
    ]


def _frustum_mesh(
    name: str,
    z0: float,
    z1: float,
    r0: float,
    r1: float,
    segments: int,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    cap_bottom: bool = False,
    cap_top: bool = False,
) -> Mesh:
    mesh = Mesh(name)
    cx, cy = center
    bottom = []
    top = []
    for i in range(segments):
        a = 2 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        bottom.append(mesh.add_vertex((cx + r0 * ca, cy + r0 * sa, z0)))
        top.append(mesh.add_vertex((cx + r1 * ca, cy + r1 * sa, z1)))
    for i in range(segments):
        j = (i + 1) % segments
        mesh.add_face(bottom[i], bottom[j], top[j])
        mesh.add_face(bottom[i], top[j], top[i])
    if cap_bottom:
        c = mesh.add_vertex((cx, cy, z0))
        for i in range(segments):
            mesh.add_face(c, bottom[i], bottom[(i + 1) % segments])
    if cap_top:
        c = mesh.add_vertex((cx, cy, z1))
        for i in range(segments):
            mesh.add_face(c, top[(i + 1) % segments], top[i])
    return mesh


def _ogive_mesh(name: str, z0: float, z1: float, radius: float,
                segments: int, stations: int = 16) -> Mesh:
    mesh = Mesh(name)
    rings: list[list[int]] = []
    for k in range(stations + 1):
        t = k / stations
        z = z0 + (z1 - z0) * t
        r = radius * max(0.0, 1.0 - t ** 1.65)
        ring = []
        if k == stations:
            tip = mesh.add_vertex((0.0, 0.0, z))
            ring = [tip] * segments
        else:
            for i in range(segments):
                a = 2 * math.pi * i / segments
                ring.append(mesh.add_vertex((r * math.cos(a),
                                             r * math.sin(a), z)))
        rings.append(ring)
    for k in range(stations):
        for i in range(segments):
            j = (i + 1) % segments
            if k == stations - 1:
                mesh.add_face(rings[k][i], rings[k][j], rings[k + 1][i])
            else:
                mesh.add_face(rings[k][i], rings[k][j], rings[k + 1][j])
                mesh.add_face(rings[k][i], rings[k + 1][j], rings[k + 1][i])
    c = mesh.add_vertex((0.0, 0.0, z0))
    for i in range(segments):
        mesh.add_face(c, rings[0][i], rings[0][(i + 1) % segments])
    return mesh


def _normal(p0, p1, p2):
    ux, uy, uz = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    vx, vy, vz = (p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2])
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    if mag == 0:
        return 0.0, 0.0, 0.0
    return nx / mag, ny / mag, nz / mag


def _degenerate_faces(mesh: Mesh) -> int:
    count = 0
    for a, b, c in mesh.faces:
        if _normal(mesh.vertices[a], mesh.vertices[b],
                   mesh.vertices[c]) == (0.0, 0.0, 0.0):
            count += 1
    return count


def _bounds_record(parts: list[CadPart]):
    xs, ys, zs = [], [], []
    for p in parts:
        for x, y, z in p.mesh.vertices:
            xs.append(x)
            ys.append(y)
            zs.append(z)
    return {
        "x_min_m": _num(min(xs)),
        "x_max_m": _num(max(xs)),
        "y_min_m": _num(min(ys)),
        "y_max_m": _num(max(ys)),
        "z_min_m": _num(min(zs)),
        "z_max_m": _num(max(zs)),
    }


def _mesh_bounds_record(mesh: Mesh):
    xs = [p[0] for p in mesh.vertices]
    ys = [p[1] for p in mesh.vertices]
    zs = [p[2] for p in mesh.vertices]
    return {
        "x_min_m": _num(min(xs)),
        "x_max_m": _num(max(xs)),
        "y_min_m": _num(min(ys)),
        "y_max_m": _num(max(ys)),
        "z_min_m": _num(min(zs)),
        "z_max_m": _num(max(zs)),
    }


def _add_check(checks: list[dict], name: str, ok: bool,
               pass_detail: str, fail_detail: str):
    checks.append({
        "name": name,
        "status": "pass" if ok else "fail",
        "detail": pass_detail if ok else fail_detail,
    })


def _engine_envelope_checks(parts: list[CadPart],
                            stages: list[CadPart]) -> list[dict]:
    checks = []
    engines = [p for p in parts if p.role == "engine_bell"]
    grouped: dict[float, list[CadPart]] = {}
    for e in engines:
        grouped.setdefault(round(e.z_max_m, 6), []).append(e)

    all_inside = True
    inside_failures = []
    for e in engines:
        stage = next((s for s in stages if abs(s.z_min_m - e.z_max_m) < 1e-6),
                     None)
        if stage is None:
            all_inside = False
            inside_failures.append(f"{e.name}: no matching stage base")
            continue
        cx, cy = _part_center_xy(e.mesh)
        if math.hypot(cx, cy) + e.diameter_m / 2 > stage.diameter_m / 2 + 1e-6:
            all_inside = False
            inside_failures.append(f"{e.name}: outside {stage.name} envelope")
    _add_check(checks, "engine_envelope", all_inside,
               "engine bells remain inside their stage diameter envelopes",
               "; ".join(inside_failures))

    clearance_ok = True
    clearance_failures = []
    for z_base, group in grouped.items():
        for i, a in enumerate(group):
            ax, ay = _part_center_xy(a.mesh)
            for b in group[i + 1:]:
                bx, by = _part_center_xy(b.mesh)
                dist = math.hypot(ax - bx, ay - by)
                required = (a.diameter_m + b.diameter_m) * 0.51
                if dist < required:
                    clearance_ok = False
                    clearance_failures.append(
                        f"{a.name}/{b.name} at z={z_base}: "
                        f"{dist:.2f} m < {required:.2f} m")
    _add_check(checks, "engine_clearance", clearance_ok,
               "same-stage engine bells have visual clearance",
               "; ".join(clearance_failures))
    return checks


def _set_axes_review_3d(ax, parts: list[CadPart]):
    bounds = _bounds_record(parts)
    x0, x1 = bounds["x_min_m"], bounds["x_max_m"]
    y0, y1 = bounds["y_min_m"], bounds["y_max_m"]
    z0, z1 = bounds["z_min_m"], bounds["z_max_m"]
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    radial_span = max(x1 - x0, y1 - y0) / 2
    r = max(radial_span * 1.55, 1.0)
    ax.set_xlim(cx - r, cx + r)
    ax.set_ylim(cy - r, cy + r)
    ax.set_zlim(z0, z1)
    try:
        ax.set_box_aspect((1, 1, 3.2))
    except AttributeError:
        pass


def _part_center_xy(mesh: Mesh):
    xs = [p[0] for p in mesh.vertices]
    ys = [p[1] for p in mesh.vertices]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def _slug(text: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9]+", "_", text).strip("_").lower()
    return slug or "part"


def _num(value):
    try:
        return round(float(value), 6)
    except (TypeError, ValueError):
        return value
