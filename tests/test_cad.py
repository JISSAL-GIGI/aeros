"""Concept CAD export invariants."""

import json
import math

from aeros.cad import build_vehicle_cad, export_vehicle_cad, inspect_cad_parts
from aeros.cli import main
from aeros.validate import falcon9_expendable


def test_build_vehicle_cad_contains_stages_fairing_and_engines():
    v = falcon9_expendable(10_000)
    parts = build_vehicle_cad(v, segments=16)
    roles = [p.role for p in parts]
    assert roles.count("propulsive_stage") == 2
    assert roles.count("payload_accommodation") == 1
    assert roles.count("engine_bell") == 10
    assert roles.count("thrust_interface") == 2
    assert roles.count("structural_interface") == 1
    assert roles.count("payload_interface") == 1
    assert all(len(p.mesh.vertices) > 0 for p in parts)
    assert all(len(p.mesh.faces) > 0 for p in parts)


def test_engine_cluster_uses_center_engine_for_nine_engine_stage():
    v = falcon9_expendable(10_000)
    parts = build_vehicle_cad(v, segments=16)
    first_stage = [p for p in parts if p.role == "propulsive_stage"][0]
    engines = [
        p for p in parts
        if p.role == "engine_bell"
        and math.isclose(p.z_max_m, first_stage.z_min_m)
    ]
    centers = []
    for e in engines:
        xs = [v[0] for v in e.mesh.vertices]
        ys = [v[1] for v in e.mesh.vertices]
        centers.append((sum(xs) / len(xs), sum(ys) / len(ys)))

    center_count = sum(math.hypot(x, y) < 1e-6 for x, y in centers)
    ring_count = sum(math.hypot(x, y) > 0.5 for x, y in centers)
    assert center_count == 1
    assert ring_count == 8


def test_cad_review_checks_pass_for_reference_vehicle():
    v = falcon9_expendable(10_000)
    review = inspect_cad_parts(build_vehicle_cad(v, segments=16))

    assert review["status"] == "pass"
    assert {c["name"] for c in review["checks"]} >= {
        "stage_stack_continuity",
        "interface_markers",
        "engine_envelope",
        "engine_clearance",
        "payload_fairing",
    }


def test_export_vehicle_cad_writes_traceable_files(tmp_path):
    v = falcon9_expendable(10_000)
    files = export_vehicle_cad(v, tmp_path, segments=16, prefix="falcon_test")

    obj = files["obj"].read_text(encoding="utf-8")
    stl = files["stl"].read_text(encoding="utf-8")
    scad = files["scad"].read_text(encoding="utf-8")
    manifest = json.loads(files["manifest"].read_text(encoding="utf-8"))
    review = json.loads(files["review_json"].read_text(encoding="utf-8"))

    assert "g stage_1_merlin_1d" in obj
    assert "g stage_1_to_2_joint" in obj
    assert obj.count("\nv ") > 100
    assert stl.startswith("solid falcon_test")
    assert "module stage_shell" in scad
    assert manifest["name"] == "Falcon 9 Block 5 (expendable)"
    assert manifest["cad_fidelity"] == "concept_mesh"
    assert manifest["glow_kg"] == v.glow_kg
    assert len(manifest["parts"]) == 17
    assert manifest["official_need_alignment"]
    assert files["review_png"].exists()
    assert review["status"] == "pass"


def test_cad_cli_exports_review_artifacts(tmp_path):
    out = tmp_path / "cli_cad"

    main([
        "cad",
        "1500 kg to 400 km",
        "--out",
        str(out),
        "--segments",
        "12",
    ])

    assert (out / "vehicle.obj").exists()
    assert (out / "vehicle_cad_review.png").exists()
    review = json.loads((out / "vehicle_cad_review.json").read_text())
    assert review["status"] == "pass"
