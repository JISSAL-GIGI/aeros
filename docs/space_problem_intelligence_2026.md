# AEROS Space Problem Intelligence - 2026 CAD Direction

This note records why the next AEROS layer is CAD generation and structured
engineering data, rather than a purely visual rocket model.

## Official Signal

NASA's FY26 Civil Space Shortfall Prioritization defines a shortfall as a
technology area requiring further development to meet future exploration,
science, and other mission needs. The 2026 process consolidated the prior
187 shortfalls into 32 broader categories after stakeholder feedback.

Relevant high-ranked needs include:

- SF25, rank 3: provide on-board advanced computing capabilities for space
  operations.
- SF28, rank 9: autonomously monitor, inspect, maintain, and repair space
  assets.
- SF32, rank 14: develop an affordable and resilient supply chain for space
  exploration.
- SF04, rank 16: deploy, assemble, and construct complex structures on the
  lunar surface.
- SF22, rank 31: provide ground support infrastructure for launch cadence.

Sources:

- NASA Civil Space Shortfalls: https://www.nasa.gov/spacetechpriorities/
- NASA FY26 Civil Space Shortfall Prioritization:
  https://www.nasa.gov/wp-content/uploads/2026/05/fy26-civil-space-shortfall-prioritization.pdf
- NASA release on 2026 technology priorities:
  https://www.nasa.gov/technology/nasa-releases-technology-priorities-to-energize-space-industry/

NASA's 2026 State-of-the-Art Small Spacecraft Technology report also shows
continued growth in small spacecraft and larger small-spacecraft
constellations, with updates across power, propulsion, GNC, structures,
thermal, avionics, communications, launch/integration/deployment, and
deorbit systems.

Source:

- NASA Small Spacecraft Technology State of the Art:
  https://www.nasa.gov/smallsat-institute/sst-soa/

ESA's Technology Strategy points in the same direction: faster technology
development, digital workflows, MBSE, modularity, standardised interfaces,
Digital Design-2-Produce, advanced manufacturing, and in-orbit servicing and
construction.

Sources:

- ESA Strategy 2040: https://www.esa.int/About_Us/ESA_Strategy_2040
- ESA Technology Strategy:
  https://www.esa.int/Enabling_Support/Space_Engineering_Technology/ESA_Technology_Strategy_for_Europe_s_future_in_space

## Product Decision

AEROS should not jump from text to arbitrary CAD. The valuable wedge is:

Mission -> physics-sized architecture -> traceable engineering geometry ->
machine-readable manifest.

That path directly supports:

- faster early design iteration,
- reviewable model-based engineering data,
- better design-to-manufacturing handoff,
- future automated inspection and repair workflows,
- future modular spacecraft and launch system libraries.

## Implemented Step

The first CAD layer exports:

- `vehicle.obj` grouped by stage, fairing, and engine bells,
- `vehicle.stl` for quick mesh inspection and 3D printing previews,
- `vehicle.scad` as a parametric OpenSCAD recipe,
- `vehicle_manifest.json` with masses, dimensions, decision traceability, and
  official-need alignment,
- `vehicle_cad_review.png` with side, engine-bay, and isometric views for
  human visual inspection,
- `vehicle_cad_review.json` with automated checks for mesh presence, triangle
  quality, stage continuity, interface markers, engine envelope, engine
  clearance, and fairing placement.

The exporter now includes conceptual thrust-transfer plates, interstage
interfaces, payload adapters, and clustered engine layouts such as the
one-center-plus-eight-ring arrangement used by Falcon 9-class first stages.

This is not fabrication-ready CAD. It is the first bridge from verified
analysis to generated geometry. The next layer should add interface objects,
loads, manufacturing constraints, and eventually STEP export through an
optional CAD kernel.
