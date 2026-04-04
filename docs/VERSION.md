# Version History

## v0.4.0 — 2026-04-04

Renamed project from `worvai.util.nav_map` to `worv.util.nav_map`. Fixed bounding box computation on non-mesh prims.

### Breaking Changes

- **Module rename** — `worvai.util.nav_map` → `worv.util.nav_map`; Python package directory `worvai/` → `worv/`; update all `.kit` files and imports accordingly

### Bug Fixes

- **Empty bbox guard** — Skip prims whose `ComputeAlignedRange()` is empty before combining into the total bounding box

### Modified Files

- `config/extension.toml` — Module name and version bump to v0.4.0
- `docs/README.md` — All references updated from `worvai` to `worv`
- `ui_builder.py` — Added `IsEmpty()` check in `_calculate_selection_world_bounds()`

---

## v0.3.0 — 2026-02-15

Added slope-based post-processing to filter steep terrain from occupancy maps.

### Features

- **Slope-based terrain filtering** — Post-process occupied cells by casting downward rays and comparing surface normals against a configurable slope threshold; traversable ground is reclassified as free space
- **Max Traversable Slope UI** — New "Max Traversable Slope (°)" field in Occupancy Map Settings (0–90°, default 0 = disabled)

### Modified Files

- `config/extension.toml` — Version bump to v0.3.0
- `impl/omap_config.py` — Added `max_traversable_slope_degrees` field to `OmapConfig`
- `impl/omap_capture.py` — Added `_compute_slope_free_mask()` method; integrated slope mask into both PhysX and mesh collision generation paths; `_save_results()` applies mask before image export
- `ui_builder.py` — Added `get_max_traversable_slope_degrees()` getter and slope angle float field in omap section
- `extension.py` — Passes `max_traversable_slope_degrees` from UI to `OmapConfig`

---

## v0.2.0 — 2026-02-12

Integrated `isaacsim.asset.gen.omap` occupancy map generation into the extension.

### Features

- **Occupancy map generation** — 2D occupancy grid via PhysX raycasting with PNG + ROS YAML output
- **Unified area definition** — Shared origin, bounds, and cell size for both orthographic and occupancy map workflows
- **Positioning helpers** — "Center to Selection" and "Bound Selection" buttons to set area from selected prims
- **Rigid body handling** — Anonymous session layer strips RigidBodyAPI from prims during mesh collision generation, leaving the stage untouched
- **Exact ROS origin** — YAML output uses user-specified origin coordinates instead of grid-aligned computed origin
- **Separate action buttons** — Create Camera, Capture Orthographic Map, Generate Occupancy Map are independent

### New Files

- `impl/omap_config.py` — Frozen `OmapConfig` dataclass with origin, bounds, cell size, and output settings
- `impl/omap_capture.py` — `OmapCapture` engine with PhysX collision and mesh collision generation paths

### Modified Files

- `config/extension.toml` — Added dependencies for omap, physx, timeline, layers, core utils
- `ui_builder.py` — Redesigned with Area Definition, Orthographic Settings, Occupancy Map Settings sections
- `extension.py` — Wires both `OrthoMapCapture` and `OmapCapture` engines to the unified UI

---

## v0.1.0 — 2026-02-09

Initial release. Migrated orthographic capture from `isaacsim.util.ortho_capture` and redesigned following AGENTS.md coding standards.

### Features

- **Orthographic top-down capture** with configurable boundary region, camera height, and resolution
- **Tiled rendering** — Automatically splits captures exceeding 2048px into tile grids to avoid VRAM limitations, then stitches tiles into a single output image
- **GUI panel** — Accessible via Tools → Utilities → Navigation Map Generator with boundary coordinates, camera settings, and output directory controls
- **Programmatic API** — `OrthoMapCapture` class usable without UI for scripted workflows
- **Immutable configuration** — All config objects (`BoundaryRegion`, `TileGrid`, `OrthoMapConfig`) are frozen dataclasses

### Architecture

- `ortho_config.py` — Frozen dataclasses for capture configuration
- `ortho_capture.py` — Standalone capture engine with camera lifecycle and tiled rendering
- `ui_builder.py` — Separated UI construction with callback delegation
- `extension.py` — Thin wiring layer between engine and UI

### Migration Notes

Renamed from `isaacsim.util.ortho_capture`. The monolithic extension class was decomposed into four modules with clear separation of concerns. Output filenames changed from `ortho_capture_*.png` to `nav_map_*.png`.

