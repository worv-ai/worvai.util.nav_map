# Navigation Map Generator

> `worv.util.nav_map` — Isaac Sim extension for generating navigation maps using orthographic capture and occupancy grid extraction.

## Overview

Navigation Map Generator creates high-resolution top-down orthographic images and 2D occupancy grids from Isaac Sim scenes. These serve as navigation maps for autonomous robots, providing bird's-eye-view representations of the environment.

The extension supports:

- **Orthographic Capture** — Creates a USD orthographic camera and renders a top-down image of a defined scene region.
- **Tiled Rendering** — Automatically splits large captures into tiles (max 2048px per tile) to avoid VRAM limitations, then stitches them into a single output image.
- **Occupancy Map** — 2D occupancy grid via PhysX raycasting with PNG + ROS YAML output, including slope-based terrain filtering.

## Installation

Enable `worv.util.nav_map` in your Isaac Sim `.kit` file or the Extension Manager.

```toml
[dependencies]
"worv.util.nav_map" = {}
```

The extension directory must be registered as an extension search path. If placed under `extsUser/`, Isaac Sim discovers it automatically.

## Usage

### GUI

1. Open **Tools → Utilities → Navigation Map Generator**
2. Configure the **Area Definition** — Origin, lower/upper bounds, and cell size
   - **Center to Selection** / **Bound Selection** — Set area from selected prims
3. **Orthographic Settings**:
   - **Z Height** — Camera altitude above the scene (default: 50.0 m)
   - **Meters per Pixel** — Spatial resolution (default: 0.01 m/px)
   - **Camera Path** — USD prim path for the camera (default: `/World/OrthoCamera`)
4. **Occupancy Map Settings**:
   - **Max Traversable Slope** — Slope threshold in degrees for terrain filtering (0 = disabled)
   - **Output Directory** for saved maps (default: `~/navigation_maps`)
5. Click **CREATE CAMERA** / **CAPTURE** for orthographic maps
6. Click **GENERATE** for occupancy maps

### Programmatic API

The capture engine is usable without the UI:

```python
from worv.util.nav_map import BoundaryRegion, OrthoMapConfig, OrthoMapCapture

boundary = BoundaryRegion(x_min=-20.0, x_max=20.0, y_min=-20.0, y_max=20.0)
tile_grid = OrthoMapConfig.compute_tile_grid(boundary, meters_per_pixel=0.01)

config = OrthoMapConfig(
    boundary=boundary,
    camera_height_meters=50.0,
    meters_per_pixel=0.01,
    camera_prim_path="/World/OrthoCamera",
    output_directory="/tmp/nav_maps",
    tile_grid=tile_grid,
)

capture = OrthoMapCapture()
capture.create_camera(config)
filepath = await capture.capture_async()
capture.destroy()
```

## Architecture

```
worv.util.nav_map/
├── config/
│   └── extension.toml          # Extension manifest and dependencies
├── docs/
│   ├── README.md               # This file
│   └── VERSION.md              # Version history
└── worv/util/nav_map/
    ├── __init__.py              # Public API exports
    ├── extension.py             # NavigationMapExtension — Kit entry point
    ├── ui_builder.py            # NavigationMapUIBuilder — UI widget construction
    └── impl/
        ├── __init__.py
        ├── ortho_config.py      # Frozen dataclasses: BoundaryRegion, TileGrid, OrthoMapConfig
        ├── ortho_capture.py     # OrthoMapCapture — camera creation + tiled rendering
        ├── omap_config.py       # Frozen OmapConfig: origin, bounds, cell size, output
        └── omap_capture.py      # OmapCapture — PhysX/mesh collision occupancy generation
```

### Module Responsibilities

| Module | Class | Role |
|---|---|---|
| `ortho_config` | `BoundaryRegion` | Axis-aligned bounding box in world meters |
| `ortho_config` | `TileGrid` | Computed tiling layout (tile count, dimensions, resolution) |
| `ortho_config` | `OrthoMapConfig` | Complete immutable capture configuration |
| `ortho_capture` | `OrthoMapCapture` | Camera lifecycle, tiled rendering, image stitching |
| `omap_config` | `OmapConfig` | Origin, bounds, cell size, slope threshold, output settings |
| `omap_capture` | `OmapCapture` | PhysX/mesh collision occupancy grid generation |
| `ui_builder` | `NavigationMapUIBuilder` | Omni.UI panel with area/camera/omap controls |
| `extension` | `NavigationMapExtension` | Thin wiring between engines and UI |

### Design Principles

- **Immutability** — All configuration objects are `@dataclass(frozen=True)`
- **Separation of Concerns** — Capture engines are standalone and usable without UI
- **Type Safety** — Full type hints on all signatures, attributes, and return types

## Dependencies

| Package | Purpose |
|---|---|
| `isaacsim.asset.gen.omap` | Occupancy map generation primitives |
| `isaacsim.core.utils` | Core Isaac Sim utilities |
| `isaacsim.gui.components` | ScrollingWindow, menu helpers, UI builders |
| `omni.kit.menu.utils` | Tools menu integration |
| `omni.kit.usd.layers` | Anonymous session layer for non-destructive edits |
| `omni.physx` | PhysX raycasting for occupancy maps |
| `omni.replicator.core` | Render products and RGB annotators |
| `omni.timeline` | Simulation timeline control |
| `omni.ui` | Widget framework |
| `omni.usd` | USD stage access |
| `PIL` (Pillow) | Image assembly and saving |
| `numpy` | Pixel buffer manipulation |

## License

Internal use — see repository for details.
