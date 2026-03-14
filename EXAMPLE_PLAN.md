# Example Plan: `cable_playground`

A single interactive example that demonstrates every routing algorithm and lets you inspect cables from any angle using `bevy_panorbit_camera_ext`.

## Scene Layout

```
        ┌───┐                    ┌───┐
        │ A │────── catenary ────│ B │
        └───┘    (slack 1.2)     └───┘
         │                        │
         │ catenary               │ catenary
         │ (slack 1.5)            │ (slack 1.0, taut)
         │                        │
        ┌───┐                    ┌───┐
        │ C │── orthogonal ──────│ D │
        └───┘                    └───┘
                  ┌─────────┐
                  │ obstacle │
                  └─────────┘
        ┌───┐                    ┌───┐
        │ E │── A* + catenary ───│ F │
        └───┘   (routes around   └───┘
                  obstacle)
```

**Nodes**: 6 cubes (A–F) at varying heights and positions. Each is a selectable, draggable mesh.

**Cables**: 5 cables demonstrating different configurations:
1. **A→B**: Standard catenary, moderate slack
2. **A→C**: Loose catenary, heavy sag
3. **B→D**: Taut catenary, minimal sag
4. **C→D**: Orthogonal routing with 90-degree bends
5. **E→F**: Composite routing — A\* pathfinding around the obstacle box, catenary fill between waypoints

**Obstacle**: A translucent box between E and F. Cable 5 routes around it. Visible so you can see why the cable bends.

**Ground**: A large flat plane for visual grounding and click-to-deselect.

## Camera & Navigation

Uses `bevy_panorbit_camera_ext` with the `visualization` feature:

- **Orbit**: Middle-mouse drag (or two-finger trackpad)
- **Pan**: Shift + middle-mouse (or two-finger drag)
- **Zoom**: Scroll wheel (or pinch)
- **Click node**: Select it, zoom-to-fit framing the node and its connected cables
- **Click cable**: Select it, zoom-to-fit framing the cable's bounding box
- **Click ground**: Deselect, zoom-to-fit the full scene
- **H key**: Home — animate back to the default overview

The camera starts at an elevated 3/4 angle showing the full scene.

## Interactive Controls

| Key | Action |
|-----|--------|
| `H` | Home — zoom to fit entire scene |
| `D` | Toggle debug gizmos (waypoints, tangents, segment boundaries) |
| `G` | Zoom-to-fit hovered entity |
| `1`–`5` | Select cable 1–5 and zoom to it |
| `Up/Down` | Adjust slack on selected catenary cable |
| `Space` | Cycle solver on selected cable (catenary → orthogonal → composite → catenary) |

## Debug Visualization (Toggle with D)

When debug gizmos are on, each cable shows:
- **Green dots** at waypoints (the structural points from the planner)
- **Yellow lines** showing tangent vectors at sample points
- **Red dots** at segment boundaries (where one `CableSegment` meets the next)
- **Blue wireframe** around obstacles (the AABBs the planner sees)

This makes it easy to understand what the routing algorithms are doing — you can orbit around and see the 3D structure.

## What the Example Exercises

From the `routing/` math layer:
- `CatenarySolver` with varying slack values
- `OrthogonalPlanner` + `LinearSolver` via `Router`
- `AStarPlanner` + `CatenarySolver` via `Router`
- `RouteRequest` with and without obstacles
- `CableGeometry` output consumed by two renderers simultaneously (gizmos + meshes)

From the `plugin/` layer:
- `Cable` component with different solver configurations
- `ComputedCableGeometry` change detection
- `CatenaryPlugin` registration alongside `PanOrbitCameraExtPlugin`

From rendering:
- Gizmo debug renderer reading `CableGeometry`
- (Phase 5) Tube mesh renderer reading the same `CableGeometry`

## Dependencies for the Example

```toml
[dev-dependencies]
bevy = { version = "0.18", features = ["default"] }
bevy_panorbit_camera_ext = { path = "../bevy_panorbit_camera_ext", features = ["visualization"] }
```

## File

```
examples/
└── cable_playground.rs
```

Single file. No example-specific modules. The example should be self-contained and readable top-to-bottom as a reference for how to use the crate.

## Startup Systems

```
setup_camera       — spawn PanOrbitCamera at overview position
setup_scene        — ground plane, directional light, ambient light
setup_nodes        — spawn 6 cubes with transforms and picking
setup_cables       — spawn 5 cables with different solvers
setup_obstacle     — spawn translucent obstacle box
setup_ui           — instruction text overlay (like extras example)
```

## Runtime Systems

```
handle_node_click     — select node, zoom-to-fit
handle_cable_click    — select cable, zoom-to-fit
handle_ground_click   — deselect, zoom-to-fit scene
handle_keyboard       — debug toggle, cable cycling, slack adjustment
update_cable_anchors  — if a node is dragged, update connected cables' anchors
toggle_debug_gizmos   — render/hide waypoints, tangents, segment boundaries
```

## Build Order

This example is built incrementally alongside the implementation phases:

- **After Phase 1** (math only): No example yet, just `cargo nextest run`
- **After Phase 2** (plugin + gizmos): First version of the example with catenary cables rendered as gizmo lines
- **After Phase 3** (pathfinding): Add obstacle and composite cable to the example
- **After Phase 4** (orthogonal): Add orthogonal cable to the example
- **After Phase 5** (meshes): Replace gizmo lines with tube meshes, keep gizmos as debug toggle
