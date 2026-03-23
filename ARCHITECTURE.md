# Architecture

## Guiding Principle: Math and Rendering are Separate

Inspired by Clay's layout library and `bevy_diegetic`, this crate splits into two layers:

```
┌─────────────────────────────────────────────────────────┐
│  routing/  — Pure math (depends only on glam)           │
│                                                         │
│  Input:   RouteRequest (start, end, obstacles, params)  │
│  Output:  CableGeometry (points, tangents, arc lengths) │
│                                                         │
│  No Bevy. No ECS. No rendering. Fully testable.         │
└──────────────────────────┬──────────────────────────────┘
                           │
                     CableGeometry
                           │
┌──────────────────────────▼──────────────────────────────┐
│  plugin/  — Bevy integration (thin layer)               │
│                                                         │
│  Components:  Cable, CableEndpoint, AttachedTo          │
│  Observer:    on_geometry_computed                       │
│  Rendering:   tube mesh generation, debug gizmos        │
└─────────────────────────────────────────────────────────┘
```

The routing layer knows nothing about Bevy. The plugin layer knows nothing about how routes are computed. `CableGeometry` is the seam between them — a render-agnostic description of what was computed.

## Why This Matters

- **Testability**: Route math is tested with plain `#[test]` — no `App`, no `World`, no systems.
- **Multiple renderers**: Tube meshes, gizmo debug lines — all consume the same `CableGeometry`.
- **Multiple algorithms**: Catenary, A\*, orthogonal, composite — all produce the same `CableGeometry`.
- **Reusability**: The routing layer could be used outside Bevy (tools, headless validation, etc.).

## Module Structure

```
src/
├── lib.rs                     # pub use re-exports from both layers
│
├── routing/                   # Pure math — no Bevy dependency
│   ├── mod.rs                 # mod + pub use
│   ├── types.rs               # Anchor, Obstacle, CableGeometry, CableSegment, RouteRequest
│   ├── constants.rs           # Named constants (no magic values)
│   ├── enums.rs               # Solver, Planner, Curve enum dispatch
│   ├── solver.rs              # RouteSolver/CurveSolver/PathPlanner traits, Router compositor
│   ├── catenary.rs            # Catenary math functions + CatenarySolver
│   ├── pathfinding.rs         # 3D A* + AStarPlanner
│   └── orthogonal.rs          # Orthogonal routing + OrthogonalPlanner
│
└── plugin/                    # Bevy integration
    ├── mod.rs                 # CatenaryPlugin
    ├── components.rs          # Cable, CableEndpoint, AttachedTo, ComputedCableGeometry
    ├── systems.rs             # compute_cable_routes, on_geometry_computed, debug gizmos
    └── mesh.rs                # CableGeometry → Mesh (tube generation, caps, elbows)
```

## Data Flow

```
1. User spawns Cable entity with child CableEndpoint entities:
   - Each endpoint has a CableEnd (Start/End) and optional AttachedTo(entity)
   - Cable holds solver choice, obstacles, and resolution

2. compute_cable_routes system:
   - Resolves endpoint positions from AttachedTo GlobalTransforms
   - Calls solver.solve(request) via Solver enum dispatch
   - Writes CableGeometry into ComputedCableGeometry

3. on_geometry_computed observer:
   - Triggers on Insert of ComputedCableGeometry
   - Generates tube mesh from CableGeometry + CableMeshConfig
   - Creates or updates mesh child entity
```

## Dependency Strategy

The `routing/` module depends only on `glam` (which `bevy_math` re-exports, so types are compatible).

The `plugin/` module depends on `bevy`. It bridges glam types from `routing/` into the ECS world.

This means the routing module can be compiled and tested without pulling in the full Bevy dependency tree.
