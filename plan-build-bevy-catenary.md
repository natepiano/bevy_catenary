# Build bevy_catenary: Full Implementation

## Overview

Build bevy_catenary from empty scaffold to working crate with catenary cables, A* pathfinding, orthogonal routing, tube mesh rendering, and an interactive example using bevy_panorbit_camera_ext. All work follows the design documents in the project root.

## Proposed Solution

### Core Approach
Two-layer architecture inspired by Clay and bevy_diegetic:
- `routing/` — pure math module (glam only), produces `CableGeometry`
- `plugin/` — thin Bevy integration consuming `CableGeometry`

Three traits (`RouteSolver`, `PathPlanner`, `CurveSolver`) composed via `Router`.

### Key Design Principles
- **Math/render separation**: Route computation is testable without Bevy
- **Trait-based extensibility**: `Box<dyn RouteSolver>` for runtime solver selection
- **Composition over inheritance**: `Router` composes any planner + any curve solver
- **Standalone functions**: Raw math callable without traits

## Implementation Strategy

### Phase 1: Foundation + Catenary Math
**Goal**: Core types, traits, catenary math — all testable with `cargo nextest run`

**Files**:
- `src/routing/types.rs` — Anchor, Obstacle, CableSegment, CableGeometry, RouteRequest
- `src/routing/constants.rs` — named constants
- `src/routing/solver.rs` — RouteSolver, CurveSolver, PathPlanner traits, Router, DirectPlanner, LinearSolver
- `src/routing/catenary.rs` — catenary math + CatenarySolver
- `src/routing/mod.rs` — module wiring
- `src/lib.rs` — pub use re-exports
- `Cargo.toml` — add glam dependency

**Validation**: `cargo build && cargo nextest run`

### Phase 2: Bevy Plugin + Gizmo Debug Rendering
**Goal**: Cable component, compute system, gizmo visualization

**Files**:
- `src/plugin/components.rs` — Cable, ComputedCableGeometry
- `src/plugin/systems.rs` — compute_cable_routes, render_cable_gizmos
- `src/plugin/mod.rs` — CatenaryPlugin
- `Cargo.toml` — add bevy dependency

**Validation**: `cargo build` (plugin compiles)

### Phase 3: A* Pathfinding
**Goal**: 3D grid-based A* around obstacles

**Files**:
- `src/routing/pathfinding.rs` — AStarPlanner

**Validation**: `cargo nextest run` (pathfinding tests pass)

### Phase 4: Orthogonal Routing
**Goal**: Axis-aligned routing with 90-degree bends

**Files**:
- `src/routing/orthogonal.rs` — OrthogonalPlanner

**Validation**: `cargo nextest run`

### Phase 5: Tube Mesh Generation
**Goal**: Procedural tube meshes from CableGeometry

**Files**:
- `src/plugin/mesh.rs` — CableGeometry → Bevy Mesh

**Validation**: `cargo build`

### Phase 6: Example
**Goal**: cable_playground example with panorbit camera

**Files**:
- `examples/cable_playground.rs`
- `Cargo.toml` — add dev-dependencies

**Validation**: `cargo build --example cable_playground`

## Testing Strategy

### Unit Tests (in routing module)
- Catenary parameter solving: convergence, edge cases (vertical, zero slack)
- Catenary sampling: endpoint accuracy, symmetry, arc length monotonicity
- A* pathfinding: single obstacle, multiple obstacles, no-path fallback
- Orthogonal planning: axis snapping, bend placement
- Router composition: planner + curve solver integration

### Validation Commands
```bash
cargo build --workspace
cargo +nightly fmt
cargo nextest run
cargo build --example cable_playground
```

## Risk Assessment

1. **Catenary Newton's method divergence**
   - Likelihood: Medium
   - Mitigation: Clamp iterations, fallback to parabolic approximation

2. **A* performance on large obstacle sets**
   - Likelihood: Low (cable scenes are small)
   - Mitigation: Bounded grid size, early termination

3. **bevy_panorbit_camera_ext API compatibility**
   - Likelihood: Low (same workspace, controlled dependency)
   - Mitigation: Reference extras.rs example directly

## Success Criteria

- [ ] `cargo build` succeeds with zero warnings
- [ ] `cargo nextest run` passes all tests
- [ ] `cargo build --example cable_playground` compiles
- [ ] Example runs and shows cables with panorbit camera navigation
- [ ] Catenary cables visibly sag with gravity
- [ ] A* cables route around obstacles
- [ ] Orthogonal cables show 90-degree bends
- [ ] Debug gizmos toggle shows waypoints and tangents
- [ ] `cargo +nightly fmt` produces no changes
- [ ] No `#[allow(dead_code)]` annotations

## Design Review Skip Notes

*This section will be populated during design reviews.*
