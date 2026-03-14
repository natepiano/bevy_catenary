# Implementation Plan

## Phase 1: Foundation + Catenary

The minimum viable crate. Pure math, fully testable, no rendering.

### Deliverables
1. `routing/types.rs` — `Anchor`, `Obstacle`, `CableSegment`, `CableGeometry`, `RouteRequest`
2. `routing/constants.rs` — named constants
3. `routing/solver.rs` — `RouteSolver`, `CurveSolver`, `PathPlanner` traits
4. `routing/catenary.rs` — standalone catenary math functions + `CatenarySolver`
5. Tests covering:
   - Catenary parameter solving (Newton's method convergence)
   - 3D catenary sampling (symmetry, endpoint accuracy, arc length)
   - Edge cases: vertical cables, zero slack, very long cables

### Catenary Math Detail

The 3D catenary problem:
1. Project the two endpoints onto a 2D plane containing both points and the gravity vector
2. In that plane, solve `y = a * cosh(x/a)` for parameter `a` given:
   - Horizontal distance between endpoints
   - Vertical distance between endpoints
   - Cable length (= straight-line distance * slack factor)
3. Use Newton's method to find `a`
4. Sample points along the curve in 2D
5. Map sampled points back to 3D using the projection basis

Public functions:
- `evaluate(x, a) -> f32` — evaluate catenary at a point
- `solve_parameter(h_dist, v_dist, cable_length) -> Option<f32>` — find `a`
- `sample_3d(start, end, slack, gravity, resolution) -> CableSegment` — the main entry point

### What This Phase Does NOT Include
- No Bevy plugin (no components, no systems)
- No rendering (no meshes, no gizmos)
- No obstacles or pathfinding
- No orthogonal routing

This phase is pure math with `cargo nextest run`.

---

## Phase 2: Bevy Plugin + Debug Rendering

Wire the math into Bevy so cables are visible.

### Deliverables
1. `plugin/components.rs` — `Cable`, `ComputedCableGeometry`
2. `plugin/systems.rs` — `compute_cable_routes` system with `Changed<Cable>` detection
3. `plugin/mod.rs` — `CatenaryPlugin` registration
4. `plugin/gizmos.rs` — debug gizmo rendering of `CableGeometry` (polyline through points)
5. Example: `examples/catenary.rs` — two cubes connected by a catenary cable

### System Design
```
compute_cable_routes (Update)
  Query<(&Cable, &mut ComputedCableGeometry), Changed<Cable>>
  → solver.solve(request) → write geometry

render_cable_gizmos (Update, after compute)
  Query<(&ComputedCableGeometry, &GlobalTransform)>
  → draw polyline through segment points
```

---

## Phase 3: Pathfinding

Obstacle-aware routing. A\* in a 3D voxel grid.

### Deliverables
1. `routing/pathfinding.rs` — `AStarPlanner` implementing `PathPlanner`
2. `routing/solver.rs` — `Router` compositor, `DirectPlanner`, `LinearSolver`
3. Tests covering:
   - Routing around a single obstacle
   - Routing around multiple obstacles
   - No-path-found fallback (direct line)
   - Grid resolution effects on path quality

### A\* Implementation Detail

3D grid-based A\*:
1. Voxelize the space between start and end (with margin)
2. Mark cells occupied by obstacles (AABB-transformed into grid coords)
3. Run A\* with 26-connectivity (all neighbors including diagonals)
4. Smooth the resulting path (remove redundant collinear waypoints)
5. Return waypoints

Heuristic: Euclidean distance (admissible in 3D).

The `Router` then feeds these waypoints to a `CurveSolver` (catenary or linear) for segment geometry.

### Composition Unlocked
After this phase, users can write:
```rust
Router::new(
    AStarPlanner::new(grid_size, margin),
    CatenarySolver::new(slack, gravity),
)
```

---

## Phase 4: Orthogonal Routing

Axis-aligned cable paths with 90-degree bends. Think circuit board traces in 3D.

### Deliverables
1. `routing/orthogonal.rs` — `OrthogonalPlanner` implementing `PathPlanner`
2. Tests covering axis snapping, bend placement, obstacle avoidance

### Algorithm Sketch
1. From start, extend in the direction closest to a configured snap angle
2. When blocked by an obstacle (or when the angle to the endpoint changes significantly), insert a 90-degree bend
3. Continue until reaching the end
4. Optimize: remove unnecessary bends, straighten segments

---

## Phase 5: Mesh Generation

Procedural tube meshes from `CableGeometry`.

### Deliverables
1. `plugin/mesh.rs` — `CableGeometry` → Bevy `Mesh`
2. Tube generation using rotation-minimizing frames (RMF) to avoid Frenet frame twisting
3. Configurable: tube radius, cross-section resolution, UV mapping
4. LOD: reduce cross-section segments and sample points at distance

### Mesh Algorithm
For each `CableSegment`:
1. Walk the points and tangents
2. Compute a rotation-minimizing frame at each point (parallel transport)
3. Place a cross-section circle at each frame
4. Connect adjacent circles with triangle strips
5. Generate UVs using arc length (U) and angle around cross-section (V)

---

## Phase 6: Polish

- Running light material/shader (animated UV offset using arc length)
- Connection system (drag-to-connect with valid/invalid feedback)
- Dynamic response (verlet integration for cable sway when endpoints move)
- Spatial query integration (auto-discover obstacles from Bevy's physics world)
