# Trait Design

## The Problem

Cable routing isn't one algorithm — it's a family:
- **Catenary**: Natural hanging curve between two points. No obstacle awareness.
- **A\* pathfinding**: Finds waypoints around obstacles in a 3D grid.
- **Orthogonal**: Axis-aligned segments with 90-degree bends.
- **Composite**: A\* finds waypoints, catenary fills each segment.

These need a common interface so the Bevy plugin doesn't care which algorithm produced the geometry.

## Three Traits, One Compositor

### `RouteSolver` — The Main Interface

The primary trait. Everything the plugin touches goes through this.

```rust
pub trait RouteSolver: Send + Sync {
    fn solve(&self, request: &RouteRequest) -> CableGeometry;
}
```

Simple solvers implement this directly. `CatenarySolver` ignores obstacles and computes a direct curve. The plugin stores `Box<dyn RouteSolver>` and doesn't know or care what's inside.

### `PathPlanner` — High-Level Waypoint Finding

Determines the structural path: which waypoints the cable passes through to avoid obstacles.

```rust
pub trait PathPlanner: Send + Sync {
    fn plan(&self, start: Vec3, end: Vec3, obstacles: &[Obstacle]) -> Vec<Vec3>;
}
```

Returns an ordered list of waypoints (always includes start and end). Implementations:
- `DirectPlanner` — returns `[start, end]` (no obstacle avoidance)
- `AStarPlanner` — 3D grid A\* around obstacles
- `OrthogonalPlanner` — axis-aligned waypoints with snapping angles

### `CurveSolver` — Low-Level Segment Geometry

Generates smooth geometry between two waypoints. No obstacle awareness — that's the planner's job.

```rust
pub trait CurveSolver: Send + Sync {
    fn solve_segment(&self, start: Vec3, end: Vec3, resolution: u32) -> CableSegment;
}
```

Implementations:
- `CatenarySolver` — hanging catenary curve
- `LinearSolver` — straight line (for orthogonal segments)
- Future: `SplineSolver`, `VerletSolver`, etc.

### `Router` — The Compositor

Composes a `PathPlanner` and `CurveSolver` into a `RouteSolver`.

```rust
pub struct Router {
    planner: Box<dyn PathPlanner>,
    curve: Box<dyn CurveSolver>,
}

impl RouteSolver for Router {
    fn solve(&self, request: &RouteRequest) -> CableGeometry {
        let waypoints = self.planner.plan(request.start, request.end, &request.obstacles);

        let segments = waypoints.windows(2)
            .map(|pair| self.curve.solve_segment(pair[0], pair[1], request.resolution))
            .collect();

        CableGeometry::from_segments(segments)
    }
}
```

## Why Traits (Not Generics, Not Enums)

### Option considered: Generics (`Cable<S: RouteSolver>`)

Rejected. Different cables need different solvers, and they must coexist in the same ECS queries. Generics would require separate component types per solver, fragmenting queries.

### Option considered: Enum dispatch

```rust
enum Solver { Catenary(CatenarySolver), AStar(AStarSolver), ... }
```

Rejected as the primary interface. Enums are a closed set — users can't add custom solvers. However, a `BuiltinSolver` enum could wrap the shipped implementations for convenience, implementing `RouteSolver` via delegation.

### Chosen: Trait objects (`Box<dyn RouteSolver>`)

- Different cables use different solvers — runtime polymorphism is the natural fit
- Route computation is not a hot inner loop — vtable overhead is negligible
- Users can implement custom `RouteSolver` for their own algorithms
- The trait is the seam between routing math and the plugin layer

## Composition Examples

### Direct catenary (no obstacles)
```rust
let solver = CatenarySolver::new(slack, gravity);
// CatenarySolver implements both CurveSolver and RouteSolver
```

### A\* waypoints with catenary fill
```rust
let solver = Router::new(
    AStarPlanner::new(grid_size, margin),
    CatenarySolver::new(slack, gravity),
);
```

### Orthogonal routing with straight segments
```rust
let solver = Router::new(
    OrthogonalPlanner::new(snap_angles),
    LinearSolver,
);
```

### Orthogonal routing with catenary fill between bends
```rust
let solver = Router::new(
    OrthogonalPlanner::new(snap_angles),
    CatenarySolver::new(slack, gravity),
);
```

## Standalone Math Functions

The trait implementations are thin wrappers around standalone functions. Users who don't need the trait abstraction can call the math directly:

```rust
// Raw catenary evaluation
let y = catenary::evaluate(x, catenary_param);

// Find catenary parameter from endpoint constraints
let a = catenary::solve_parameter(horizontal_dist, vertical_dist, cable_length)?;

// Sample a 3D catenary curve
let segment = catenary::sample_3d(start, end, slack, gravity_dir, resolution);
```

This means the catenary math is usable without routing, without traits, without Bevy — just `f32` in, `Vec3` out.
