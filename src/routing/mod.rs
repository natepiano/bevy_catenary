//! Pure math routing module for cable geometry computation.
//!
//! This module depends only on `glam` (via `bevy::math`) — no Bevy ECS, no rendering.
//! It produces [`CableGeometry`] from a [`RouteRequest`] via the [`RouteSolver`] trait.

mod catenary;
mod constants;
#[allow(
    clippy::used_underscore_binding,
    reason = "false positive on enum variant fields"
)]
mod enums;
mod orthogonal;
mod pathfinding;
mod solver;
mod types;

pub use catenary::CatenarySolver;
pub use catenary::evaluate;
pub use catenary::sample_3d;
pub use catenary::solve_parameter;
pub use constants::DEFAULT_GRAVITY;
pub use constants::DEFAULT_RESOLUTION;
pub use constants::DEFAULT_SLACK;
pub use constants::MIN_SEGMENT_LENGTH;
pub use enums::CurveKind;
pub use enums::PathStrategy;
pub use enums::Solver;
pub use orthogonal::AxisOrder;
pub use orthogonal::OrthogonalPlanner;
pub use pathfinding::AStarPlanner;
pub use solver::CurveSolver;
pub use solver::DirectPlanner;
pub use solver::LinearSolver;
pub use solver::PathPlanner;
pub use solver::RouteSolver;
pub use solver::Router;
pub use types::Anchor;
pub use types::CableGeometry;
pub use types::CableSegment;
pub use types::Obstacle;
pub use types::RouteRequest;
