//! `bevy_catenary` — Physics-based 3D cable routing for Bevy.
//!
//! Provides catenary curves, A* pathfinding, and orthogonal routing for 3D cables,
//! with a clean separation between route math and Bevy rendering.
//!
//! # Architecture
//!
//! The crate is split into two layers:
//! - [`routing`] — Pure math module (depends only on `glam`). Produces [`CableGeometry`].
//! - [`plugin`] — Thin Bevy integration. Consumes [`CableGeometry`] for rendering.
//!
//! # Quick Start
//!
//! ```ignore
//! use bevy::prelude::*;
//! use bevy_catenary::*;
//!
//! App::new()
//!     .add_plugins(DefaultPlugins)
//!     .add_plugins(CatenaryPlugin)
//!     .add_systems(Startup, setup)
//!     .run();
//!
//! fn setup(mut commands: Commands) {
//!     commands.spawn(Cable {
//!         start: Vec3::new(-2.0, 2.0, 0.0),
//!         end: Vec3::new(2.0, 2.0, 0.0),
//!         solver: Box::new(CatenarySolver::new().with_slack(1.3)),
//!         obstacles: vec![],
//!         resolution: 0,
//!     });
//! }
//! ```

mod plugin;
mod routing;

// Routing types
// Bevy plugin
pub use plugin::Cable;
pub use plugin::CableDebugEnabled;
pub use plugin::CableGizmoGroup;
pub use plugin::CatenaryPlugin;
pub use plugin::ComputedCableGeometry;
pub use plugin::TubeMeshConfig;
pub use plugin::generate_tube_mesh;
// Solvers
pub use routing::AStarPlanner;
pub use routing::Anchor;
pub use routing::CableGeometry;
pub use routing::CableSegment;
pub use routing::CatenarySolver;
// Routing traits
pub use routing::CurveSolver;
// Constants
pub use routing::DEFAULT_GRAVITY;
pub use routing::DEFAULT_RESOLUTION;
pub use routing::DEFAULT_SLACK;
pub use routing::DirectPlanner;
pub use routing::LinearSolver;
pub use routing::Obstacle;
pub use routing::OrthogonalPlanner;
pub use routing::PathPlanner;
pub use routing::RouteRequest;
pub use routing::RouteSolver;
pub use routing::Router;
// Standalone catenary math
pub use routing::evaluate;
pub use routing::sample_3d;
pub use routing::solve_parameter;
