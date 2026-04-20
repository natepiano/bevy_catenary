//! `bevy_catenary` — Physics-based 3D cable routing for Bevy.
//!
//! Provides catenary curves, A* pathfinding, and orthogonal routing for 3D cables,
//! with a clean separation between route math and Bevy rendering.
//!
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
//!     commands
//!         .spawn(Cable {
//!             solver: Solver::Catenary(CatenarySolver::new().with_slack(1.3)),
//!             obstacles: vec![],
//!             resolution: 0,
//!         })
//!         .with_children(|parent| {
//!             parent.spawn(CableEndpoint::new(CableEnd::Start, Vec3::new(-2.0, 2.0, 0.0)));
//!             parent.spawn(CableEndpoint::new(CableEnd::End, Vec3::new(2.0, 2.0, 0.0)));
//!         });
//! }
//! ```

mod plugin;
mod routing;

// Bevy plugin
pub use plugin::AttachedEndpoints;
pub use plugin::AttachedTo;
pub use plugin::Cable;
pub use plugin::CableEnd;
pub use plugin::CableEndpoint;
pub use plugin::CableGizmoGroup;
pub use plugin::CableMeshChild;
pub use plugin::CableMeshConfig;
pub use plugin::CableMeshHandle;
pub use plugin::CapConfig;
pub use plugin::Capping;
pub use plugin::CatenaryPlugin;
pub use plugin::ComputedCableGeometry;
pub use plugin::DebugGizmos;
pub use plugin::ElbowConfig;
pub use plugin::ElbowMetadata;
pub use plugin::EndpointAlignment;
pub use plugin::FaceSides;
pub use plugin::OnDetach;
pub use plugin::TrimConfig;
pub use plugin::TubeConfig;
pub use plugin::compute_elbow_metadata;
pub use plugin::generate_tube_mesh;
// Routing
pub use routing::AStarPlanner;
pub use routing::Anchor;
pub use routing::AxisOrder;
pub use routing::CableGeometry;
pub use routing::CableSegment;
pub use routing::CatenarySolver;
pub use routing::CurveKind;
pub use routing::CurveSolver;
pub use routing::DEFAULT_GRAVITY;
pub use routing::DEFAULT_RESOLUTION;
pub use routing::DEFAULT_SLACK;
pub use routing::DirectPlanner;
pub use routing::LinearSolver;
pub use routing::Obstacle;
pub use routing::OrthogonalPlanner;
pub use routing::PathPlanner;
pub use routing::PathStrategy;
pub use routing::RouteRequest;
pub use routing::RouteSolver;
pub use routing::Router;
pub use routing::Solver;
pub use routing::evaluate;
pub use routing::sample_3d;
pub use routing::solve_parameter;
