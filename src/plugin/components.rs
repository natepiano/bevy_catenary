//! Bevy components for cable entities.

use bevy::prelude::*;

use crate::routing::CableGeometry;
use crate::routing::Obstacle;
use crate::routing::RouteSolver;

/// A cable entity. Declares the endpoints, solver, and obstacles.
///
/// Adding this component triggers route computation via the
/// `compute_cable_routes` system. Results are written to [`ComputedCableGeometry`].
#[derive(Component)]
#[require(ComputedCableGeometry, Transform, Visibility)]
pub struct Cable {
    /// Starting position of the cable (world space).
    pub start:      Vec3,
    /// Ending position of the cable (world space).
    pub end:        Vec3,
    /// The routing algorithm to use.
    pub solver:     Box<dyn RouteSolver>,
    /// Obstacles to route around.
    pub obstacles:  Vec<Obstacle>,
    /// Number of sample points per segment (0 = use solver default).
    pub resolution: u32,
}

/// Computed cable geometry, populated by the `compute_cable_routes` system.
///
/// Rendering systems read from this component.
#[derive(Component, Default)]
pub struct ComputedCableGeometry {
    /// The computed geometry, or `None` if not yet computed.
    pub geometry: Option<CableGeometry>,
}
