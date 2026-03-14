//! Bevy plugin for cable routing and visualization.
//!
//! Provides [`CatenaryPlugin`], which adds cable route computation,
//! debug gizmo rendering, and tube mesh generation for [`Cable`] entities.

mod components;
mod mesh;
mod systems;

use bevy::prelude::*;
pub use components::Cable;
pub use components::ComputedCableGeometry;
pub use mesh::TubeMeshConfig;
pub use mesh::generate_tube_mesh;
pub use systems::CableDebugEnabled;
use systems::compute_cable_routes;
use systems::render_cable_gizmos;
use systems::render_debug_gizmos;

/// Gizmo group for cable debug wireframes.
///
/// Enable or disable via Bevy's `GizmoConfigStore`.
#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct CableGizmoGroup;

/// Plugin that adds cable routing support to a Bevy app.
///
/// Registers:
/// - [`CableDebugEnabled`] resource (default: off).
/// - Cable route computation system (runs in `Update`).
/// - Gizmo debug renderer (runs in `Update` after computation).
/// - [`CableGizmoGroup`] for controlling debug visibility.
pub struct CatenaryPlugin;

impl Plugin for CatenaryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CableDebugEnabled>()
            .init_gizmo_group::<CableGizmoGroup>()
            .add_systems(
                Update,
                (
                    compute_cable_routes,
                    render_cable_gizmos,
                    render_debug_gizmos,
                )
                    .chain(),
            );
    }
}
