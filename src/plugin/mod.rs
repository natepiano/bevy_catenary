//! Bevy plugin for cable routing and visualization.
//!
//! Provides [`CatenaryPlugin`], which adds cable route computation,
//! debug gizmo rendering, and tube mesh generation for [`Cable`] entities.

mod components;
mod constants;
#[allow(
    clippy::used_underscore_binding,
    reason = "false positive on enum variant fields"
)]
mod mesh;
mod systems;

use bevy::prelude::*;
pub use components::AttachedEndpoints;
pub use components::AttachedTo;
pub use components::Cable;
pub use components::CableEnd;
pub use components::CableEndpoint;
pub use components::ComputedCableGeometry;
pub use components::EndpointAlignment;
pub use components::OnDetach;
pub use mesh::CableMeshConfig;
pub use mesh::CapConfig;
pub use mesh::Capping;
pub use mesh::ElbowConfig;
pub use mesh::ElbowMetadata;
pub use mesh::FaceSides;
pub use mesh::TrimConfig;
pub use mesh::TubeConfig;
pub use mesh::compute_elbow_metadata;
pub use mesh::generate_tube_mesh;
pub use systems::CableGizmoGroup;
pub use systems::CableMeshChild;
pub use systems::CableMeshHandle;
pub use systems::DebugGizmos;

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
        app.init_resource::<DebugGizmos>()
            .init_gizmo_group::<CableGizmoGroup>()
            .add_systems(
                Update,
                (
                    systems::compute_cable_routes,
                    systems::render_cable_gizmos,
                    systems::render_debug_gizmos,
                )
                    .chain(),
            )
            .add_observer(systems::on_endpoint_detached)
            .add_observer(systems::on_endpoint_alignment_update)
            .add_observer(systems::on_geometry_computed);
    }
}
