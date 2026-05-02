mod constants;
mod debug;
mod render;

use bevy::prelude::*;
pub use debug::CableGizmoGroup;
pub use debug::DebugGizmos;

use crate::cable::CableSystems;

pub(super) struct GizmosPlugin;

impl Plugin for GizmosPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugGizmos>()
            .init_gizmo_group::<CableGizmoGroup>()
            .add_systems(
                Update,
                (render::render_cable_gizmos, render::render_debug_gizmos)
                    .chain()
                    .after(CableSystems::Compute),
            );
    }
}
