use bevy::prelude::*;

/// Gizmo group for cable debug wireframes.
///
/// Enable or disable via Bevy's `GizmoConfigStore`.
#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct CableGizmoGroup;

/// Resource that toggles detailed debug visualization.
#[derive(Clone, Debug, Default, PartialEq, Eq, Resource)]
pub enum DebugGizmos {
    /// Debug gizmos are rendered.
    Enabled,
    /// Debug gizmos are hidden.
    #[default]
    Disabled,
}
