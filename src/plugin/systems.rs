//! Bevy systems for cable route computation and debug rendering.

use bevy::prelude::*;

use super::CableGizmoGroup;
use super::components::AttachedTo;
use super::components::Cable;
use super::components::CableEnd;
use super::components::CableEndpoint;
use super::components::ComputedCableGeometry;
use super::components::DetachPolicy;
use super::mesh::CableMeshConfig;
use super::mesh;
use crate::routing::MIN_SEGMENT_LENGTH;
use crate::routing::RouteRequest;

/// Stores the mesh asset handle for a cable's generated tube mesh.
/// The library manages this — users don't need to interact with it directly.
#[derive(Component)]
pub struct CableMeshHandle(pub Handle<Mesh>);

/// Stores the entity ID of the mesh child spawned for a cable.
#[derive(Component)]
pub struct CableMeshChild(pub Entity);

/// Query type for accessing cable geometry, config, and mesh handles.
type CableMeshQuery<'w> = (
    &'w ComputedCableGeometry,
    &'w CableMeshConfig,
    &'w Children,
    Option<&'w CableMeshHandle>,
    Option<&'w CableMeshChild>,
);

/// Computes cable routes reactively when endpoints or targets change.
///
/// Triggers on:
/// - `Cable` component changes (solver, obstacles, resolution)
/// - `CableEndpoint` component changes (offset, cap style)
/// - Attached target entity's `GlobalTransform` changes
pub fn compute_cable_routes(
    mut commands: Commands,
    cables: Query<(Entity, Ref<Cable>, &Children)>,
    endpoints: Query<(Ref<CableEndpoint>, Option<&AttachedTo>)>,
    transforms: Query<&GlobalTransform>,
    changed_transforms: Query<(), Changed<GlobalTransform>>,
) {
    for (entity, cable, children) in &cables {
        let mut needs_update = cable.is_changed();
        let mut start_pos = None;
        let mut end_pos = None;

        for child in children.iter() {
            let Ok((endpoint, attached_to)) = endpoints.get(child) else {
                continue;
            };

            if endpoint.is_changed() {
                needs_update = true;
            }

            let pos = if let Some(attached) = attached_to {
                if let Ok(target_tf) = transforms.get(attached.0) {
                    if changed_transforms.get(attached.0).is_ok() {
                        needs_update = true;
                    }
                    target_tf.transform_point(endpoint.offset)
                } else {
                    // Target despawned — fall back to raw offset
                    endpoint.offset
                }
            } else {
                // World-attached — offset IS the world position
                endpoint.offset
            };

            match endpoint.end {
                CableEnd::Start => start_pos = Some(pos),
                CableEnd::End => end_pos = Some(pos),
            }
        }

        if !needs_update {
            continue;
        }

        let (Some(start), Some(end)) = (start_pos, end_pos) else {
            continue;
        };

        // Guard: skip degenerate zero-length cables
        if start.distance(end) < MIN_SEGMENT_LENGTH {
            continue;
        }

        let request = RouteRequest {
            start,
            end,
            obstacles: &cable.obstacles,
            resolution: cable.resolution,
        };

        let geometry = cable.solver.solve(&request);
        // Re-insert to trigger OnInsert<ComputedCableGeometry> for mesh generation
        commands.entity(entity).insert(ComputedCableGeometry {
            geometry: Some(geometry),
        });
    }
}

/// Renders cable geometry as gizmo lines (only when debug is enabled).
pub fn render_cable_gizmos(
    cables: Query<(&ComputedCableGeometry, &GlobalTransform)>,
    mut gizmos: Gizmos<CableGizmoGroup>,
    debug_enabled: Res<CableDebugEnabled>,
) {
    if !debug_enabled.0 {
        return;
    }
    for (computed, _global_transform) in &cables {
        let Some(geometry) = &computed.geometry else {
            continue;
        };

        // Draw cable segments as polylines
        for segment in &geometry.segments {
            if segment.points.len() < 2 {
                continue;
            }

            for pair in segment.points.windows(2) {
                gizmos.line(pair[0], pair[1], Color::srgb(1.0, 0.6, 0.0));
            }
        }

        // Draw waypoints as small crosses
        for &wp in &geometry.waypoints {
            draw_dot(&mut gizmos, wp, 0.05, Color::srgb(0.0, 1.0, 0.0));
        }
    }
}

/// Renders detailed debug info: tangent vectors and segment boundaries.
pub fn render_debug_gizmos(
    cables: Query<&ComputedCableGeometry>,
    mut gizmos: Gizmos<CableGizmoGroup>,
    debug_enabled: Res<CableDebugEnabled>,
) {
    if !debug_enabled.0 {
        return;
    }

    for computed in &cables {
        let Some(geometry) = &computed.geometry else {
            continue;
        };

        for segment in &geometry.segments {
            // Draw tangent vectors at every 4th point
            for (i, (point, tangent)) in segment.points.iter().zip(&segment.tangents).enumerate() {
                if i % 4 == 0 {
                    gizmos.line(*point, *point + *tangent * 0.1, Color::srgb(1.0, 1.0, 0.0));
                }
            }

            // Draw segment start/end as red dots
            if let Some(first) = segment.points.first() {
                draw_dot(&mut gizmos, *first, 0.03, Color::srgb(1.0, 0.0, 0.0));
            }
            if let Some(last) = segment.points.last() {
                draw_dot(&mut gizmos, *last, 0.03, Color::srgb(1.0, 0.0, 0.0));
            }
        }
    }
}

/// Draw a small cross at a point (debug marker).
fn draw_dot(gizmos: &mut Gizmos<CableGizmoGroup>, point: Vec3, size: f32, color: Color) {
    let s = size;
    gizmos.line(point - Vec3::X * s, point + Vec3::X * s, color);
    gizmos.line(point - Vec3::Y * s, point + Vec3::Y * s, color);
    gizmos.line(point - Vec3::Z * s, point + Vec3::Z * s, color);
}

/// Resource that toggles detailed debug visualization.
#[derive(Default, Resource)]
pub struct CableDebugEnabled(pub bool);

/// Observer that generates or updates the cable mesh when geometry is (re)computed.
///
/// On first insert: creates a `Mesh` asset, spawns a mesh child entity, stores
/// `CableMeshHandle` and `CableMeshChild` on the cable.
/// On subsequent inserts: mutates the existing mesh asset in place (no entity churn).
pub fn on_geometry_computed(
    trigger: On<Insert, ComputedCableGeometry>,
    cables: Query<CableMeshQuery>,
    endpoints: Query<&CableEndpoint>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    mut commands: Commands,
) {
    let Some(mut meshes) = meshes else {
        return;
    };
    let cable_entity = trigger.event_target();
    let Ok((computed, config, children, mesh_handle, _mesh_child)) = cables.get(cable_entity)
    else {
        return;
    };

    let Some(geometry) = &computed.geometry else {
        return;
    };

    // Read endpoint cap styles from children
    let mut cap_start = config.cap_start.clone();
    let mut cap_end = config.cap_end.clone();
    for child in children.iter() {
        if let Ok(ep) = endpoints.get(child) {
            match ep.end {
                CableEnd::Start => cap_start = ep.cap_style.clone(),
                CableEnd::End => cap_end = ep.cap_style.clone(),
            }
        }
    }

    // Build the config with endpoint cap styles applied
    let mut mesh_config = config.clone();
    mesh_config.cap_start = cap_start;
    mesh_config.cap_end = cap_end;

    let new_mesh = mesh::generate_tube_mesh(geometry, &mesh_config);

    if let Some(handle) = mesh_handle {
        // Update existing mesh asset in place
        if let Some(existing) = meshes.get_mut(&handle.0) {
            *existing = new_mesh;
        }
    } else {
        // First time: create asset, spawn mesh child
        let handle = meshes.add(new_mesh);
        let mut child_commands = commands.spawn((Mesh3d(handle.clone()), ChildOf(cable_entity)));
        if let Some(ref mat) = config.material {
            child_commands.insert(MeshMaterial3d(mat.clone()));
        }
        let child = child_commands.id();
        commands
            .entity(cable_entity)
            .insert((CableMeshHandle(handle), CableMeshChild(child)));
    }
}

/// Observer that handles endpoint detachment when a target entity is despawned.
///
/// Bevy auto-removes `AttachedTo` when the target entity is despawned, which
/// triggers `OnRemove<AttachedTo>`. This observer reads the endpoint's
/// [`DetachPolicy`] and acts accordingly.
pub fn on_endpoint_detached(
    trigger: On<Remove, AttachedTo>,
    mut endpoints: Query<(&mut CableEndpoint, &ChildOf)>,
    mut commands: Commands,
) {
    let endpoint_entity = trigger.event_target();
    let Ok((endpoint, child_of)) = endpoints.get_mut(endpoint_entity) else {
        return;
    };

    match endpoint.detach_policy {
        DetachPolicy::HangInPlace | DetachPolicy::HangLoose => {
            // HangLoose falls back to HangInPlace for now.
            // The offset is already a world position for world-attached endpoints.
            // For entity-attached, we'd ideally store the last resolved world pos,
            // but since the target is being despawned, the offset stays as-is.
            // The cable will keep its last computed geometry.
        },
        DetachPolicy::Despawn => {
            let cable_entity = child_of.parent();
            commands.entity(cable_entity).despawn();
        },
    }
}
