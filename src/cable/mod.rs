//! Cable components and route computation — `Cable`, `CableEndpoint`, and supporting
//! relationship/policy types, plus the observers that align attached targets and handle
//! detachment.

mod endpoint;

use bevy::prelude::*;
pub use endpoint::AttachedEndpoints;
pub use endpoint::AttachedTo;
pub use endpoint::CableEnd;
pub use endpoint::CableEndpoint;
pub use endpoint::EndpointAlignment;
pub use endpoint::OnDetach;

use crate::mesh::CableMeshConfig;
use crate::routing::CableGeometry;
use crate::routing::MIN_SEGMENT_LENGTH;
use crate::routing::Obstacle;
use crate::routing::RouteRequest;
use crate::routing::Solver;

/// `SystemSet` for cross-plugin ordering. `GizmosPlugin`'s render systems run
/// `.after(CableSystems::Compute)` to observe freshly-computed geometry in the
/// same frame.
#[derive(SystemSet, Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub(crate) enum CableSystems {
    Compute,
}

pub(crate) struct CablePlugin;

impl Plugin for CablePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, compute_cable_routes.in_set(CableSystems::Compute))
            .add_observer(endpoint::on_endpoint_alignment_update)
            .add_observer(endpoint::on_endpoint_detached);
    }
}

/// A cable entity. Route computation is driven by its child [`CableEndpoint`] entities.
///
/// The cable itself stores the solver, obstacles, and resolution. Endpoint positions
/// come from child entities with [`CableEndpoint`] components.
#[derive(Component, Clone, Debug, Reflect)]
#[reflect(Component)]
#[require(ComputedCableGeometry, CableMeshConfig, Transform, Visibility)]
pub struct Cable {
    /// The routing algorithm to use.
    pub solver:     Solver,
    /// Obstacles to route around.
    pub obstacles:  Vec<Obstacle>,
    /// Number of sample points per segment (0 = use solver default).
    pub resolution: u32,
}

/// Computed cable geometry, populated by the `compute_cable_routes` system.
///
/// Rendering systems read from this component.
#[derive(Component, Clone, Default)]
pub struct ComputedCableGeometry {
    /// The computed geometry, or `None` if not yet computed.
    pub geometry: Option<CableGeometry>,
}

/// Computes cable routes reactively when endpoints or targets change.
///
/// Triggers on:
/// - `Cable` component changes (solver, obstacles, resolution)
/// - `CableEndpoint` component changes (offset, cap style)
/// - Attached target entity's `GlobalTransform` changes
fn compute_cable_routes(
    mut commands: Commands,
    cables: Query<(Entity, Ref<Cable>, &Children)>,
    mut endpoints: Query<(
        Ref<CableEndpoint>,
        Option<&AttachedTo>,
        Option<&mut endpoint::ResolvedEndpointPosition>,
    )>,
    transforms: Query<&GlobalTransform>,
    changed_transforms: Query<(), Changed<GlobalTransform>>,
) {
    for (entity, cable, children) in &cables {
        let mut needs_update = cable.is_changed();
        let mut start_pos = None;
        let mut end_pos = None;

        for child in children.iter() {
            let Ok((endpoint, attached_to, resolved_endpoint_position)) = endpoints.get_mut(child)
            else {
                continue;
            };

            if endpoint.is_changed() {
                needs_update = true;
            }

            let pos = if let Some(attached) = attached_to
                && let Ok(target_transform) = transforms.get(attached.0)
            {
                if changed_transforms.get(attached.0).is_ok() {
                    needs_update = true;
                }
                target_transform.transform_point(endpoint.offset)
            } else {
                endpoint.offset
            };

            if let Some(mut resolved) = resolved_endpoint_position {
                if resolved.0 != pos {
                    resolved.0 = pos;
                }
            } else {
                commands
                    .entity(child)
                    .insert(endpoint::ResolvedEndpointPosition(pos));
            }

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
        commands.entity(entity).insert(ComputedCableGeometry {
            geometry: Some(geometry),
        });
    }
}
