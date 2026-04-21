//! Cable components and route computation — `Cable`, `CableEndpoint`, and supporting
//! relationship/policy types, plus the observers that align attached targets and handle
//! detachment.

mod endpoint;

use bevy::ecs::entity::EntityHashSet;
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

/// Cables queued for geometry recomputation this frame. Drained by
/// [`recompute_dirty_cables`].
#[derive(Resource, Default)]
struct DirtyCables(EntityHashSet);

pub(crate) struct CablePlugin;

impl Plugin for CablePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DirtyCables>()
            .add_systems(
                Update,
                (
                    (
                        queue_changed_cables,
                        queue_endpoint_changes,
                        queue_attached_target_moves,
                    ),
                    recompute_dirty_cables,
                )
                    .chain()
                    .in_set(CableSystems::Compute),
            )
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

/// Computed cable geometry, populated by the cable recompute queue.
///
/// Rendering systems read from this component.
#[derive(Component, Clone, Default)]
pub struct ComputedCableGeometry {
    /// The computed geometry, or `None` if not yet computed.
    pub geometry: Option<CableGeometry>,
}

/// Queues cables whose own `Cable` component was inserted or mutated.
fn queue_changed_cables(
    cables: Query<Entity, Changed<Cable>>,
    mut dirty_cables: ResMut<DirtyCables>,
) {
    for cable_entity in &cables {
        dirty_cables.0.insert(cable_entity);
    }
}

/// Queues the parent cable of any endpoint whose `CableEndpoint` or `AttachedTo`
/// component was inserted or mutated.
fn queue_endpoint_changes(
    endpoints: Query<&ChildOf, Or<(Changed<CableEndpoint>, Changed<AttachedTo>)>>,
    mut dirty_cables: ResMut<DirtyCables>,
) {
    for child_of in &endpoints {
        dirty_cables.0.insert(child_of.parent());
    }
}

/// Queues cables whose attached targets had their world transform change.
fn queue_attached_target_moves(
    targets: Query<&AttachedEndpoints, Changed<GlobalTransform>>,
    endpoint_parents: Query<&ChildOf>,
    mut dirty_cables: ResMut<DirtyCables>,
) {
    for attached_endpoints in &targets {
        for endpoint in attached_endpoints.iter() {
            let Ok(child_of) = endpoint_parents.get(endpoint) else {
                continue;
            };
            dirty_cables.0.insert(child_of.parent());
        }
    }
}

/// Drains [`DirtyCables`] and recomputes geometry for each queued cable.
fn recompute_dirty_cables(
    mut commands: Commands,
    mut dirty_cables: ResMut<DirtyCables>,
    cables: Query<(&Cable, &Children)>,
    mut endpoints: Query<(
        &CableEndpoint,
        Option<&AttachedTo>,
        Option<&mut endpoint::ResolvedEndpointPosition>,
    )>,
    transforms: Query<&GlobalTransform>,
) {
    for cable_entity in dirty_cables.0.drain() {
        recompute_cable_route(
            cable_entity,
            &mut commands,
            &cables,
            &mut endpoints,
            &transforms,
        );
    }
}

fn recompute_cable_route(
    cable_entity: Entity,
    commands: &mut Commands,
    cables: &Query<(&Cable, &Children)>,
    endpoints: &mut Query<(
        &CableEndpoint,
        Option<&AttachedTo>,
        Option<&mut endpoint::ResolvedEndpointPosition>,
    )>,
    transforms: &Query<&GlobalTransform>,
) {
    let Ok((cable, children)) = cables.get(cable_entity) else {
        return;
    };

    let mut start_pos = None;
    let mut end_pos = None;

    for child in children.iter() {
        let Ok((endpoint, attached_to, resolved_endpoint_position)) = endpoints.get_mut(child)
        else {
            continue;
        };

        let pos = if let Some(attached) = attached_to
            && let Ok(target_transform) = transforms.get(attached.0)
        {
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

    let (Some(start), Some(end)) = (start_pos, end_pos) else {
        return;
    };

    if start.distance(end) < MIN_SEGMENT_LENGTH {
        return;
    }

    let request = RouteRequest {
        start,
        end,
        obstacles: &cable.obstacles,
        resolution: cable.resolution,
    };

    let geometry = cable.solver.solve(&request);
    commands.entity(cable_entity).insert(ComputedCableGeometry {
        geometry: Some(geometry),
    });
}
