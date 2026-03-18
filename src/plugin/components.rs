//! Bevy components for cable entities.

use bevy::prelude::*;

use crate::plugin::mesh::CapStyle;
use crate::routing::CableGeometry;
use crate::routing::Obstacle;
use crate::routing::Solver;

/// Which end of the cable an endpoint represents.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Reflect)]
pub enum CableEnd {
    /// The starting end of the cable.
    Start,
    /// The ending end of the cable.
    End,
}

/// A cable endpoint entity. Spawned as a child of a [`Cable`] entity.
///
/// `offset` has different semantics depending on whether [`AttachedTo`] is present:
/// - **World-attached** (no `AttachedTo`): `offset` is the world-space position.
/// - **Entity-attached** (with `AttachedTo`): `offset` is in the target entity's local space. The
///   system transforms it to world space via the target's [`GlobalTransform`].
#[derive(Component, Clone, Debug, Reflect)]
pub struct CableEndpoint {
    /// Which end of the cable this represents.
    pub end:           CableEnd,
    /// Position offset. World-space for world-attached, local-space for entity-attached.
    pub offset:        Vec3,
    /// How to cap this end of the tube mesh.
    pub cap_style:     CapStyle,
    /// What happens when the target entity is despawned.
    pub detach_policy: DetachPolicy,
}

impl CableEndpoint {
    /// Create a new endpoint with default cap style (`None`) and detach policy (`HangInPlace`).
    pub fn new(end: CableEnd, offset: Vec3) -> Self {
        Self {
            end,
            offset,
            cap_style: CapStyle::default(),
            detach_policy: DetachPolicy::default(),
        }
    }

    /// Set the cap style for this endpoint.
    pub fn with_cap(mut self, cap_style: CapStyle) -> Self {
        self.cap_style = cap_style;
        self
    }

    /// Set the detach policy for this endpoint.
    pub fn with_detach_policy(mut self, policy: DetachPolicy) -> Self {
        self.detach_policy = policy;
        self
    }
}

/// What happens when an endpoint's target entity is despawned.
///
/// When a target entity is despawned, Bevy auto-removes the [`AttachedTo`] relationship.
/// An `OnRemove<AttachedTo>` observer fires and reads this policy to decide behavior.
#[derive(Clone, Debug, Default, Reflect)]
pub enum DetachPolicy {
    /// Convert to world-attached at last resolved position. Cable keeps its shape.
    #[default]
    HangInPlace,
    /// (Future) Animate slack increase — cable "drops" from the detached end.
    /// Currently falls back to `HangInPlace`.
    HangLoose,
    /// Despawn the entire cable when this endpoint's target is removed.
    Despawn,
}

/// Relationship: endpoint → target entity it follows.
///
/// When present on a [`CableEndpoint`] entity, the endpoint's `offset` is interpreted
/// as a local-space offset from the target's [`GlobalTransform`]. The cable recomputes
/// reactively when the target's transform changes.
///
/// If the target entity is despawned, the endpoint's [`DetachPolicy`] determines behavior.
#[derive(Component)]
#[relationship(relationship_target = AttachedEndpoints)]
pub struct AttachedTo(pub Entity);

/// Back-reference on target entities. Auto-populated by Bevy's relationship system.
///
/// Query this to find all cable endpoints attached to a given entity.
#[derive(Component)]
#[relationship_target(relationship = AttachedTo)]
pub struct AttachedEndpoints(Vec<Entity>);

/// A cable entity. Route computation is driven by its child [`CableEndpoint`] entities.
///
/// The cable itself stores the solver, obstacles, and resolution. Endpoint positions
/// come from child entities with [`CableEndpoint`] components.
#[derive(Component, Clone, Debug, Reflect)]
#[require(ComputedCableGeometry, Transform, Visibility)]
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
#[derive(Component, Default)]
pub struct ComputedCableGeometry {
    /// The computed geometry, or `None` if not yet computed.
    pub geometry: Option<CableGeometry>,
}
