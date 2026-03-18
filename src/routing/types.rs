//! Core data structures for cable routing.
//!
//! These types form the API boundary between route computation and rendering.
//! They depend only on `glam` (re-exported by `bevy::math`) and carry no Bevy ECS types.

use bevy::math::Quat;
use bevy::math::Vec3;
use bevy::reflect::Reflect;

/// Where a cable connects to an object.
#[derive(Clone, Copy, Debug)]
pub struct Anchor {
    /// World-space position of the connection point.
    pub position:  Vec3,
    /// Preferred exit direction at this anchor (for tangent continuity).
    /// When `None`, the solver chooses the natural direction.
    pub direction: Option<Vec3>,
}

impl Anchor {
    /// Create an anchor at a position with no preferred direction.
    pub fn new(position: Vec3) -> Self {
        Self {
            position,
            direction: None,
        }
    }

    /// Create an anchor with a preferred exit direction.
    pub fn with_direction(position: Vec3, direction: Vec3) -> Self {
        Self {
            position,
            direction: Some(direction),
        }
    }
}

/// An axis-aligned bounding box with a world transform, used as a routing obstacle.
#[derive(Clone, Copy, Debug, Reflect)]
pub struct Obstacle {
    /// Half-extents of the AABB in local space.
    pub half_extents: Vec3,
    /// World-space position of the obstacle center.
    pub position:     Vec3,
    /// World-space rotation of the obstacle.
    pub rotation:     Quat,
}

impl Obstacle {
    /// Create an axis-aligned obstacle (no rotation).
    pub fn new(half_extents: Vec3, position: Vec3) -> Self {
        Self {
            half_extents,
            position,
            rotation: Quat::IDENTITY,
        }
    }

    /// Create a rotated obstacle.
    pub fn with_rotation(half_extents: Vec3, position: Vec3, rotation: Quat) -> Self {
        Self {
            half_extents,
            position,
            rotation,
        }
    }

    /// World-space AABB minimum corner (ignoring rotation for axis-aligned tests).
    pub fn aabb_min(&self) -> Vec3 { self.position - self.half_extents }

    /// World-space AABB maximum corner (ignoring rotation for axis-aligned tests).
    pub fn aabb_max(&self) -> Vec3 { self.position + self.half_extents }
}

/// Everything a solver needs to compute a route.
#[derive(Clone, Debug)]
pub struct RouteRequest<'a> {
    /// Starting position of the cable.
    pub start:      Vec3,
    /// Ending position of the cable.
    pub end:        Vec3,
    /// Obstacles to route around (may be empty).
    pub obstacles:  &'a [Obstacle],
    /// Number of sample points per segment.
    pub resolution: u32,
}

/// A single continuous curve between two waypoints.
#[derive(Clone, Debug, Default)]
pub struct CableSegment {
    /// Sampled positions along the curve.
    pub points:      Vec<Vec3>,
    /// Unit tangent at each sample point.
    pub tangents:    Vec<Vec3>,
    /// Cumulative arc length at each sample point.
    pub arc_lengths: Vec<f32>,
    /// Total arc length of this segment.
    pub length:      f32,
}

impl CableSegment {
    /// Create a segment from points, computing tangents and arc lengths.
    pub fn from_points(points: Vec<Vec3>) -> Self {
        if points.is_empty() {
            return Self::default();
        }

        let n = points.len();
        let mut tangents = Vec::with_capacity(n);
        let mut arc_lengths = Vec::with_capacity(n);
        let mut cumulative = 0.0_f32;

        arc_lengths.push(0.0);

        for i in 0..n {
            // Compute tangent via finite differences
            let tangent = if n == 1 {
                Vec3::Y
            } else if i == 0 {
                (points[1] - points[0]).normalize_or_zero()
            } else if i == n - 1 {
                (points[n - 1] - points[n - 2]).normalize_or_zero()
            } else {
                (points[i + 1] - points[i - 1]).normalize_or_zero()
            };
            tangents.push(tangent);

            if i > 0 {
                cumulative += points[i].distance(points[i - 1]);
                arc_lengths.push(cumulative);
            }
        }

        Self {
            points,
            tangents,
            arc_lengths,
            length: cumulative,
        }
    }
}

/// The complete geometry of a routed cable. This is the render-agnostic output
/// that bridges route computation and rendering.
#[derive(Clone, Debug, Default)]
pub struct CableGeometry {
    /// Curve segments between waypoints.
    pub segments:     Vec<CableSegment>,
    /// Total arc length across all segments.
    pub total_length: f32,
    /// Structural waypoints (start, intermediate bends, end).
    pub waypoints:    Vec<Vec3>,
}

impl CableGeometry {
    /// Build a `CableGeometry` from a list of segments and the waypoints that produced them.
    pub fn from_segments(segments: Vec<CableSegment>, waypoints: Vec<Vec3>) -> Self {
        let total_length = segments.iter().map(|s| s.length).sum();
        Self {
            segments,
            total_length,
            waypoints,
        }
    }

    /// Iterate over all sample points across all segments.
    pub fn all_points(&self) -> impl Iterator<Item = &Vec3> {
        self.segments.iter().flat_map(|s| &s.points)
    }
}
