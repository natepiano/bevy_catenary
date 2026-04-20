//! Core data structures for cable routing.
//!
//! These types form the API boundary between route computation and rendering.
//! They depend only on `glam` (re-exported by `bevy::math`) and carry no Bevy ECS types.

use bevy::math::Quat;
use bevy::math::Vec3;
use bevy::reflect::Reflect;
use bevy_kana::ToF32;

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
    #[must_use]
    pub fn new(position: impl Into<Vec3>) -> Self {
        Self {
            position:  position.into(),
            direction: None,
        }
    }

    /// Create an anchor with a preferred exit direction.
    #[must_use]
    pub fn with_direction(position: impl Into<Vec3>, direction: Vec3) -> Self {
        Self {
            position:  position.into(),
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
    #[must_use]
    pub fn new(half_extents: Vec3, position: impl Into<Vec3>) -> Self {
        Self {
            half_extents,
            position: position.into(),
            rotation: Quat::IDENTITY,
        }
    }

    /// Create a rotated obstacle.
    #[must_use]
    pub fn with_rotation(half_extents: Vec3, position: impl Into<Vec3>, rotation: Quat) -> Self {
        Self {
            half_extents,
            position: position.into(),
            rotation,
        }
    }

    /// World-space AABB minimum corner (ignoring rotation for axis-aligned tests).
    #[must_use]
    pub fn aabb_min(&self) -> Vec3 { self.position - self.half_extents }

    /// World-space AABB maximum corner (ignoring rotation for axis-aligned tests).
    #[must_use]
    pub fn aabb_max(&self) -> Vec3 { self.position + self.half_extents }

    /// Check if a point is inside this obstacle's AABB, expanded by `margin`.
    #[must_use]
    pub fn contains_point(&self, pos: impl Into<Vec3>, margin: f32) -> bool {
        let pos: Vec3 = pos.into();
        let min = self.aabb_min() - Vec3::splat(margin);
        let max = self.aabb_max() + Vec3::splat(margin);
        pos.x >= min.x
            && pos.x <= max.x
            && pos.y >= min.y
            && pos.y <= max.y
            && pos.z >= min.z
            && pos.z <= max.z
    }
}

/// Check if a point is inside any obstacle's AABB, expanded by `margin`.
#[must_use]
pub(super) fn is_point_in_any_obstacle(pos: Vec3, obstacles: &[Obstacle], margin: f32) -> bool {
    obstacles.iter().any(|obs| obs.contains_point(pos, margin))
}

/// Check if any obstacle intersects a line segment by sampling `steps` evenly-spaced points.
#[must_use]
pub(super) fn is_segment_blocked(
    start: Vec3,
    end: Vec3,
    obstacles: &[Obstacle],
    margin: f32,
    steps: u32,
) -> bool {
    (0..=steps).any(|i| {
        let t = i.to_f32() / steps.to_f32();
        let point = start.lerp(end, t);
        is_point_in_any_obstacle(point, obstacles, margin)
    })
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

impl RouteRequest<'_> {
    /// Returns the request's resolution if set, otherwise falls back to `default`.
    #[must_use]
    pub const fn effective_resolution(&self, default: u32) -> u32 {
        if self.resolution > 0 {
            self.resolution
        } else {
            default
        }
    }
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
    #[must_use]
    pub fn from_points(points: Vec<Vec3>) -> Self {
        if points.is_empty() {
            return Self::default();
        }

        let n = points.len();

        let tangents: Vec<Vec3> = points
            .iter()
            .enumerate()
            .map(|(i, _)| {
                if n == 1 {
                    Vec3::Y
                } else if i == 0 {
                    (points[1] - points[0]).normalize_or_zero()
                } else if i == n - 1 {
                    (points[n - 1] - points[n - 2]).normalize_or_zero()
                } else {
                    (points[i + 1] - points[i - 1]).normalize_or_zero()
                }
            })
            .collect();

        let mut cumulative = 0.0_f32;
        let arc_lengths: Vec<f32> = std::iter::once(0.0)
            .chain(points.windows(2).map(|pair| {
                cumulative += pair[0].distance(pair[1]);
                cumulative
            }))
            .collect();

        Self {
            points,
            tangents,
            arc_lengths,
            length: cumulative,
        }
    }

    /// Create a segment by evenly sampling `n` points along a straight line.
    #[must_use]
    pub fn straight_line(start: impl Into<Vec3>, end: impl Into<Vec3>, n: usize) -> Self {
        let start: Vec3 = start.into();
        let end: Vec3 = end.into();
        let n = n.max(2);
        let points: Vec<Vec3> = (0..n)
            .map(|i| {
                let t = i.to_f32() / (n - 1).to_f32();
                start.lerp(end, t)
            })
            .collect();
        Self::from_points(points)
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
    #[must_use]
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
