//! `Obstacle` `AABB` type and the point/segment blocking helpers that operate on it.

use bevy::math::Quat;
use bevy::math::Vec3;
use bevy::reflect::Reflect;
use bevy_kana::ToF32;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum PointContainment {
    Outside,
    Inside,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum Blockage {
    Clear,
    Blocked,
}

/// An axis-aligned bounding box with a world transform, used as a routing obstacle.
#[derive(Clone, Copy, Debug, Reflect)]
pub struct Obstacle {
    /// Half-extents of the `AABB` in local space.
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

    /// World-space `AABB` minimum corner (ignoring rotation for axis-aligned tests).
    #[must_use]
    pub fn aabb_min(&self) -> Vec3 { self.position - self.half_extents }

    /// World-space `AABB` maximum corner (ignoring rotation for axis-aligned tests).
    #[must_use]
    pub fn aabb_max(&self) -> Vec3 { self.position + self.half_extents }

    fn point_containment(&self, pos: Vec3, margin: f32) -> PointContainment {
        let min = self.aabb_min() - Vec3::splat(margin);
        let max = self.aabb_max() + Vec3::splat(margin);
        if pos.x >= min.x
            && pos.x <= max.x
            && pos.y >= min.y
            && pos.y <= max.y
            && pos.z >= min.z
            && pos.z <= max.z
        {
            PointContainment::Inside
        } else {
            PointContainment::Outside
        }
    }

    /// Check if a point is inside this obstacle's `AABB`, expanded by `margin`.
    #[must_use]
    pub fn contains_point(&self, pos: impl Into<Vec3>, margin: f32) -> bool {
        matches!(
            self.point_containment(pos.into(), margin),
            PointContainment::Inside
        )
    }
}

/// Check whether a point falls inside any obstacle's `AABB`, expanded by `margin`.
#[must_use]
pub(super) fn is_point_in_any_obstacle(
    pos: Vec3,
    obstacles: &[Obstacle],
    margin: f32,
) -> PointContainment {
    for obstacle in obstacles {
        match obstacle.point_containment(pos, margin) {
            PointContainment::Inside => return PointContainment::Inside,
            PointContainment::Outside => {},
        }
    }

    PointContainment::Outside
}

/// Check whether any obstacle intersects a line segment by sampling `steps`
/// evenly-spaced points.
#[must_use]
pub(super) fn is_segment_blocked(
    start: Vec3,
    end: Vec3,
    obstacles: &[Obstacle],
    margin: f32,
    steps: u32,
) -> Blockage {
    for i in 0..=steps {
        let t = i.to_f32() / steps.to_f32();
        let point = start.lerp(end, t);
        match is_point_in_any_obstacle(point, obstacles, margin) {
            PointContainment::Inside => return Blockage::Blocked,
            PointContainment::Outside => {},
        }
    }

    Blockage::Clear
}
