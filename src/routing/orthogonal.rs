//! Orthogonal routing — axis-aligned cable paths with 90-degree bends.

use bevy::math::Vec3;
use bevy_kana::ToF32;

use super::constants::DEFAULT_OBSTACLE_MARGIN;
use super::solver::PathPlanner;
use super::types::Obstacle;

/// Plans axis-aligned cable paths with 90-degree bends.
///
/// Routes cables along the primary axes (X, Y, Z), choosing the most
/// direct orthogonal path from start to end. Avoids obstacles when present.
#[derive(Clone, Debug)]
pub struct OrthogonalPlanner {
    /// Clearance around obstacles.
    pub margin:         f32,
    /// When true, prefer routing vertically first then horizontally.
    /// When false, prefer horizontal first.
    pub vertical_first: bool,
}

impl Default for OrthogonalPlanner {
    fn default() -> Self {
        Self {
            margin:         DEFAULT_OBSTACLE_MARGIN,
            vertical_first: false,
        }
    }
}

impl OrthogonalPlanner {
    /// Create an orthogonal planner with default settings.
    #[must_use]
    pub fn new() -> Self { Self::default() }

    /// Set the obstacle clearance margin.
    #[must_use]
    pub const fn with_margin(mut self, margin: f32) -> Self {
        self.margin = margin;
        self
    }

    /// Prefer vertical-first routing.
    #[must_use]
    pub const fn vertical_first(mut self) -> Self {
        self.vertical_first = true;
        self
    }

    /// Check if an axis-aligned segment between two points is blocked.
    fn is_segment_blocked(&self, start: Vec3, end: Vec3, obstacles: &[Obstacle]) -> bool {
        let steps = 10;
        for i in 0..=steps {
            let t = i.to_f32() / steps.to_f32();
            let point = start.lerp(end, t);
            if self.is_point_blocked(point, obstacles) {
                return true;
            }
        }
        false
    }

    /// Check if a point is inside any obstacle (with margin).
    fn is_point_blocked(&self, pos: Vec3, obstacles: &[Obstacle]) -> bool {
        let margin = self.margin;
        obstacles.iter().any(|obs| {
            let min = obs.aabb_min() - Vec3::splat(margin);
            let max = obs.aabb_max() + Vec3::splat(margin);
            pos.x >= min.x
                && pos.x <= max.x
                && pos.y >= min.y
                && pos.y <= max.y
                && pos.z >= min.z
                && pos.z <= max.z
        })
    }

    /// Build an axis-aligned path moving one axis at a time in the given order.
    /// Each step changes exactly one of X, Y, Z.
    fn axis_path(start: Vec3, end: Vec3, order: &[usize]) -> Vec<Vec3> {
        let delta = end - start;
        let mut current = start;
        let mut waypoints = vec![start];

        for &axis in order {
            let step = match axis {
                0 => Vec3::new(delta.x, 0.0, 0.0),
                1 => Vec3::new(0.0, delta.y, 0.0),
                _ => Vec3::new(0.0, 0.0, delta.z),
            };

            if step.length_squared() > f32::EPSILON {
                current += step;
                waypoints.push(current);
            }
        }

        // Ensure we end exactly at the target
        if waypoints
            .last()
            .is_some_and(|last| last.distance(end) > f32::EPSILON)
        {
            waypoints.push(end);
        }

        waypoints
    }

    /// Generate a U-shaped orthogonal path that routes around obstacles by
    /// going out, across, and back in. Each segment is axis-aligned.
    fn u_path(&self, start: Vec3, end: Vec3, obstacles: &[Obstacle]) -> Vec<Vec3> {
        // Find the maximum obstacle extent to route around
        let offset = self.margin.mul_add(
            2.0,
            obstacles.iter().fold(0.0_f32, |acc, obs| {
                let extent = obs.half_extents.max_element();
                acc.max(extent)
            }),
        );

        // Route below obstacles: go down, across X, across Z, up
        let below = start.y.min(end.y) - offset;
        let p1 = Vec3::new(start.x, below, start.z);
        let p2 = Vec3::new(end.x, below, start.z);
        let p3 = Vec3::new(end.x, below, end.z);
        vec![start, p1, p2, p3, end]
    }
}

impl PathPlanner for OrthogonalPlanner {
    fn plan(&self, start: Vec3, end: Vec3, obstacles: &[Obstacle]) -> Vec<Vec3> {
        // Axis orders to try: preferred first, then alternatives
        let orders: &[&[usize]] = if self.vertical_first {
            &[
                &[1, 0, 2], // Y, X, Z
                &[0, 1, 2], // X, Y, Z
                &[0, 2, 1], // X, Z, Y
                &[2, 0, 1], // Z, X, Y
            ]
        } else {
            &[
                &[0, 2, 1], // X, Z, Y
                &[0, 1, 2], // X, Y, Z
                &[1, 0, 2], // Y, X, Z
                &[2, 1, 0], // Z, Y, X
            ]
        };

        for order in orders {
            let path = Self::axis_path(start, end, order);
            if obstacles.is_empty() || !self.is_path_blocked(&path, obstacles) {
                return path;
            }
        }

        // Fallback: U-path around obstacles
        self.u_path(start, end, obstacles)
    }
}

impl OrthogonalPlanner {
    /// Check if any segment of a multi-waypoint path is blocked.
    fn is_path_blocked(&self, waypoints: &[Vec3], obstacles: &[Obstacle]) -> bool {
        waypoints
            .windows(2)
            .any(|pair| self.is_segment_blocked(pair[0], pair[1], obstacles))
    }
}
