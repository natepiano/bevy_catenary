//! Catenary curve math and solver.
//!
//! The catenary curve `y = a * cosh(x/a)` describes how a cable hangs under gravity
//! between two fixed points. This module provides standalone math functions and a
//! [`CatenarySolver`] that implements [`RouteSolver`] and [`CurveSolver`].
//!
//! # 3D Catenary Approach
//!
//! 1. Project the problem into a 2D plane containing both endpoints and the gravity vector
//! 2. Solve the 2D catenary for parameter `a` using Newton's method
//! 3. Sample points along the 2D curve
//! 4. Map sampled points back to 3D

use bevy::math::Vec3;

use super::constants::DEFAULT_GRAVITY;
use super::constants::DEFAULT_RESOLUTION;
use super::constants::DEFAULT_SLACK;
use super::constants::MAX_NEWTON_ITERATIONS;
use super::constants::MIN_CATENARY_PARAM;
use super::constants::MIN_SEGMENT_LENGTH;
use super::constants::NEWTON_TOLERANCE;
use super::solver::CurveSolver;
use super::solver::RouteSolver;
use super::types::CableGeometry;
use super::types::CableSegment;
use super::types::RouteRequest;

/// Evaluate the catenary function: `a * cosh(x / a)`.
pub fn evaluate(x: f32, a: f32) -> f32 { a * (x / a).cosh() }

/// Solve for the catenary parameter `a` given horizontal distance, vertical distance,
/// and cable length, using Newton's method.
///
/// The cable length `L` must satisfy `L > sqrt(h^2 + v^2)` (longer than the straight line).
///
/// Returns `None` if the problem is degenerate or Newton's method fails to converge.
pub fn solve_parameter(horizontal_dist: f32, vertical_dist: f32, cable_length: f32) -> Option<f32> {
    let h = horizontal_dist.abs();
    let v = vertical_dist;
    let l = cable_length;

    // Cable must be longer than straight-line distance
    let straight = (h * h + v * v).sqrt();
    if l <= straight + MIN_SEGMENT_LENGTH {
        return None;
    }

    // If horizontal distance is near zero, degenerate to vertical hang
    if h < MIN_SEGMENT_LENGTH {
        return None;
    }

    // We need to solve: L^2 - v^2 = (2a * sinh(h / (2a)))^2
    // Let f(a) = 2a * sinh(h/(2a)) - sqrt(L^2 - v^2)
    // Newton: a_{n+1} = a_n - f(a_n) / f'(a_n)
    let target = (l * l - v * v).sqrt();

    // Initial guess: start with a = h (reasonable for moderate sag)
    let mut a = h;

    for _ in 0..MAX_NEWTON_ITERATIONS {
        if a < MIN_CATENARY_PARAM {
            a = MIN_CATENARY_PARAM;
        }

        let half_h_over_a = h / (2.0 * a);
        let sinh_val = half_h_over_a.sinh();
        let cosh_val = half_h_over_a.cosh();

        let f = 2.0 * a * sinh_val - target;
        // f'(a) = 2*sinh(h/(2a)) - (h/a)*cosh(h/(2a))
        let f_prime = 2.0 * sinh_val - (h / a) * cosh_val;

        if f_prime.abs() < f32::EPSILON {
            // Derivative too small, can't continue
            break;
        }

        let delta = f / f_prime;
        a -= delta;

        if delta.abs() < NEWTON_TOLERANCE {
            return (a > MIN_CATENARY_PARAM).then_some(a);
        }
    }

    // Failed to converge — return current best if reasonable
    (a > MIN_CATENARY_PARAM).then_some(a)
}

/// Sample points along a 3D catenary curve between `start` and `end`.
///
/// The curve sags in the direction of `gravity_dir` (should be normalized).
/// `slack` is the ratio of cable length to straight-line distance (1.0 = taut).
/// `resolution` is the number of sample points (minimum 2).
pub fn sample_3d(
    start: Vec3,
    end: Vec3,
    slack: f32,
    gravity_dir: Vec3,
    resolution: u32,
) -> CableSegment {
    let n = resolution.max(2) as usize;
    let chord = end - start;
    let chord_length = chord.length();

    // Degenerate: endpoints are the same point
    if chord_length < MIN_SEGMENT_LENGTH {
        return CableSegment::from_points(vec![start; n]);
    }

    let cable_length = chord_length * slack.max(1.0);
    let gravity_norm = gravity_dir.normalize_or_zero();

    // If gravity is zero, fall back to straight line
    if gravity_norm.length_squared() < 0.5 {
        return sample_straight_line(start, end, n);
    }

    // Project the problem into a 2D plane:
    // - horizontal axis: along the chord direction, projected onto the plane perpendicular to
    //   gravity
    // - vertical axis: along gravity

    // Decompose chord into horizontal and vertical components
    let vertical_component = chord.dot(gravity_norm);
    let horizontal_vec = chord - vertical_component * gravity_norm;
    let horizontal_dist = horizontal_vec.length();

    // If cable is purely vertical, handle as a special case
    if horizontal_dist < MIN_SEGMENT_LENGTH {
        return sample_vertical_hang(start, end, gravity_norm, cable_length, n);
    }

    let h_axis = horizontal_vec / horizontal_dist;

    // Solve for catenary parameter
    let catenary_a = match solve_parameter(horizontal_dist, vertical_component, cable_length) {
        Some(a) => a,
        None => return sample_parabolic_fallback(start, end, gravity_norm, slack, n),
    };

    // The 2D catenary: y = a * cosh((x - x_offset) / a) + y_offset
    // With boundary conditions at x=0 (start) and x=h (end)

    // Find the horizontal offset of the catenary's lowest point
    // x_offset = h/2 - a * arcsinh(v / (2a * sinh(h/(2a))))
    let half_h = horizontal_dist / 2.0;
    let sinh_half_h_a = (half_h / catenary_a).sinh();
    let x_offset = if sinh_half_h_a.abs() > f32::EPSILON {
        half_h - catenary_a * (vertical_component / (2.0 * catenary_a * sinh_half_h_a)).asinh()
    } else {
        half_h
    };

    // y_offset positions the curve so it passes through the start point
    let y_at_start = catenary_a * ((0.0 - x_offset) / catenary_a).cosh();
    let y_offset = -y_at_start;

    // Sample points along the 2D catenary and map back to 3D
    let mut points = Vec::with_capacity(n);

    for i in 0..n {
        let t = i as f32 / (n - 1) as f32;
        let x_2d = t * horizontal_dist;
        let y_2d = catenary_a * ((x_2d - x_offset) / catenary_a).cosh() + y_offset;

        // Map back to 3D: horizontal position + vertical sag
        let point = start + x_2d * h_axis + y_2d * gravity_norm;
        points.push(point);
    }

    CableSegment::from_points(points)
}

/// Fallback for degenerate cases: straight line between two points.
fn sample_straight_line(start: Vec3, end: Vec3, n: usize) -> CableSegment {
    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f32 / (n - 1) as f32;
        points.push(start.lerp(end, t));
    }
    CableSegment::from_points(points)
}

/// Handle purely vertical cables (start and end aligned with gravity).
fn sample_vertical_hang(
    start: Vec3,
    end: Vec3,
    gravity_norm: Vec3,
    cable_length: f32,
    n: usize,
) -> CableSegment {
    let vertical_dist = (end - start).dot(gravity_norm);
    let excess = cable_length - vertical_dist.abs();

    if excess < MIN_SEGMENT_LENGTH {
        return sample_straight_line(start, end, n);
    }

    // Vertical cable with slack forms a U-shape hanging down then back up
    // Midpoint hangs down by excess/2
    let midpoint = (start + end) / 2.0 + gravity_norm * (excess / 2.0);
    let mut points = Vec::with_capacity(n);
    let half_n = n / 2;

    for i in 0..half_n {
        let t = i as f32 / half_n as f32;
        points.push(start.lerp(midpoint, t));
    }
    for i in 0..(n - half_n) {
        let t = i as f32 / (n - half_n - 1).max(1) as f32;
        points.push(midpoint.lerp(end, t));
    }

    CableSegment::from_points(points)
}

/// Parabolic approximation when Newton's method fails to find a catenary parameter.
/// Uses `y = 4 * sag * t * (1 - t)` for a simple droop.
fn sample_parabolic_fallback(
    start: Vec3,
    end: Vec3,
    gravity_norm: Vec3,
    slack: f32,
    n: usize,
) -> CableSegment {
    let chord = end - start;
    let chord_length = chord.length();
    let sag = chord_length * (slack - 1.0).max(0.05) * 0.5;

    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f32 / (n - 1) as f32;
        let base = start + t * chord;
        let droop = 4.0 * sag * t * (1.0 - t);
        points.push(base + droop * gravity_norm);
    }

    CableSegment::from_points(points)
}

/// Solver that computes catenary curves between cable endpoints.
///
/// Implements both [`CurveSolver`] (for use with [`Router`]) and [`RouteSolver`]
/// (for standalone use without obstacle avoidance).
#[derive(Clone, Debug)]
pub struct CatenarySolver {
    /// Cable length / straight-line distance. Values > 1.0 add sag.
    pub slack:      f32,
    /// Gravity direction (not necessarily normalized; magnitude is ignored).
    pub gravity:    Vec3,
    /// Default sample resolution when not specified by the request.
    pub resolution: u32,
}

impl Default for CatenarySolver {
    fn default() -> Self {
        Self {
            slack:      DEFAULT_SLACK,
            gravity:    DEFAULT_GRAVITY,
            resolution: DEFAULT_RESOLUTION,
        }
    }
}

impl CatenarySolver {
    /// Create a catenary solver with default parameters.
    pub fn new() -> Self { Self::default() }

    /// Set the slack factor.
    pub fn with_slack(mut self, slack: f32) -> Self {
        self.slack = slack;
        self
    }

    /// Set the gravity vector.
    pub fn with_gravity(mut self, gravity: Vec3) -> Self {
        self.gravity = gravity;
        self
    }

    /// Set the default sample resolution.
    pub fn with_resolution(mut self, resolution: u32) -> Self {
        self.resolution = resolution;
        self
    }
}

impl CurveSolver for CatenarySolver {
    fn solve_segment(&self, start: Vec3, end: Vec3, resolution: u32) -> CableSegment {
        let gravity_dir = self.gravity.normalize_or_zero();
        sample_3d(start, end, self.slack, gravity_dir, resolution)
    }
}

impl RouteSolver for CatenarySolver {
    fn solve(&self, request: &RouteRequest) -> CableGeometry {
        let resolution = if request.resolution > 0 {
            request.resolution
        } else {
            self.resolution
        };

        let segment = self.solve_segment(request.start, request.end, resolution);
        let waypoints = vec![request.start, request.end];
        CableGeometry::from_segments(vec![segment], waypoints)
    }
}
