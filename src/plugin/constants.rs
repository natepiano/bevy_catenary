//! Named constants for the plugin module. No magic values.

use bevy::color::Color;

// Cable gizmos
pub(super) const CABLE_GIZMO_COLOR: Color = Color::srgb(1.0, 0.6, 0.0);

// Cap defaults
pub(super) const MIN_CAP_RINGS: u32 = 8;

// Elbow defaults
pub(super) const DEFAULT_ARM_MULTIPLIER: f32 = 1.0;
pub(super) const DEFAULT_ELBOW_ANGLE_THRESHOLD_DEG: f32 = 25.0;
pub(super) const DEFAULT_ELBOW_BEND_RADIUS_MULTIPLIER: f32 = 1.0;
pub(super) const DEFAULT_ELBOW_RINGS_PER_RIGHT_ANGLE: u32 = 32;
pub(super) const DEFAULT_MIN_ELBOW_RADIUS_MULTIPLIER: f32 = 0.5;
/// Maximum ratio of arm length to fillet reach, preventing self-intersecting loops.
pub(super) const MAX_ARM_RATIO: f32 = 0.95;

// Segment boundaries
pub(super) const SEGMENT_BOUNDARY_COLOR: Color = Color::srgb(1.0, 0.0, 0.0);
pub(super) const SEGMENT_BOUNDARY_DOT_SIZE: f32 = 0.03;

// Tangent gizmos
pub(super) const TANGENT_GIZMO_COLOR: Color = Color::srgb(1.0, 1.0, 0.0);
pub(super) const TANGENT_SAMPLING_INTERVAL: usize = 4;
pub(super) const TANGENT_VECTOR_SCALE: f32 = 0.1;

// Tube defaults
pub(super) const DEFAULT_TUBE_RADIUS: f32 = 0.06;
pub(super) const DEFAULT_TUBE_SIDES: u32 = 32;

// Perpendicular detection
/// Dot-product threshold above which a vector is considered near-parallel to an axis.
pub(super) const PERPENDICULAR_AXIS_THRESHOLD: f32 = 0.9;

// Waypoints
pub(super) const WAYPOINT_DOT_COLOR: Color = Color::srgb(0.0, 1.0, 0.0);
pub(super) const WAYPOINT_DOT_SIZE: f32 = 0.05;
