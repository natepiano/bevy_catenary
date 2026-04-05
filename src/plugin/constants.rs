//! Named constants for the plugin module. No magic values.

use bevy::color::Color;

// Cable gizmos
pub const CABLE_GIZMO_COLOR: Color = Color::srgb(1.0, 0.6, 0.0);

// Elbow defaults
pub const DEFAULT_ARM_MULTIPLIER: f32 = 1.0;
pub const DEFAULT_ELBOW_ANGLE_THRESHOLD_DEG: f32 = 25.0;
pub const DEFAULT_ELBOW_BEND_RADIUS_MULTIPLIER: f32 = 1.0;
pub const DEFAULT_ELBOW_RINGS_PER_RIGHT_ANGLE: u32 = 32;
pub const DEFAULT_MIN_ELBOW_RADIUS_MULTIPLIER: f32 = 0.5;

// Segment boundaries
pub const SEGMENT_BOUNDARY_COLOR: Color = Color::srgb(1.0, 0.0, 0.0);
pub const SEGMENT_BOUNDARY_DOT_SIZE: f32 = 0.03;

// Tangent gizmos
pub const TANGENT_GIZMO_COLOR: Color = Color::srgb(1.0, 1.0, 0.0);
pub const TANGENT_SAMPLING_INTERVAL: usize = 4;
pub const TANGENT_VECTOR_SCALE: f32 = 0.1;

// Tube defaults
pub const DEFAULT_TUBE_RADIUS: f32 = 0.06;
pub const DEFAULT_TUBE_SIDES: u32 = 32;

// Waypoints
pub const WAYPOINT_DOT_COLOR: Color = Color::srgb(0.0, 1.0, 0.0);
pub const WAYPOINT_DOT_SIZE: f32 = 0.05;
