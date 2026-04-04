//! Named constants for the plugin module. No magic values.

use bevy::color::Color;

/// Color for cable gizmo lines (orange).
pub const CABLE_GIZMO_COLOR: Color = Color::srgb(1.0, 0.6, 0.0);

/// Color for waypoint debug dots (green).
pub const WAYPOINT_DOT_COLOR: Color = Color::srgb(0.0, 1.0, 0.0);

/// Radius of waypoint debug dots.
pub const WAYPOINT_DOT_SIZE: f32 = 0.05;

/// Color for tangent vector gizmo lines (yellow).
pub const TANGENT_GIZMO_COLOR: Color = Color::srgb(1.0, 1.0, 0.0);

/// Scale factor for tangent vector gizmo lines.
pub const TANGENT_VECTOR_SCALE: f32 = 0.1;

/// Draw a tangent vector every N-th point along the cable.
pub const TANGENT_SAMPLING_INTERVAL: usize = 4;

/// Radius of segment boundary debug dots.
pub const SEGMENT_BOUNDARY_DOT_SIZE: f32 = 0.03;

/// Color for segment boundary debug dots (red).
pub const SEGMENT_BOUNDARY_COLOR: Color = Color::srgb(1.0, 0.0, 0.0);
