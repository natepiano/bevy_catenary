//! Named constants for the routing module. No magic values.

use bevy::math::Vec3;

/// Default number of sample points per cable segment.
pub const DEFAULT_RESOLUTION: u32 = 32;

/// Default slack factor (cable length / straight-line distance).
/// 1.0 = taut (straight line), values > 1.0 add sag.
pub const DEFAULT_SLACK: f32 = 1.2;

/// Default gravity direction and magnitude.
pub const DEFAULT_GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// Default voxel size for A* grid pathfinding.
pub const DEFAULT_GRID_SIZE: f32 = 0.5;

/// Default clearance around obstacles during pathfinding.
pub const DEFAULT_OBSTACLE_MARGIN: f32 = 0.2;

/// Segments shorter than this are collapsed to a single point.
pub const MIN_SEGMENT_LENGTH: f32 = 0.001;

/// Maximum Newton's method iterations for catenary parameter solving.
pub const MAX_NEWTON_ITERATIONS: u32 = 50;

/// Convergence tolerance for Newton's method.
pub const NEWTON_TOLERANCE: f32 = 1e-6;

/// Minimum catenary parameter to avoid division by near-zero.
pub const MIN_CATENARY_PARAM: f32 = 1e-4;

/// Slack values below this degrade to a straight line instead of attempting a catenary solve.
pub const STRAIGHT_LINE_THRESHOLD: f32 = 1.005;
