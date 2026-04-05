//! Named constants for the routing module. No magic values.

use bevy::math::Vec3;

// Catenary solver
pub const MAX_NEWTON_ITERATIONS: u32 = 50;
pub const MIN_CATENARY_PARAM: f32 = 1e-4;
pub const MIN_SEGMENT_LENGTH: f32 = 0.001;
pub const NEWTON_TOLERANCE: f32 = 1e-6;
pub const STRAIGHT_LINE_THRESHOLD: f32 = 1.005;

// Defaults
/// Default gravity direction and magnitude.
pub const DEFAULT_GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
pub const DEFAULT_GRID_SIZE: f32 = 0.5;
pub const DEFAULT_OBSTACLE_MARGIN: f32 = 0.2;
/// Default number of sample points per cable segment.
pub const DEFAULT_RESOLUTION: u32 = 32;
/// Default slack factor (cable length / straight-line distance).
/// 1.0 = taut (straight line), values > 1.0 add sag.
pub const DEFAULT_SLACK: f32 = 1.2;

// Grid pathfinding
pub const ASTAR_SEGMENT_SAMPLE_STEPS: u32 = 20;
pub const COLLINEARITY_THRESHOLD: f32 = 0.999;
pub const DEFAULT_ASTAR_MAX_CELLS: usize = 10_000;
pub const ORTHOGONAL_SEGMENT_SAMPLE_STEPS: u32 = 10;
