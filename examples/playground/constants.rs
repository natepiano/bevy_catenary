//! Constants for the playground example, grouped by domain.

use bevy::prelude::*;

// Cable
pub(crate) const OBSTACLE_HALF_EXTENTS: Vec3 = Vec3::new(0.8, 0.8, 0.8);
pub(crate) const SLACK_NORMAL: f32 = 1.15;

// Camera
pub(crate) const NAV_DURATION_MS: u64 = 1200;
pub(crate) const ZOOM_DURATION_MS: u64 = 1000;
pub(crate) const ZOOM_MARGIN_MESH: f32 = 0.15;
pub(crate) const ZOOM_MARGIN_NAV: f32 = 0.12;

// Colors
pub(crate) const CABLE_COLOR: Color = Color::srgb(0.9, 0.5, 0.1);
pub(crate) const DESPAWN_GREEN: Color = Color::srgb(0.3, 0.8, 0.3);
pub(crate) const DESPAWN_RED: Color = Color::srgb(0.8, 0.3, 0.3);
pub(crate) const DETACH_BUMP_BLUE: Color = Color::srgb(0.3, 0.5, 0.9);
pub(crate) const DRAGGABLE_COLOR: Color = Color::srgb(0.2, 0.7, 0.7);
pub(crate) const NODE_COLOR: Color = Color::srgba(0.4, 0.6, 0.8, 0.4);
pub(crate) const OBSTACLE_COLOR: Color = Color::srgba(0.8, 0.2, 0.2, 0.25);
pub(crate) const POINT_LIGHT_COLOR: Color = Color::srgb(1.0, 0.95, 0.8);
pub(crate) const TRANSPARENT_TUBE_COLOR: Color = Color::srgba(0.85, 0.55, 0.2, 0.2);

// Ground
pub(crate) const GROUND_DEPTH: f32 = 14.0;
pub(crate) const GROUND_WIDTH: f32 = 160.0;

// Layout
pub(crate) const CAP_STYLE_RADIUS_MULTIPLIER: f32 = 5.0;
pub(crate) const CAP_STYLE_TUBE_OFFSET: f32 = 0.8;
pub(crate) const CAP_STYLE_TUBE_SPACING: f32 = 2.0;
pub(crate) const DRAGGABLE_CUBE_SIZE: f32 = 0.45;
pub(crate) const HUB_SPHERE_RADIUS: f32 = 0.35;
pub(crate) const INSIDE_VIEW_RADIUS_MULTIPLIER: f32 = 25.0;
pub(crate) const NODE_CUBE_SIZE: f32 = 0.3;
pub(crate) const NODE_Y: f32 = 2.0;
pub(crate) const RAY_EPSILON: f32 = 1e-6;
pub(crate) const SECTION_COUNT: usize = 9;
pub(crate) const SECTION_SPACING: f32 = 16.0;
pub(crate) const SECTION_TITLES: [&str; SECTION_COUNT] = [
    "Simple Catenary",
    "Cap Styles",
    "Solver Comparison",
    "Entity Attachment",
    "Shared Hub",
    "A* Routing",
    "Detach Policy",
    "Inside View",
    "Connector Model",
];
pub(crate) const SECTION_X: [f32; SECTION_COUNT] = [
    -4.0 * SECTION_SPACING,
    -3.0 * SECTION_SPACING,
    -2.0 * SECTION_SPACING,
    -SECTION_SPACING,
    0.0 * SECTION_SPACING,
    1.0 * SECTION_SPACING,
    2.0 * SECTION_SPACING,
    3.0 * SECTION_SPACING,
    4.0 * SECTION_SPACING,
];
pub(crate) const SLACK_ADJUSTMENT_STEP: f32 = 0.01;
pub(crate) const SPAN_HALF_X: f32 = 3.0;

// Light
pub(crate) const DIRECTIONAL_LIGHT_ILLUMINANCE: f32 = 3000.0;
pub(crate) const POINT_LIGHT_INTENSITY: f32 = 20000.0;
pub(crate) const POINT_LIGHT_RANGE: f32 = 2.0;

// Tube mesh
pub(crate) const JOINT_RADIUS_MULTIPLIER: f32 = 1.5;
pub(crate) const JOINT_SPHERE_SEGMENTS: u32 = 16;
pub(crate) const TUBE_RADIUS: f32 = 0.06;
pub(crate) const TUBE_SIDES: u32 = 32;

// UI
pub(crate) const NAV_FONT_SIZE: f32 = 16.0;
pub(crate) const UI_FONT_SIZE: f32 = 14.0;
