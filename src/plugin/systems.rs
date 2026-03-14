//! Bevy systems for cable route computation and debug rendering.

use bevy::prelude::*;

use super::CableGizmoGroup;
use super::components::Cable;
use super::components::ComputedCableGeometry;
use crate::routing::RouteRequest;

/// Computes cable routes when a `Cable` component changes.
pub fn compute_cable_routes(
    mut cables: Query<(&Cable, &mut ComputedCableGeometry), Changed<Cable>>,
) {
    for (cable, mut computed) in &mut cables {
        let request = RouteRequest {
            start:      cable.start,
            end:        cable.end,
            obstacles:  &cable.obstacles,
            resolution: cable.resolution,
        };

        let geometry = cable.solver.solve(&request);
        computed.geometry = Some(geometry);
    }
}

/// Renders cable geometry as gizmo lines (only when debug is enabled).
pub fn render_cable_gizmos(
    cables: Query<(&ComputedCableGeometry, &GlobalTransform)>,
    mut gizmos: Gizmos<CableGizmoGroup>,
    debug_enabled: Res<CableDebugEnabled>,
) {
    if !debug_enabled.0 {
        return;
    }
    for (computed, _global_transform) in &cables {
        let Some(geometry) = &computed.geometry else {
            continue;
        };

        // Draw cable segments as polylines
        for segment in &geometry.segments {
            if segment.points.len() < 2 {
                continue;
            }

            for pair in segment.points.windows(2) {
                gizmos.line(pair[0], pair[1], Color::srgb(1.0, 0.6, 0.0));
            }
        }

        // Draw waypoints as small crosses
        for &wp in &geometry.waypoints {
            let size = 0.05;
            gizmos.line(
                wp - Vec3::X * size,
                wp + Vec3::X * size,
                Color::srgb(0.0, 1.0, 0.0),
            );
            gizmos.line(
                wp - Vec3::Y * size,
                wp + Vec3::Y * size,
                Color::srgb(0.0, 1.0, 0.0),
            );
            gizmos.line(
                wp - Vec3::Z * size,
                wp + Vec3::Z * size,
                Color::srgb(0.0, 1.0, 0.0),
            );
        }
    }
}

/// Renders detailed debug info: tangent vectors and segment boundaries.
pub fn render_debug_gizmos(
    cables: Query<&ComputedCableGeometry>,
    mut gizmos: Gizmos<CableGizmoGroup>,
    debug_enabled: Res<CableDebugEnabled>,
) {
    if !debug_enabled.0 {
        return;
    }

    for computed in &cables {
        let Some(geometry) = &computed.geometry else {
            continue;
        };

        for segment in &geometry.segments {
            // Draw tangent vectors at every 4th point
            for (i, (point, tangent)) in segment.points.iter().zip(&segment.tangents).enumerate() {
                if i % 4 == 0 {
                    gizmos.line(*point, *point + *tangent * 0.1, Color::srgb(1.0, 1.0, 0.0));
                }
            }

            // Draw segment start/end as red dots
            if let Some(first) = segment.points.first() {
                draw_dot(&mut gizmos, *first, Color::srgb(1.0, 0.0, 0.0));
            }
            if let Some(last) = segment.points.last() {
                draw_dot(&mut gizmos, *last, Color::srgb(1.0, 0.0, 0.0));
            }
        }
    }
}

/// Draw a small cross at a point (debug marker).
fn draw_dot(gizmos: &mut Gizmos<CableGizmoGroup>, point: Vec3, color: Color) {
    let s = 0.03;
    gizmos.line(point - Vec3::X * s, point + Vec3::X * s, color);
    gizmos.line(point - Vec3::Y * s, point + Vec3::Y * s, color);
    gizmos.line(point - Vec3::Z * s, point + Vec3::Z * s, color);
}

/// Resource that toggles detailed debug visualization.
#[derive(Resource)]
pub struct CableDebugEnabled(pub bool);

impl Default for CableDebugEnabled {
    fn default() -> Self { Self(false) }
}
