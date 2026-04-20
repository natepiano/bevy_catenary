use bevy::prelude::*;
use bevy_kana::ToF32;
use bevy_kana::ToU32;

use super::config;
use super::config::CableMeshConfig;
use super::path;
use crate::plugin::constants::MAX_ARM_RATIO;
use crate::routing::CableGeometry;

/// Metadata about a single elbow fillet, for visualization and interactive editing.
#[derive(Clone, Debug)]
pub struct ElbowMetadata {
    /// Fillet start point (on incoming straight section).
    pub p0:           Vec3,
    /// First control point (along incoming direction from `p0`).
    pub p1:           Vec3,
    /// Second control point (along outgoing direction toward `p3`).
    pub p2:           Vec3,
    /// Fillet end point (on outgoing straight section).
    pub p3:           Vec3,
    /// Incoming segment direction at the elbow.
    pub dir_in:       Vec3,
    /// Outgoing segment direction at the elbow.
    pub dir_out:      Vec3,
    /// Arm length for P1.
    pub control1_arm: f32,
    /// Arm length for P2.
    pub control2_arm: f32,
    /// Fillet reach distance.
    pub fillet_reach: f32,
}

/// Pre-computed elbow detection parameters extracted from `CableMeshConfig`.
struct ElbowParams {
    angle_threshold_cos: f32,
    bend_radius:         f32,
    min_bend_radius:     f32,
}

impl ElbowParams {
    fn from_config(config: &CableMeshConfig) -> Self {
        let tube_radius = config.tube.radius;
        Self {
            angle_threshold_cos: config.elbow.angle_threshold_deg.to_radians().cos(),
            bend_radius:         tube_radius * config.elbow.bend_radius_multiplier,
            min_bend_radius:     tube_radius * config.elbow.min_radius_multiplier,
        }
    }
}

/// Compute `ElbowMetadata` for a single corner point, if the bend is sharp enough.
fn compute_elbow_at_corner(
    dir_in: Vec3,
    dir_out: Vec3,
    corner: Vec3,
    config: &CableMeshConfig,
    elbow_idx: usize,
    params: &ElbowParams,
) -> Option<ElbowMetadata> {
    let cos_angle = dir_in.dot(dir_out).clamp(-1.0, 1.0);
    if cos_angle >= params.angle_threshold_cos {
        return None;
    }

    if params.bend_radius < params.min_bend_radius {
        return None;
    }

    let theta = cos_angle.acos();
    let half_theta = theta * 0.5;
    let fillet_reach = params.bend_radius * half_theta.tan();

    let p0 = corner - dir_in * fillet_reach;
    let p3 = corner + dir_out * fillet_reach;
    let max_arm = fillet_reach * MAX_ARM_RATIO;
    let (control1_arm, control2_arm) =
        config::resolve_elbow_arms(config, elbow_idx, p0, p3, max_arm);
    let p1 = p0 + dir_in * control1_arm;
    let p2 = p3 - dir_out * control2_arm;

    Some(ElbowMetadata {
        p0,
        p1,
        p2,
        p3,
        dir_in,
        dir_out,
        control1_arm,
        control2_arm,
        fillet_reach,
    })
}

/// Smooth sharp bends in the path using cubic Bezier fillets.
pub(super) fn insert_knee_rings(
    points: Vec<Vec3>,
    arc_lengths: Vec<f32>,
    config: &CableMeshConfig,
) -> (Vec<Vec3>, Vec<Vec3>, Vec<f32>) {
    let point_count = points.len();
    if point_count < 2 {
        let tangents = path::recompute_tangents(&points);
        return (points, tangents, arc_lengths);
    }

    let params = ElbowParams::from_config(config);
    let rings_per_right_angle = config.elbow.rings_per_right_angle;
    let mut output_points = Vec::with_capacity(point_count * 2);
    let mut output_arc_lengths = Vec::with_capacity(point_count * 2);

    output_points.push(points[0]);
    output_arc_lengths.push(arc_lengths[0]);

    let mut elbow_idx = 0_usize;
    let mut i = 1;
    while i < point_count {
        let dir_in = (points[i] - points[i - 1]).normalize_or_zero();
        let Some(next_point) = points.get(i + 1).copied() else {
            output_points.push(points[i]);
            output_arc_lengths.push(arc_lengths[i]);
            i += 1;
            continue;
        };
        let dir_out = (next_point - points[i]).normalize_or_zero();

        let Some(metadata) =
            compute_elbow_at_corner(dir_in, dir_out, points[i], config, elbow_idx, &params)
        else {
            output_points.push(points[i]);
            output_arc_lengths.push(arc_lengths[i]);
            i += 1;
            continue;
        };
        elbow_idx += 1;

        while output_points.len() > 1 {
            let last = output_points[output_points.len() - 1];
            if (last - metadata.p0).dot(dir_in) > 0.0 {
                output_points.pop();
                output_arc_lengths.pop();
            } else {
                break;
            }
        }

        let base_arc = output_arc_lengths.last().copied().unwrap_or(0.0);
        let distance_to_fillet_start = output_points
            .last()
            .map_or(0.0, |last| last.distance(metadata.p0));
        output_points.push(metadata.p0);
        output_arc_lengths.push(base_arc + distance_to_fillet_start);

        let theta = metadata
            .dir_in
            .dot(metadata.dir_out)
            .clamp(-1.0, 1.0)
            .acos();
        let num_rings = ((theta / std::f32::consts::FRAC_PI_2) * rings_per_right_angle.to_f32())
            .ceil()
            .max(3.0)
            .to_u32();

        let fillet_base_arc = output_arc_lengths[output_arc_lengths.len() - 1];
        let q1 = 0.125 * (metadata.p0 + 3.0 * metadata.p1 + 3.0 * metadata.p2 + metadata.p3)
            - 0.0625 * (3.0 * metadata.p0 + metadata.p3);
        let fillet_length = metadata
            .p0
            .distance(q1)
            .mul_add(2.0, q1.distance(metadata.p3) * 2.0);

        for k in 1..=num_rings {
            let t = k.to_f32() / num_rings.to_f32();
            let one_minus_t = 1.0 - t;
            let position = one_minus_t * one_minus_t * one_minus_t * metadata.p0
                + 3.0 * one_minus_t * one_minus_t * t * metadata.p1
                + 3.0 * one_minus_t * t * t * metadata.p2
                + t * t * t * metadata.p3;

            output_points.push(position);
            output_arc_lengths.push(fillet_base_arc + t * fillet_length);
        }

        i += 1;
        while i < point_count {
            if (points[i] - metadata.p3).dot(dir_out) < 0.0 {
                i += 1;
            } else {
                break;
            }
        }
    }

    let output_tangents = path::recompute_tangents(&output_points);
    (output_points, output_tangents, output_arc_lengths)
}

/// Compute elbow metadata for all fillet bends in the geometry.
#[must_use]
pub fn compute_elbow_metadata(
    geometry: &CableGeometry,
    config: &CableMeshConfig,
) -> Vec<ElbowMetadata> {
    let flat = path::flatten_geometry(geometry);
    let mut points = flat.points;
    let mut arc_lengths = flat.arc_lengths;

    if points.len() < 3 {
        return Vec::new();
    }

    if config.trim.start > 0.0 || config.trim.end > 0.0 {
        let mut tangents = path::recompute_tangents(&points);
        path::trim_path(
            &mut points,
            &mut tangents,
            &mut arc_lengths,
            config.trim.start,
            config.trim.end,
        );
    }

    let params = ElbowParams::from_config(config);
    let mut elbows = Vec::new();
    let mut elbow_idx = 0_usize;

    for i in 1..points.len() - 1 {
        let dir_in = (points[i] - points[i - 1]).normalize_or_zero();
        let dir_out = (points[i + 1] - points[i]).normalize_or_zero();

        if let Some(metadata) =
            compute_elbow_at_corner(dir_in, dir_out, points[i], config, elbow_idx, &params)
        {
            elbows.push(metadata);
            elbow_idx += 1;
        }
    }

    elbows
}
