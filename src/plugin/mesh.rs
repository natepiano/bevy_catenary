//! Procedural tube mesh generation from `CableGeometry`.
//!
//! Generates a tube mesh by sweeping a circular cross-section along the cable path,
//! using rotation-minimizing frames to avoid twisting artifacts.

use bevy::mesh::Indices;
use bevy::mesh::PrimitiveTopology;
use bevy::prelude::*;
use bevy_kana::ToF32;
use bevy_kana::ToU32;
use bevy_kana::ToUsize;

use crate::routing::CableGeometry;

/// Default elbow bend radius multiplier (used in `CableMeshConfig::default`).
const ELBOW_BEND_RADIUS_MULTIPLIER: f32 = 1.0;

/// Default minimum elbow radius multiplier (used in `CableMeshConfig::default`).
const MIN_ELBOW_RADIUS_MULTIPLIER: f32 = 0.5;

/// Default rings per 90 degrees of bend (used in `CableMeshConfig::default`).
const KNEE_RINGS_PER_RIGHT_ANGLE: u32 = 32;

/// Default arm multiplier for Bézier control points (used in `CableMeshConfig::default`).
const DEFAULT_ARM_MULTIPLIER: f32 = 1.0;

/// Push a triangle with correct winding. When `flip` is true, the winding is reversed
/// so that the face normal points the opposite direction.
fn push_tri(indices: &mut Vec<u32>, a: u32, b: u32, c: u32, flip: bool) {
    indices.push(a);
    if flip {
        indices.push(c);
        indices.push(b);
    } else {
        indices.push(b);
        indices.push(c);
    }
}

/// Push a quad (two triangles) between two ring edges. When `flip` is true,
/// the winding is reversed so that face normals point the opposite direction.
fn push_quad(indices: &mut Vec<u32>, a: u32, b: u32, c: u32, d: u32, flip: bool) {
    push_tri(indices, a, b, c, flip);
    push_tri(indices, b, d, c, flip);
}

/// Mutable references to the mesh attribute buffers being built.
///
/// Bundles the four output vectors that every mesh-building helper needs,
/// reducing argument counts on internal functions.
struct MeshBuffers<'a> {
    positions: &'a mut Vec<[f32; 3]>,
    normals:   &'a mut Vec<[f32; 3]>,
    uvs:       &'a mut Vec<[f32; 2]>,
    indices:   &'a mut Vec<u32>,
}

/// Resolve elbow arm lengths from per-elbow overrides or the global multiplier.
fn resolve_elbow_arms(
    config: &CableMeshConfig,
    elbow_idx: usize,
    p0: Vec3,
    p3: Vec3,
    max_arm: f32,
) -> (f32, f32) {
    config
        .elbow_arm_overrides
        .as_ref()
        .and_then(|overrides| overrides.get(elbow_idx))
        .map_or_else(
            || {
                let arm = (p0.distance(p3) / 3.0 * config.elbow_arm_multiplier).min(max_arm);
                (arm, arm)
            },
            |&(a1, a2)| (a1.clamp(0.0, max_arm), a2.clamp(0.0, max_arm)),
        )
}

/// How to cap each end of a tube mesh.
///
/// Surface normal is only relevant for [`CapStyle::Flat`] caps and is encoded directly
/// in the variant — invalid states (normal on a [`CapStyle::Round`] cap) are unrepresentable.
#[derive(Clone, Debug, Default, Reflect)]
pub enum CapStyle {
    /// Open end — no cap geometry. Used internally for shared hub junctions.
    None,
    /// Hemisphere cap (smooth rounded end).
    #[default]
    Round,
    /// Flat disc cap. `normal` determines orientation.
    /// `None` uses the cable's tangent direction.
    Flat {
        /// Cap orientation normal. `None` = use cable tangent.
        normal: Option<Vec3>,
    },
}

impl CapStyle {
    /// Flat cap using the cable's tangent direction.
    #[must_use]
    pub const fn flat() -> Self { Self::Flat { normal: None } }

    /// Flat cap with an explicit orientation normal.
    #[must_use]
    pub const fn flat_with_normal(normal: Vec3) -> Self {
        Self::Flat {
            normal: Some(normal),
        }
    }
}

/// Which sides of the tube surface to render.
#[derive(Clone, Debug, Default, Reflect)]
pub enum FaceSides {
    /// Render only the outside (default — backface culling hides interior).
    #[default]
    Outside,
    /// Render only the inside (useful for tunnels viewed from within).
    Inside,
    /// Render both sides (outside + interior faces).
    Both,
}

/// Configuration for cable mesh generation. Attach to a [`Cable`] entity to control
/// how its tube mesh is rendered.
#[derive(Component, Clone, Debug, Reflect)]
pub struct CableMeshConfig {
    /// Radius of the tube cross-section.
    pub radius:                       f32,
    /// Number of vertices around the cross-section circle.
    pub sides:                        u32,
    /// Cap style for the start end of the tube.
    pub cap_start:                    CapStyle,
    /// Cap style for the end of the tube.
    pub cap_end:                      CapStyle,
    /// Distance to trim from the start (tube begins this far along the path).
    pub trim_start:                   f32,
    /// Distance to trim from the end (tube ends this far before the path end).
    pub trim_end:                     f32,
    /// Elbow bend radius multiplier relative to tube radius.
    pub elbow_bend_radius_multiplier: f32,
    /// Minimum elbow radius multiplier — below this, elbows are skipped.
    pub elbow_min_radius_multiplier:  f32,
    /// Number of rings per 90 degrees of elbow bend.
    pub elbow_rings_per_right_angle:  u32,
    /// Minimum angle (degrees) between consecutive tangents to trigger an elbow.
    pub elbow_angle_threshold_deg:    f32,
    /// Multiplier for Bézier arm length at elbows (default 1.0).
    /// Controls how far P1/P2 extend from P0/P3: `arm = (distance / 3.0) * multiplier`.
    pub elbow_arm_multiplier:         f32,
    /// Which sides of the tube surface to render.
    pub face_sides:                   FaceSides,
    /// Material to apply to the generated mesh. If `None`, no material is added.
    pub material:                     Option<Handle<StandardMaterial>>,
    /// Per-elbow arm overrides as `(p1_arm, p2_arm)` distances.
    /// When set, each elbow uses its own independent arm lengths instead of the global
    /// `elbow_arm_multiplier`. `None` = use the global multiplier for all elbows.
    pub elbow_arm_overrides:          Option<Vec<(f32, f32)>>,
}

impl Default for CableMeshConfig {
    fn default() -> Self {
        Self {
            radius:                       0.02,
            sides:                        8,
            cap_start:                    CapStyle::Round,
            cap_end:                      CapStyle::Round,
            trim_start:                   0.0,
            trim_end:                     0.0,
            elbow_bend_radius_multiplier: ELBOW_BEND_RADIUS_MULTIPLIER,
            elbow_min_radius_multiplier:  MIN_ELBOW_RADIUS_MULTIPLIER,
            elbow_rings_per_right_angle:  KNEE_RINGS_PER_RIGHT_ANGLE,
            elbow_angle_threshold_deg:    25.0,
            elbow_arm_multiplier:         DEFAULT_ARM_MULTIPLIER,
            elbow_arm_overrides:          None,
            face_sides:                   FaceSides::default(),
            material:                     None,
        }
    }
}

/// Flatten all geometry segments into one continuous polyline, deduplicating boundary points.
fn flatten_geometry(geometry: &CableGeometry) -> (Vec<Vec3>, Vec<Vec3>, Vec<f32>) {
    let mut points: Vec<Vec3> = Vec::new();
    let mut tangents: Vec<Vec3> = Vec::new();
    let mut arc_lengths: Vec<f32> = Vec::new();
    let mut arc_offset = 0.0_f32;

    for segment in &geometry.segments {
        if segment.points.len() < 2 {
            arc_offset += segment.length;
            continue;
        }

        let start_idx = usize::from(!points.is_empty());

        for i in start_idx..segment.points.len() {
            points.push(segment.points[i]);
            tangents.push(segment.tangents[i]);
            arc_lengths.push(segment.arc_lengths[i] + arc_offset);
        }

        arc_offset += segment.length;
    }

    (points, tangents, arc_lengths)
}

/// Add start and end caps to the tube mesh.
fn add_end_caps(
    all_points: &[Vec3],
    all_tangents: &[Vec3],
    frames: &[(Vec3, Vec3)],
    config: &CableMeshConfig,
    sides: u32,
    point_count: usize,
    buffers: &mut MeshBuffers,
) {
    let cap_rings = sides.max(8);

    if point_count < 2 {
        return;
    }

    match config.cap_start {
        CapStyle::Round => {
            let (start_normal, start_binormal) = frames[0];
            add_hemisphere_cap(
                &all_points[0],
                &(-all_tangents[0]),
                &start_normal,
                &start_binormal,
                config.radius,
                sides,
                cap_rings,
                0,
                true,
                buffers,
            );
        },
        CapStyle::Flat { ref normal } => {
            let dir = normal.unwrap_or(-all_tangents[0]);
            add_flat_cap(&all_points[0], &dir, 0, sides, true, buffers);
        },
        CapStyle::None => {},
    }

    let last = point_count - 1;
    let last_ring_base = (last * sides.to_usize()).to_u32();
    match config.cap_end {
        CapStyle::Round => {
            let (end_normal, end_binormal) = frames[last];
            add_hemisphere_cap(
                &all_points[last],
                &all_tangents[last],
                &end_normal,
                &end_binormal,
                config.radius,
                sides,
                cap_rings,
                last_ring_base,
                false,
                buffers,
            );
        },
        CapStyle::Flat { ref normal } => {
            let dir = normal.unwrap_or(all_tangents[last]);
            add_flat_cap(
                &all_points[last],
                &dir,
                last_ring_base,
                sides,
                false,
                buffers,
            );
        },
        CapStyle::None => {},
    }
}

/// Generate a tube `Mesh` from cable geometry.
///
/// All segments are flattened into a single continuous polyline,
/// producing one seamless tube for the entire cable path.
#[must_use]
pub fn generate_tube_mesh(geometry: &CableGeometry, config: &CableMeshConfig) -> Mesh {
    let sides = config.sides.max(3);
    let total_length = geometry.total_length.max(0.001);

    let (mut all_points, mut all_tangents, mut all_arc_lengths) = flatten_geometry(geometry);

    if all_points.len() < 2 {
        return Mesh::new(PrimitiveTopology::TriangleList, default());
    }

    // Trim start/end of the path if configured (for junction hiding).
    if config.trim_start > 0.0 || config.trim_end > 0.0 {
        trim_path(
            &mut all_points,
            &mut all_tangents,
            &mut all_arc_lengths,
            config.trim_start,
            config.trim_end,
        );
    }

    if all_points.len() < 2 {
        return Mesh::new(PrimitiveTopology::TriangleList, default());
    }

    // Insert elbow joints at sharp bends to create smooth rounded curves.
    let (all_points, all_tangents, all_arc_lengths) =
        insert_knee_rings(all_points, all_arc_lengths, config);
    let point_count = all_points.len();

    // Compute rotation-minimizing frames along the entire path
    let frames = compute_rmf(&all_points, &all_tangents);

    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(point_count * sides.to_usize());
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(point_count * sides.to_usize());
    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(point_count * sides.to_usize());
    let mut indices: Vec<u32> = Vec::new();

    for (i, ((point, ..), (frame_normal, binormal))) in all_points
        .iter()
        .zip(&all_tangents)
        .zip(&frames)
        .enumerate()
    {
        let arc_u = all_arc_lengths[i] / total_length;

        // Generate cross-section ring
        for j in 0..sides {
            let angle = (j.to_f32() / sides.to_f32()) * std::f32::consts::TAU;
            let (sin_a, cos_a) = angle.sin_cos();

            let offset = *frame_normal * cos_a * config.radius + *binormal * sin_a * config.radius;
            let vertex_pos = *point + offset;
            let vertex_normal = offset.normalize_or_zero();

            positions.push(vertex_pos.to_array());
            normals.push(vertex_normal.to_array());
            uvs.push([arc_u, j.to_f32() / sides.to_f32()]);
        }

        // Connect to previous ring with triangles
        if i > 0 {
            let base = ((i - 1) * sides.to_usize()).to_u32();
            let next_base = (i * sides.to_usize()).to_u32();

            for j in 0..sides {
                let j_next = (j + 1) % sides;

                let curr_j = base + j;
                let curr_j_next = base + j_next;
                let next_j = next_base + j;
                let next_j_next = next_base + j_next;

                match config.face_sides {
                    FaceSides::Outside => push_quad(
                        &mut indices,
                        curr_j,
                        curr_j_next,
                        next_j,
                        next_j_next,
                        false,
                    ),
                    FaceSides::Inside => {
                        push_quad(&mut indices, curr_j, curr_j_next, next_j, next_j_next, true);
                    },
                    FaceSides::Both => {
                        push_quad(
                            &mut indices,
                            curr_j,
                            curr_j_next,
                            next_j,
                            next_j_next,
                            false,
                        );
                        push_quad(&mut indices, curr_j, curr_j_next, next_j, next_j_next, true);
                    },
                }
            }
        }
    }

    // Add caps at ends
    let mut buffers = MeshBuffers {
        positions: &mut positions,
        normals:   &mut normals,
        uvs:       &mut uvs,
        indices:   &mut indices,
    };
    add_end_caps(
        &all_points,
        &all_tangents,
        &frames,
        config,
        sides,
        point_count,
        &mut buffers,
    );

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

/// Trim the start and/or end of a path by removing points within the trim distance.
/// Interpolates a new endpoint at the exact trim boundary.
fn trim_path(
    points: &mut Vec<Vec3>,
    tangents: &mut Vec<Vec3>,
    arc_lengths: &mut Vec<f32>,
    trim_start: f32,
    trim_end: f32,
) {
    let total = *arc_lengths.last().unwrap_or(&0.0);

    // Trim start: remove points before trim_start distance
    if trim_start > 0.0 && points.len() >= 2 {
        let cut = arc_lengths
            .iter()
            .position(|&al| al >= trim_start)
            .unwrap_or(0);
        if cut > 0 && cut < points.len() {
            // Interpolate a new start point at the trim boundary
            let prev = cut - 1;
            let seg_len = arc_lengths[cut] - arc_lengths[prev];
            if seg_len > f32::EPSILON {
                let t = (trim_start - arc_lengths[prev]) / seg_len;
                let new_point = points[prev].lerp(points[cut], t);
                let new_tangent = tangents[prev].lerp(tangents[cut], t).normalize_or_zero();
                points[prev] = new_point;
                tangents[prev] = new_tangent;
                arc_lengths[prev] = trim_start;
            }
            points.drain(..prev);
            tangents.drain(..prev);
            arc_lengths.drain(..prev);
        }
    }

    // Trim end: remove points after (total - trim_end) distance
    if trim_end > 0.0 && points.len() >= 2 {
        let end_boundary = total - trim_end;
        let mut cut = points.len();
        for i in (0..points.len()).rev() {
            if arc_lengths[i] <= end_boundary {
                cut = i + 1;
                break;
            }
        }
        if cut > 0 && cut < points.len() {
            // Interpolate a new end point at the trim boundary
            let seg_len = arc_lengths[cut] - arc_lengths[cut - 1];
            if seg_len > f32::EPSILON {
                let t = (end_boundary - arc_lengths[cut - 1]) / seg_len;
                let new_point = points[cut - 1].lerp(points[cut], t);
                let new_tangent = tangents[cut - 1].lerp(tangents[cut], t).normalize_or_zero();
                points[cut] = new_point;
                tangents[cut] = new_tangent;
                arc_lengths[cut] = end_boundary;
            }
            points.truncate(cut + 1);
            tangents.truncate(cut + 1);
            arc_lengths.truncate(cut + 1);
        }
    }
}

/// Smooth sharp bends in the path using cubic Bézier fillets.
///
/// At each sharp bend, the corner region is replaced by a smooth cubic Bézier
/// curve. Uses actual segment directions (not stored central-difference tangents)
/// for bend detection and fillet geometry. Tangents are recomputed from the
/// final path positions before returning, ensuring consistency with `compute_rmf`.
fn insert_knee_rings(
    points: Vec<Vec3>,
    arc_lengths: Vec<f32>,
    config: &CableMeshConfig,
) -> (Vec<Vec3>, Vec<Vec3>, Vec<f32>) {
    let point_count = points.len();
    if point_count < 2 {
        let tangents = recompute_tangents(&points);
        return (points, tangents, arc_lengths);
    }

    let tube_radius = config.radius;
    let bend_radius = tube_radius * config.elbow_bend_radius_multiplier;
    let angle_threshold_cos = (config.elbow_angle_threshold_deg.to_radians()).cos();
    let rings_per_right_angle = config.elbow_rings_per_right_angle;
    let min_bend_r = tube_radius * config.elbow_min_radius_multiplier;

    let mut out_pts = Vec::with_capacity(point_count * 2);
    let mut out_arc = Vec::with_capacity(point_count * 2);

    out_pts.push(points[0]);
    out_arc.push(arc_lengths[0]);

    let mut elbow_idx = 0_usize;
    let mut i = 1;
    while i < point_count {
        // Use actual segment directions, not stored central-difference tangents.
        let dir_in = if i > 0 {
            (points[i] - points[i - 1]).normalize_or_zero()
        } else {
            Vec3::Z
        };
        let dir_out = if i + 1 < point_count {
            (points[i + 1] - points[i]).normalize_or_zero()
        } else {
            // Last point — no outgoing segment, just pass through.
            out_pts.push(points[i]);
            out_arc.push(arc_lengths[i]);
            i += 1;
            continue;
        };

        let cos_a = dir_in.dot(dir_out).clamp(-1.0, 1.0);

        if cos_a >= angle_threshold_cos {
            out_pts.push(points[i]);
            out_arc.push(arc_lengths[i]);
            i += 1;
            continue;
        }

        let theta = cos_a.acos();

        if bend_radius < min_bend_r {
            out_pts.push(points[i]);
            out_arc.push(arc_lengths[i]);
            i += 1;
            continue;
        }

        let half = theta * 0.5;
        let fillet_reach = bend_radius * half.tan();
        let corner = points[i];

        // Fillet start/end computed from actual segment directions.
        let fillet_start = corner - dir_in * fillet_reach;
        let fillet_end = corner + dir_out * fillet_reach;

        // Remove output points that overlap with the fillet's backward reach.
        while out_pts.len() > 1 {
            let last = out_pts[out_pts.len() - 1];
            if (last - fillet_start).dot(dir_in) > 0.0 {
                out_pts.pop();
                out_arc.pop();
            } else {
                break;
            }
        }

        // Add explicit fillet_start point for precise join location.
        let base_arc = out_arc.last().copied().unwrap_or(0.0);
        let dist_to_fillet_start = out_pts
            .last()
            .map_or(0.0, |last| last.distance(fillet_start));
        out_pts.push(fillet_start);
        out_arc.push(base_arc + dist_to_fillet_start);

        // Cubic Bézier with tangent-matched endpoints:
        // P0 = fillet_start, tangent at start = dir_in
        // P3 = fillet_end,   tangent at end   = dir_out
        let p0 = fillet_start;
        let p3 = fillet_end;
        let max_arm = fillet_reach * 0.95;
        let (p1_arm, p2_arm) = resolve_elbow_arms(config, elbow_idx, p0, p3, max_arm);
        let p1 = p0 + dir_in * p1_arm;
        let p2 = p3 - dir_out * p2_arm;
        elbow_idx += 1;

        let num_rings = ((theta / std::f32::consts::FRAC_PI_2) * rings_per_right_angle.to_f32())
            .ceil()
            .max(3.0)
            .to_u32();

        let fillet_base_arc = out_arc[out_arc.len() - 1];

        // Estimate fillet length via sampled chord distances.
        let q1 = 0.125 * (p0 + 3.0 * p1 + 3.0 * p2 + p3) - 0.0625 * (3.0 * p0 + p3);
        let fillet_len = p0.distance(q1).mul_add(2.0, q1.distance(p3) * 2.0);

        for k in 1..=num_rings {
            let t = k.to_f32() / num_rings.to_f32();
            let omt = 1.0 - t;

            // Cubic Bézier: B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
            let pos = omt * omt * omt * p0
                + 3.0 * omt * omt * t * p1
                + 3.0 * omt * t * t * p2
                + t * t * t * p3;

            let al = fillet_base_arc + t * fillet_len;

            out_pts.push(pos);
            out_arc.push(al);
        }

        // Skip forward input points that overlap with the fillet's forward reach.
        i += 1;
        while i < point_count {
            if (points[i] - fillet_end).dot(dir_out) < 0.0 {
                i += 1;
            } else {
                break;
            }
        }
    }

    // Recompute tangents from the modified path positions.
    // This ensures tangents are consistent with actual segment directions,
    // which is critical for `compute_rmf` to produce correct frames.
    let out_tan = recompute_tangents(&out_pts);

    (out_pts, out_tan, out_arc)
}

/// Recompute tangents from path positions using segment directions.
///
/// Interior points use the average of incoming and outgoing directions (central difference).
/// Endpoints use the direction of their single adjacent segment.
fn recompute_tangents(points: &[Vec3]) -> Vec<Vec3> {
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![Vec3::Z];
    }

    let mut tangents = Vec::with_capacity(n);

    // First point: forward direction
    tangents.push((points[1] - points[0]).normalize_or_zero());

    // Interior points: average of incoming and outgoing directions
    for i in 1..n - 1 {
        let dir_in = (points[i] - points[i - 1]).normalize_or_zero();
        let dir_out = (points[i + 1] - points[i]).normalize_or_zero();
        tangents.push((dir_in + dir_out).normalize_or_zero());
    }

    // Last point: backward direction
    tangents.push((points[n - 1] - points[n - 2]).normalize_or_zero());

    tangents
}

/// Metadata about a single elbow fillet, for visualization and interactive editing.
#[derive(Clone, Debug)]
pub struct ElbowMetadata {
    /// Fillet start point (on incoming straight section).
    pub p0:      Vec3,
    /// First control point (along incoming direction from `p0`).
    pub p1:      Vec3,
    /// Second control point (along outgoing direction toward `p3`).
    pub p2:      Vec3,
    /// Fillet end point (on outgoing straight section).
    pub p3:      Vec3,
    /// Incoming segment direction at the elbow.
    pub dir_in:  Vec3,
    /// Outgoing segment direction at the elbow.
    pub dir_out: Vec3,
    /// Arm length for P1 (distance from `p0` to `p1` along `dir_in`).
    pub p1_arm:  f32,
    /// Arm length for P2 (distance from `p3` to `p2` along `-dir_out`).
    pub p2_arm:  f32,
    /// Fillet reach distance — maximum arm before loops form.
    pub d:       f32,
}

/// Compute elbow metadata for all fillet bends in the geometry.
///
/// Returns one `ElbowMetadata` per detected sharp bend, containing the four
/// Bézier control points and constraint directions. This is useful for
/// debug visualization and interactive editing of elbow curves.
#[must_use]
pub fn compute_elbow_metadata(
    geometry: &CableGeometry,
    config: &CableMeshConfig,
) -> Vec<ElbowMetadata> {
    // Flatten geometry into a single polyline (same as generate_tube_mesh).
    let mut points: Vec<Vec3> = Vec::new();
    let mut arc_lengths: Vec<f32> = Vec::new();
    let mut arc_offset = 0.0_f32;

    for segment in &geometry.segments {
        if segment.points.len() < 2 {
            arc_offset += segment.length;
            continue;
        }
        let start_idx = usize::from(!points.is_empty());
        for i in start_idx..segment.points.len() {
            points.push(segment.points[i]);
            arc_lengths.push(segment.arc_lengths[i] + arc_offset);
        }
        arc_offset += segment.length;
    }

    if points.len() < 3 {
        return Vec::new();
    }

    // Trim (same logic as generate_tube_mesh).
    if config.trim_start > 0.0 || config.trim_end > 0.0 {
        let mut tangents = recompute_tangents(&points);
        trim_path(
            &mut points,
            &mut tangents,
            &mut arc_lengths,
            config.trim_start,
            config.trim_end,
        );
    }

    let n = points.len();
    let tube_radius = config.radius;
    let bend_radius = tube_radius * config.elbow_bend_radius_multiplier;
    let angle_threshold_cos = (config.elbow_angle_threshold_deg.to_radians()).cos();
    let min_bend_r = tube_radius * config.elbow_min_radius_multiplier;

    let mut elbows = Vec::new();
    let mut elbow_idx = 0_usize;

    for i in 1..n - 1 {
        let dir_in = (points[i] - points[i - 1]).normalize_or_zero();
        let dir_out = (points[i + 1] - points[i]).normalize_or_zero();
        let cos_a = dir_in.dot(dir_out).clamp(-1.0, 1.0);

        if cos_a >= angle_threshold_cos {
            continue;
        }

        if bend_radius < min_bend_r {
            continue;
        }

        let theta = cos_a.acos();
        let half = theta * 0.5;
        let fillet_reach = bend_radius * half.tan();
        let corner = points[i];

        let p0 = corner - dir_in * fillet_reach;
        let p3 = corner + dir_out * fillet_reach;
        let max_arm = fillet_reach * 0.95;
        let (p1_arm, p2_arm) = resolve_elbow_arms(config, elbow_idx, p0, p3, max_arm);
        let p1 = p0 + dir_in * p1_arm;
        let p2 = p3 - dir_out * p2_arm;

        elbows.push(ElbowMetadata {
            p0,
            p1,
            p2,
            p3,
            dir_in,
            dir_out,
            p1_arm,
            p2_arm,
            d: fillet_reach,
        });
        elbow_idx += 1;
    }

    elbows
}

/// Adds a hemisphere cap to the mesh, connecting to an existing tube ring.
///
/// `equator_ring_base` is the index of the first vertex in the existing tube ring
/// that forms the equator of this hemisphere. The hemisphere sweeps from that ring
/// toward a pole in the `cap_direction`.
#[allow(clippy::too_many_arguments)]
fn add_hemisphere_cap(
    center: &Vec3,
    cap_direction: &Vec3,
    frame_normal: &Vec3,
    binormal: &Vec3,
    radius: f32,
    sides: u32,
    cap_rings: u32,
    equator_ring_base: u32,
    flip_winding: bool,
    buffers: &mut MeshBuffers,
) {
    let mut prev_ring_base = equator_ring_base;

    for k in 1..cap_rings {
        let phi = (k.to_f32() / cap_rings.to_f32()) * std::f32::consts::FRAC_PI_2;
        let ring_radius = phi.cos() * radius;
        let along_offset = phi.sin() * radius;
        let ring_center = *center + *cap_direction * along_offset;

        let new_ring_base = buffers.positions.len().to_u32();

        for j in 0..sides {
            let angle = (j.to_f32() / sides.to_f32()) * std::f32::consts::TAU;
            let (sin_a, cos_a) = angle.sin_cos();

            let radial = *frame_normal * cos_a + *binormal * sin_a;
            let vertex_pos = ring_center + radial * ring_radius;
            let vertex_normal = (radial * phi.cos() + *cap_direction * phi.sin()).normalize();

            buffers.positions.push(vertex_pos.to_array());
            buffers.normals.push(vertex_normal.to_array());
            buffers.uvs.push([0.5, 0.5]);
        }

        // Connect to previous ring
        for j in 0..sides {
            let j_next = (j + 1) % sides;

            let a = prev_ring_base + j;
            let b = prev_ring_base + j_next;
            let c = new_ring_base + j;
            let d = new_ring_base + j_next;

            push_quad(buffers.indices, a, b, c, d, flip_winding);
        }

        prev_ring_base = new_ring_base;
    }

    // Pole vertex (tip of hemisphere)
    let pole_pos = *center + *cap_direction * radius;
    let pole_idx = buffers.positions.len().to_u32();
    buffers.positions.push(pole_pos.to_array());
    buffers.normals.push(cap_direction.to_array());
    buffers.uvs.push([0.5, 0.5]);

    // Fan from last ring to pole
    for j in 0..sides {
        let j_next = (j + 1) % sides;
        push_tri(
            buffers.indices,
            prev_ring_base + j,
            prev_ring_base + j_next,
            pole_idx,
            flip_winding,
        );
    }
}

/// Adds a flat disc cap to the mesh, connecting to an existing tube ring.
///
/// The disc is perpendicular to the tube tangent at the endpoint.
/// A center vertex is added and a triangle fan connects it to the ring.
fn add_flat_cap(
    center: &Vec3,
    cap_direction: &Vec3,
    ring_base: u32,
    sides: u32,
    flip_winding: bool,
    buffers: &mut MeshBuffers,
) {
    // Center vertex at the tube endpoint, normal facing outward along cap direction
    let center_idx = buffers.positions.len().to_u32();
    buffers.positions.push(center.to_array());
    buffers.normals.push(cap_direction.to_array());
    buffers.uvs.push([0.5, 0.5]);

    // Triangle fan from ring to center
    for j in 0..sides {
        let j_next = (j + 1) % sides;
        push_tri(
            buffers.indices,
            ring_base + j,
            ring_base + j_next,
            center_idx,
            flip_winding,
        );
    }
}

/// Compute rotation-minimizing frames (parallel transport) along a curve.
///
/// Returns a Vec of (normal, binormal) pairs for each point.
fn compute_rmf(points: &[Vec3], tangents: &[Vec3]) -> Vec<(Vec3, Vec3)> {
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }

    let mut frames = Vec::with_capacity(n);

    // Initial frame: find a vector not parallel to the first tangent
    let t0 = tangents[0];
    let initial_normal = find_perpendicular(t0);
    let initial_binormal = t0.cross(initial_normal).normalize_or_zero();

    frames.push((initial_normal, initial_binormal));

    // Propagate frames using parallel transport (double reflection method)
    for i in 1..n {
        let (prev_normal, ..) = frames[i - 1];
        let prev_tangent = tangents[i - 1];
        let curr_tangent = tangents[i];

        // Reflect previous frame through the plane bisecting the two tangents
        let v1 = points[i] - points[i - 1];
        let c1 = v1.dot(v1);

        if c1 < f32::EPSILON {
            frames.push(frames[i - 1]);
            continue;
        }

        // First reflection
        let r_l = prev_normal - (2.0 / c1) * v1.dot(prev_normal) * v1;
        let r_t = prev_tangent - (2.0 / c1) * v1.dot(prev_tangent) * v1;

        // Second reflection (to align reflected tangent with current tangent)
        let v2 = curr_tangent - r_t;
        let c2 = v2.dot(v2);

        let new_normal = if c2 < f32::EPSILON {
            r_l
        } else {
            r_l - (2.0 / c2) * v2.dot(r_l) * v2
        }
        .normalize_or_zero();

        let new_binormal = curr_tangent.cross(new_normal).normalize_or_zero();

        frames.push((new_normal, new_binormal));
    }

    frames
}

/// Find a vector perpendicular to the given direction.
fn find_perpendicular(v: Vec3) -> Vec3 {
    let candidate = if v.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    v.cross(candidate).normalize_or_zero()
}
