use bevy::light::NotShadowCaster;
use bevy::prelude::*;
use bevy_catenary::CatenarySolver;
use bevy_catenary::CurveKind;
use bevy_catenary::Obstacle;
use bevy_catenary::PathStrategy;
use bevy_catenary::Solver;
use bevy_catenary::DEFAULT_SLACK;
use bevy_kana::Position;

use super::super::constants::NODE_Y;
use super::super::constants::OBSTACLE_COLOR;
use super::super::constants::OBSTACLE_HALF_EXTENTS;
use super::super::constants::SECTION_X;
use super::super::constants::SPAN_HALF_X;
use super::super::entities;
use super::super::input;

/// Section 5: A* pathfinding around an obstacle.
pub(super) fn setup_section_astar(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[5];
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    let obstacle_position = Position::new(cx, NODE_Y, 0.0);
    let obstacle = Obstacle::new(OBSTACLE_HALF_EXTENTS, obstacle_position);

    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(
        commands,
        start,
        end,
        Solver::Routed {
            path_strategy: PathStrategy::AStar,
            curve_kind:    CurveKind::Catenary(CatenarySolver::new().with_slack(DEFAULT_SLACK)),
            resolution:    0,
        },
        vec![obstacle],
        cable_mat,
    );

    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                OBSTACLE_HALF_EXTENTS.x * 2.0,
                OBSTACLE_HALF_EXTENTS.y * 2.0,
                OBSTACLE_HALF_EXTENTS.z * 2.0,
            ))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: OBSTACLE_COLOR,
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_translation(*obstacle_position),
            NotShadowCaster,
        ))
        .observe(input::on_mesh_clicked);
}
