use bevy::prelude::*;
use bevy_catenary::CatenarySolver;
use bevy_catenary::CurveKind;
use bevy_catenary::PathStrategy;
use bevy_catenary::Solver;

use crate::constants::NODE_Y;
use crate::constants::SECTION_X;
use crate::constants::SLACK_NORMAL;
use crate::constants::SPAN_HALF_X;
use crate::entities;

/// Section 2: Catenary, linear, and orthogonal solvers side by side.
pub(super) fn setup_section_solver_comparison(
    commands: &mut Commands,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[2];

    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, -1.5);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, -1.5);
    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(
        commands,
        start,
        end,
        Solver::Catenary(CatenarySolver::new().with_slack(SLACK_NORMAL)),
        vec![],
        cable_mat,
    );

    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(commands, start, end, Solver::Linear, vec![], cable_mat);

    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y - 0.5, 1.5);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y + 0.5, 1.5);
    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(
        commands,
        start,
        end,
        Solver::Routed {
            path_strategy: PathStrategy::Orthogonal,
            curve_kind:    CurveKind::Linear,
            resolution:    0,
        },
        vec![],
        cable_mat,
    );
}
