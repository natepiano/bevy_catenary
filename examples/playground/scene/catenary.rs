use bevy::prelude::*;
use bevy_catenary::CatenarySolver;
use bevy_catenary::Solver;

use crate::constants::NODE_Y;
use crate::constants::SECTION_X;
use crate::constants::SLACK_NORMAL;
use crate::constants::SPAN_HALF_X;
use crate::entities;

/// Section 0: Simple catenary cable between two nodes.
pub(super) fn setup_section_catenary(
    commands: &mut Commands,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[0];
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(
        commands,
        start,
        end,
        Solver::Catenary(CatenarySolver::new().with_slack(SLACK_NORMAL)),
        vec![],
        cable_mat,
    );
}
