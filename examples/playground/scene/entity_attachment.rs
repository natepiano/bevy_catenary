use bevy::prelude::*;
use bevy_catenary::AttachedTo;
use bevy_catenary::Cable;
use bevy_catenary::CableEnd;
use bevy_catenary::CableEndpoint;
use bevy_catenary::CableMeshConfig;
use bevy_catenary::CatenarySolver;
use bevy_catenary::Solver;

use super::super::constants::DRAGGABLE_COLOR;
use super::super::constants::DRAGGABLE_CUBE_SIZE;
use super::super::constants::NODE_Y;
use super::super::constants::SECTION_X;
use super::super::constants::SLACK_NORMAL;
use super::super::constants::SPAN_HALF_X;
use super::super::entities::Draggable;
use super::super::entities::NodeCube;
use super::super::input;

/// Section 3: Cables attached to draggable cubes.
pub(super) fn setup_section_entity_attachment(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[3];
    let drag_mesh = meshes.add(Cuboid::new(
        DRAGGABLE_CUBE_SIZE * 2.0,
        DRAGGABLE_CUBE_SIZE * 2.0,
        DRAGGABLE_CUBE_SIZE * 2.0,
    ));
    let drag_mat = materials.add(StandardMaterial {
        base_color: DRAGGABLE_COLOR,
        ..default()
    });

    let left_cube = commands
        .spawn((
            Mesh3d(drag_mesh.clone()),
            MeshMaterial3d(drag_mat.clone()),
            Transform::from_translation(Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0)),
            Draggable,
            NodeCube,
        ))
        .observe(input::on_drag_start)
        .id();

    let right_cube = commands
        .spawn((
            Mesh3d(drag_mesh),
            MeshMaterial3d(drag_mat),
            Transform::from_translation(Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0)),
            Draggable,
            NodeCube,
        ))
        .observe(input::on_drag_start)
        .id();

    commands
        .spawn((
            Cable {
                solver:     Solver::Catenary(CatenarySolver::new().with_slack(SLACK_NORMAL)),
                obstacles:  vec![],
                resolution: 0,
            },
            CableMeshConfig {
                material: Some(cable_mat.clone()),
                ..default()
            },
        ))
        .with_children(|parent| {
            parent.spawn((
                CableEndpoint::new(CableEnd::Start, Vec3::ZERO),
                AttachedTo(left_cube),
            ));
            parent.spawn((
                CableEndpoint::new(CableEnd::End, Vec3::ZERO),
                AttachedTo(right_cube),
            ));
        });
}
