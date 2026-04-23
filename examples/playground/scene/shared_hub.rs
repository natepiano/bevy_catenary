use bevy::prelude::*;
use bevy_catenary::AttachedTo;
use bevy_catenary::Cable;
use bevy_catenary::CableEnd;
use bevy_catenary::CableEndpoint;
use bevy_catenary::CableMeshConfig;
use bevy_catenary::CatenarySolver;
use bevy_catenary::DEFAULT_SLACK;
use bevy_catenary::Solver;

use super::super::constants::DRAGGABLE_COLOR;
use super::super::constants::HUB_SPHERE_RADIUS;
use super::super::constants::NODE_Y;
use super::super::constants::SECTION_X;
use super::super::entities;
use super::super::entities::Draggable;
use super::super::entities::NodeCube;
use super::super::input;

/// Section 4: Three cables from a central draggable hub.
pub(super) fn setup_section_shared_hub(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[4];
    let drag_mesh = meshes.add(Sphere::new(HUB_SPHERE_RADIUS).mesh().uv(32, 32));
    let drag_mat = materials.add(StandardMaterial {
        base_color: DRAGGABLE_COLOR,
        ..default()
    });

    let hub = commands
        .spawn((
            Mesh3d(drag_mesh),
            MeshMaterial3d(drag_mat),
            Transform::from_translation(Vec3::new(cx, NODE_Y, 0.0)),
            Draggable,
            NodeCube,
        ))
        .observe(input::on_drag_start)
        .id();

    let spokes = [
        Vec3::new(cx - 3.0, NODE_Y + 0.5, -1.5),
        Vec3::new(cx + 3.0, NODE_Y + 0.5, -1.5),
        Vec3::new(cx, NODE_Y + 0.5, 2.0),
    ];

    for spoke_pos in spokes {
        entities::spawn_node_cube(commands, node_mesh, node_mat, spoke_pos);
        commands
            .spawn((
                Cable {
                    solver:     Solver::Catenary(CatenarySolver::new().with_slack(DEFAULT_SLACK)),
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
                    AttachedTo(hub),
                ));
                parent.spawn(CableEndpoint::new(CableEnd::End, spoke_pos));
            });
    }
}
