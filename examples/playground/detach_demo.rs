//! Section 6: detach demo — cables respond to endpoint detach events.

use bevy::prelude::*;
use bevy_catenary::AttachedTo;
use bevy_catenary::Cable;
use bevy_catenary::CableEnd;
use bevy_catenary::CableEndpoint;
use bevy_catenary::CableMeshConfig;
use bevy_catenary::CatenarySolver;
use bevy_catenary::OnDetach;
use bevy_catenary::Solver;

use super::constants::DESPAWN_GREEN;
use super::constants::DESPAWN_RED;
use super::constants::DETACH_BUMP_BLUE;
use super::constants::HUB_SPHERE_RADIUS;
use super::constants::NODE_Y;
use super::constants::SECTION_X;
use super::constants::SLACK_NORMAL;
use super::entities;
use super::entities::Despawnable;
use super::input;

/// Marker for entities belonging to the detach demo section (for reset).
#[derive(Component)]
pub(crate) struct DetachDemoEntity;

struct DetachDemoAssets<'a> {
    sphere_mesh: Handle<Mesh>,
    node_mesh:   &'a Handle<Mesh>,
    node_mat:    &'a Handle<StandardMaterial>,
    cable_mat:   &'a Handle<StandardMaterial>,
}

struct DetachDemoRow {
    z:            f32,
    sphere_color: Color,
    solver:       CatenarySolver,
    policy:       OnDetach,
}

pub(crate) fn spawn_detach_demo(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[6];
    let assets = DetachDemoAssets {
        sphere_mesh: meshes.add(Sphere::new(HUB_SPHERE_RADIUS).mesh().uv(16, 16)),
        node_mesh,
        node_mat,
        cable_mat,
    };

    let rows = [
        DetachDemoRow {
            z:            -1.5,
            sphere_color: DESPAWN_GREEN,
            solver:       CatenarySolver::new().with_slack(SLACK_NORMAL),
            policy:       OnDetach::Remain,
        },
        DetachDemoRow {
            z:            0.0,
            sphere_color: DETACH_BUMP_BLUE,
            solver:       CatenarySolver::new()
                .with_slack(SLACK_NORMAL)
                .with_detach_slack_bump(0.35),
            policy:       OnDetach::Remain,
        },
        DetachDemoRow {
            z:            1.5,
            sphere_color: DESPAWN_RED,
            solver:       CatenarySolver::new().with_slack(SLACK_NORMAL),
            policy:       OnDetach::Despawn,
        },
    ];

    for row in rows {
        spawn_detach_demo_row(commands, materials, &assets, cx, row);
    }
}

fn spawn_detach_demo_row(
    commands: &mut Commands,
    materials: &mut Assets<StandardMaterial>,
    assets: &DetachDemoAssets,
    cx: f32,
    row: DetachDemoRow,
) {
    let sphere_mat = materials.add(StandardMaterial {
        base_color: row.sphere_color,
        ..default()
    });
    let sphere = commands
        .spawn((
            Mesh3d(assets.sphere_mesh.clone()),
            MeshMaterial3d(sphere_mat),
            Transform::from_translation(Vec3::new(cx - 2.0, NODE_Y, row.z)),
            Despawnable,
            DetachDemoEntity,
        ))
        .observe(input::on_despawnable_clicked)
        .id();

    let anchor_pos = Vec3::new(cx + 2.0, NODE_Y, row.z);
    entities::spawn_node_cube(commands, assets.node_mesh, assets.node_mat, anchor_pos)
        .insert(DetachDemoEntity);

    commands
        .spawn((
            Cable {
                solver:     Solver::Catenary(row.solver),
                obstacles:  vec![],
                resolution: 0,
            },
            CableMeshConfig {
                material: Some(assets.cable_mat.clone()),
                ..default()
            },
            DetachDemoEntity,
        ))
        .with_children(|parent| {
            parent.spawn((
                CableEndpoint::new(CableEnd::Start, Vec3::ZERO).with_detach_policy(row.policy),
                AttachedTo(sphere),
            ));
            parent.spawn(CableEndpoint::new(CableEnd::End, anchor_pos));
        });
}
