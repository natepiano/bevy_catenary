use bevy::prelude::*;
use bevy_catenary::Cable;
use bevy_catenary::CableEnd;
use bevy_catenary::CableEndpoint;
use bevy_catenary::CableMeshConfig;
use bevy_catenary::Capping;
use bevy_catenary::FaceSides;
use bevy_catenary::Solver;
use bevy_catenary::TubeConfig;

use super::super::constants::INSIDE_VIEW_RADIUS_MULTIPLIER;
use super::super::constants::NODE_Y;
use super::super::constants::SECTION_X;
use super::super::constants::TUBE_RADIUS;
use super::RadiusMultiplier;

/// Section 7: Inside view — large tube rendered inside-only.
pub(super) fn setup_section_inside_view(
    commands: &mut Commands,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[7];
    let start = Vec3::new(cx + 0.8, NODE_Y + 0.8, 3.0);
    let end = Vec3::new(cx - 0.8, NODE_Y - 1.5, -3.0);
    commands
        .spawn((
            Cable {
                solver:     Solver::Linear,
                obstacles:  vec![],
                resolution: 0,
            },
            CableMeshConfig {
                tube: TubeConfig {
                    radius: TUBE_RADIUS * INSIDE_VIEW_RADIUS_MULTIPLIER,
                    sides:  64,
                    faces:  FaceSides::Both,
                },
                material: Some(cable_mat.clone()),
                ..default()
            },
            RadiusMultiplier(INSIDE_VIEW_RADIUS_MULTIPLIER),
        ))
        .with_children(|parent| {
            parent.spawn(CableEndpoint::new(CableEnd::Start, start).with_cap(Capping::None));
            parent.spawn(CableEndpoint::new(CableEnd::End, end).with_cap(Capping::None));
        });
}
