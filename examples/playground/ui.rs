//! Inspector settings, UI overlays, and cable-config sync.

use bevy::picking::Pickable;
use bevy::prelude::*;
use bevy_catenary::Cable;
use bevy_catenary::CableMeshConfig;
use bevy_catenary::ComputedCableGeometry;
use bevy_inspector_egui::inspector_options::std_options::NumberDisplay;
use bevy_inspector_egui::prelude::*;

use crate::constants::JOINT_RADIUS_MULTIPLIER;
use crate::constants::JOINT_SPHERE_SEGMENTS;
use crate::constants::TUBE_RADIUS;
use crate::constants::TUBE_SIDES;
use crate::constants::UI_FONT_SIZE;
use crate::scene::RadiusMultiplier;
use crate::scene::SceneEntities;
use crate::sections::SectionInfo;

#[derive(Default, Resource)]
pub(crate) enum InspectorVisibility {
    Visible,
    #[default]
    Hidden,
}

#[derive(Resource, Reflect, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
pub(crate) struct CableSettings {
    pub(crate) tube:  TubeSettings,
    pub(crate) joint: JointSettings,
    pub(crate) elbow: ElbowSettings,
}

#[derive(Reflect, InspectorOptions)]
#[reflect(InspectorOptions)]
pub(crate) struct TubeSettings {
    #[inspector(min = 0.01, max = 0.3, display = NumberDisplay::Slider)]
    pub(crate) radius: f32,
    #[inspector(min = 1, max = 64, display = NumberDisplay::Slider)]
    pub(crate) sides:  u32,
}

#[derive(Reflect, InspectorOptions)]
#[reflect(InspectorOptions)]
pub(crate) struct JointSettings {
    #[inspector(min = 1.0, max = 4.0, display = NumberDisplay::Slider)]
    pub(crate) radius_multiplier: f32,
    #[inspector(min = 8, max = 32, display = NumberDisplay::Slider)]
    pub(crate) segments:          u32,
}

#[derive(Reflect, InspectorOptions)]
#[reflect(InspectorOptions)]
pub(crate) struct ElbowSettings {
    #[inspector(min = 1.0, max = 20.0, display = NumberDisplay::Slider)]
    pub(crate) bend_radius_multiplier: f32,
    #[inspector(min = 0.5, max = 5.0, display = NumberDisplay::Slider)]
    pub(crate) min_radius_multiplier:  f32,
    #[inspector(min = 2, max = 32, display = NumberDisplay::Slider)]
    pub(crate) rings_per_right_angle:  u32,
    #[inspector(min = 1.0, max = 90.0, display = NumberDisplay::Slider)]
    pub(crate) angle_threshold_deg:    f32,
    #[inspector(min = 0.1, max = 3.0, display = NumberDisplay::Slider)]
    pub(crate) arm_multiplier:         f32,
}

impl Default for CableSettings {
    fn default() -> Self {
        Self {
            tube:  TubeSettings {
                radius: TUBE_RADIUS,
                sides:  TUBE_SIDES,
            },
            joint: JointSettings {
                radius_multiplier: JOINT_RADIUS_MULTIPLIER,
                segments:          JOINT_SPHERE_SEGMENTS,
            },
            elbow: ElbowSettings {
                bend_radius_multiplier: 1.0,
                min_radius_multiplier:  0.5,
                rings_per_right_angle:  32,
                angle_threshold_deg:    25.0,
                arm_multiplier:         1.0,
            },
        }
    }
}

pub(crate) fn setup_ui(mut commands: Commands, scene: Res<SceneEntities>) {
    spawn_help_text(&mut commands, scene.camera);
    spawn_keyboard_shortcuts(&mut commands, scene.camera);
    crate::navigation::spawn_nav_bar(&mut commands, scene.camera);
    spawn_section_infos(&mut commands, scene.camera);
}

fn spawn_section_infos(commands: &mut Commands, camera: Entity) {
    let section_texts: [(usize, &str); 7] = [
        (
            1,
            "Round (transparent) / Flat / None\nEnd caps are independent\nEsc - Pause lights",
        ),
        (2, "Catenary    Linear    Orthogonal"),
        (3, "Drag blue boxes"),
        (4, "Drag sphere"),
        (
            6,
            "Click green sphere - cable freezes\n\
             Click red sphere - cable disappears\n\
             R - Reset",
        ),
        (7, "Look, it's a tube!"),
        (
            8,
            "Front: Fixed (no roll)\nMiddle: AsSpawned (plug keeps its spawn orientation)\nBack: Rotating (follows twist)\nDrag the plugs to compare",
        ),
    ];

    for (section, text) in section_texts {
        commands.spawn((
            Text::new(text),
            TextFont {
                font_size: UI_FONT_SIZE,
                ..default()
            },
            TextColor(Color::WHITE),
            TextLayout::new_with_justify(Justify::Center),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(60.0),
                left: Val::Percent(50.0),
                margin: UiRect::left(Val::Px(-200.0)),
                width: Val::Px(400.0),
                justify_content: JustifyContent::Center,
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.5)),
            Pickable::IGNORE,
            UiTargetCamera(camera),
            SectionInfo(section),
            Visibility::Hidden,
        ));
    }
}

fn spawn_help_text(commands: &mut Commands, camera: Entity) {
    commands.spawn((
        Text::new(
            "Orbit: Middle-mouse (or trackpad)\n\
             Pan: Shift + middle-mouse\n\
             Zoom: Scroll wheel (or pinch)",
        ),
        TextFont {
            font_size: UI_FONT_SIZE,
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        Pickable::IGNORE,
        UiTargetCamera(camera),
    ));
}

fn spawn_keyboard_shortcuts(commands: &mut Commands, camera: Entity) {
    commands.spawn((
        Text::new(
            "D - Debug gizmos\n\
             F - Full scene\n\
             I - Inspector\n\
             +/- Slack",
        ),
        TextFont {
            font_size: UI_FONT_SIZE,
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        Pickable::IGNORE,
        UiTargetCamera(camera),
    ));
}

pub(crate) fn sync_cable_settings(
    settings: Res<CableSettings>,
    mut commands: Commands,
    cables: Query<
        (
            Entity,
            &CableMeshConfig,
            &ComputedCableGeometry,
            Option<&RadiusMultiplier>,
        ),
        With<Cable>,
    >,
) {
    for (entity, config, computed, multiplier) in &cables {
        let mut new_config = config.clone();
        let mult = multiplier.map_or(1.0, |m| m.0);
        new_config.tube.radius = settings.tube.radius * mult;
        new_config.tube.sides = settings.tube.sides;
        new_config.elbow.bend_radius_multiplier = settings.elbow.bend_radius_multiplier;
        new_config.elbow.min_radius_multiplier = settings.elbow.min_radius_multiplier;
        new_config.elbow.rings_per_right_angle = settings.elbow.rings_per_right_angle;
        new_config.elbow.angle_threshold_deg = settings.elbow.angle_threshold_deg;
        new_config.elbow.arm_multiplier = settings.elbow.arm_multiplier;
        commands
            .entity(entity)
            .insert((new_config, computed.clone()));
    }
}
