//! Section navigation UI: nav bar, left/right arrows, and keyboard shortcuts.

use std::time::Duration;

use bevy::math::curve::easing::EaseFunction;
use bevy::picking::Pickable;
use bevy::prelude::*;
use bevy_lagrange::ZoomToFit;

use crate::constants::NAV_DURATION_MS;
use crate::constants::NAV_FONT_SIZE;
use crate::constants::SECTION_COUNT;
use crate::constants::SECTION_TITLES;
use crate::constants::ZOOM_MARGIN_NAV;
use crate::scene::SceneEntities;
use crate::sections::CurrentSection;
use crate::sections::SectionBounds;

#[derive(Component)]
pub(crate) struct NavLabel;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum NavDirection {
    Left,
    Right,
}

#[derive(Component)]
pub(crate) struct NavButton(pub(crate) NavDirection);

pub(crate) fn navigate_to_section(
    commands: &mut Commands,
    section: usize,
    current: &mut ResMut<CurrentSection>,
    scene: &Res<SceneEntities>,
    bounds: &Res<SectionBounds>,
    label_query: &mut Query<&mut Text, With<NavLabel>>,
) {
    current.0 = section;
    commands.trigger(
        ZoomToFit::new(scene.camera, bounds.0[section])
            .margin(ZOOM_MARGIN_NAV)
            .duration(Duration::from_millis(NAV_DURATION_MS))
            .easing(EaseFunction::CubicInOut),
    );
    update_nav_label(label_query, section);
}

pub(crate) fn update_nav_label(label_query: &mut Query<&mut Text, With<NavLabel>>, section: usize) {
    for mut text in label_query.iter_mut() {
        **text = format!(
            "{} / {SECTION_COUNT} - {}",
            section + 1,
            SECTION_TITLES[section]
        );
    }
}

pub(crate) fn handle_nav_buttons(
    interactions: Query<(&Interaction, &NavButton), Changed<Interaction>>,
    mut commands: Commands,
    mut current: ResMut<CurrentSection>,
    scene: Res<SceneEntities>,
    bounds: Res<SectionBounds>,
    mut label_query: Query<&mut Text, With<NavLabel>>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    let mut new_section = None;

    for (interaction, nav) in &interactions {
        if *interaction == Interaction::Pressed {
            match nav.0 {
                NavDirection::Left if current.0 > 0 => {
                    new_section = Some(current.0 - 1);
                },
                NavDirection::Right if current.0 < SECTION_COUNT - 1 => {
                    new_section = Some(current.0 + 1);
                },
                _ => {},
            }
        }
    }

    if keyboard.just_pressed(KeyCode::ArrowLeft) && current.0 > 0 {
        new_section = Some(current.0 - 1);
    }
    if keyboard.just_pressed(KeyCode::ArrowRight) && current.0 < SECTION_COUNT - 1 {
        new_section = Some(current.0 + 1);
    }

    let number_keys = [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Digit4,
        KeyCode::Digit5,
        KeyCode::Digit6,
        KeyCode::Digit7,
        KeyCode::Digit8,
        KeyCode::Digit9,
    ];
    for (i, key) in number_keys.iter().enumerate() {
        if keyboard.just_pressed(*key) && i < SECTION_COUNT {
            new_section = Some(i);
        }
    }

    if let Some(section) = new_section {
        navigate_to_section(
            &mut commands,
            section,
            &mut current,
            &scene,
            &bounds,
            &mut label_query,
        );
    }
}

pub(crate) fn spawn_nav_bar(commands: &mut Commands, camera: Entity) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                bottom: Val::Px(16.0),
                left: Val::Percent(50.0),
                margin: UiRect::left(Val::Px(-150.0)),
                flex_direction: FlexDirection::Row,
                align_items: AlignItems::Center,
                column_gap: Val::Px(12.0),
                padding: UiRect::axes(Val::Px(12.0), Val::Px(8.0)),
                border_radius: BorderRadius::all(Val::Px(6.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)),
            Pickable::IGNORE,
            UiTargetCamera(camera),
        ))
        .with_children(|parent| {
            parent
                .spawn((
                    Button,
                    Node {
                        padding: UiRect::axes(Val::Px(10.0), Val::Px(4.0)),
                        border_radius: BorderRadius::all(Val::Px(4.0)),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.15)),
                    NavButton(NavDirection::Left),
                ))
                .with_child((
                    Text::new("<"),
                    TextFont {
                        font_size: NAV_FONT_SIZE,
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));

            parent
                .spawn(Node {
                    width: Val::Px(260.0),
                    justify_content: JustifyContent::Center,
                    ..default()
                })
                .with_child((
                    Text::new(format!("1 / {SECTION_COUNT} - {}", SECTION_TITLES[0])),
                    TextFont {
                        font_size: NAV_FONT_SIZE,
                        ..default()
                    },
                    TextColor(Color::WHITE),
                    NavLabel,
                ));

            parent
                .spawn((
                    Button,
                    Node {
                        padding: UiRect::axes(Val::Px(10.0), Val::Px(4.0)),
                        border_radius: BorderRadius::all(Val::Px(4.0)),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.15)),
                    NavButton(NavDirection::Right),
                ))
                .with_child((
                    Text::new(">"),
                    TextFont {
                        font_size: NAV_FONT_SIZE,
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
        });
}
