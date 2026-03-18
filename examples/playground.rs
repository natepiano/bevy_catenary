//! Interactive cable playground demonstrating catenary, A*, and orthogonal routing.
//!
//! Each cable type is laid out side by side with a 3D label above it.
//!
//! Controls:
//! - Orbit: Middle-mouse drag (or two-finger trackpad)
//! - Pan: Shift + middle-mouse
//! - Zoom: Scroll wheel (or pinch)
//! - D: Toggle debug gizmos
//! - H: Home — zoom to fit entire scene
//! - I: Toggle inspector

use std::f32::consts::PI;
use std::time::Duration;

use bevy::light::NotShadowCaster;
use bevy::math::curve::easing::EaseFunction;
use bevy::picking::Pickable;
use bevy::picking::mesh_picking::MeshPickingPlugin;
use bevy::prelude::*;
use bevy_brp_extras::BrpExtrasPlugin;
use bevy_catenary::AStarPlanner;
use bevy_catenary::Cable;
use bevy_catenary::CableDebugEnabled;
use bevy_catenary::CapStyle;
use bevy_catenary::CatenaryPlugin;
use bevy_catenary::CatenarySolver;
use bevy_catenary::ComputedCableGeometry;
use bevy_catenary::LinearSolver;
use bevy_catenary::Obstacle;
use bevy_catenary::OrthogonalPlanner;
use bevy_catenary::Router;
use bevy_catenary::TubeMeshConfig;
use bevy_catenary::generate_tube_mesh;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::inspector_options::std_options::NumberDisplay;
use bevy_inspector_egui::prelude::*;
use bevy_inspector_egui::quick::ResourceInspectorPlugin;
use bevy_panorbit_camera::PanOrbitCamera;
use bevy_panorbit_camera::PanOrbitCameraPlugin;
use bevy_panorbit_camera::TrackpadBehavior;
use bevy_panorbit_camera_ext::AnimateToFit;
use bevy_panorbit_camera_ext::PanOrbitCameraExtPlugin;
use bevy_panorbit_camera_ext::ZoomToFit;

// ============================================================================
// Layout — five sections spread along the X axis
// ============================================================================

/// Spacing between section centers along X.
const SECTION_SPACING: f32 = 10.0;

/// Node height (Y) for cable endpoints.
const NODE_Y: f32 = 2.0;

/// Half-width of each cable span (endpoints at ±SPAN_HALF_X from section center).
const SPAN_HALF_X: f32 = 3.0;

/// Section center X positions (5 sections: -20, -10, 0, 10, 20).
const SECTION_X: [f32; 5] = [
    -2.0 * SECTION_SPACING,
    -1.0 * SECTION_SPACING,
    0.0,
    1.0 * SECTION_SPACING,
    2.0 * SECTION_SPACING,
];

// Node cube size (half-extent per axis)
const NODE_CUBE_SIZE: f32 = 0.3;

// Cable slack values
const SLACK_NORMAL: f32 = 1.2;
const SLACK_LOOSE: f32 = 1.5;
const SLACK_TAUT: f32 = 1.05;

// Obstacle (for A* section)
const OBSTACLE_HALF_EXTENTS: Vec3 = Vec3::new(0.8, 0.8, 0.8);

// Ground plane
const GROUND_WIDTH: f32 = 56.0;
const GROUND_DEPTH: f32 = 14.0;

// Camera
const CAMERA_START_YAW: f32 = 0.0;
const CAMERA_START_PITCH: f32 = 0.45;
const CAMERA_START_RADIUS: f32 = 38.0;
const CAMERA_FOCUS: Vec3 = Vec3::new(0.0, 0.5, 0.0);
const ANIMATE_FIT_DURATION_MS: u64 = 1200;
const ZOOM_DURATION_MS: u64 = 1000;
const ZOOM_MARGIN: f32 = 0.1;
const ZOOM_MARGIN_MESH: f32 = 0.15;
const ZOOM_MARGIN_SCENE: f32 = 0.08;

// Light
const DIRECTIONAL_LIGHT_ILLUMINANCE: f32 = 3000.0;

// Tube mesh
const TUBE_RADIUS: f32 = 0.06;
const TUBE_SIDES: u32 = 16;
const JOINT_RADIUS_MULTIPLIER: f32 = 1.5;
const JOINT_SPHERE_SEGMENTS: u32 = 16;

// Cable material color (orange)
const CABLE_COLOR: Color = Color::srgb(0.9, 0.5, 0.1);

// Obstacle material
const OBSTACLE_COLOR: Color = Color::srgba(0.8, 0.2, 0.2, 0.25);

// Node colors
const NODE_COLOR: Color = Color::srgba(0.4, 0.6, 0.8, 0.4);

// Label height above nodes
const LABEL_Y_OFFSET: f32 = 0.8;

// UI
const UI_FONT_SIZE: f32 = 14.0;

// ============================================================================
// Components and resources
// ============================================================================

/// Marker for the scene bounds entity used by zoom-to-fit.
#[derive(Resource)]
struct SceneEntities {
    camera:       Entity,
    scene_bounds: Entity,
}

/// Marker for cable entities that already have a mesh child spawned.
#[derive(Component)]
struct CableMeshSpawned;

/// Marker for the currently selected entity.
#[derive(Component)]
struct Selected;

/// Shared cable material handle.
#[derive(Resource)]
struct CableMaterial(Handle<StandardMaterial>);

/// Inspector-tunable cable mesh settings.
#[derive(Resource, Reflect, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
struct CableTuneSettings {
    #[inspector(min = 0.01, max = 0.3, display = NumberDisplay::Slider)]
    tube_radius:                  f32,
    #[inspector(min = 3, max = 32, display = NumberDisplay::Slider)]
    tube_sides:                   u32,
    #[inspector(min = 1.0, max = 4.0, display = NumberDisplay::Slider)]
    joint_radius_multiplier:      f32,
    #[inspector(min = 8, max = 32, display = NumberDisplay::Slider)]
    joint_sphere_segments:        u32,
    #[inspector(min = 1.0, max = 20.0, display = NumberDisplay::Slider)]
    elbow_bend_radius_multiplier: f32,
    #[inspector(min = 0.5, max = 5.0, display = NumberDisplay::Slider)]
    elbow_min_radius_multiplier:  f32,
    #[inspector(min = 2, max = 32, display = NumberDisplay::Slider)]
    elbow_rings_per_right_angle:  u32,
    #[inspector(min = 1.0, max = 90.0, display = NumberDisplay::Slider)]
    elbow_angle_threshold_deg:    f32,
    #[inspector(min = 0.1, max = 3.0, display = NumberDisplay::Slider)]
    elbow_arm_multiplier:         f32,
}

impl Default for CableTuneSettings {
    fn default() -> Self {
        Self {
            tube_radius:                  TUBE_RADIUS,
            tube_sides:                   TUBE_SIDES,
            joint_radius_multiplier:      JOINT_RADIUS_MULTIPLIER,
            joint_sphere_segments:        JOINT_SPHERE_SEGMENTS,
            elbow_bend_radius_multiplier: 1.0,
            elbow_min_radius_multiplier:  0.5,
            elbow_rings_per_right_angle:  32,
            elbow_angle_threshold_deg:    25.0,
            elbow_arm_multiplier:         1.0,
        }
    }
}

/// Marker for spawned hub sphere entities so they can be despawned on rebuild.
#[derive(Component)]
struct HubSphere;

/// Marker for node endpoint cubes.
#[derive(Component)]
struct NodeCube;

/// When true, the inspector panel is visible.
#[derive(Resource)]
struct InspectorVisible(bool);

impl Default for InspectorVisible {
    fn default() -> Self { Self(false) }
}

// ============================================================================
// App entry point
// ============================================================================

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Playground".into(),
                    ..default()
                }),
                ..default()
            }),
            EguiPlugin::default(),
            PanOrbitCameraPlugin,
            PanOrbitCameraExtPlugin,
            MeshPickingPlugin,
            BrpExtrasPlugin::default(),
            CatenaryPlugin,
            ResourceInspectorPlugin::<CableTuneSettings>::default()
                .run_if(|visible: Res<InspectorVisible>| visible.0),
        ))
        .init_resource::<CableTuneSettings>()
        .init_resource::<InspectorVisible>()
        .add_systems(
            Startup,
            (setup_camera, (setup_scene, setup_sections, setup_ui)).chain(),
        )
        .add_systems(
            Update,
            (
                generate_cable_meshes,
                handle_keyboard,
                rebuild_on_settings_change.run_if(resource_changed::<CableTuneSettings>),
                update_label_positions,
                on_label_clicked,
            ),
        )
        .run();
}

// ============================================================================
// Startup systems
// ============================================================================

fn setup_camera(mut commands: Commands) {
    let camera = commands
        .spawn(PanOrbitCamera {
            button_orbit: MouseButton::Middle,
            button_pan: MouseButton::Middle,
            modifier_pan: Some(KeyCode::ShiftLeft),
            trackpad_behavior: TrackpadBehavior::BlenderLike {
                modifier_pan:  Some(KeyCode::ShiftLeft),
                modifier_zoom: Some(KeyCode::ControlLeft),
            },
            trackpad_pinch_to_zoom_enabled: true,
            trackpad_sensitivity: 0.3,
            focus: CAMERA_FOCUS,
            target_focus: CAMERA_FOCUS,
            yaw: Some(CAMERA_START_YAW),
            pitch: Some(CAMERA_START_PITCH),
            radius: Some(CAMERA_START_RADIUS),
            ..default()
        })
        .with_child(SpotLight {
            intensity: 50_000.0,
            range: 100.0,
            outer_angle: 0.8,
            inner_angle: 0.6,
            shadows_enabled: false,
            ..default()
        })
        .id();

    commands.insert_resource(SceneEntities {
        camera,
        scene_bounds: Entity::PLACEHOLDER,
    });
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut scene: ResMut<SceneEntities>,
) {
    // Wide ground plane
    let ground = commands
        .spawn((
            Mesh3d(meshes.add(Plane3d::default().mesh().size(GROUND_WIDTH, GROUND_DEPTH))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.3, 0.5, 0.3),
                ..default()
            })),
        ))
        .observe(on_ground_clicked)
        .id();

    scene.scene_bounds = ground;

    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: DIRECTIONAL_LIGHT_ILLUMINANCE,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::ZYX, 0.0, PI / 4.0, -PI / 4.0)),
    ));
}

/// Spawn all five cable sections side by side with labels.
fn setup_sections(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Shared materials
    let cable_mat = materials.add(StandardMaterial {
        base_color: CABLE_COLOR,
        ..default()
    });
    commands.insert_resource(CableMaterial(cable_mat));

    let node_mesh = meshes.add(Cuboid::new(
        NODE_CUBE_SIZE * 2.0,
        NODE_CUBE_SIZE * 2.0,
        NODE_CUBE_SIZE * 2.0,
    ));
    let node_mat = materials.add(StandardMaterial {
        base_color: NODE_COLOR,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    // --- Section 0: Catenary (normal slack) ---
    let cx = SECTION_X[0];
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    spawn_node_pair(&mut commands, &node_mesh, &node_mat, start, end);
    commands.spawn(Cable {
        start,
        end,
        solver: Box::new(CatenarySolver::new().with_slack(SLACK_NORMAL)),
        obstacles: vec![],
        resolution: 0,
    });
    spawn_label(
        &mut commands,
        &mut meshes,
        &mut materials,
        cx,
        "Catenary\nslack 1.2",
    );

    // --- Section 1: Catenary (loose) ---
    let cx = SECTION_X[1];
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    spawn_node_pair(&mut commands, &node_mesh, &node_mat, start, end);
    commands.spawn(Cable {
        start,
        end,
        solver: Box::new(CatenarySolver::new().with_slack(SLACK_LOOSE)),
        obstacles: vec![],
        resolution: 0,
    });
    spawn_label(
        &mut commands,
        &mut meshes,
        &mut materials,
        cx,
        "Catenary\nslack 1.5",
    );

    // --- Section 2: Catenary (taut) ---
    let cx = SECTION_X[2];
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    spawn_node_pair(&mut commands, &node_mesh, &node_mat, start, end);
    commands.spawn(Cable {
        start,
        end,
        solver: Box::new(CatenarySolver::new().with_slack(SLACK_TAUT)),
        obstacles: vec![],
        resolution: 0,
    });
    spawn_label(
        &mut commands,
        &mut meshes,
        &mut materials,
        cx,
        "Catenary\nslack 1.05 (taut)",
    );

    // --- Section 3: Orthogonal routing ---
    let cx = SECTION_X[3];
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y - 0.5, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y + 0.5, 0.0);
    spawn_node_pair(&mut commands, &node_mesh, &node_mat, start, end);
    commands.spawn(Cable {
        start,
        end,
        solver: Box::new(Router::new(OrthogonalPlanner::new(), LinearSolver)),
        obstacles: vec![],
        resolution: 0,
    });
    spawn_label(
        &mut commands,
        &mut meshes,
        &mut materials,
        cx,
        "Orthogonal\n+ Linear",
    );

    // --- Section 4: A* routing around obstacle ---
    let cx = SECTION_X[4];
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    let obstacle_pos = Vec3::new(cx, NODE_Y, 0.0);
    let obstacle = Obstacle::new(OBSTACLE_HALF_EXTENTS, obstacle_pos);
    spawn_node_pair(&mut commands, &node_mesh, &node_mat, start, end);
    commands.spawn(Cable {
        start,
        end,
        solver: Box::new(Router::new(
            AStarPlanner::new(),
            CatenarySolver::new().with_slack(SLACK_NORMAL),
        )),
        obstacles: vec![obstacle],
        resolution: 0,
    });
    // Obstacle box
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
            Transform::from_translation(obstacle_pos),
            NotShadowCaster,
        ))
        .observe(on_mesh_clicked);
    spawn_label(
        &mut commands,
        &mut meshes,
        &mut materials,
        cx,
        "A* + Catenary\n(obstacle)",
    );
}

/// Spawn a pair of node cubes at the given positions.
fn spawn_node_pair(
    commands: &mut Commands,
    mesh: &Handle<Mesh>,
    material: &Handle<StandardMaterial>,
    start: Vec3,
    end: Vec3,
) {
    for pos in [start, end] {
        commands
            .spawn((
                Mesh3d(mesh.clone()),
                MeshMaterial3d(material.clone()),
                Transform::from_translation(pos),
                NodeCube,
            ))
            .observe(on_mesh_clicked);
    }
}

/// Links a UI label to its world-space anchor and bounds entity for zoom-to-fit.
#[derive(Component)]
struct SectionLabel {
    world_pos:     Vec3,
    bounds_entity: Entity,
}

/// Marker for the invisible bounds mesh of a section.
#[derive(Component)]
struct SectionBounds;

/// Spawn an invisible bounds mesh, and a clickable UI label anchored to the section.
fn spawn_label(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    center_x: f32,
    text: &str,
) {
    // Invisible bounds cube spanning the section (for ZoomToFit targeting)
    let bounds_entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(SPAN_HALF_X * 2.0, NODE_Y + 1.0, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.0, 0.0, 0.0, 0.0),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_translation(Vec3::new(center_x, NODE_Y * 0.5, 0.0)),
            Pickable::IGNORE,
            SectionBounds,
        ))
        .id();

    // Clickable UI label with dark background
    commands.spawn((
        Button,
        Node {
            position_type: PositionType::Absolute,
            padding: UiRect::axes(Val::Px(8.0), Val::Px(4.0)),
            border_radius: BorderRadius::all(Val::Px(4.0)),
            justify_content: JustifyContent::Center,
            align_items: AlignItems::Center,
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)),
        SectionLabel {
            world_pos: Vec3::new(center_x, NODE_Y + LABEL_Y_OFFSET, 0.0),
            bounds_entity,
        },
        children![(
            Text::new(text),
            TextFont {
                font_size: 13.0,
                ..default()
            },
            TextColor(Color::WHITE),
            TextLayout::new_with_justify(Justify::Center),
        )],
    ));
}

/// Project label world positions to screen each frame. Hide if off-screen.
fn update_label_positions(
    mut labels: Query<(&SectionLabel, &mut Node, &mut Visibility)>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
) {
    let Ok((camera, cam_tf)) = camera_query.single() else {
        return;
    };
    let Some(viewport_size) = camera.logical_viewport_size() else {
        return;
    };

    for (label, mut node, mut vis) in &mut labels {
        let cam_forward = cam_tf.forward().as_vec3();
        let to_label = label.world_pos - cam_tf.translation();
        if cam_forward.dot(to_label) < 0.0 {
            *vis = Visibility::Hidden;
            continue;
        }
        match camera.world_to_viewport(cam_tf, label.world_pos) {
            Ok(screen_pos) => {
                let x = screen_pos.x - 55.0;
                let y = screen_pos.y - 20.0;
                if x < -80.0 || x > viewport_size.x || y < -10.0 || y > viewport_size.y {
                    *vis = Visibility::Hidden;
                } else {
                    *vis = Visibility::Inherited;
                    node.left = Val::Px(x);
                    node.top = Val::Px(y);
                }
            },
            Err(_) => {
                *vis = Visibility::Hidden;
            },
        }
    }
}

/// When a label button is clicked, zoom-to-fit on its section bounds.
fn on_label_clicked(
    interactions: Query<(&Interaction, &SectionLabel), Changed<Interaction>>,
    mut commands: Commands,
    scene: Res<SceneEntities>,
) {
    for (interaction, label) in &interactions {
        if *interaction == Interaction::Pressed {
            commands.trigger(
                ZoomToFit::new(scene.camera, label.bounds_entity)
                    .margin(ZOOM_MARGIN_MESH)
                    .duration(Duration::from_millis(ZOOM_DURATION_MS))
                    .easing(EaseFunction::CubicOut),
            );
        }
    }
}

fn setup_ui(mut commands: Commands, scene: Res<SceneEntities>) {
    commands.spawn((
        Text::new(
            "Orbit: Middle-mouse (or two-finger trackpad)\n\
             Pan: Shift + middle-mouse\n\
             Zoom: Scroll wheel (or pinch)\n\n\
             Click label - Zoom to cable\n\
             Click ground - Zoom back to scene\n\
             D - Toggle debug gizmos\n\
             H - Zoom to fit entire scene\n\
             I - Toggle inspector\n\
             N - Toggle node cubes",
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
        UiTargetCamera(scene.camera),
    ));
}

// ============================================================================
// Runtime systems
// ============================================================================

/// Generates tube meshes for cables that have computed geometry but no mesh child yet.
/// Detects shared endpoints, trims tubes at junctions, and spawns hub spheres.
fn generate_cable_meshes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    cable_material: Option<Res<CableMaterial>>,
    settings: Res<CableTuneSettings>,
    cables: Query<(Entity, &Cable, &ComputedCableGeometry), Without<CableMeshSpawned>>,
) {
    let Some(cable_material) = cable_material else {
        return;
    };

    // Collect all cable endpoints to detect shared positions
    let mut endpoint_counts: std::collections::HashMap<[i32; 3], u32> =
        std::collections::HashMap::new();
    for (_entity, cable, _computed) in &cables {
        let start_key = quantize_position(cable.start);
        let end_key = quantize_position(cable.end);
        *endpoint_counts.entry(start_key).or_default() += 1;
        *endpoint_counts.entry(end_key).or_default() += 1;
    }

    let joint_radius = settings.tube_radius * settings.joint_radius_multiplier;
    let trim_distance = joint_radius;
    let joint_mesh = meshes.add(Sphere::new(joint_radius).mesh().uv(
        settings.joint_sphere_segments,
        settings.joint_sphere_segments,
    ));

    // Track which positions already have a hub spawned
    let mut hub_spawned: std::collections::HashSet<[i32; 3]> = std::collections::HashSet::new();

    for (entity, cable, computed) in &cables {
        let Some(geometry) = &computed.geometry else {
            continue;
        };

        let start_key = quantize_position(cable.start);
        let end_key = quantize_position(cable.end);
        let start_shared = endpoint_counts.get(&start_key).copied().unwrap_or(0) > 1;
        let end_shared = endpoint_counts.get(&end_key).copied().unwrap_or(0) > 1;

        let config = TubeMeshConfig {
            radius:                       settings.tube_radius,
            sides:                        settings.tube_sides,
            cap_start:                    if start_shared {
                CapStyle::None
            } else {
                CapStyle::Round
            },
            cap_end:                      if end_shared {
                CapStyle::None
            } else {
                CapStyle::Round
            },
            trim_start:                   if start_shared { trim_distance } else { 0.0 },
            trim_end:                     if end_shared { trim_distance } else { 0.0 },
            elbow_bend_radius_multiplier: settings.elbow_bend_radius_multiplier,
            elbow_min_radius_multiplier:  settings.elbow_min_radius_multiplier,
            elbow_rings_per_right_angle:  settings.elbow_rings_per_right_angle,
            elbow_angle_threshold_deg:    settings.elbow_angle_threshold_deg,
            elbow_arm_multiplier:         settings.elbow_arm_multiplier,
            elbow_arm_overrides:          None,
        };

        let mesh = generate_tube_mesh(geometry, &config);
        let mesh_handle = meshes.add(mesh);

        commands
            .spawn((
                Mesh3d(mesh_handle),
                MeshMaterial3d(cable_material.0.clone()),
                ChildOf(entity),
            ))
            .observe(on_mesh_clicked);

        // Spawn hub sphere at shared endpoints (once per unique position)
        if start_shared && hub_spawned.insert(start_key) {
            commands.spawn((
                Mesh3d(joint_mesh.clone()),
                MeshMaterial3d(cable_material.0.clone()),
                Transform::from_translation(cable.start),
                HubSphere,
            ));
        }
        if end_shared && hub_spawned.insert(end_key) {
            commands.spawn((
                Mesh3d(joint_mesh.clone()),
                MeshMaterial3d(cable_material.0.clone()),
                Transform::from_translation(cable.end),
                HubSphere,
            ));
        }

        commands.entity(entity).insert(CableMeshSpawned);
    }
}

/// When inspector settings change, despawn old meshes so they get regenerated.
fn rebuild_on_settings_change(
    mut commands: Commands,
    cable_meshes: Query<Entity, With<CableMeshSpawned>>,
    hub_spheres: Query<Entity, With<HubSphere>>,
    mesh_children: Query<(Entity, &ChildOf), With<Mesh3d>>,
) {
    for entity in &hub_spheres {
        commands.entity(entity).despawn();
    }
    for entity in &cable_meshes {
        commands.entity(entity).remove::<CableMeshSpawned>();
    }
    for (child_entity, child_of) in &mesh_children {
        if cable_meshes.get(child_of.parent()).is_ok() {
            commands.entity(child_entity).despawn();
        }
    }
}

/// Handles keyboard input for debug toggle and zoom-to-fit.
fn handle_keyboard(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut debug_enabled: ResMut<CableDebugEnabled>,
    mut inspector_visible: ResMut<InspectorVisible>,
    scene: Res<SceneEntities>,
    mut node_cubes: Query<&mut Visibility, With<NodeCube>>,
) {
    if keyboard.just_pressed(KeyCode::KeyD) {
        debug_enabled.0 = !debug_enabled.0;
    }

    if keyboard.just_pressed(KeyCode::KeyI) {
        inspector_visible.0 = !inspector_visible.0;
    }

    // N: Toggle node endpoint cubes
    if keyboard.just_pressed(KeyCode::KeyN) {
        for mut vis in &mut node_cubes {
            *vis = match *vis {
                Visibility::Hidden => Visibility::Inherited,
                _ => Visibility::Hidden,
            };
        }
    }

    if keyboard.just_pressed(KeyCode::KeyH) {
        commands.trigger(
            AnimateToFit::new(scene.camera, scene.scene_bounds)
                .yaw(CAMERA_START_YAW)
                .pitch(CAMERA_START_PITCH)
                .margin(ZOOM_MARGIN)
                .duration(Duration::from_millis(ANIMATE_FIT_DURATION_MS))
                .easing(EaseFunction::CubicOut),
        );
    }
}

// ============================================================================
// Pointer interaction
// ============================================================================

/// Click a mesh to zoom-to-fit on it.
fn on_mesh_clicked(
    click: On<Pointer<Click>>,
    mut commands: Commands,
    selected: Query<Entity, With<Selected>>,
) {
    for entity in &selected {
        commands.entity(entity).remove::<Selected>();
    }

    let clicked = click.entity;
    let camera = click.hit.camera;
    commands.entity(clicked).insert(Selected);
    commands.trigger(
        ZoomToFit::new(camera, clicked)
            .margin(ZOOM_MARGIN_MESH)
            .duration(Duration::from_millis(ZOOM_DURATION_MS))
            .easing(EaseFunction::CubicOut),
    );
}

/// Click the ground to deselect and zoom back to the full scene.
fn on_ground_clicked(
    click: On<Pointer<Click>>,
    mut commands: Commands,
    scene: Res<SceneEntities>,
    selected: Query<Entity, With<Selected>>,
) {
    for entity in &selected {
        commands.entity(entity).remove::<Selected>();
    }

    let camera = click.hit.camera;
    commands.trigger(
        ZoomToFit::new(camera, scene.scene_bounds)
            .margin(ZOOM_MARGIN_SCENE)
            .duration(Duration::from_millis(ZOOM_DURATION_MS))
            .easing(EaseFunction::CubicOut),
    );
}

// ============================================================================
// Utilities
// ============================================================================

/// Quantize a position to a grid key for detecting shared endpoints.
const QUANTIZE_SCALE: f32 = 1000.0;

fn quantize_position(pos: Vec3) -> [i32; 3] {
    [
        (pos.x * QUANTIZE_SCALE).round() as i32,
        (pos.y * QUANTIZE_SCALE).round() as i32,
        (pos.z * QUANTIZE_SCALE).round() as i32,
    ]
}
