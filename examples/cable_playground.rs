//! Interactive cable playground demonstrating catenary, A*, and orthogonal routing.
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
use bevy::picking::mesh_picking::MeshPickingPlugin;
use bevy::picking::Pickable;
use bevy::prelude::*;
use bevy_brp_extras::BrpExtrasPlugin;
use bevy_catenary::generate_tube_mesh;
use bevy_catenary::AStarPlanner;
use bevy_catenary::Cable;
use bevy_catenary::CableDebugEnabled;
use bevy_catenary::CatenaryPlugin;
use bevy_catenary::CatenarySolver;
use bevy_catenary::ComputedCableGeometry;
use bevy_catenary::LinearSolver;
use bevy_catenary::Obstacle;
use bevy_catenary::OrthogonalPlanner;
use bevy_catenary::Router;
use bevy_catenary::TubeMeshConfig;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::inspector_options::std_options::NumberDisplay;
use bevy_inspector_egui::prelude::*;
use bevy_inspector_egui::quick::ResourceInspectorPlugin;
use bevy_panorbit_camera::PanOrbitCamera;
use bevy_panorbit_camera::PanOrbitCameraPlugin;
use bevy_panorbit_camera::TrackpadBehavior;
use bevy_panorbit_camera_ext::AnimateToFit;
use bevy_panorbit_camera_ext::FitVisualization;
use bevy_panorbit_camera_ext::PanOrbitCameraExtPlugin;
use bevy_panorbit_camera_ext::ZoomToFit;

// ============================================================================
// Constants
// ============================================================================

// Node positions
const NODE_A_POS: Vec3 = Vec3::new(-3.0, 2.0, -2.0);
const NODE_B_POS: Vec3 = Vec3::new(3.0, 2.5, -2.0);
const NODE_C_POS: Vec3 = Vec3::new(-3.0, 1.0, 2.0);
const NODE_D_POS: Vec3 = Vec3::new(3.0, 1.5, 2.0);
const NODE_E_POS: Vec3 = Vec3::new(-3.0, 1.8, 5.0);
const NODE_F_POS: Vec3 = Vec3::new(3.0, 1.8, 5.0);

// Node cube size (half-extent per axis)
const NODE_CUBE_SIZE: f32 = 0.3;

// Cable slack values
const CABLE_AB_SLACK: f32 = 1.2;
const CABLE_AC_SLACK: f32 = 1.5;
const CABLE_BD_SLACK: f32 = 1.05;

// Obstacle between E and F
const OBSTACLE_POS: Vec3 = Vec3::new(0.0, 1.8, 5.0);
const OBSTACLE_HALF_EXTENTS: Vec3 = Vec3::new(0.8, 0.8, 0.8);

// Ground plane
const GROUND_SIZE: f32 = 16.0;

// Camera
const CAMERA_START_YAW: f32 = -0.4;
const CAMERA_START_PITCH: f32 = 0.5;
const CAMERA_START_RADIUS: f32 = 15.0;
const CAMERA_FOCUS: Vec3 = Vec3::new(0.0, 1.5, 1.5);
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
const CABLE_COLOR_R: f32 = 0.9;
const CABLE_COLOR_G: f32 = 0.5;
const CABLE_COLOR_B: f32 = 0.1;

// Obstacle material
const OBSTACLE_ALPHA: f32 = 0.25;
const OBSTACLE_COLOR_R: f32 = 0.8;
const OBSTACLE_COLOR_G: f32 = 0.2;
const OBSTACLE_COLOR_B: f32 = 0.2;

// Node colors
const NODE_COLOR_R: f32 = 0.4;
const NODE_COLOR_G: f32 = 0.6;
const NODE_COLOR_B: f32 = 0.8;
const NODE_ALPHA: f32 = 0.8;

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
        }
    }
}

/// Marker for spawned hub sphere entities so they can be despawned on rebuild.
#[derive(Component)]
struct HubSphere;

/// When false, clicking meshes does not trigger zoom-to-fit.
#[derive(Resource)]
struct ClickToZoomEnabled(bool);

impl Default for ClickToZoomEnabled {
    fn default() -> Self { Self(true) }
}

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
            DefaultPlugins,
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
        .init_resource::<ClickToZoomEnabled>()
        .init_resource::<InspectorVisible>()
        .add_systems(
            Startup,
            (
                setup_camera,
                (
                    setup_scene,
                    setup_nodes,
                    setup_cables,
                    setup_obstacle,
                    setup_ui,
                ),
            )
                .chain(),
        )
        .add_systems(
            Update,
            (
                generate_cable_meshes,
                handle_keyboard,
                rebuild_on_settings_change.run_if(resource_changed::<CableTuneSettings>),
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
            intensity: 500_000.0,
            range: 100.0,
            outer_angle: 0.8,
            inner_angle: 0.6,
            shadows_enabled: false,
            ..default()
        })
        .id();

    // Store camera entity — scene_bounds will be set when ground spawns
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
    // Ground plane (click to deselect and zoom back to scene)
    let ground = commands
        .spawn((
            Mesh3d(meshes.add(Plane3d::default().mesh().size(GROUND_SIZE, GROUND_SIZE))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.3, 0.5, 0.3),
                ..default()
            })),
        ))
        .observe(on_ground_clicked)
        .id();

    scene.scene_bounds = ground;

    // Directional light with shadows
    commands.spawn((
        DirectionalLight {
            illuminance: DIRECTIONAL_LIGHT_ILLUMINANCE,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::ZYX, 0.0, PI / 4.0, -PI / 4.0)),
    ));
}

fn setup_nodes(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_mesh = meshes.add(Cuboid::new(
        NODE_CUBE_SIZE * 2.0,
        NODE_CUBE_SIZE * 2.0,
        NODE_CUBE_SIZE * 2.0,
    ));
    let node_material = materials.add(StandardMaterial {
        base_color: Color::srgba(NODE_COLOR_R, NODE_COLOR_G, NODE_COLOR_B, NODE_ALPHA),
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    let positions = [
        NODE_A_POS, NODE_B_POS, NODE_C_POS, NODE_D_POS, NODE_E_POS, NODE_F_POS,
    ];

    for pos in positions {
        commands
            .spawn((
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(node_material.clone()),
                Transform::from_translation(pos),
            ))
            .observe(on_mesh_clicked);
    }
}

fn setup_cables(mut commands: Commands, mut materials: ResMut<Assets<StandardMaterial>>) {
    // Store the shared cable material
    let cable_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(CABLE_COLOR_R, CABLE_COLOR_G, CABLE_COLOR_B),
        ..default()
    });
    commands.insert_resource(CableMaterial(cable_mat));

    // Cable 1: A -> B, catenary with slack 1.2
    commands.spawn(Cable {
        start:      NODE_A_POS,
        end:        NODE_B_POS,
        solver:     Box::new(CatenarySolver::new().with_slack(CABLE_AB_SLACK)),
        obstacles:  vec![],
        resolution: 0,
    });

    // Cable 2: A -> C, catenary with slack 1.5 (loose)
    commands.spawn(Cable {
        start:      NODE_A_POS,
        end:        NODE_C_POS,
        solver:     Box::new(CatenarySolver::new().with_slack(CABLE_AC_SLACK)),
        obstacles:  vec![],
        resolution: 0,
    });

    // Cable 3: B -> D, catenary with slack 1.05 (taut)
    commands.spawn(Cable {
        start:      NODE_B_POS,
        end:        NODE_D_POS,
        solver:     Box::new(CatenarySolver::new().with_slack(CABLE_BD_SLACK)),
        obstacles:  vec![],
        resolution: 0,
    });

    // Cable 4: C -> D, orthogonal planner + linear solver via `Router`
    commands.spawn(Cable {
        start:      NODE_C_POS,
        end:        NODE_D_POS,
        solver:     Box::new(Router::new(OrthogonalPlanner::new(), LinearSolver)),
        obstacles:  vec![],
        resolution: 0,
    });

    // Cable 5: E -> F, A* planner + catenary solver via `Router`, with obstacle
    let obstacle = Obstacle::new(OBSTACLE_HALF_EXTENTS, OBSTACLE_POS);
    commands.spawn(Cable {
        start:      NODE_E_POS,
        end:        NODE_F_POS,
        solver:     Box::new(Router::new(
            AStarPlanner::new(),
            CatenarySolver::new().with_slack(CABLE_AB_SLACK),
        )),
        obstacles:  vec![obstacle],
        resolution: 0,
    });
}

fn setup_obstacle(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Translucent box between E and F
    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                OBSTACLE_HALF_EXTENTS.x * 2.0,
                OBSTACLE_HALF_EXTENTS.y * 2.0,
                OBSTACLE_HALF_EXTENTS.z * 2.0,
            ))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(
                    OBSTACLE_COLOR_R,
                    OBSTACLE_COLOR_G,
                    OBSTACLE_COLOR_B,
                    OBSTACLE_ALPHA,
                ),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_translation(OBSTACLE_POS),
            NotShadowCaster,
        ))
        .observe(on_mesh_clicked);
}

fn setup_ui(mut commands: Commands, scene: Res<SceneEntities>) {
    commands.spawn((
        Text::new(
            "Cable Playground\n\n\
             Orbit: Middle-mouse (or two-finger trackpad)\n\
             Pan: Shift + middle-mouse\n\
             Zoom: Scroll wheel (or pinch)\n\n\
             Click mesh - Zoom to fit\n\
             Click ground - Zoom back to scene\n\
             Esc - Toggle click-to-zoom\n\
             D - Toggle debug gizmos\n\
             H - Zoom to fit entire scene\n\
             I - Toggle inspector",
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
            cap_start:                    !start_shared,
            cap_end:                      !end_shared,
            trim_start:                   if start_shared { trim_distance } else { 0.0 },
            trim_end:                     if end_shared { trim_distance } else { 0.0 },
            elbow_bend_radius_multiplier: settings.elbow_bend_radius_multiplier,
            elbow_min_radius_multiplier:  settings.elbow_min_radius_multiplier,
            elbow_rings_per_right_angle:  settings.elbow_rings_per_right_angle,
            elbow_angle_threshold_deg:    settings.elbow_angle_threshold_deg,
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
    // Despawn hub spheres
    for entity in &hub_spheres {
        commands.entity(entity).despawn();
    }

    // Despawn mesh children of cables and remove the spawned marker
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
    mut click_to_zoom: ResMut<ClickToZoomEnabled>,
    mut inspector_visible: ResMut<InspectorVisible>,
    scene: Res<SceneEntities>,
    viz_query: Query<(), With<FitVisualization>>,
) {
    // Esc: Toggle click-to-zoom
    if keyboard.just_pressed(KeyCode::Escape) {
        click_to_zoom.0 = !click_to_zoom.0;
        info!(
            "Click-to-zoom: {}",
            if click_to_zoom.0 { "ON" } else { "OFF" }
        );
    }

    // D: Toggle debug gizmos + fit visualization
    if keyboard.just_pressed(KeyCode::KeyD) {
        debug_enabled.0 = !debug_enabled.0;
        if viz_query.get(scene.camera).is_ok() {
            commands.entity(scene.camera).remove::<FitVisualization>();
        } else {
            commands.entity(scene.camera).insert(FitVisualization);
        }
    }

    // I: Toggle inspector panel
    if keyboard.just_pressed(KeyCode::KeyI) {
        inspector_visible.0 = !inspector_visible.0;
    }

    // H: Zoom to fit entire scene
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
    enabled: Res<ClickToZoomEnabled>,
) {
    if !enabled.0 {
        return;
    }

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
    enabled: Res<ClickToZoomEnabled>,
) {
    if !enabled.0 {
        return;
    }

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
/// Uses millimeter precision to avoid floating point equality issues.
const QUANTIZE_SCALE: f32 = 1000.0;

fn quantize_position(pos: Vec3) -> [i32; 3] {
    [
        (pos.x * QUANTIZE_SCALE).round() as i32,
        (pos.y * QUANTIZE_SCALE).round() as i32,
        (pos.z * QUANTIZE_SCALE).round() as i32,
    ]
}
