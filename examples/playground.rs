//! Progressive cable playground — showcases `bevy_catenary` features from simple to complex.
//!
//! 7 sections navigated with arrow buttons or Left/Right keys. Camera animates
//! to frame each section.
//!
//! **Known limitation**: Cable-to-cable attachment (chaining cables by attaching one
//! endpoint to another cable's endpoint entity) does not work yet because `CableEndpoint`
//! entities lack a `GlobalTransform` that tracks their resolved world position.
//!
//! Controls:
//! - Left/Right arrow keys or on-screen arrows: navigate sections
//! - Orbit: Middle-mouse drag (or two-finger trackpad)
//! - Pan: Shift + middle-mouse
//! - Zoom: Scroll wheel (or pinch)
//! - D: Toggle debug gizmos
//! - H: Home -zoom to fit entire scene
//! - I: Toggle inspector
//! - N: Toggle node cubes
//! - R: Reset detach demo (section 6)

use std::f32::consts::PI;
use std::time::Duration;

use bevy::light::NotShadowCaster;
use bevy::math::curve::easing::EaseFunction;
use bevy::picking::Pickable;
use bevy::picking::mesh_picking::MeshPickingPlugin;
use bevy::prelude::*;
use bevy_brp_extras::BrpExtrasPlugin;
use bevy_catenary::AttachedTo;
use bevy_catenary::Cable;
use bevy_catenary::CableDebugEnabled;
use bevy_catenary::CableEnd;
use bevy_catenary::CableEndpoint;
use bevy_catenary::CableMeshChild;
use bevy_catenary::CableMeshConfig;
use bevy_catenary::CapStyle;
use bevy_catenary::CatenaryPlugin;
use bevy_catenary::CatenarySolver;
use bevy_catenary::ComputedCableGeometry;
use bevy_catenary::Curve;
use bevy_catenary::DetachPolicy;
use bevy_catenary::Obstacle;
use bevy_catenary::Planner;
use bevy_catenary::Solver;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::inspector_options::std_options::NumberDisplay;
use bevy_inspector_egui::prelude::*;
use bevy_inspector_egui::quick::ResourceInspectorPlugin;
use bevy_panorbit_camera::PanOrbitCamera;
use bevy_panorbit_camera::PanOrbitCameraPlugin;
use bevy_panorbit_camera::TrackpadBehavior;
use bevy_panorbit_camera_ext::PanOrbitCameraExtPlugin;
use bevy_panorbit_camera_ext::ZoomToFit;

// ============================================================================
// Layout -7 sections spread along X
// ============================================================================

const SECTION_COUNT: usize = 8;
const SECTION_SPACING: f32 = 16.0;
const NODE_Y: f32 = 2.0;
const SPAN_HALF_X: f32 = 3.0;
const NODE_CUBE_SIZE: f32 = 0.3;
const DRAGGABLE_CUBE_SIZE: f32 = 0.45;

const SECTION_X: [f32; SECTION_COUNT] = [
    -3.5 * SECTION_SPACING,
    -2.5 * SECTION_SPACING,
    -1.5 * SECTION_SPACING,
    -0.5 * SECTION_SPACING,
    0.5 * SECTION_SPACING,
    1.5 * SECTION_SPACING,
    2.5 * SECTION_SPACING,
    3.5 * SECTION_SPACING,
];

const SECTION_TITLES: [&str; SECTION_COUNT] = [
    "Simple Catenary",
    "Cap Styles",
    "Solver Comparison",
    "Entity Attachment",
    "Shared Hub",
    "A* Routing",
    "Detach Policy",
    "Inside View",
];

// Cable
const SLACK_NORMAL: f32 = 1.3;
const OBSTACLE_HALF_EXTENTS: Vec3 = Vec3::new(0.8, 0.8, 0.8);

// Ground
const GROUND_WIDTH: f32 = 128.0;
const GROUND_DEPTH: f32 = 14.0;

// Camera
const NAV_DURATION_MS: u64 = 1200;
const ZOOM_DURATION_MS: u64 = 1000;
const ZOOM_MARGIN_MESH: f32 = 0.15;
const ZOOM_MARGIN_NAV: f32 = 0.12;

// Light
const DIRECTIONAL_LIGHT_ILLUMINANCE: f32 = 3000.0;

// Tube mesh
const TUBE_RADIUS: f32 = 0.06;
const TUBE_SIDES: u32 = 16;
const JOINT_RADIUS_MULTIPLIER: f32 = 1.5;
const JOINT_SPHERE_SEGMENTS: u32 = 16;

// Colors
const CABLE_COLOR: Color = Color::srgb(0.9, 0.5, 0.1);
const OBSTACLE_COLOR: Color = Color::srgba(0.8, 0.2, 0.2, 0.25);
const NODE_COLOR: Color = Color::srgba(0.4, 0.6, 0.8, 0.4);
const DRAGGABLE_COLOR: Color = Color::srgb(0.2, 0.7, 0.7);
const DESPAWN_GREEN: Color = Color::srgb(0.3, 0.8, 0.3);
const DESPAWN_RED: Color = Color::srgb(0.8, 0.3, 0.3);

// UI
const UI_FONT_SIZE: f32 = 14.0;
const NAV_FONT_SIZE: f32 = 16.0;

// ============================================================================
// Components and resources
// ============================================================================

#[derive(Resource)]
struct SceneEntities {
    camera: Entity,
    ground: Entity,
}

#[derive(Resource)]
struct CurrentSection(usize);

#[derive(Resource)]
struct SectionBounds(Vec<Entity>);

#[derive(Resource, Default)]
struct DragState {
    entity:      Option<Entity>,
    y_height:    f32,
    /// Offset from the cursor hit point to the entity center (XZ only).
    grab_offset: Vec2,
}

#[derive(Component)]
struct Selected;

/// Shared cable material handle for all cable meshes.
#[derive(Resource)]
struct SharedCableMaterial(Handle<StandardMaterial>);

#[derive(Component)]
struct NodeCube;

#[derive(Component)]
struct Draggable;

#[derive(Component)]
struct Despawnable;

/// Marker for entities belonging to the detach demo section (for reset).
#[derive(Component)]
struct DetachDemoEntity;

/// Marker for the inside-view cable (section 7) — uses `FaceSides::Inside` and large radius.
#[derive(Component)]
struct InsideViewCable;

#[derive(Component)]
struct NavLabel;

#[derive(Clone, Copy, PartialEq, Eq)]
enum NavDirection {
    Left,
    Right,
}

#[derive(Component)]
struct NavButton(NavDirection);

#[derive(Default, Resource)]
struct InspectorVisible(bool);

#[derive(Resource, Reflect, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
struct CableSettings {
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

impl Default for CableSettings {
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
            ResourceInspectorPlugin::<CableSettings>::default()
                .run_if(|visible: Res<InspectorVisible>| visible.0),
        ))
        .init_resource::<CableSettings>()
        .init_resource::<InspectorVisible>()
        .init_resource::<DragState>()
        .insert_resource(CurrentSection(0))
        .add_systems(
            Startup,
            (setup_camera, (setup_scene, setup_sections, setup_ui)).chain(),
        )
        .add_systems(
            Update,
            (
                attach_click_to_cable_meshes,
                update_current_section_from_camera,
                handle_keyboard,
                handle_nav_buttons,
                handle_drag,
                sync_cable_settings.run_if(resource_changed::<CableSettings>),
            ),
        )
        .run();
}

// ============================================================================
// Startup: Camera
// ============================================================================

fn setup_camera(mut commands: Commands) {
    // Camera starts offset in front of section 0 so ZoomToFit gets a good angle
    let focus = Vec3::new(SECTION_X[0], NODE_Y * 0.5, 0.0);
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
            focus,
            target_focus: focus,
            yaw: Some(0.0),
            pitch: Some(0.45),
            radius: Some(12.0),
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
        ground: Entity::PLACEHOLDER,
    });
}

// ============================================================================
// Startup: Scene (ground, light)
// ============================================================================

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut scene: ResMut<SceneEntities>,
) {
    // Ground plane
    let ground = commands
        .spawn((
            Mesh3d(meshes.add(Plane3d::default().mesh().size(GROUND_WIDTH, GROUND_DEPTH))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.2, 0.9, 0.2, 0.09),
                alpha_mode: AlphaMode::Blend,
                double_sided: true,
                cull_mode: None,
                ..default()
            })),
        ))
        .observe(on_ground_clicked)
        .id();
    scene.ground = ground;

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

// ============================================================================
// Startup: All 7 sections
// ============================================================================

fn setup_sections(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cable_mat = materials.add(StandardMaterial {
        base_color: CABLE_COLOR,
        ..default()
    });
    commands.insert_resource(SharedCableMaterial(cable_mat.clone()));

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

    let mut bounds = Vec::new();

    bounds.push(spawn_section_bounds(
        &mut commands,
        &mut meshes,
        &mut materials,
        SECTION_X[0],
    ));
    setup_section_catenary(&mut commands, &node_mesh, &node_mat, &cable_mat);

    bounds.push(spawn_section_bounds(
        &mut commands,
        &mut meshes,
        &mut materials,
        SECTION_X[1],
    ));
    setup_section_cap_styles(&mut commands, &node_mesh, &node_mat, &cable_mat);

    bounds.push(spawn_section_bounds(
        &mut commands,
        &mut meshes,
        &mut materials,
        SECTION_X[2],
    ));
    setup_section_solver_comparison(&mut commands, &node_mesh, &node_mat, &cable_mat);

    bounds.push(spawn_section_bounds(
        &mut commands,
        &mut meshes,
        &mut materials,
        SECTION_X[3],
    ));
    setup_section_entity_attachment(&mut commands, &mut meshes, &mut materials, &cable_mat);

    bounds.push(spawn_section_bounds(
        &mut commands,
        &mut meshes,
        &mut materials,
        SECTION_X[4],
    ));
    setup_section_shared_hub(
        &mut commands,
        &mut meshes,
        &mut materials,
        &node_mesh,
        &node_mat,
        &cable_mat,
    );

    bounds.push(spawn_section_bounds(
        &mut commands,
        &mut meshes,
        &mut materials,
        SECTION_X[5],
    ));
    setup_section_astar(
        &mut commands,
        &mut meshes,
        &mut materials,
        &node_mesh,
        &node_mat,
        &cable_mat,
    );

    bounds.push(spawn_section_bounds(
        &mut commands,
        &mut meshes,
        &mut materials,
        SECTION_X[6],
    ));
    spawn_detach_demo(
        &mut commands,
        &mut meshes,
        &mut materials,
        &node_mesh,
        &node_mat,
        &cable_mat,
    );

    bounds.push(spawn_section_bounds(
        &mut commands,
        &mut meshes,
        &mut materials,
        SECTION_X[7],
    ));
    setup_section_inside_view(&mut commands, &cable_mat);

    commands.insert_resource(SectionBounds(bounds));
}

/// Section 0: Simple catenary cable between two nodes.
fn setup_section_catenary(
    commands: &mut Commands,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[0];
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    spawn_node_pair(commands, node_mesh, node_mat, start, end);
    spawn_cable(
        commands,
        start,
        end,
        Solver::Catenary(CatenarySolver::new().with_slack(SLACK_NORMAL)),
        vec![],
        &cable_mat,
    );
}

/// Section 1: Three cables demonstrating different cap styles.
fn setup_section_cap_styles(
    commands: &mut Commands,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[1];
    for (z, start_cap, end_cap, _label) in [
        (-1.5_f32, CapStyle::Round, CapStyle::Round, "Round / Round"),
        (0.0, CapStyle::flat(), CapStyle::flat(), "Flat / Flat"),
        (1.5, CapStyle::Round, CapStyle::None, "Round / None"),
    ] {
        let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, z);
        let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, z);
        spawn_node_pair(commands, node_mesh, node_mat, start, end);
        spawn_cable_with_caps(
            commands,
            start,
            end,
            Solver::Catenary(CatenarySolver::new().with_slack(1.2)),
            vec![],
            start_cap,
            end_cap,
            cable_mat,
        );
    }
}

/// Section 2: Catenary, linear, and orthogonal solvers side by side.
fn setup_section_solver_comparison(
    commands: &mut Commands,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[2];

    // Catenary
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, -1.5);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, -1.5);
    spawn_node_pair(commands, node_mesh, node_mat, start, end);
    spawn_cable(
        commands,
        start,
        end,
        Solver::Catenary(CatenarySolver::new().with_slack(SLACK_NORMAL)),
        vec![],
        cable_mat,
    );

    // Linear
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    spawn_node_pair(commands, node_mesh, node_mat, start, end);
    spawn_cable(commands, start, end, Solver::Linear, vec![], cable_mat);

    // Orthogonal
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y - 0.5, 1.5);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y + 0.5, 1.5);
    spawn_node_pair(commands, node_mesh, node_mat, start, end);
    spawn_cable(
        commands,
        start,
        end,
        Solver::Routed {
            planner:    Planner::Orthogonal,
            curve:      Curve::Linear,
            resolution: 0,
        },
        vec![],
        cable_mat,
    );
}

/// Section 3: Cables attached to draggable cubes.
fn setup_section_entity_attachment(
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
        .observe(on_drag_start)
        .id();

    let right_cube = commands
        .spawn((
            Mesh3d(drag_mesh),
            MeshMaterial3d(drag_mat),
            Transform::from_translation(Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0)),
            Draggable,
            NodeCube,
        ))
        .observe(on_drag_start)
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

/// Section 4: Three cables from a central draggable hub.
fn setup_section_shared_hub(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[4];
    let drag_mesh = meshes.add(Sphere::new(0.35).mesh().uv(16, 16));
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
        .observe(on_drag_start)
        .id();

    let spokes = [
        Vec3::new(cx - 3.0, NODE_Y + 0.5, -1.5),
        Vec3::new(cx + 3.0, NODE_Y + 0.5, -1.5),
        Vec3::new(cx, NODE_Y + 0.5, 2.0),
    ];

    for spoke_pos in spokes {
        spawn_node_cube(commands, node_mesh, node_mat, spoke_pos);
        commands
            .spawn((
                Cable {
                    solver:     Solver::Catenary(CatenarySolver::new().with_slack(1.2)),
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

/// Section 5: A* pathfinding around an obstacle.
fn setup_section_astar(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[5];
    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    let obstacle_pos = Vec3::new(cx, NODE_Y, 0.0);
    let obstacle = Obstacle::new(OBSTACLE_HALF_EXTENTS, obstacle_pos);

    spawn_node_pair(commands, node_mesh, node_mat, start, end);
    spawn_cable(
        commands,
        start,
        end,
        Solver::Routed {
            planner:    Planner::AStar,
            curve:      Curve::Catenary(CatenarySolver::new().with_slack(1.2)),
            resolution: 0,
        },
        vec![obstacle],
        cable_mat,
    );

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
}

/// Section 7: Inside view — large tube rendered inside-only.
fn setup_section_inside_view(commands: &mut Commands, cable_mat: &Handle<StandardMaterial>) {
    let cx = SECTION_X[7];
    // Tube angled to match the camera's viewing angle (yaw ~-0.48, pitch ~0.5).
    // The near opening faces the camera; the tube extends away and slightly down-left.
    // sin/cos(0.48) ≈ 0.46/0.88 for X offset, tan(0.5) ≈ 0.55 for Y drop.
    // Tube lies mostly flat along Z with a slight sideways lean.
    // The near opening faces the camera; we see inside it.
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
                radius: 1.5,
                sides: 64,
                face_sides: bevy_catenary::FaceSides::Both,
                material: Some(cable_mat.clone()),
                ..default()
            },
            InsideViewCable,
        ))
        .with_children(|parent| {
            parent.spawn(CableEndpoint::new(CableEnd::Start, start).with_cap(CapStyle::None));
            parent.spawn(CableEndpoint::new(CableEnd::End, end).with_cap(CapStyle::None));
        });
}

/// Spawn the detach demo section (can be called again on reset).
fn spawn_detach_demo(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[6];
    let sphere_mesh = meshes.add(Sphere::new(0.35).mesh().uv(16, 16));

    // Top: HangInPlace (green sphere)
    let green_mat = materials.add(StandardMaterial {
        base_color: DESPAWN_GREEN,
        ..default()
    });
    let green_sphere = commands
        .spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(green_mat),
            Transform::from_translation(Vec3::new(cx - 2.0, NODE_Y, -1.5)),
            Despawnable,
            DetachDemoEntity,
        ))
        .observe(on_despawnable_clicked)
        .id();

    let anchor_pos = Vec3::new(cx + 2.0, NODE_Y, -1.5);
    spawn_node_cube_with_marker(commands, node_mesh, node_mat, anchor_pos);

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
            DetachDemoEntity,
        ))
        .with_children(|parent| {
            parent.spawn((
                CableEndpoint::new(CableEnd::Start, Vec3::ZERO),
                AttachedTo(green_sphere),
            ));
            parent.spawn(CableEndpoint::new(CableEnd::End, anchor_pos));
        });

    // Bottom: Despawn (red sphere)
    let red_mat = materials.add(StandardMaterial {
        base_color: DESPAWN_RED,
        ..default()
    });
    let red_sphere = commands
        .spawn((
            Mesh3d(sphere_mesh),
            MeshMaterial3d(red_mat),
            Transform::from_translation(Vec3::new(cx - 2.0, NODE_Y, 1.5)),
            Despawnable,
            DetachDemoEntity,
        ))
        .observe(on_despawnable_clicked)
        .id();

    let anchor_pos = Vec3::new(cx + 2.0, NODE_Y, 1.5);
    spawn_node_cube_with_marker(commands, node_mesh, node_mat, anchor_pos);

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
            DetachDemoEntity,
        ))
        .with_children(|parent| {
            parent.spawn((
                CableEndpoint::new(CableEnd::Start, Vec3::ZERO)
                    .with_detach_policy(DetachPolicy::Despawn),
                AttachedTo(red_sphere),
            ));
            parent.spawn(CableEndpoint::new(CableEnd::End, anchor_pos));
        });
}

// ============================================================================
// Startup: UI
// ============================================================================

fn setup_ui(mut commands: Commands, scene: Res<SceneEntities>) {
    spawn_help_text(&mut commands, scene.camera);
    spawn_keyboard_shortcuts(&mut commands, scene.camera);
    spawn_nav_bar(&mut commands, scene.camera);
}

/// Navigation help panel (top-left corner).
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

/// Keyboard shortcuts panel (bottom-left corner).
fn spawn_keyboard_shortcuts(commands: &mut Commands, camera: Entity) {
    commands.spawn((
        Text::new(
            "D - Debug gizmos\n\
             F - Full scene\n\
             I - Inspector\n\
             N - Toggle nodes",
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

/// Section navigation bar (bottom-center).
fn spawn_nav_bar(commands: &mut Commands, camera: Entity) {
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
            // Left arrow
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

            // Section title (fixed width so bar doesn't resize)
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

            // Right arrow
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

// ============================================================================
// Spawn helpers
// ============================================================================

fn spawn_section_bounds(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    center_x: f32,
) -> Entity {
    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                SPAN_HALF_X.mul_add(2.0, 2.0),
                NODE_Y + 2.0,
                5.0,
            ))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.0, 0.0, 0.0, 0.0),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_translation(Vec3::new(center_x, NODE_Y * 0.5, 0.0)),
            Pickable::IGNORE,
        ))
        .id()
}

fn spawn_cable(
    commands: &mut Commands,
    start: Vec3,
    end: Vec3,
    solver: Solver,
    obstacles: Vec<Obstacle>,
    material: &Handle<StandardMaterial>,
) {
    commands
        .spawn((
            Cable {
                solver,
                obstacles,
                resolution: 0,
            },
            CableMeshConfig {
                material: Some(material.clone()),
                ..default()
            },
        ))
        .with_children(|parent| {
            parent.spawn(CableEndpoint::new(CableEnd::Start, start));
            parent.spawn(CableEndpoint::new(CableEnd::End, end));
        });
}

fn spawn_cable_with_caps(
    commands: &mut Commands,
    start: Vec3,
    end: Vec3,
    solver: Solver,
    obstacles: Vec<Obstacle>,
    cap_start: CapStyle,
    cap_end: CapStyle,
    material: &Handle<StandardMaterial>,
) {
    commands
        .spawn((
            Cable {
                solver,
                obstacles,
                resolution: 0,
            },
            CableMeshConfig {
                material: Some(material.clone()),
                ..default()
            },
        ))
        .with_children(|parent| {
            parent.spawn(CableEndpoint::new(CableEnd::Start, start).with_cap(cap_start));
            parent.spawn(CableEndpoint::new(CableEnd::End, end).with_cap(cap_end));
        });
}

fn spawn_node_pair(
    commands: &mut Commands,
    mesh: &Handle<Mesh>,
    material: &Handle<StandardMaterial>,
    start: Vec3,
    end: Vec3,
) {
    for pos in [start, end] {
        spawn_node_cube(commands, mesh, material, pos);
    }
}

fn spawn_node_cube(
    commands: &mut Commands,
    mesh: &Handle<Mesh>,
    material: &Handle<StandardMaterial>,
    pos: Vec3,
) {
    commands
        .spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_translation(pos),
            NodeCube,
        ))
        .observe(on_mesh_clicked);
}

fn spawn_node_cube_with_marker(
    commands: &mut Commands,
    mesh: &Handle<Mesh>,
    material: &Handle<StandardMaterial>,
    pos: Vec3,
) {
    commands
        .spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_translation(pos),
            NodeCube,
            DetachDemoEntity,
        ))
        .observe(on_mesh_clicked);
}

// ============================================================================
// Navigation
// ============================================================================

fn navigate_to_section(
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
    for mut text in label_query.iter_mut() {
        **text = format!(
            "{} / {SECTION_COUNT} - {}",
            section + 1,
            SECTION_TITLES[section]
        );
    }
}

fn handle_nav_buttons(
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

    // Number keys 1-7 jump directly to that section
    let number_keys = [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Digit4,
        KeyCode::Digit5,
        KeyCode::Digit6,
        KeyCode::Digit7,
        KeyCode::Digit8,
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

// ============================================================================
// Drag system
// ============================================================================

fn handle_drag(
    mut drag_state: ResMut<DragState>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
    mut draggables: Query<&mut Transform, With<Draggable>>,
) {
    // End drag on mouse release
    if mouse_buttons.just_released(MouseButton::Left) {
        drag_state.entity = None;
        return;
    }

    // Start drag on mouse press over a draggable
    // (handled by the observer below -this system only does move + release)

    // Move during drag
    let Some(dragged) = drag_state.entity else {
        return;
    };
    if !mouse_buttons.pressed(MouseButton::Left) {
        drag_state.entity = None;
        return;
    }

    let Ok((camera, cam_tf)) = cameras.single() else {
        return;
    };
    let Ok(window) = windows.single() else {
        return;
    };
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(cam_tf, cursor) else {
        return;
    };

    // Intersect ray with Y = drag_state.y_height plane
    let denom = ray.direction.y;
    if denom.abs() < 1e-6 {
        return;
    }
    let t = (drag_state.y_height - ray.origin.y) / denom;
    if t < 0.0 {
        return;
    }
    let hit = ray.origin + ray.direction * t;

    if let Ok(mut transform) = draggables.get_mut(dragged) {
        transform.translation.x = hit.x + drag_state.grab_offset.x;
        transform.translation.z = hit.z + drag_state.grab_offset.y;
    }
}

fn on_drag_start(
    click: On<Pointer<Press>>,
    mut drag_state: ResMut<DragState>,
    transforms: Query<&Transform, With<Draggable>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
) {
    if click.button != PointerButton::Primary {
        return;
    }
    let entity = click.entity;
    let Ok(tf) = transforms.get(entity) else {
        return;
    };
    drag_state.entity = Some(entity);
    drag_state.y_height = tf.translation.y;
    drag_state.grab_offset = Vec2::ZERO;

    // Raycast the cursor to the entity's Y-plane to get a consistent grab offset
    let Ok((camera, cam_tf)) = cameras.single() else {
        return;
    };
    let Ok(window) = windows.single() else {
        return;
    };
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(cam_tf, cursor) else {
        return;
    };
    let denom = ray.direction.y;
    if denom.abs() < 1e-6 {
        return;
    }
    let t = (tf.translation.y - ray.origin.y) / denom;
    if t < 0.0 {
        return;
    }
    let hit = ray.origin + ray.direction * t;
    drag_state.grab_offset = Vec2::new(tf.translation.x - hit.x, tf.translation.z - hit.z);
}

// ============================================================================
// Detach demo interaction
// ============================================================================

fn on_despawnable_clicked(click: On<Pointer<Click>>, mut commands: Commands) {
    commands.entity(click.entity).despawn();
}

// ============================================================================
// Runtime systems
// ============================================================================

/// Track which section the camera is nearest to and update the nav label.
fn update_current_section_from_camera(
    cameras: Query<&PanOrbitCamera>,
    mut current: ResMut<CurrentSection>,
    mut label_query: Query<&mut Text, With<NavLabel>>,
) {
    let Ok(cam) = cameras.single() else {
        return;
    };
    let cam_x = cam.focus.x;
    let nearest = SECTION_X
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            (cam_x - *a)
                .abs()
                .partial_cmp(&(cam_x - *b).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(i, _)| i)
        .unwrap_or(0);

    if nearest != current.0 {
        current.0 = nearest;
        for mut text in label_query.iter_mut() {
            **text = format!(
                "{} / {SECTION_COUNT} - {}",
                nearest + 1,
                SECTION_TITLES[nearest]
            );
        }
    }
}

/// Attach click-to-zoom observer to cable mesh children spawned by the library.
fn attach_click_to_cable_meshes(
    mut commands: Commands,
    new_cables: Query<&CableMeshChild, Added<CableMeshChild>>,
) {
    for mesh_child in &new_cables {
        commands.entity(mesh_child.0).observe(on_mesh_clicked);
    }
}

/// Sync `CableSettings` resource to each cable's `CableMeshConfig` when the inspector changes
/// values.
fn sync_cable_settings(
    settings: Res<CableSettings>,
    mut commands: Commands,
    cables: Query<
        (Entity, &CableMeshConfig, &ComputedCableGeometry),
        (With<Cable>, Without<InsideViewCable>),
    >,
) {
    for (entity, config, computed) in &cables {
        let mut new_config = config.clone();
        new_config.radius = settings.tube_radius;
        new_config.sides = settings.tube_sides;
        new_config.elbow_bend_radius_multiplier = settings.elbow_bend_radius_multiplier;
        new_config.elbow_min_radius_multiplier = settings.elbow_min_radius_multiplier;
        new_config.elbow_rings_per_right_angle = settings.elbow_rings_per_right_angle;
        new_config.elbow_angle_threshold_deg = settings.elbow_angle_threshold_deg;
        new_config.elbow_arm_multiplier = settings.elbow_arm_multiplier;
        // Re-insert both to trigger the mesh rebuild observer
        commands
            .entity(entity)
            .insert((new_config, computed.clone()));
    }
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn handle_keyboard(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut debug_enabled: ResMut<CableDebugEnabled>,
    mut inspector_visible: ResMut<InspectorVisible>,
    scene: Res<SceneEntities>,
    mut node_cubes: Query<&mut Visibility, With<NodeCube>>,
    // Reset detach demo
    shared_cable_mat: Res<SharedCableMaterial>,
    detach_entities: Query<Entity, With<DetachDemoEntity>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    node_mesh_query: Query<&Mesh3d, (With<NodeCube>, Without<Draggable>, Without<Despawnable>)>,
    node_mat_query: Query<
        &MeshMaterial3d<StandardMaterial>,
        (With<NodeCube>, Without<Draggable>, Without<Despawnable>),
    >,
) {
    if keyboard.just_pressed(KeyCode::KeyD) {
        debug_enabled.0 = !debug_enabled.0;
    }

    if keyboard.just_pressed(KeyCode::KeyI) {
        inspector_visible.0 = !inspector_visible.0;
    }

    // F: Full scene — zoom out to see everything
    if keyboard.just_pressed(KeyCode::KeyF) {
        commands.trigger(
            ZoomToFit::new(scene.camera, scene.ground)
                .margin(0.05)
                .duration(Duration::from_millis(NAV_DURATION_MS))
                .easing(EaseFunction::CubicOut),
        );
    }

    if keyboard.just_pressed(KeyCode::KeyN) {
        for mut vis in &mut node_cubes {
            *vis = match *vis {
                Visibility::Hidden => Visibility::Inherited,
                _ => Visibility::Hidden,
            };
        }
    }

    // R: Reset detach demo
    if keyboard.just_pressed(KeyCode::KeyR) {
        // Get mesh/mat handles before despawning
        let node_mesh = node_mesh_query.iter().next().map(|m| m.0.clone());
        let node_mat = node_mat_query.iter().next().map(|m| m.0.clone());

        for entity in &detach_entities {
            commands.entity(entity).despawn();
        }

        if let (Some(nm), Some(nmat)) = (node_mesh, node_mat) {
            spawn_detach_demo(
                &mut commands,
                &mut meshes,
                &mut materials,
                &nm,
                &nmat,
                &shared_cable_mat.0,
            );
        }
    }
}

// ============================================================================
// Pointer interaction
// ============================================================================

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

fn on_ground_clicked(
    _click: On<Pointer<Click>>,
    mut commands: Commands,
    selected: Query<Entity, With<Selected>>,
    scene: Res<SceneEntities>,
    bounds: Res<SectionBounds>,
    current: Res<CurrentSection>,
) {
    for entity in &selected {
        commands.entity(entity).remove::<Selected>();
    }

    // Zoom back to current section (not full scene)
    commands.trigger(
        ZoomToFit::new(scene.camera, bounds.0[current.0])
            .margin(ZOOM_MARGIN_NAV)
            .duration(Duration::from_millis(ZOOM_DURATION_MS))
            .easing(EaseFunction::CubicOut),
    );
}
