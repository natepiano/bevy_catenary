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
use bevy_catenary::CableEnd;
use bevy_catenary::CableEndpoint;
use bevy_catenary::CableMeshChild;
use bevy_catenary::CableMeshConfig;
use bevy_catenary::Capping;
use bevy_catenary::CatenaryPlugin;
use bevy_catenary::CatenarySolver;
use bevy_catenary::ComputedCableGeometry;
use bevy_catenary::CurveKind;
use bevy_catenary::DEFAULT_SLACK;
use bevy_catenary::DebugGizmos;
use bevy_catenary::EndpointAlignment;
use bevy_catenary::FaceSides;
use bevy_catenary::Obstacle;
use bevy_catenary::OnDetach;
use bevy_catenary::PathStrategy;
use bevy_catenary::Solver;
use bevy_catenary::TubeConfig;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::inspector_options::std_options::NumberDisplay;
use bevy_inspector_egui::prelude::*;
use bevy_inspector_egui::quick::ResourceInspectorPlugin;
use bevy_kana::Position;
use bevy_lagrange::InputControl;
use bevy_lagrange::LagrangePlugin;
use bevy_lagrange::OrbitCam;
use bevy_lagrange::TrackpadBehavior;
use bevy_lagrange::TrackpadInput;
use bevy_lagrange::ZoomToFit;

mod animation;
mod constants;

use animation::LightAnimation;
use animation::TubeLight;
use constants::CABLE_COLOR;
use constants::CAP_STYLE_RADIUS_MULTIPLIER;
use constants::CAP_STYLE_TUBE_OFFSET;
use constants::CAP_STYLE_TUBE_SPACING;
use constants::DESPAWN_GREEN;
use constants::DESPAWN_RED;
use constants::DETACH_BUMP_BLUE;
use constants::DIRECTIONAL_LIGHT_ILLUMINANCE;
use constants::DRAGGABLE_COLOR;
use constants::DRAGGABLE_CUBE_SIZE;
use constants::GROUND_DEPTH;
use constants::GROUND_WIDTH;
use constants::HUB_SPHERE_RADIUS;
use constants::INSIDE_VIEW_RADIUS_MULTIPLIER;
use constants::JOINT_RADIUS_MULTIPLIER;
use constants::JOINT_SPHERE_SEGMENTS;
use constants::NAV_DURATION_MS;
use constants::NAV_FONT_SIZE;
use constants::NODE_COLOR;
use constants::NODE_CUBE_SIZE;
use constants::NODE_Y;
use constants::OBSTACLE_COLOR;
use constants::OBSTACLE_HALF_EXTENTS;
use constants::POINT_LIGHT_COLOR;
use constants::POINT_LIGHT_INTENSITY;
use constants::POINT_LIGHT_RANGE;
use constants::RAY_EPSILON;
use constants::SECTION_COUNT;
use constants::SECTION_TITLES;
use constants::SECTION_X;
use constants::SLACK_ADJUSTMENT_STEP;
use constants::SLACK_NORMAL;
use constants::SPAN_HALF_X;
use constants::TRANSPARENT_TUBE_COLOR;
use constants::TUBE_RADIUS;
use constants::TUBE_SIDES;
use constants::UI_FONT_SIZE;
use constants::ZOOM_DURATION_MS;
use constants::ZOOM_MARGIN_MESH;
use constants::ZOOM_MARGIN_NAV;

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

/// Marker to exclude a cable from global +/- slack adjustment.
#[derive(Component)]
struct SlackLocked;

/// Marker for cables with a radius multiplier relative to the inspector setting.
/// The `sync_cable_settings` system applies `radius * multiplier` instead of raw radius.
#[derive(Component)]
struct RadiusMultiplier(f32);

/// UI text that is only visible when viewing a specific section.
#[derive(Component)]
struct SectionInfo(usize);

#[derive(Clone, Copy)]
enum SlackAdjustment {
    Increase,
    Decrease,
    None,
}

impl SlackAdjustment {
    fn from_keyboard(keyboard: &ButtonInput<KeyCode>) -> Self {
        match (
            keyboard.pressed(KeyCode::Equal),
            keyboard.pressed(KeyCode::Minus),
        ) {
            (true, false) => Self::Increase,
            (false, true) => Self::Decrease,
            _ => Self::None,
        }
    }

    const fn delta(self) -> f32 {
        match self {
            Self::Increase => SLACK_ADJUSTMENT_STEP,
            Self::Decrease => -SLACK_ADJUSTMENT_STEP,
            Self::None => 0.0,
        }
    }
}

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
enum InspectorVisibility {
    Visible,
    #[default]
    Hidden,
}

#[derive(Resource, Reflect, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
struct CableSettings {
    tube:  TubeSettings,
    joint: JointSettings,
    elbow: ElbowSettings,
}

#[derive(Reflect, InspectorOptions)]
#[reflect(InspectorOptions)]
struct TubeSettings {
    #[inspector(min = 0.01, max = 0.3, display = NumberDisplay::Slider)]
    radius: f32,
    #[inspector(min = 1, max = 64, display = NumberDisplay::Slider)]
    sides:  u32,
}

#[derive(Reflect, InspectorOptions)]
#[reflect(InspectorOptions)]
struct JointSettings {
    #[inspector(min = 1.0, max = 4.0, display = NumberDisplay::Slider)]
    radius_multiplier: f32,
    #[inspector(min = 8, max = 32, display = NumberDisplay::Slider)]
    segments:          u32,
}

#[derive(Reflect, InspectorOptions)]
#[reflect(InspectorOptions)]
struct ElbowSettings {
    #[inspector(min = 1.0, max = 20.0, display = NumberDisplay::Slider)]
    bend_radius_multiplier: f32,
    #[inspector(min = 0.5, max = 5.0, display = NumberDisplay::Slider)]
    min_radius_multiplier:  f32,
    #[inspector(min = 2, max = 32, display = NumberDisplay::Slider)]
    rings_per_right_angle:  u32,
    #[inspector(min = 1.0, max = 90.0, display = NumberDisplay::Slider)]
    angle_threshold_deg:    f32,
    #[inspector(min = 0.1, max = 3.0, display = NumberDisplay::Slider)]
    arm_multiplier:         f32,
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
            LagrangePlugin,
            MeshPickingPlugin,
            BrpExtrasPlugin::default(),
            CatenaryPlugin,
            ResourceInspectorPlugin::<CableSettings>::default().run_if(
                |visible: Res<InspectorVisibility>| {
                    matches!(*visible, InspectorVisibility::Visible)
                },
            ),
        ))
        .init_resource::<CableSettings>()
        .init_resource::<InspectorVisibility>()
        .init_resource::<DragState>()
        .init_resource::<LightAnimation>()
        .insert_resource(CurrentSection(0))
        .add_systems(
            Startup,
            (setup_camera, (setup_scene, setup_sections, setup_ui)).chain(),
        )
        .add_systems(
            Update,
            (
                update_current_section_from_camera,
                update_section_info_visibility,
                handle_keyboard,
                handle_nav_buttons,
                handle_drag,
                sync_cable_settings.run_if(resource_changed::<CableSettings>),
                animation::animate_tube_light,
            ),
        )
        .add_observer(on_cable_mesh_child_added)
        .run();
}

// ============================================================================
// Startup: Camera
// ============================================================================

fn setup_camera(mut commands: Commands) {
    // Camera starts offset in front of section 0 so `ZoomToFit` gets a good angle.
    let focus = Vec3::new(SECTION_X[0], NODE_Y * 0.5, 0.0);
    let camera = commands
        .spawn(OrbitCam {
            button_orbit: MouseButton::Middle,
            button_pan: MouseButton::Middle,
            modifier_pan: Some(KeyCode::ShiftLeft),
            input_control: Some(InputControl {
                trackpad: Some(TrackpadInput {
                    behavior:    TrackpadBehavior::BlenderLike {
                        modifier_pan:  Some(KeyCode::ShiftLeft),
                        modifier_zoom: Some(KeyCode::ControlLeft),
                    },
                    sensitivity: 0.3,
                }),
                ..Default::default()
            }),
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
    asset_server: Res<AssetServer>,
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

    let bounds = spawn_all_sections(
        &mut commands,
        &mut meshes,
        &mut materials,
        &node_mesh,
        &node_mat,
        &cable_mat,
        &asset_server,
    );
    commands.insert_resource(SectionBounds(bounds));
}

fn spawn_all_sections(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    node_mesh: &Handle<Mesh>,
    node_mat: &Handle<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
    asset_server: &AssetServer,
) -> Vec<Entity> {
    let mut bounds = Vec::new();

    bounds.push(spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[0],
    ));
    setup_section_catenary(commands, node_mesh, node_mat, cable_mat);

    bounds.push(spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[1],
    ));
    setup_section_cap_styles(commands, materials, cable_mat);

    bounds.push(spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[2],
    ));
    setup_section_solver_comparison(commands, node_mesh, node_mat, cable_mat);

    bounds.push(spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[3],
    ));
    setup_section_entity_attachment(commands, meshes, materials, cable_mat);

    bounds.push(spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[4],
    ));
    setup_section_shared_hub(commands, meshes, materials, node_mesh, node_mat, cable_mat);

    bounds.push(spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[5],
    ));
    setup_section_astar(commands, meshes, materials, node_mesh, node_mat, cable_mat);

    bounds.push(spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[6],
    ));
    spawn_detach_demo(commands, meshes, materials, node_mesh, node_mat, cable_mat);

    bounds.push(spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[7],
    ));
    setup_section_inside_view(commands, cable_mat);

    bounds.push(spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[8],
    ));
    setup_section_connector(commands, cable_mat, asset_server);

    bounds
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
        cable_mat,
    );
}

/// Section 1: Three cables with different cap combinations — each end is freely choosable.
/// Left: Round/Round, Middle: Round/Flat, Right: Round/None.
/// The near end (facing the camera) varies to show that each end's cap is freely choosable.
fn spawn_cap_style_tube(
    commands: &mut Commands,
    material: Handle<StandardMaterial>,
    start: Vec3,
    end: Vec3,
    start_cap: Capping,
    end_cap: Capping,
) {
    commands
        .spawn((
            Cable {
                solver:     Solver::Linear,
                obstacles:  vec![],
                resolution: 0,
            },
            CableMeshConfig {
                tube: TubeConfig {
                    radius: TUBE_RADIUS * CAP_STYLE_RADIUS_MULTIPLIER,
                    faces: FaceSides::Both,
                    ..default()
                },
                material: Some(material),
                ..default()
            },
            RadiusMultiplier(CAP_STYLE_RADIUS_MULTIPLIER),
        ))
        .with_children(|parent| {
            parent.spawn(CableEndpoint::new(CableEnd::Start, start).with_cap(start_cap));
            parent.spawn(CableEndpoint::new(CableEnd::End, end).with_cap(end_cap));
        });
}

fn setup_section_cap_styles(
    commands: &mut Commands,
    materials: &mut Assets<StandardMaterial>,
    cable_mat: &Handle<StandardMaterial>,
) {
    let cx = SECTION_X[1];

    // Transparent material for the left tube (Round/Round) — 80% transparent
    let transparent_mat = materials.add(StandardMaterial {
        base_color: TRANSPARENT_TUBE_COLOR,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    // Left: Round/Round — transparent, both sides, light inside shines through
    let left_start = Vec3::new(
        2.0f32.mul_add(-CAP_STYLE_TUBE_SPACING, cx),
        NODE_Y,
        -CAP_STYLE_TUBE_OFFSET,
    );
    let left_end = Vec3::new(cx - CAP_STYLE_TUBE_SPACING, NODE_Y, CAP_STYLE_TUBE_OFFSET);
    spawn_cap_style_tube(
        commands,
        transparent_mat,
        left_start,
        left_end,
        Capping::Round,
        Capping::Round,
    );

    // Middle: Round/Flat — open back (None on start) so we can see the flat cap lit from inside
    let mid_start = Vec3::new(
        cx - CAP_STYLE_TUBE_SPACING / 2.0,
        NODE_Y,
        -CAP_STYLE_TUBE_OFFSET,
    );
    let mid_end = Vec3::new(
        cx + CAP_STYLE_TUBE_SPACING / 2.0,
        NODE_Y,
        CAP_STYLE_TUBE_OFFSET,
    );
    spawn_cap_style_tube(
        commands,
        cable_mat.clone(),
        mid_start,
        mid_end,
        Capping::None,
        Capping::flat(),
    );

    // Right: Round/None — open front, round cap at back
    let right_start = Vec3::new(cx + CAP_STYLE_TUBE_SPACING, NODE_Y, -CAP_STYLE_TUBE_OFFSET);
    let right_end = Vec3::new(
        2.0f32.mul_add(CAP_STYLE_TUBE_SPACING, cx),
        NODE_Y,
        CAP_STYLE_TUBE_OFFSET,
    );
    spawn_cap_style_tube(
        commands,
        cable_mat.clone(),
        right_start,
        right_end,
        Capping::Round,
        Capping::None,
    );

    // Animated point lights inside each tube, staggered start positions.
    // All oscillate at the same speed (2s each direction, 0.5s pause at ends).
    let tubes = [
        (left_start, left_end, 0.3),   // left: start near middle
        (mid_start, mid_end, 0.7),     // middle: start near far end
        (right_start, right_end, 0.0), // right: start at open end
    ];
    for (start, end, initial_t) in tubes {
        commands.spawn((
            PointLight {
                color: POINT_LIGHT_COLOR,
                intensity: POINT_LIGHT_INTENSITY,
                range: POINT_LIGHT_RANGE,
                shadows_enabled: false,
                ..default()
            },
            Transform::from_translation(start.lerp(end, initial_t)),
            NotShadowCaster,
            TubeLight { start, end },
        ));
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
            planner:    PathStrategy::Orthogonal,
            curve:      CurveKind::Linear,
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
    let obstacle_pos = Position::new(cx, NODE_Y, 0.0);
    let obstacle = Obstacle::new(OBSTACLE_HALF_EXTENTS, obstacle_pos);

    spawn_node_pair(commands, node_mesh, node_mat, start, end);
    spawn_cable(
        commands,
        start,
        end,
        Solver::Routed {
            planner:    PathStrategy::AStar,
            curve:      CurveKind::Catenary(CatenarySolver::new().with_slack(DEFAULT_SLACK)),
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
            Transform::from_translation(*obstacle_pos),
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

/// Section 8: Connector model — GLTF power plug at one cable end.
///
/// # Connector model origin convention
///
/// When attaching a 3D model (GLTF/GLB) to a cable end, the model's **origin must be
/// at the cable attachment point** — the point where the cable tube meets the connector.
/// For a power plug, this is the cable exit (strain relief opening). For a TRS jack,
/// it would be the back of the barrel where the cable enters.
///
/// The cable endpoint uses `AttachedTo` with `Vec3::ZERO` offset, meaning the cable
/// terminates at the connector's origin. If the origin is elsewhere (e.g. center of
/// the plug body), the cable will visually end inside the model.
///
/// In Blender, set the origin by shifting the mesh vertices so the attachment point
/// sits at (0, 0, 0), then re-export the GLB. The connector's local +Y axis (Blender +Z)
/// should point along the cable-exit direction so the alignment system can orient it
/// to match the cable tangent.
fn setup_section_connector(
    commands: &mut Commands,
    cable_mat: &Handle<StandardMaterial>,
    asset_server: &AssetServer,
) {
    let cx = SECTION_X[8];
    let plug_scene: Handle<Scene> = asset_server.load("models/power_plug.glb#Scene0");

    // Three cables side by side: front is Fixed (no roll), middle is AsSpawned (plug keeps
    // its spawn orientation), back is Rotating (follows twist).
    let configs = [
        (
            Vec3::new(cx - SPAN_HALF_X, NODE_Y, 1.5),
            Vec3::new(cx + SPAN_HALF_X, NODE_Y, 1.5),
            EndpointAlignment::Fixed,
        ),
        (
            Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0),
            Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0),
            EndpointAlignment::AsSpawned,
        ),
        (
            Vec3::new(cx - SPAN_HALF_X, NODE_Y, -1.5),
            Vec3::new(cx + SPAN_HALF_X, NODE_Y, -1.5),
            EndpointAlignment::Rotating,
        ),
    ];

    for (start, end, alignment) in configs {
        let cable = commands
            .spawn((
                Cable {
                    solver:     Solver::Linear,
                    obstacles:  vec![],
                    resolution: 0,
                },
                CableMeshConfig {
                    material: Some(cable_mat.clone()),
                    ..default()
                },
            ))
            .id();

        let plug = commands
            .spawn((
                SceneRoot(plug_scene.clone()),
                Transform::from_translation(end).with_scale(Vec3::splat(15.0)),
                Draggable,
            ))
            .observe(on_drag_start)
            .id();

        commands.entity(cable).with_children(|parent| {
            parent.spawn(CableEndpoint::new(CableEnd::Start, start));
            parent.spawn((
                CableEndpoint::new(CableEnd::End, Vec3::ZERO)
                    .with_cap(Capping::None)
                    .with_alignment(alignment),
                AttachedTo(plug),
            ));
        });
    }
}

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
    let assets = DetachDemoAssets {
        sphere_mesh: meshes.add(Sphere::new(HUB_SPHERE_RADIUS).mesh().uv(16, 16)),
        node_mesh,
        node_mat,
        cable_mat,
    };

    let rows = [
        // Top: default Remain — cable keeps its shape on detach.
        DetachDemoRow {
            z:            -1.5,
            sphere_color: DESPAWN_GREEN,
            solver:       CatenarySolver::new().with_slack(SLACK_NORMAL),
            policy:       OnDetach::Remain,
        },
        // Middle: Remain + catenary detach_slack_bump — cable droops visibly on detach.
        DetachDemoRow {
            z:            0.0,
            sphere_color: DETACH_BUMP_BLUE,
            solver:       CatenarySolver::new()
                .with_slack(SLACK_NORMAL)
                .with_detach_slack_bump(0.35),
            policy:       OnDetach::Remain,
        },
        // Bottom: Despawn — whole cable despawns on detach.
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
        .observe(on_despawnable_clicked)
        .id();

    let anchor_pos = Vec3::new(cx + 2.0, NODE_Y, row.z);
    spawn_node_cube(commands, assets.node_mesh, assets.node_mat, anchor_pos)
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

// ============================================================================
// Startup: UI
// ============================================================================

fn setup_ui(mut commands: Commands, scene: Res<SceneEntities>) {
    spawn_help_text(&mut commands, scene.camera);
    spawn_keyboard_shortcuts(&mut commands, scene.camera);
    spawn_nav_bar(&mut commands, scene.camera);
    spawn_section_infos(&mut commands, scene.camera);
}

/// Spawn per-section info text (top-center, hidden by default).
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

fn spawn_node_cube<'a>(
    commands: &'a mut Commands,
    mesh: &Handle<Mesh>,
    material: &Handle<StandardMaterial>,
    pos: Vec3,
) -> EntityCommands<'a> {
    let mut ec = commands.spawn((
        Mesh3d(mesh.clone()),
        MeshMaterial3d(material.clone()),
        Transform::from_translation(pos),
        NodeCube,
    ));
    ec.observe(on_mesh_clicked);
    ec
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
    update_nav_label(label_query, section);
}

fn update_nav_label(label_query: &mut Query<&mut Text, With<NavLabel>>, section: usize) {
    for mut text in label_query.iter_mut() {
        **text = format!(
            "{} / {SECTION_COUNT} - {}",
            section + 1,
            SECTION_TITLES[section]
        );
    }
}

fn deselect_all(commands: &mut Commands, selected: &Query<Entity, With<Selected>>) {
    for entity in selected {
        commands.entity(entity).remove::<Selected>();
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

    // Number keys 1-9 jump directly to that section
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

// ============================================================================
// Drag system
// ============================================================================

/// Intersect a camera ray with a horizontal Y-plane.
fn cursor_ray_y_plane(
    camera: &Camera,
    cam_tf: &GlobalTransform,
    window: &Window,
    y_height: f32,
) -> Option<Vec3> {
    let cursor = window.cursor_position()?;
    let ray = camera.viewport_to_world(cam_tf, cursor).ok()?;
    let denom = ray.direction.y;
    if denom.abs() < RAY_EPSILON {
        return None;
    }
    let t = (y_height - ray.origin.y) / denom;
    if t < 0.0 {
        return None;
    }
    Some(ray.origin + ray.direction * t)
}

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
    let Some(hit) = cursor_ray_y_plane(camera, cam_tf, window, drag_state.y_height) else {
        return;
    };

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
    let Some(hit) = cursor_ray_y_plane(camera, cam_tf, window, tf.translation.y) else {
        return;
    };
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

/// Show/hide per-section info text based on current section.
fn update_section_info_visibility(
    current: Res<CurrentSection>,
    mut infos: Query<(&SectionInfo, &mut Visibility)>,
) {
    for (info, mut vis) in &mut infos {
        *vis = if info.0 == current.0 {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
    }
}

/// Track which section the camera is nearest to and update the nav label.
fn update_current_section_from_camera(
    cameras: Query<&OrbitCam>,
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
        .map_or(0, |(i, _)| i);

    if nearest != current.0 {
        current.0 = nearest;
        update_nav_label(&mut label_query, nearest);
    }
}

/// Attach click-to-zoom observers to cable mesh children spawned by the library.
fn on_cable_mesh_child_added(
    trigger: On<Insert, CableMeshChild>,
    cables: Query<&CableMeshChild>,
    mut commands: Commands,
) {
    let Ok(mesh_child) = cables.get(trigger.event_target()) else {
        return;
    };
    commands.entity(mesh_child.0).observe(on_mesh_clicked);
}

/// Sync `CableSettings` resource to each cable's `CableMeshConfig` when the inspector changes
/// values.
fn sync_cable_settings(
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
        // Re-insert both to trigger the mesh rebuild observer
        commands
            .entity(entity)
            .insert((new_config, computed.clone()));
    }
}

fn handle_keyboard(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut debug_enabled: ResMut<DebugGizmos>,
    mut inspector_visible: ResMut<InspectorVisibility>,
    scene: Res<SceneEntities>,
    // Slack adjustment (exclude `SlackLocked` cables)
    mut cables: Query<&mut Cable, Without<SlackLocked>>,
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
        *debug_enabled = match *debug_enabled {
            DebugGizmos::Enabled => DebugGizmos::Disabled,
            DebugGizmos::Disabled => DebugGizmos::Enabled,
        };
    }

    if keyboard.just_pressed(KeyCode::KeyI) {
        *inspector_visible = match *inspector_visible {
            InspectorVisibility::Visible => InspectorVisibility::Hidden,
            InspectorVisibility::Hidden => InspectorVisibility::Visible,
        };
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

    // +/-: Adjust catenary slack
    let slack_delta = SlackAdjustment::from_keyboard(&keyboard).delta();
    if slack_delta != 0.0 {
        for mut cable in &mut cables {
            match &mut cable.solver {
                Solver::Catenary(catenary)
                | Solver::Routed {
                    curve: CurveKind::Catenary(catenary),
                    ..
                } => {
                    catenary.slack = (catenary.slack + slack_delta).max(1.0);
                },
                _ => {},
            }
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
    deselect_all(&mut commands, &selected);

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
    deselect_all(&mut commands, &selected);

    // Zoom back to current section (not full scene)
    commands.trigger(
        ZoomToFit::new(scene.camera, bounds.0[current.0])
            .margin(ZOOM_MARGIN_NAV)
            .duration(Duration::from_millis(ZOOM_DURATION_MS))
            .easing(EaseFunction::CubicOut),
    );
}
