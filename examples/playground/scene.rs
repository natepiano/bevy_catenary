//! Camera, ground, directional light, and all 7 section scene setups.

use std::f32::consts::PI;

use bevy::light::NotShadowCaster;
use bevy::prelude::*;
use bevy_catenary::AttachedTo;
use bevy_catenary::Cable;
use bevy_catenary::CableEnd;
use bevy_catenary::CableEndpoint;
use bevy_catenary::CableMeshConfig;
use bevy_catenary::Capping;
use bevy_catenary::CatenarySolver;
use bevy_catenary::CurveKind;
use bevy_catenary::DEFAULT_SLACK;
use bevy_catenary::FaceSides;
use bevy_catenary::Obstacle;
use bevy_catenary::PathStrategy;
use bevy_catenary::Solver;
use bevy_catenary::TubeConfig;
use bevy_kana::Position;
use bevy_lagrange::InputControl;
use bevy_lagrange::OrbitCam;
use bevy_lagrange::TrackpadBehavior;
use bevy_lagrange::TrackpadInput;

use super::animation::TubeLight;
use super::connector;
use super::constants::CABLE_COLOR;
use super::constants::CAP_STYLE_RADIUS_MULTIPLIER;
use super::constants::CAP_STYLE_TUBE_OFFSET;
use super::constants::CAP_STYLE_TUBE_SPACING;
use super::constants::DIRECTIONAL_LIGHT_ILLUMINANCE;
use super::constants::DRAGGABLE_COLOR;
use super::constants::DRAGGABLE_CUBE_SIZE;
use super::constants::GROUND_DEPTH;
use super::constants::GROUND_WIDTH;
use super::constants::HUB_SPHERE_RADIUS;
use super::constants::INSIDE_VIEW_RADIUS_MULTIPLIER;
use super::constants::NODE_COLOR;
use super::constants::NODE_CUBE_SIZE;
use super::constants::NODE_Y;
use super::constants::OBSTACLE_COLOR;
use super::constants::OBSTACLE_HALF_EXTENTS;
use super::constants::POINT_LIGHT_COLOR;
use super::constants::POINT_LIGHT_INTENSITY;
use super::constants::POINT_LIGHT_RANGE;
use super::constants::SECTION_X;
use super::constants::SLACK_NORMAL;
use super::constants::SPAN_HALF_X;
use super::constants::TRANSPARENT_TUBE_COLOR;
use super::constants::TUBE_RADIUS;
use super::detach_demo;
use super::entities;
use super::entities::Draggable;
use super::entities::NodeCube;
use super::input;
use super::sections;
use super::sections::SectionBounds;

#[derive(Resource)]
pub(crate) struct SceneEntities {
    pub(crate) camera: Entity,
    pub(crate) ground: Entity,
}

/// Shared cable material handle for all cable meshes.
#[derive(Resource)]
pub(crate) struct SharedCableMaterial(pub(crate) Handle<StandardMaterial>);

/// Marker for cables with a radius multiplier relative to the inspector setting.
/// The `sync_cable_settings` system applies `radius * multiplier` instead of raw radius.
#[derive(Component)]
pub(crate) struct RadiusMultiplier(pub(crate) f32);

pub(crate) fn setup_camera(mut commands: Commands) {
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

pub(crate) fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut scene: ResMut<SceneEntities>,
) {
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
        .observe(input::on_ground_clicked)
        .id();
    scene.ground = ground;

    commands.spawn((
        DirectionalLight {
            illuminance: DIRECTIONAL_LIGHT_ILLUMINANCE,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::ZYX, 0.0, PI / 4.0, -PI / 4.0)),
    ));
}

pub(crate) fn setup_sections(
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

    bounds.push(sections::spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[0],
    ));
    setup_section_catenary(commands, node_mesh, node_mat, cable_mat);

    bounds.push(sections::spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[1],
    ));
    setup_section_cap_styles(commands, materials, cable_mat);

    bounds.push(sections::spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[2],
    ));
    setup_section_solver_comparison(commands, node_mesh, node_mat, cable_mat);

    bounds.push(sections::spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[3],
    ));
    setup_section_entity_attachment(commands, meshes, materials, cable_mat);

    bounds.push(sections::spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[4],
    ));
    setup_section_shared_hub(commands, meshes, materials, node_mesh, node_mat, cable_mat);

    bounds.push(sections::spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[5],
    ));
    setup_section_astar(commands, meshes, materials, node_mesh, node_mat, cable_mat);

    bounds.push(sections::spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[6],
    ));
    detach_demo::spawn_detach_demo(commands, meshes, materials, node_mesh, node_mat, cable_mat);

    bounds.push(sections::spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[7],
    ));
    setup_section_inside_view(commands, cable_mat);

    bounds.push(sections::spawn_section_bounds(
        commands,
        meshes,
        materials,
        SECTION_X[8],
    ));
    connector::setup_section_connector(commands, cable_mat, asset_server);

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
    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(
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

    let transparent_mat = materials.add(StandardMaterial {
        base_color: TRANSPARENT_TUBE_COLOR,
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

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

    let tubes = [
        (left_start, left_end, 0.3),
        (mid_start, mid_end, 0.7),
        (right_start, right_end, 0.0),
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

    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, -1.5);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, -1.5);
    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(
        commands,
        start,
        end,
        Solver::Catenary(CatenarySolver::new().with_slack(SLACK_NORMAL)),
        vec![],
        cable_mat,
    );

    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y, 0.0);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y, 0.0);
    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(commands, start, end, Solver::Linear, vec![], cable_mat);

    let start = Vec3::new(cx - SPAN_HALF_X, NODE_Y - 0.5, 1.5);
    let end = Vec3::new(cx + SPAN_HALF_X, NODE_Y + 0.5, 1.5);
    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(
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

    entities::spawn_node_pair(commands, node_mesh, node_mat, start, end);
    entities::spawn_cable(
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
        .observe(input::on_mesh_clicked);
}

/// Section 7: Inside view — large tube rendered inside-only.
fn setup_section_inside_view(commands: &mut Commands, cable_mat: &Handle<StandardMaterial>) {
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
