#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    reason = "tests should panic on unexpected values"
)]
//! Integration tests for entity-based cable endpoints.
//!
//! Uses `MinimalPlugins` for headless testing — no window, no renderer.

use bevy::app::App;
use bevy::math::Vec3;
use bevy::prelude::*;
use bevy_catenary::AttachedTo;
use bevy_catenary::Cable;
use bevy_catenary::CableEnd;
use bevy_catenary::CableEndpoint;
use bevy_catenary::CatenaryPlugin;
use bevy_catenary::CatenarySolver;
use bevy_catenary::ComputedCableGeometry;
use bevy_catenary::DetachPolicy;
use bevy_catenary::Solver;

/// Spawn a world-attached cable and return the cable entity.
fn spawn_world_cable(app: &mut App, start: Vec3, end: Vec3) -> Entity {
    let cable = app
        .world_mut()
        .spawn(Cable {
            solver:     Solver::Catenary(CatenarySolver::new().with_slack(1.2)),
            obstacles:  vec![],
            resolution: 0,
        })
        .id();

    let start_ep = app
        .world_mut()
        .spawn((CableEndpoint::new(CableEnd::Start, start), ChildOf(cable)))
        .id();

    let end_ep = app
        .world_mut()
        .spawn((CableEndpoint::new(CableEnd::End, end), ChildOf(cable)))
        .id();

    let _ = (start_ep, end_ep);
    cable
}

fn build_test_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(bevy::asset::AssetPlugin::default());
    app.add_plugins(bevy::transform::TransformPlugin);
    app.add_plugins(bevy::gizmos::GizmoPlugin);
    app.add_plugins(CatenaryPlugin);
    app
}

#[test]
fn world_attached_cable_computes_geometry() {
    let mut app = build_test_app();
    let cable = spawn_world_cable(
        &mut app,
        Vec3::new(-3.0, 2.0, 0.0),
        Vec3::new(3.0, 2.0, 0.0),
    );

    app.update();

    let computed = app.world().get::<ComputedCableGeometry>(cable).unwrap();
    assert!(
        computed.geometry.is_some(),
        "Cable should have computed geometry after one update"
    );

    let geometry = computed.geometry.as_ref().unwrap();
    assert!(
        !geometry.segments.is_empty(),
        "Geometry should have at least one segment"
    );
    assert!(
        geometry.total_length > 0.0,
        "Cable should have positive length"
    );
}

#[test]
fn entity_attached_cable_follows_target() {
    let mut app = build_test_app();

    // Spawn a target entity with a transform
    let target = app
        .world_mut()
        .spawn(Transform::from_translation(Vec3::new(5.0, 0.0, 0.0)))
        .id();

    // Spawn cable with one entity-attached endpoint
    let cable = app
        .world_mut()
        .spawn(Cable {
            solver:     Solver::Catenary(CatenarySolver::new().with_slack(1.2)),
            obstacles:  vec![],
            resolution: 0,
        })
        .id();

    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::Start, Vec3::new(0.5, 0.0, 0.0)),
        AttachedTo(target),
        ChildOf(cable),
    ));
    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::End, Vec3::new(-3.0, 2.0, 0.0)),
        ChildOf(cable),
    ));

    // First update: compute initial geometry
    app.update();

    let computed = app.world().get::<ComputedCableGeometry>(cable).unwrap();
    assert!(
        computed.geometry.is_some(),
        "Entity-attached cable should compute geometry"
    );

    // Move the target
    app.world_mut()
        .get_mut::<Transform>(target)
        .unwrap()
        .translation = Vec3::new(10.0, 0.0, 0.0);

    // Update to propagate transform and recompute
    app.update();
    // Second update may be needed for transform propagation
    app.update();

    let computed = app.world().get::<ComputedCableGeometry>(cable).unwrap();
    let geometry = computed.geometry.as_ref().unwrap();

    // The cable should have been recomputed with the new target position.
    // Start point should be near (10.0 + 0.5, 0.0, 0.0) = (10.5, 0, 0)
    let first_point = geometry.segments[0].points[0];
    assert!(
        (first_point.x - 10.5).abs() < 1.0,
        "Start point should follow moved target, got {first_point}"
    );
}

#[test]
fn zero_length_cable_does_not_panic() {
    let mut app = build_test_app();

    let cable = app
        .world_mut()
        .spawn(Cable {
            solver:     Solver::Catenary(CatenarySolver::new().with_slack(1.2)),
            obstacles:  vec![],
            resolution: 0,
        })
        .id();

    // Both endpoints at the same position
    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::Start, Vec3::ZERO),
        ChildOf(cable),
    ));
    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::End, Vec3::ZERO),
        ChildOf(cable),
    ));

    // Should not panic
    app.update();

    let computed = app.world().get::<ComputedCableGeometry>(cable).unwrap();
    assert!(
        computed.geometry.is_none(),
        "Zero-length cable should skip computation"
    );
}

#[test]
fn missing_target_does_not_panic() {
    let mut app = build_test_app();

    // Spawn a real target, then despawn it before the cable computes
    let target = app
        .world_mut()
        .spawn(Transform::from_translation(Vec3::new(5.0, 0.0, 0.0)))
        .id();

    let cable = app
        .world_mut()
        .spawn(Cable {
            solver:     Solver::Catenary(CatenarySolver::new().with_slack(1.2)),
            obstacles:  vec![],
            resolution: 0,
        })
        .id();

    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::Start, Vec3::new(1.0, 0.0, 0.0)),
        AttachedTo(target),
        ChildOf(cable),
    ));
    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::End, Vec3::new(-1.0, 0.0, 0.0)),
        ChildOf(cable),
    ));

    // Despawn the target before the first update
    app.world_mut().despawn(target);

    // Should not panic — falls back to raw offset
    app.update();

    let computed = app.world().get::<ComputedCableGeometry>(cable).unwrap();
    assert!(
        computed.geometry.is_some(),
        "Cable with despawned target should still compute (using fallback offset)"
    );
}

#[test]
fn detach_policy_despawn_removes_cable() {
    let mut app = build_test_app();

    let target = app
        .world_mut()
        .spawn(Transform::from_translation(Vec3::new(5.0, 0.0, 0.0)))
        .id();

    let cable = app
        .world_mut()
        .spawn(Cable {
            solver:     Solver::Catenary(CatenarySolver::new().with_slack(1.2)),
            obstacles:  vec![],
            resolution: 0,
        })
        .id();

    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::Start, Vec3::new(0.5, 0.0, 0.0))
            .with_detach_policy(DetachPolicy::Despawn),
        AttachedTo(target),
        ChildOf(cable),
    ));
    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::End, Vec3::new(-3.0, 2.0, 0.0)),
        ChildOf(cable),
    ));

    // Initial update
    app.update();
    assert!(
        app.world().get_entity(cable).is_ok(),
        "Cable should exist before target despawn"
    );

    // Despawn the target
    app.world_mut().despawn(target);

    // Update to trigger the OnRemove<AttachedTo> observer
    app.update();

    assert!(
        app.world().get_entity(cable).is_err(),
        "Cable should be despawned after target despawn with DetachPolicy::Despawn"
    );
}

#[test]
fn detach_policy_hang_in_place_keeps_cable() {
    let mut app = build_test_app();

    let target = app
        .world_mut()
        .spawn(Transform::from_translation(Vec3::new(5.0, 0.0, 0.0)))
        .id();

    let cable = app
        .world_mut()
        .spawn(Cable {
            solver:     Solver::Catenary(CatenarySolver::new().with_slack(1.2)),
            obstacles:  vec![],
            resolution: 0,
        })
        .id();

    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::Start, Vec3::new(0.5, 0.0, 0.0)),
        AttachedTo(target),
        ChildOf(cable),
    ));
    app.world_mut().spawn((
        CableEndpoint::new(CableEnd::End, Vec3::new(-3.0, 2.0, 0.0)),
        ChildOf(cable),
    ));

    // Initial update
    app.update();

    // Despawn the target
    app.world_mut().despawn(target);
    app.update();

    // Cable should still exist (HangInPlace is the default)
    assert!(
        app.world().get_entity(cable).is_ok(),
        "Cable should survive target despawn with DetachPolicy::HangInPlace"
    );

    let computed = app.world().get::<ComputedCableGeometry>(cable).unwrap();
    assert!(
        computed.geometry.is_some(),
        "Cable should retain its last computed geometry"
    );
}
