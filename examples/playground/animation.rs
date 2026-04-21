//! Point-light animation along tube center axes.

use bevy::prelude::*;

/// Animates a point light along a tube's center axis, oscillating front to back.
#[derive(Component)]
pub(crate) struct TubeLight {
    pub(crate) start: Vec3,
    pub(crate) end:   Vec3,
}

/// Tracks play/pause state for tube light animation.
#[derive(Resource, Default)]
pub(crate) enum LightAnimation {
    #[default]
    Running,
    Paused {
        frozen_at: f32,
    },
}

#[derive(Clone, Copy)]
enum LightTravelPhase {
    HoldStart,
    MoveForward,
    HoldEnd,
    MoveBackward,
}

impl LightTravelPhase {
    fn sample_position(self, phase: f32) -> f32 {
        match self {
            Self::HoldStart => 0.0,
            Self::MoveForward => (phase - 0.5) / 2.0,
            Self::HoldEnd => 1.0,
            Self::MoveBackward => 1.0 - (phase - 3.0) / 2.0,
        }
    }
}

impl From<f32> for LightTravelPhase {
    fn from(phase: f32) -> Self {
        match phase {
            phase if phase < 0.5 => Self::HoldStart,
            phase if phase < 2.5 => Self::MoveForward,
            phase if phase < 3.0 => Self::HoldEnd,
            _ => Self::MoveBackward,
        }
    }
}

/// Oscillates a point light along a tube's center axis.
/// 2s travel each direction, 0.5s pause at each end. Esc toggles pause.
pub(crate) fn animate_tube_light(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut animation: ResMut<LightAnimation>,
    mut lights: Query<(&TubeLight, &mut Transform)>,
) {
    if keyboard.just_pressed(KeyCode::Escape) {
        *animation = match *animation {
            LightAnimation::Running => LightAnimation::Paused {
                frozen_at: time.elapsed_secs(),
            },
            LightAnimation::Paused { .. } => LightAnimation::Running,
        };
    }

    let elapsed = match *animation {
        LightAnimation::Paused { frozen_at } => frozen_at,
        LightAnimation::Running => time.elapsed_secs(),
    };

    // 0.5s pause | 2s travel forward | 0.5s pause | 2s travel back = 5s cycle
    let cycle = 5.0_f32;
    let phase = elapsed % cycle;

    let light_phase = LightTravelPhase::from(phase);
    let t = light_phase.sample_position(phase);

    for (tube, mut transform) in &mut lights {
        transform.translation = tube.start.lerp(tube.end, t);
    }
}
