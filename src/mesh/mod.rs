//! Procedural tube mesh generation from `CableGeometry`.

mod buffers;
mod caps;
#[allow(
    clippy::used_underscore_binding,
    reason = "false positive on Reflect derive for Capping::Flat { normal }"
)]
mod config;
mod constants;
mod elbows;
mod frames;
mod handle;
mod path;
mod tube;

use bevy::prelude::*;
pub use config::CableMeshConfig;
pub use config::CapConfig;
pub use config::Capping;
pub use config::ElbowConfig;
pub use config::FaceSides;
pub use config::TrimConfig;
pub use config::TubeConfig;
pub use elbows::ElbowMetadata;
pub use elbows::compute_elbow_metadata;
pub use handle::CableMeshChild;
pub use handle::CableMeshHandle;
pub use tube::generate_tube_mesh;

pub(crate) struct MeshPlugin;

impl Plugin for MeshPlugin {
    fn build(&self, app: &mut App) { app.add_observer(handle::on_geometry_computed); }
}
