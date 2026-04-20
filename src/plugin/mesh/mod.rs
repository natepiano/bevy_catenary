//! Procedural tube mesh generation from `CableGeometry`.

mod buffers;
mod caps;
mod config;
mod elbows;
mod frames;
mod path;
mod tube;

pub use config::CableMeshConfig;
pub use config::CapConfig;
pub use config::Capping;
pub use config::ElbowConfig;
pub use config::FaceSides;
pub use config::TrimConfig;
pub use config::TubeConfig;
pub use elbows::ElbowMetadata;
pub use elbows::compute_elbow_metadata;
pub use tube::generate_tube_mesh;
