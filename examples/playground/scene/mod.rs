//! Camera, ground, directional light, and all 7 section scene setups.

mod astar;
mod cap_styles;
mod catenary;
mod entity_attachment;
mod inside_view;
mod scaffold;
mod shared_hub;
mod solver_comparison;

pub(super) use scaffold::RadiusMultiplier;
pub(super) use scaffold::SceneEntities;
pub(super) use scaffold::SharedCableMaterial;
pub(super) use scaffold::setup_camera;
pub(super) use scaffold::setup_scene;
pub(super) use scaffold::setup_sections;
