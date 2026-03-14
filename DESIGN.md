# bevy_catenary

A Bevy library for physics-based 3D cables with catenary geometry and automatic routing.

## Core Concept

Cables hang naturally between two anchor points using catenary curve math (`y = a * cosh(x/a)`). When objects sit between endpoints, the cable routes around them via waypoints — each segment between waypoints is itself a catenary.

## Feature Set

### Geometry
- Catenary curve math for natural hanging between two anchor points
- Procedural tube mesh generation along the cable path
- Variable tension/slack control per cable
- Transparency/opacity support

### Physics
- Gravity simulation — cables hang naturally
- Verlet integration for dynamic response to movement
- Extension/contraction when endpoints move
- Optional rigid-body coupling — pulling a cable can pull attached objects

### Routing & Layout
- Automatic pathfinding through 3D space around intervening objects
- Routed paths are catenary segments between waypoints
- Configurable anchor positions on connected objects (side, top, etc.)
- 3D-aware routing (not constrained to 2D)

### Rendering
- LOD support — different visual detail at different distances
- Pluggable visual styles/materials
- Running light effect — animated flow direction indicator along the cable

### Connection System
- Connect/disconnect two endpoints
- Drag-to-connect interaction model
- Visual feedback during connection (valid/invalid target highlighting)

## Out of Scope

These are application-layer concerns, not the cable library's job:

- Data visualization on cables (oscilloscope, MIDI, video)
- Signal manipulation (attenuation, inversion)
- Typed data transport and conversion
- Polyphonic support
- Modulation routing models
- Domain-specific port/node logic

## Research References

- Catenary math: Alan Zucconi's catenary series
- Verlet integration: `bevy_verlet` crate
- Procedural mesh: `bevy-procedural/meshes`
- Point meshes: `bevy_points` crate
- Physics: avian3d
