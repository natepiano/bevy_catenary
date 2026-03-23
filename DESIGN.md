# bevy_catenary

A Bevy library for 3D cables with catenary geometry and automatic routing.

## Core Concept

Cables hang naturally between two anchor points using catenary curve math (`y = a * cosh(x/a)`). When objects sit between endpoints, the cable routes around them via waypoints — each segment between waypoints is itself a catenary.

## Implemented Features

### Geometry
- Catenary curve math for natural hanging between two anchor points
- Procedural tube mesh generation along the cable path
- Variable tension/slack control per cable
- Configurable cap styles (round, flat, none) per endpoint
- Double-sided rendering (outside, inside, or both)
- Elbow fillet smoothing at sharp bends with configurable Bézier curves
- Trim start/end for junction hiding

### Routing & Layout
- Automatic pathfinding through 3D space around intervening objects (A*)
- Orthogonal routing (axis-aligned 90-degree bends)
- Composite routing: any planner + any curve solver
- Configurable anchor positions on connected objects
- 3D-aware routing (not constrained to 2D)

### Connection System
- Entity-based endpoints (`CableEndpoint` + `AttachedTo`)
- Automatic position tracking via `GlobalTransform`
- Detach policies (hang in place, despawn) when attached entities are removed
- Observer-driven mesh pipeline — no manual mesh management

## Future Possibilities

These are not currently planned but could be built on top of the existing architecture:

- Physics: Verlet integration for dynamic response, rigid-body coupling
- LOD: different visual detail at different distances
- Running light effects along cables
- Drag-to-connect interaction model

## Out of Scope

These are application-layer concerns, not the cable library's job:

- Data visualization on cables (oscilloscope, MIDI, video)
- Signal manipulation (attenuation, inversion)
- Typed data transport and conversion
- Domain-specific port/node logic
