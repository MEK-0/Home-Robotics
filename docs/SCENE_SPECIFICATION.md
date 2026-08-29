# Home Robotics — Scene Specification

## 1. Purpose

This document defines the physical and semantic specification of the Home Robotics simulation scene.

Its purpose is to ensure that:

- the world layout is fixed early,
- robot placement remains stable,
- work surfaces are explicitly defined,
- objects have known semantic roles,
- collision geometry is predictable,
- later planning code does not invent coordinates,
- and Codex has a clear spatial contract before implementation begins.

This document defines the scene conceptually.

Exact numeric poses belong in:

```text
config/scene.yaml
config/robots.yaml
config/objects.yaml
config/locations.yaml
```

The scene specification and configuration files must remain consistent.

---

## 2. Scene Goal

The scene represents a structured home / kitchen environment designed for tabletop manipulation.

The final environment should visually resemble a compact domestic manipulation workspace containing:

- two Franka Panda robot arms,
- multiple work surfaces,
- household / kitchen objects,
- target containers,
- static furniture,
- and visually rich but computationally efficient environment assets.

The scene must support both:

```text
portfolio-quality visual presentation
```

and:

```text
stable robotic manipulation research
```

The visual appearance should not compromise physics stability.

---

## 3. Final-Layout-First Principle

The project follows a final-layout-first scene strategy.

This means that the major scene layout created during Phase 1 is intended to remain stable for the rest of the project.

The following should not be casually changed after scene validation:

```text
world origin
robot mounting positions
main counters
main tables
major work surfaces
container regions
shared manipulation zones
workspace spacing
robot-to-table relationships
```

Visual details may improve later, but major geometry should not shift.

The objective is to avoid breaking:

- reachability assumptions,
- motion-planning tests,
- benchmark results,
- grasp offsets,
- collision tests,
- and coordinate mappings.

---

## 4. Coordinate Convention

The global coordinate system is:

```text
X → forward
Y → left
Z → up
```

Root frame:

```text
world
```

All scene measurements use SI units.

```text
1 simulation distance unit = 1 meter
```

All major scene assets must be positioned relative to the `world` frame.

---

## 5. Scene Zones

The environment should be divided into semantic spatial zones.

Recommended high-level zoning:

```text
HOME ROBOTICS SCENE

┌─────────────────────────────────────────────┐
│                LEFT SIDE                    │
│                                             │
│   Work Surface A   Work Surface B   A3      │
│                                             │
├────────────── ROBOT CORRIDOR ───────────────┤
│                                             │
│        Panda 1        Panda 2               │
│                                             │
├─────────────────────────────────────────────┤
│                                             │
│   Work Surface B1  Work Surface B2  B3      │
│                RIGHT SIDE                   │
└─────────────────────────────────────────────┘
```

This is a semantic layout, not an exact metric drawing.

The exact placement will be defined in `scene.yaml`.

---

## 6. Major Scene Components

The scene should contain the following major component classes:

### 6.1 Floor

The environment has a fixed floor plane.

Requirements:

- static,
- non-slip,
- no visible penetration,
- large enough to contain the complete scene,
- and visually suitable for a domestic / laboratory kitchen setting.

Decorative floor tiles may be used.

Collision should remain a simple plane or box.

---

### 6.2 Central Robot Mounting Area

The two Franka Panda robots are mounted in a central manipulation corridor.

The mounting area should:

- support both robots physically,
- keep their bases fixed,
- create realistic access to nearby work surfaces,
- and leave enough clearance for future dual-arm operation.

The robot bases must not be repositioned after Phase 1 validation without an explicit architectural change.

---

### 6.3 Work Surfaces

The scene contains multiple table / counter surfaces around the robots.

The visual reference contains three work sections on each side, producing approximately six distinct functional work surfaces.

The scene should therefore support semantic surface IDs such as:

```text
surface_left_1
surface_left_2
surface_left_3

surface_right_1
surface_right_2
surface_right_3
```

Alternative naming may be used if kept consistent.

Each surface must expose:

```text
pose
dimensions
top_height
collision_bounds
semantic role
workspace ownership
```

These values must be configuration-driven.

---

## 7. Work Surface Requirements

Each work surface should use:

```text
detailed visual geometry
+
simple collision geometry
```

Preferred collision representation:

```text
rectangular box
```

unless a more complex shape is physically necessary.

Tables must not use triangle-mesh collision by default.

Each surface should define a safe manipulation region smaller than its complete physical top.

Example concept:

```text
physical tabletop
┌────────────────────────────┐
│                            │
│   ┌────────────────────┐   │
│   │ safe object region │   │
│   └────────────────────┘   │
│                            │
└────────────────────────────┘
```

This prevents objects from spawning too close to edges.

---

## 8. Robot Configuration in Scene

The scene contains:

```text
panda1
panda2
```

Both use:

```text
Franka Panda
+
standard Franka Hand
```

### 8.1 Panda 1

Early phases:

```text
active = true
```

Panda 1 is the primary manipulation robot.

Its workspace should include several object and destination locations.

### 8.2 Panda 2

Early phases:

```text
active = false
```

Panda 2 remains physically present in the scene.

Its geometry remains relevant for:

- visual composition,
- future collision planning,
- workspace design,
- and later dual-arm activation.

Panda 2 must not be deleted simply because it is inactive.

---

## 9. Robot Base Placement Principles

Robot base poses must satisfy all of the following:

1. realistic mounting,
2. no initial collisions,
3. usable reach to intended work surfaces,
4. reasonable joint posture at home,
5. future dual-arm compatibility,
6. sufficient separation between robot bases,
7. access to a future shared workspace.

The final numeric poses will be validated empirically in MuJoCo before they are locked.

---

## 10. Robot Home State

Each Panda must define a named home configuration.

Example semantic names:

```text
panda1_home
panda2_home
```

The home pose should:

- avoid self-collision,
- avoid scene collision,
- keep the gripper clear of objects,
- keep the robot visually neutral,
- and provide a reproducible reset state.

The home configuration belongs in `robots.yaml`.

---

## 11. Initial Manipulation Objects

The initial supported object set is:

```text
apple
purple_ball
cube
bowl
pan
```

These objects are intentionally selected to represent different geometric and manipulation classes.

---

## 12. Apple

### Role

```text
pickable
```

Potential later role:

```text
placeable into bowl
```

### Geometry

The apple should visually resemble a small red apple.

Collision should use a simple primitive approximation, preferably:

```text
sphere
```

or a small combination of primitives if needed.

### Physical Properties

The apple should use:

- realistic scale,
- realistic but stable mass,
- moderate friction,
- and a center of mass near its geometric center.

### Manipulation Characteristics

Suitable grasp types may include:

```text
side grasp
top-biased side grasp
```

---

## 13. Purple Ball

### Role

```text
pickable
```

### Geometry

Visual:

```text
purple sphere
```

Collision:

```text
sphere
```

### Purpose

The ball is useful for testing:

- symmetric grasp generation,
- friction sensitivity,
- rolling behavior,
- and placement stability.

It should not spawn close to table edges.

---

## 14. Cube

### Role

```text
pickable
calibration object
benchmark object
```

### Geometry

Visual:

```text
box / cube
```

Collision:

```text
box
```

### Purpose

The cube is the primary low-complexity manipulation object.

It should be used before apple and ball tests because it provides predictable:

- contact surfaces,
- grasp normals,
- support geometry,
- and orientation.

---

## 15. Bowl

### Initial Role

```text
destination
container
```

### Geometry

The bowl should be visually recognizable.

Its collision model must be chosen carefully.

A simple fully solid cylinder is not sufficient if objects are expected to be placed inside it.

Preferred collision approaches include:

```text
multiple simple wall primitives
```

or:

```text
validated convex decomposition
```

The bowl interior must remain physically accessible.

### Placement Semantics

The bowl should define a named destination volume rather than only a single point.

For example:

```text
bowl_inner_region
```

Placement success should later be verified using object containment or support conditions.

---

## 16. Pan

### Initial Role

```text
environment object
destination candidate
```

### Future Role

```text
pickable by handle
```

### Geometry

Visual mesh may be detailed.

Collision should be simplified.

Suggested approximation:

```text
shallow cylinder for body
+
capsule / box for handle
```

The pan should not use unnecessarily complex mesh collision.

---

## 17. Additional Visual Assets

The final scene may contain:

- laptop,
- game controller,
- stove,
- sink,
- chairs,
- baskets,
- cabinets,
- decorative containers,
- and other domestic objects.

These assets are categorized as:

```text
decorative
static environment
future interactive
```

Each asset must be explicitly assigned a category.

A decorative object must not accidentally participate in physics unless required.

---

## 18. Scene Object Categories

Every scene entity should belong to one of the following categories:

### STATIC_STRUCTURE

Examples:

```text
floor
tables
counters
robot mounts
sink structure
stove structure
cabinets
```

### DECORATIVE_STATIC

Examples:

```text
chairs
laptop
controller
non-interactive containers
visual kitchen accessories
```

### MANIPULABLE

Examples:

```text
apple
purple_ball
cube
```

### CONTAINER

Examples:

```text
bowl
basket
bin
```

### FUTURE_MANIPULABLE

Examples:

```text
pan
cup
utensils
```

### ROBOT

Examples:

```text
panda1
panda2
```

---

## 19. Object Naming Rules

All object names must:

- use lowercase snake_case,
- be unique,
- be stable across runs,
- avoid spaces,
- and describe semantic identity rather than visual appearance only.

Good examples:

```text
apple
purple_ball
cube
bowl
pan
left_bin
kitchen_counter
```

Bad examples:

```text
Object001
redThing
mesh_4
ball-final-v2
```

---

## 20. Visual Geometry vs Collision Geometry

The project explicitly separates:

```text
visual representation
```

from:

```text
physical collision representation
```

### Visual Geometry

May include:

- detailed meshes,
- textures,
- rounded edges,
- realistic furniture,
- decorative detail.

### Collision Geometry

Should prefer:

- box,
- sphere,
- capsule,
- cylinder,
- convex simplified mesh.

### Rule

Collision geometry must be selected for stable physics, not visual accuracy.

---

## 21. Static vs Dynamic Bodies

Furniture should generally be static.

Examples:

```text
tables
floor
stove
sink
cabinets
robot mounts
```

Manipulation objects should be dynamic.

Examples:

```text
apple
purple_ball
cube
```

Containers may be static unless a later task requires moving them.

Examples:

```text
bowl → initially static
pan → initially static
```

This may be revised in later phases.

---

## 22. Object Spawn Rules

Objects must spawn only inside predefined safe regions.

Do not spawn objects directly:

- on table edges,
- intersecting other objects,
- inside collision geometry,
- under robot links,
- inside gripper fingers,
- or outside intended robot workspace.

Each spawn region should define:

```text
surface
x range
y range
z rule
orientation rule
minimum edge clearance
minimum inter-object clearance
```

---

## 23. Support-Surface Semantics

Objects should know which surface initially supports them.

Example:

```text
apple:
  support_surface: surface_left_2
```

This semantic relation is useful for:

- reset validation,
- scene-state reasoning,
- LLM descriptions,
- and placement verification.

---

## 24. Named Locations

Task-level destinations should be semantic.

Examples:

```text
bowl
left_bin
right_bin
surface_left_1_center
surface_right_2_drop_zone
handover_zone
```

Each location should define:

```text
reference frame
position or region
allowed orientation
clearance
supported object classes
```

Exact definitions belong in `locations.yaml`.

---

## 25. Placement Regions Instead of Single Points

Whenever possible, destinations should be represented as valid regions rather than exact points.

For example:

```text
bowl
```

should define:

```text
valid placement volume
```

rather than only:

```text
x, y, z
```

Likewise, a table drop location may define a rectangular placement area.

This provides:

- more robust planning,
- less brittle execution,
- better benchmarking,
- and future compatibility with optimization.

---

## 26. Safe Manipulation Regions

Each work surface should define a manipulation-safe region.

A safe region must consider:

- table edges,
- robot reach,
- neighboring objects,
- neighboring robot workspace,
- and gripper approach clearance.

Objects used in tests should begin inside these regions.

---

## 27. Panda 1 Primary Workspace

Panda 1 should initially own a defined workspace.

Conceptually:

```text
panda1_primary_workspace
```

This area should contain:

- initial pick objects,
- at least one container,
- one or more placement zones,
- sufficient approach clearance.

The exact boundaries belong in `scene.yaml`.

---

## 28. Panda 2 Primary Workspace

A future workspace should already be reserved:

```text
panda2_primary_workspace
```

Panda 2 does not use it actively during early phases, but the scene layout must preserve it.

---

## 29. Shared Workspace

The scene should reserve a physically reachable region for both robots.

Semantic name:

```text
shared_workspace
```

Possible future use:

- object handover,
- cooperative tasks,
- dual-arm manipulation,
- shared object access.

This region should not be heavily cluttered.

---

## 30. Handover Zone

A future semantic subregion may be defined:

```text
handover_zone
```

It should satisfy:

- reachable by both robots,
- safe from fixed obstacles,
- clear line of approach,
- and adequate spacing for two grippers.

It does not need active implementation in Phase 1.

---

## 31. Scene Integrity Principle

A manipulation task must preserve scene integrity.

The scene is not merely background geometry.

The system must detect or prevent:

```text
unrelated object fall
unintended object displacement
furniture penetration
robot collision
container knock-over
held-object collision
```

A successful target placement with unacceptable collateral disturbance is a failed task.

---

## 32. Initial Scene Stability

Before manipulation begins, the idle scene must remain stable.

Required conditions:

```text
no object drift
no spontaneous rolling
no object falling
no interpenetration
no robot motion
no constraint instability
```

The stability check should run for a defined simulated duration.

---

## 33. Deterministic Reset

Scene reset must restore:

- all robot joint configurations,
- gripper state,
- all dynamic object poses,
- all dynamic object velocities,
- all temporary constraints,
- and all task-related simulation state.

Reset must reproduce the same initial scene.

The initial validation target is:

```text
100 deterministic resets
```

with no invalid initial state.

---

## 34. Collision Groups

The implementation should organize collision categories clearly.

Conceptual groups may include:

```text
robot
robot_gripper
static_environment
manipulable_object
container
decorative_static
```

Collision filtering should only be introduced when physically justified.

Do not disable collisions globally to make manipulation easier.

---

## 35. Robot–Object Collision Expectations

Before grasp:

```text
gripper ↔ target object collision allowed
arm ↔ target object collision controlled / avoided
```

During grasp:

```text
finger ↔ target object contact required
```

During transport:

```text
held object ↔ environment collision avoided
```

Unrelated-object contact should generally be considered invalid.

---

## 36. Robot–Robot Collision Expectations

Early phases:

Panda 2 is inactive but remains part of the scene.

Panda 1 must avoid physically intersecting Panda 2.

Later dual-arm phases will add:

```text
cross-robot collision checking
shared planning
execution coordination
```

---

## 37. Gripper Clearance

Object placement and scene layout must preserve enough space for:

```text
finger opening
pre-grasp pose
approach path
wrist orientation
retreat path
```

An object that is geometrically reachable but impossible to approach safely is not considered a valid manipulation location.

---

## 38. Scene Mapping Contract

Every functional object or region must have a stable semantic ID.

The mapping should conceptually support:

```text
world
│
├── robots
│   ├── panda1
│   └── panda2
│
├── structures
│   ├── floor
│   ├── surface_left_1
│   ├── surface_left_2
│   ├── surface_left_3
│   ├── surface_right_1
│   ├── surface_right_2
│   └── surface_right_3
│
├── objects
│   ├── apple
│   ├── purple_ball
│   ├── cube
│   ├── bowl
│   └── pan
│
└── semantic_regions
    ├── panda1_primary_workspace
    ├── panda2_primary_workspace
    ├── shared_workspace
    └── handover_zone
```

Exact numeric mapping is defined separately in `SCENE_MAP.md` and configuration files.

---

## 39. Scene Construction Order

The implementation should construct and validate the scene in this order:

```text
1. world + floor
2. major static structures
3. robot mounts
4. Panda 1
5. Panda 2
6. work surfaces
7. container objects
8. manipulation objects
9. decorative assets
10. visual polish
```

Each stage should be validated before the next stage is added.

---

## 40. Scene Validation Sequence

Recommended validation:

### Stage A — Geometry

Check:

```text
asset loading
scale
pose
orientation
visual alignment
```

### Stage B — Static Collision

Check:

```text
floor
tables
mounts
furniture
```

### Stage C — Robot Initial State

Check:

```text
joint state
self-collision
environment collision
reachability
```

### Stage D — Dynamic Objects

Check:

```text
gravity
friction
mass
support
stability
```

### Stage E — Reset

Check:

```text
repeatability
velocity clearing
constraint clearing
```

---

## 41. Performance Constraints

The scene must be designed for the available VM.

Target development environment:

```text
4 CPU cores
8 GB RAM
ARM64
no NVIDIA GPU
```

Therefore:

- unnecessary high-poly collision meshes are prohibited,
- decorative assets should not create excessive body counts,
- physics timestep should be selected carefully,
- dynamic bodies should remain limited,
- and rendering should not dominate development runtime.

Visual quality should be optimized mainly for demonstration runs.

---

## 42. Asset Policy

Assets may come from:

- original project meshes,
- permissively licensed public assets,
- MuJoCo / robotics model repositories,
- or custom primitive geometry.

All third-party assets must have their source and license documented.

Portfolio-quality visual assets should not introduce unclear licensing.

---

## 43. Scene Versioning

Once the scene passes Phase 1 acceptance, a baseline scene version should be recorded.

Example:

```text
scene_version: 1.0
```

Subsequent changes should distinguish:

```text
visual-only change
physics change
layout change
semantic change
```

Layout changes should be rare and explicitly documented.

---

## 44. Scene Acceptance Criteria

Phase 1 scene acceptance requires:

- both Panda robots are present,
- Panda 1 is active-ready,
- Panda 2 is correctly mounted,
- no initial robot collisions exist,
- all major work surfaces exist,
- initial objects exist,
- object scales are realistic,
- object masses are plausible,
- collision geometry is valid,
- no object spawns in penetration,
- idle physics remains stable,
- named semantic regions are defined,
- all functional entities have stable IDs,
- deterministic reset works,
- and the scene matches the intended visual composition.

---

## 45. Phase 1 Exit Test

Before moving to robot-control development, the complete scene should pass:

```text
100 reset cycles
```

and an idle stability test after each reset.

Target:

```text
0 unintended object falls
0 invalid object poses
0 robot self-collisions
0 robot-environment collisions
0 stale constraints
0 initial penetration
0 unexplained dynamic drift
```

---

## 46. Rules for Codex

When Codex implements the scene:

1. Do not invent object coordinates outside configuration.
2. Do not move robot bases to solve a local grasp problem.
3. Do not change table dimensions without updating scene documentation.
4. Do not use visual meshes as collision meshes by default.
5. Do not disable collision checks to hide geometry problems.
6. Do not add unregistered scene entities.
7. Do not hard-code support-surface heights in manipulation code.
8. Do not make Panda 2 active before its assigned phase.
9. Do not change the global frame convention.
10. Do not alter scene scale.
11. Do not place test objects near table edges.
12. Do not introduce decorative dynamic bodies without need.
13. Keep all functional entity names stable.
14. Preserve deterministic reset behavior.
15. Update `SCENE_MAP.md` and config files when functional scene geometry changes.

---

## 47. Relationship to Other Documents

This document defines what the scene must contain and how it should behave.

Related documents:

```text
SCENE_MAP.md
```

defines exact semantic mapping and spatial organization.

```text
COORDINATE_FRAMES.md
```

defines all frame relationships.

```text
OBJECT_REGISTRY.md
```

defines object metadata and roles.

```text
MUJOCO_PHYSICS_SPEC.md
```

defines physics and solver conventions.

```text
ROBOT_AND_GRIPPER_SPEC.md
```

defines Panda and Franka Hand behavior.

```text
PICK_PLACE_SPEC.md
```

defines manipulation behavior within this scene.

---

## 48. Final Scene Principle

The scene must behave like a real robotic workspace, not a stage prop.

It should be:

```text
visually coherent
physically stable
semantically mapped
configuration-driven
collision-safe
repeatable
research-ready
```

The scene is the physical foundation of the entire Home Robotics system.

If the scene is unstable or ambiguously defined, all later manipulation, planning, and LLM behavior becomes unreliable.

For that reason, Phase 1 scene quality is treated as a core engineering deliverable rather than a cosmetic task.
