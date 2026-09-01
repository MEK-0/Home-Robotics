# Home Robotics — Scene Map

## 1. Purpose

This document is the authoritative semantic map of the Home Robotics simulation world.

It defines:

- the world coordinate convention,
- the major spatial zones,
- rail placement and robot carriage logic,
- work-surface identities,
- object identities,
- destination regions,
- workspace ownership,
- shared-space rules,
- scene invariants,
- spatial relationships,
- and the contract that implementation code must follow.

This document is intentionally stricter than a normal scene description.

The goal is to prevent later implementation from introducing:

- duplicated coordinates,
- ambiguous object names,
- inconsistent frame usage,
- ad-hoc offsets,
- accidental workspace overlap,
- or scene changes made only to make one manipulation test pass.

The exact numeric values are stored in configuration files.

The semantic meaning of those values is defined here.

---

# 2. Source-of-Truth Hierarchy

The scene has one authoritative hierarchy.

```text
SCENE_MAP.md
    ↓
defines semantic meaning

config/scene.yaml
config/robots.yaml
config/objects.yaml
config/locations.yaml
    ↓
define numeric values

MuJoCo scene generation
    ↓
creates physical world

Scene State Provider
    ↓
exposes runtime state

MoveIt Planning Scene
    ↓
mirrors planning-relevant geometry
```

No application code may invent a second scene map.

The following are prohibited:

```text
hard-coded table poses in C++ / Python
hard-coded apple pose in manipulation code
separate MoveIt-only table coordinates
separate LLM-only location coordinates
duplicated fixed rail transforms
```

If a coordinate changes, the configuration must change first.

---

# 3. Coordinate System

The world frame is:

```text
world
```

Axis convention:

```text
X → forward
Y → left
Z → up
```

All positions use meters.

All orientations must use one explicitly documented convention in implementation.

Preferred representation for configuration:

```text
position: [x, y, z]
orientation_rpy: [roll, pitch, yaw]
```

or:

```text
position: [x, y, z]
orientation_quat: [x, y, z, w]
```

A single representation should be selected per configuration schema and used consistently.

---

# 4. World Origin

The world origin is a permanent project-level reference.

The origin must not be moved later to make coordinates easier.

Recommended interpretation:

```text
world origin
=
center of the central robot mounting / interaction area projected onto floor level
```

This gives the complete scene a stable geometric center.

The exact scene implementation may refine the mechanical origin, but once Phase 1 is accepted it becomes immutable.

---

# 5. Scene Orientation

The environment is organized around a central robot corridor.

Conceptually:

```text
                           +X FORWARD

                  FAR / FORWARD SIDE
                         ↑
                         │

        +Y LEFT          │          -Y RIGHT

  ┌─────────────────────────────────────────────┐
  │                                             │
  │  LEFT WORK SURFACES      RIGHT WORK SURFACES│
  │                                             │
  │     L3     L2     L1    R1     R2     R3   │
  │                                             │
  ├─────────────────────────────────────────────┤
  │                                             │
  │          PANDA 1        PANDA 2             │
  │                                             │
  │         CENTRAL ROBOT CORRIDOR              │
  │                                             │
  └─────────────────────────────────────────────┘

                         │
                         ↓
                    -X / REAR
```

This diagram is semantic.

Final exact geometry belongs in `config/scene.yaml`.

---

# 6. Canonical Scene IDs

The following IDs are canonical and should remain stable.

## 6.1 Robots and Linear Rails

```text
shared_rail
panda1
panda1_carriage
panda2
panda2_carriage
```

## 6.2 Work Surfaces

```text
surface_left_1
surface_left_2
surface_left_3

surface_right_1
surface_right_2
surface_right_3
```

## 6.3 Initial Objects

```text
apple
purple_ball
cube
bowl
pan
```

## 6.4 Semantic Regions

```text
panda1_primary_workspace
panda2_primary_workspace
shared_workspace
handover_zone
```

## 6.5 Optional Container / Drop Regions

Reserved names:

```text
left_bin
right_bin
left_basket
right_basket
```

These should only become active if the matching physical assets exist.

---

# 7. Stable Naming Contract

Every functional entity must have exactly one canonical name.

Aliases may exist only at the LLM / language layer.

Example:

```text
canonical:
purple_ball

possible language aliases:
"purple ball"
"ball"
"purple sphere"
```

The simulator, registry, planner, and Task Executor must use:

```text
purple_ball
```

not the natural-language alias.

---

# 8. Major Static Geometry

The scene is divided into the following static geometry groups.

```text
floor
central_mount
surface_left_1
surface_left_2
surface_left_3
surface_right_1
surface_right_2
surface_right_3
fixed_kitchen_assets
decorative_static_assets
```

Every static structure must be registered in `scene.yaml`.

---

# 9. Work-Surface Mapping

Each work surface is a first-class semantic entity.

A work surface must define:

```text
id
pose
dimensions
top_height
safe_region
workspace_owner
collision_geometry
visual_asset
semantic_role
```

The surface ID is not merely a visual label.

It is used by:

- object spawning,
- placement verification,
- scene-state summaries,
- workspace reasoning,
- and later LLM task interpretation.

---

# 10. Work-Surface Numbering

Numbering is defined relative to the central robot corridor.

For each side:

```text
1 → nearest primary robot interaction zone
2 → middle section
3 → farthest / outer section
```

Therefore:

```text
surface_left_1
```

and:

```text
surface_right_1
```

are the central / primary interaction surfaces on their respective sides.

This convention must not be reversed later.

---

# 11. Surface Geometry Contract

Every surface must distinguish:

```text
visual_bounds
physical_collision_bounds
usable_top_region
safe_spawn_region
safe_place_region
```

These are not necessarily identical.

Example:

```text
table top physical area
┌─────────────────────────────────┐
│                                 │
│   safe manipulation region      │
│   ┌─────────────────────────┐   │
│   │                         │   │
│   │                         │   │
│   └─────────────────────────┘   │
│                                 │
└─────────────────────────────────┘
```

The safe region must exclude an edge margin.

---

# 12. Surface Edge Clearance

Objects must never be intentionally spawned directly at the physical edge of a work surface.

Each surface must define:

```text
edge_clearance
```

The initial recommended design value should be conservative.

Exact value must be tuned in configuration.

The important invariant is:

```text
safe_spawn_region ⊂ physical_surface_region
```

with non-zero margin on every exposed edge.

---

# 13. Panda 1 Mapping

Canonical ID:

```text
panda1
```

Primary role:

```text
initial active manipulation robot
```

Required scene metadata:

```text
base_pose
home_joint_configuration
controller_active
primary_workspace
reachable_surfaces
shared_workspace_access
```

Initial active state:

```text
active: true
```

---

# 14. Panda 2 Mapping

Canonical ID:

```text
panda2
```

Primary role during early phases:

```text
physically present
controller inactive
future manipulation robot
```

Required scene metadata:

```text
base_pose
home_joint_configuration
controller_active
primary_workspace
reachable_surfaces
shared_workspace_access
```

Initial active state:

```text
active: false
```

Panda 2 must not be omitted from the world.

---

# 15. Linear Rail Mapping

Both Pandas are mounted on independent carriages on one shared fixed rail.

Canonical entities:

```text
shared_rail
panda1_rail_joint
panda1_carriage
panda2_rail_joint
panda2_carriage
```

Nominal rail axis:

```text
world X
```

Purpose:

```text
near table access
↕
middle table access
↕
far table access
```

The rail is a controlled prismatic DOF.

It must be included in:

- MuJoCo state,
- ROS joint states,
- ros2_control,
- TF,
- MoveIt planning groups,
- collision checking,
- reset state,
- and benchmark logs.

The robot base world transform is derived from the carriage state:

```text
T_world_base
=
T_world_rail
×
T_rail_carriage(q_rail)
×
T_carriage_base
```

The rail assembly itself is fixed after Phase 1 scene lock.

The carriage is mobile.

Directly teleporting the Panda base is prohibited.

The shared rail and carriage configurations must define:

```text
axis
lower_limit
upper_limit
home_position
max_velocity
max_acceleration
carriage_collision_geometry
```

---

# 16. Robot Base Pose Invariant

Fixed rail assembly poses are scene-level constants. Panda base world pose is dynamic because it is derived from the rail carriage position.

Once Phase 1 scene acceptance is complete:

```text
panda1 rail pose
panda2 rail pose
rail travel limits
```

must not be changed to solve:

- a difficult grasp,
- a difficult trajectory,
- a bad object placement,
- or a reachability bug.

If reachability is invalid, the scene configuration must be reviewed as a deliberate scene revision.

---

# 16. Robot Facing Direction

Each robot must have an explicitly defined nominal facing direction.

The base yaw must be documented in `robots.yaml`.

No implementation should infer facing direction from mesh appearance.

The semantic forward direction for each robot must be derivable from TF.

---

# 17. Robot Workspace Ownership

The world is partitioned into logical workspaces.

## 17.1 Panda 1 Primary Workspace

```text
panda1_primary_workspace
```

This is the preferred operating area for Panda 1.

Initial manipulation objects should be placed here.

## 17.2 Panda 2 Primary Workspace

```text
panda2_primary_workspace
```

Reserved for later dual-arm phases.

## 17.3 Shared Workspace

```text
shared_workspace
```

Reachable by both robots.

Used later for:

- shared object access,
- cooperative tasks,
- handovers.

---

# 18. Workspace Ownership Is Logical, Not a Collision Wall

Workspace regions do not create invisible physical barriers.

They are semantic planning constraints.

A robot may later enter another region only when:

- the task explicitly requires it,
- collision conditions are satisfied,
- ownership / resource rules permit it.

---

# 19. Shared Workspace Design

The shared workspace must remain relatively uncluttered.

It should not contain large permanent decorative objects that make later dual-arm operation impossible.

The shared area should provide:

```text
clear approach volume
sufficient vertical clearance
reasonable reach for both arms
collision-safe handover geometry
```

---

# 20. Handover Zone

Canonical ID:

```text
handover_zone
```

This is a subregion of:

```text
shared_workspace
```

The handover zone should be:

- reachable by both Panda arms,
- clear of table edges,
- clear of major furniture,
- sufficiently above the support surface,
- and suitable for gripper-to-gripper object transfer.

It remains inactive in early phases.

---

# 21. Initial Object Map

Initial canonical objects:

| ID | Initial Class | Dynamic | Initial Function |
|---|---|---:|---|
| `cube` | benchmark object | yes | first pick/place validation |
| `apple` | household object | yes | semantic pick/place |
| `purple_ball` | spherical object | yes | contact/friction test |
| `bowl` | container | initially no | place destination |
| `pan` | kitchen object | initially no | environment / future manipulation |

---

# 22. Initial Development Priority

Although all objects exist in the final scene, manipulation development should proceed in this order:

```text
cube
 ↓
apple
 ↓
purple_ball
 ↓
bowl placement
 ↓
pan-related interactions
```

Reason:

```text
cube
=
most predictable collision and grasp geometry
```

The project must not begin reliability tuning with the ball.

---

# 23. Object Placement Philosophy

Initial object positions must be chosen for:

```text
safe reachability
clear grasp approach
adequate inter-object spacing
realistic scene appearance
collision-free reset
```

They must not be chosen merely to create an impressive screenshot.

---

# 24. Object Minimum Separation

The scene configuration should enforce a minimum separation between dynamic objects at reset.

This protects against:

- accidental object-object contacts,
- unstable initial dynamics,
- ambiguous grasp approach,
- and unrelated-object disturbance.

The exact numerical threshold belongs in `scene.yaml` or `objects.yaml`.

---

# 25. Cube Map

Canonical ID:

```text
cube
```

Recommended initial role:

```text
primary Phase 4 benchmark pick object
```

Preferred initial support:

```text
panda1_primary_workspace
```

Preferred surface:

```text
surface_left_1
```

or whichever primary Panda 1 surface is validated as best reachable.

The final exact surface assignment is locked in configuration after reachability testing.

---

# 26. Apple Map

Canonical ID:

```text
apple
```

Role:

```text
pickable household object
```

Preferred initial placement:

- on a Panda 1 reachable work surface,
- separated from cube,
- not near an edge,
- with vertical clearance for top / side approach.

Likely semantic task:

```text
pick("apple")
place("bowl")
```

---

# 27. Purple Ball Map

Canonical ID:

```text
purple_ball
```

Role:

```text
pickable contact-sensitive object
```

It must spawn in a region with enough margin to prevent rolling off the surface.

Its reset velocity must always be zeroed.

The surface friction parameters must be tuned so that idle state remains stable while realistic rolling remains possible after contact.

---

# 28. Bowl Map

Canonical ID:

```text
bowl
```

Role:

```text
container
place destination
```

The bowl must define:

```text
body pose
rim height
interior placement region
interior center frame or reference
valid containment volume
```

The placement system should reason about the bowl interior, not just the bowl body origin.

---

# 29. Pan Map

Canonical ID:

```text
pan
```

Initial role:

```text
static kitchen context
future manipulation object
```

The pan should be positioned so that:

- it contributes to the visual scene,
- it does not obstruct early cube/apple tasks,
- its handle does not create unexpected collision traps,
- and future manipulation remains possible.

---

# 30. Container Mapping

A container is not represented as a single target coordinate.

Each container should define:

```text
container body
interior region
rim / boundary
valid object center region
minimum drop clearance
```

For example:

```text
bowl
└── bowl_inner_region
```

The semantic target for:

```text
place("bowl")
```

is the interior region, not the bowl mesh origin.

---

# 31. Named Location Map

`locations.yaml` should eventually contain named regions such as:

```text
bowl
surface_left_1_center
surface_left_1_drop_zone
surface_left_2_center
surface_right_1_center
shared_workspace_center
handover_zone
```

Task code must request semantic locations.

It must not request raw coordinates unless operating inside the lower-level motion layer.

---

# 32. Region Definition Format

A semantic region should support one of the following forms.

## Point + Tolerance

```yaml
type: point_region
center: [...]
position_tolerance: [...]
```

## Box Region

```yaml
type: box_region
center: [...]
size: [...]
```

## Container Volume

```yaml
type: container_region
reference: bowl
inner_bounds: ...
```

## Surface Region

```yaml
type: surface_region
surface: surface_left_1
margin: ...
```

The exact schema will be finalized in configuration documentation.

---

# 33. Scene Graph

Conceptually, the scene graph is:

```text
world
│
├── floor
│
├── central_mount
│
│   ├── panda1_base
│
│   └── panda2_base
│
├── surfaces
│   ├── surface_left_1
│   ├── surface_left_2
│   ├── surface_left_3
│   ├── surface_right_1
│   ├── surface_right_2
│   └── surface_right_3
│
├── objects
│   ├── cube
│   ├── apple
│   ├── purple_ball
│   ├── bowl
│   └── pan
│
├── regions
│   ├── panda1_primary_workspace
│   ├── panda2_primary_workspace
│   ├── shared_workspace
│   └── handover_zone
│
└── decorative_environment
    ├── sink
    ├── stove
    ├── laptop
    ├── controller
    ├── chairs
    └── storage_assets
```

Not all decorative assets need ROS-level representation.

---

# 34. Physics-Relevant vs Planning-Relevant Entities

An entity may be:

```text
physics relevant
planning relevant
semantic relevant
visual only
```

Example classification:

| Entity | Physics | MoveIt | Semantic | Visual |
|---|---:|---:|---:|---:|
| table | yes | yes | yes | yes |
| cube | yes | yes | yes | yes |
| apple | yes | yes | yes | yes |
| bowl | yes | yes | yes | yes |
| chair outside workspace | maybe | maybe | low | yes |
| wall decoration | no | no | no | yes |

Only relevant geometry should be mirrored into MoveIt.

---

# 35. MoveIt Scene Mapping

Planning-relevant MuJoCo entities must map predictably to MoveIt collision objects.

Suggested ID convention:

```text
MuJoCo body ID:
surface_left_1

MoveIt collision object ID:
surface_left_1
```

Use identical semantic names whenever possible.

Avoid translation tables such as:

```text
MuJoCo: table_a
MoveIt: collision_box_17
```

---

# 36. Runtime Object State Mapping

For every manipulable object, the runtime scene-state interface should expose:

```text
id
pose_world
orientation_world
linear_velocity
angular_velocity
support_surface
held_by
contact_summary
state_valid
```

Example conceptual state:

```yaml
id: apple
pose_world: ...
support_surface: surface_left_1
held_by: null
state_valid: true
```

---

# 37. Held Object Mapping

When an object is successfully grasped:

```text
held_by = panda1
```

and its semantic support relation changes from:

```text
support_surface = surface_left_1
```

to:

```text
support_surface = null
```

The physical state remains owned by MuJoCo.

---

# 38. Placement Mapping

After successful placement into a bowl:

```text
held_by = null
container = bowl
support relation = bowl / bowl interior
```

The system should not report successful placement solely because the gripper opened near the target.

---

# 39. Scene Integrity Baseline

Before each manipulation benchmark, a scene-integrity snapshot should be recorded.

Example tracked entities:

```text
cube
apple
purple_ball
bowl
pan
```

After the task, unrelated entities should remain within allowed pose tolerances.

This allows the project to quantify:

```text
collateral object disturbance
```

---

# 40. Unrelated Object Motion Rule

An unrelated object is any dynamic or semi-dynamic object not participating in the current task.

Example:

Task:

```text
apple → bowl
```

Unrelated:

```text
cube
purple_ball
pan
```

If the robot displaces them beyond configured tolerances, the task should be flagged.

---

# 41. Reachability Validation Map

Before locking numeric object positions, the scene must run a reachability study.

For each primary task location:

```text
Panda 1 IK valid?
collision-free pre-grasp valid?
approach path valid?
lift path valid?
retreat path valid?
```

A visually pleasing coordinate is not accepted until it passes these checks.

---

# 42. Reachability Matrix

The project should eventually maintain a matrix similar to:

| Region | Panda 1 | Panda 2 | Notes |
|---|---:|---:|---|
| `panda1_primary_workspace` | yes | limited/no | primary P1 |
| `panda2_primary_workspace` | limited/no | yes | primary P2 |
| `shared_workspace` | yes | yes | coordinated |
| `handover_zone` | yes | yes | future handover |
| `surface_left_1` | yes | optional | early tasks |
| `surface_right_1` | optional | yes | later tasks |

This matrix must be confirmed by IK / collision tests, not assumption.

---

# 43. No-Magic-Offset Rule

The following pattern is forbidden:

```text
target = object_pose
target.x += magic_number
target.z += another_magic_number
```

unless the offset comes from a named concept such as:

```text
grasp profile
tool frame
object geometry
pre-grasp clearance
approach distance
```

Every offset must have semantic meaning.

---

# 44. Scene Configuration Responsibilities

## `config/scene.yaml`

Owns:

```text
world settings
static furniture poses
surface dimensions
workspace region definitions
scene version
```

## `config/robots.yaml`

Owns:

```text
robot base poses
home joints
activation state
robot namespaces
workspace assignment
```

## `config/objects.yaml`

Owns:

```text
object IDs
object geometry
mass
dynamic/static role
initial spawn
support surface
```

## `config/locations.yaml`

Owns:

```text
semantic destinations
container interiors
drop zones
handover zone
```

---

# 45. Proposed `scene.yaml` Shape

Illustrative only:

```yaml
scene:
  version: "1.0"
  world_frame: world
  units: SI

surfaces:
  surface_left_1:
    pose: ...
    size: ...
    top_height: ...
    safe_spawn_region: ...
    safe_place_region: ...
    workspace_owner: panda1

regions:
  panda1_primary_workspace:
    type: box_region
    center: ...
    size: ...

  shared_workspace:
    type: box_region
    center: ...
    size: ...
```

The numeric values must be validated before being considered final.

---

# 46. Proposed `robots.yaml` Shape

Illustrative only:

```yaml
robots:
  panda1:
    model: franka_panda
    namespace: panda1
    active: true
    base_pose: ...
    home_joints: ...
    primary_workspace: panda1_primary_workspace

  panda2:
    model: franka_panda
    namespace: panda2
    active: false
    base_pose: ...
    home_joints: ...
    primary_workspace: panda2_primary_workspace
```

---

# 47. Proposed `objects.yaml` Shape

Illustrative only:

```yaml
objects:
  cube:
    class: manipulable
    dynamic: true
    support_surface: surface_left_1
    initial_pose: ...

  apple:
    class: manipulable
    dynamic: true
    support_surface: surface_left_1
    initial_pose: ...

  bowl:
    class: container
    dynamic: false
    support_surface: surface_left_2
    initial_pose: ...
```

---

# 48. Proposed `locations.yaml` Shape

Illustrative only:

```yaml
locations:
  bowl:
    type: container_region
    object_ref: bowl
    frame: world
    valid_region: ...

  handover_zone:
    type: box_region
    frame: world
    center: ...
    size: ...
```

---

# 49. Scene Lock Procedure

The scene must not be declared final immediately after it visually looks correct.

Locking procedure:

```text
Step 1 — visual layout complete
Step 2 — collision geometry validated
Step 3 — robot bases validated
Step 4 — IK reachability tested
Step 5 — object spawn safety tested
Step 6 — idle stability tested
Step 7 — 100 deterministic resets
Step 8 — planning-scene synchronization tested
Step 9 — scene map/config consistency checked
Step 10 — scene baseline tagged
```

Only then:

```text
scene_version = 1.0
```

---

# 50. Scene Lock Invariants

After scene version 1.0 is locked, the following are frozen unless a documented migration is approved:

```text
world origin
axis convention
robot base poses
surface IDs
surface major poses
surface dimensions
workspace identities
initial object IDs
container IDs
semantic region IDs
```

---

# 51. Allowed Post-Lock Changes

Allowed without changing scene semantics:

```text
texture improvements
visual material changes
lighting
camera placement
decorative mesh refinement
non-functional visual props
```

Allowed only with validation:

```text
collision geometry refinement
friction tuning
object mass tuning
solver tuning
```

Requires architecture / scene revision:

```text
moving the fixed shared rail assembly
moving primary surfaces
renaming canonical entities
changing world origin
changing axis convention
removing shared workspace
```

---

# 52. Scene Change Classification

Every scene-related commit should fall into one class:

```text
VISUAL
PHYSICS
SEMANTIC
LAYOUT
```

Example:

```text
VISUAL:
change table material

PHYSICS:
adjust apple friction

SEMANTIC:
add new named drop zone

LAYOUT:
move surface_left_2
```

LAYOUT changes require the most review.

---

# 53. Scene Integrity Test Inputs

A deterministic scene test should record:

```text
robot joint states
robot base transforms
object poses
object velocities
surface transforms
active constraints
contact anomalies
```

These values should be compared against expected reset values.

---

# 54. Scene Mapping Test

An automated test should eventually verify:

```text
every registered object exists in MuJoCo
every registered surface exists
every robot exists
every named location resolves
every object support surface resolves
every workspace region resolves
no duplicate semantic IDs exist
no config entity points to missing references
```

This test should run before manipulation tests.

---

# 55. Geometry Consistency Test

For each dynamic object at reset:

```text
object bottom
≈
support surface top
```

within an acceptable tolerance.

The object must not:

```text
float visibly
penetrate the table
spawn with contact explosion
```

---

# 56. Initial Collision Test

At reset, there must be:

```text
0 robot self-collision
0 robot-table penetration
0 robot-object penetration
0 object-object penetration
0 object-furniture penetration
```

Expected resting contacts such as:

```text
object ↔ table
```

are valid.

---

# 57. Codex Scene Implementation Contract

Codex must follow this sequence when implementing the map:

```text
read SCENE_SPECIFICATION.md
read SCENE_MAP.md
read COORDINATE_FRAMES.md
read config files
then modify scene code
```

Codex must never infer the scene solely from a screenshot after these specifications exist.

---

# 58. Codex Prohibited Actions

Codex must not:

1. rename canonical scene IDs without approval,
2. move robot bases to solve local failures,
3. create hidden coordinate constants,
4. duplicate location data,
5. disable collisions globally,
6. make static furniture dynamic without reason,
7. attach objects before grasp verification,
8. use decorative meshes as physics truth,
9. treat the bowl center as automatically valid placement,
10. put objects directly on edges,
11. modify world axes,
12. activate Panda 2 early,
13. bypass registry lookup,
14. silently change units,
15. change scene layout in a manipulation bugfix.

---

# 59. First Scene Implementation Target

The first implementation should produce the complete final-layout world with:

```text
floor
central robot mounting area
panda1
panda2
six main work surfaces
cube
apple
purple_ball
bowl
pan
major fixed kitchen assets
major decorative assets
```

The scene should already visually represent the intended project.

However, manipulation remains disabled until scene validation passes.

---

# 60. M1 Scene Acceptance Criteria

The Scene Map is considered correctly implemented when:

- every canonical scene ID exists,
- both Panda robots and both linear rails are correctly placed,
- six work surfaces exist,
- Panda 1 has a valid primary workspace,
- Panda 2 has a reserved primary workspace,
- shared workspace exists,
- handover zone is reserved,
- initial objects are correctly registered,
- bowl interior is a semantic destination,
- object support surfaces are known,
- all dimensions are metric,
- no duplicate coordinate definitions exist,
- MoveIt-relevant objects can be mapped one-to-one,
- and deterministic reset succeeds.

---

# 61. M1 Deterministic Validation Target

Required target:

```text
100 reset cycles
```

For every cycle:

```text
same fixed rail transforms
same rail reset positions
same robot joint reset state
same object initial poses
zero stale grasp constraints
zero unintended collisions
zero unintended object falls
zero spontaneous dynamic drift
```

Any repeated instability blocks progression to manipulation development.

---

# 62. Final Scene Map Principle

The Scene Map exists to make the physical world predictable.

The project should always be able to answer:

```text
What is this object?
Where is it?
What supports it?
Which robot owns this workspace?
Which robot can reach it?
Where may it be placed?
Which frame defines the measurement?
Which configuration file owns the value?
```

If any of these questions requires searching through implementation code, the scene architecture is incomplete.

The final rule is:

> The scene must be understood from documentation and configuration before reading manipulation code.

That rule is essential for keeping Home Robotics deterministic, debuggable, academically reproducible, and safe to extend.
