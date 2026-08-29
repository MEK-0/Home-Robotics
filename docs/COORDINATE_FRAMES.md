# Home Robotics — Coordinate Frames

## 1. Purpose

This document defines the coordinate-frame contract for the Home Robotics project.

Its purpose is to ensure that MuJoCo, ROS 2, TF, MoveIt 2, configuration files, object registries, task execution, and later perception systems all use a consistent spatial representation.

This document is authoritative for:

- frame naming,
- frame hierarchy,
- world-axis convention,
- linear-rail transforms,
- robot-base transforms,
- end-effector transforms,
- object frames,
- container frames,
- semantic-region frames,
- pose ownership,
- transform direction,
- and frame-related implementation rules.

All code and configuration must follow this contract.

---

## 2. Global World Convention

The root frame is:

```text
world
```

The global axis convention is:

```text
X → forward
Y → left
Z → up
```

All linear dimensions use meters.

All angular values use radians unless a document explicitly states otherwise.

The world frame is static.

It must never be shifted, rotated, or redefined to simplify a local task.

---

## 3. World Origin

The world origin represents the fixed global reference of the scene.

Recommended interpretation:

```text
origin =
center reference of the central rail / robot corridor
projected to floor level
```

Once Phase 1 scene validation is complete, the world origin is frozen.

Any later scene element must be positioned relative to this fixed reference.

---

## 4. Frame Naming Rules

Frame names must:

- use lowercase snake_case,
- be globally unique,
- identify the owning robot or object,
- avoid spaces,
- avoid temporary suffixes such as `_final`, `_test`, or `_new`,
- remain stable once published.

Examples:

```text
world
panda1_rail
panda1_carriage
panda1_base
panda1_link0
panda1_hand
panda1_tcp

panda2_rail
panda2_carriage
panda2_base
panda2_link0
panda2_hand
panda2_tcp

apple
bowl
bowl_inner
```

---

## 5. Core Transform Principle

All spatial relationships should be expressed as explicit transforms.

The project should reason in terms of:

```text
T_parent_child
```

rather than manually adding world-space offsets.

For example:

```text
T_world_panda1_base
=
T_world_panda1_rail
×
T_panda1_rail_panda1_carriage
×
T_panda1_carriage_panda1_base
```

This principle is essential because Panda base position changes dynamically as the rail moves.

---

## 6. Panda 1 Frame Tree

Canonical Panda 1 frame hierarchy:

```text
world
└── panda1_rail
    └── panda1_carriage
        └── panda1_base
            └── panda1_link0
                └── panda1_link1
                    └── panda1_link2
                        └── panda1_link3
                            └── panda1_link4
                                └── panda1_link5
                                    └── panda1_link6
                                        └── panda1_link7
                                            └── panda1_hand
                                                └── panda1_tcp
```

The exact internal Franka link names should match the validated robot model.

The semantic frames above must remain consistent.

---

## 7. Panda 2 Frame Tree

Canonical Panda 2 frame hierarchy:

```text
world
└── panda2_rail
    └── panda2_carriage
        └── panda2_base
            └── panda2_link0
                └── panda2_link1
                    └── panda2_link2
                        └── panda2_link3
                            └── panda2_link4
                                └── panda2_link5
                                    └── panda2_link6
                                        └── panda2_link7
                                            └── panda2_hand
                                                └── panda2_tcp
```

Panda 2 remains inactive during early phases but its frame tree should still be well-defined.

---

## 8. Linear Rail Frame Model

Each robot is mounted on an independent linear rail.

Canonical frames:

```text
panda1_rail
panda1_carriage

panda2_rail
panda2_carriage
```

The rail frame is static relative to `world`.

The carriage frame is dynamic relative to the rail frame.

Conceptually:

```text
world
  ↓ static
panda1_rail
  ↓ prismatic joint
panda1_carriage
```

The same applies to Panda 2.

---

## 9. Rail Joint Axis

The nominal rail joint axis is:

```text
+X / -X in the rail frame
```

The rail frame should be aligned so that its local X axis matches the intended longitudinal travel direction.

The expected relation is:

```text
rail local X ≈ world X
```

unless a documented mechanical transform requires otherwise.

The project should avoid rotated rail frames unless there is a strong geometric reason.

---

## 10. Rail Position Variable

Each rail exposes one prismatic joint coordinate:

```text
q_rail
```

Conceptually:

```text
panda1_rail_joint
panda2_rail_joint
```

The carriage transform is:

```text
T_rail_carriage(q_rail)
```

where translation occurs only along the defined rail axis.

No rotational component should be introduced by normal rail motion.

---

## 11. Rail Home Frame State

Each rail has a configured home position:

```text
q_rail_home
```

The home position belongs in:

```text
config/robots.yaml
```

It is not necessarily zero.

Reset must restore the rail to its configured home position.

---

## 12. Robot Base Frame

The Panda base frame is:

```text
panda1_base
panda2_base
```

The base frame is rigidly attached to the carriage.

Therefore:

```text
T_carriage_base
```

is constant.

But:

```text
T_world_base
```

is dynamic because the carriage moves along the rail.

This distinction is critical.

---

## 13. Base Teleportation Is Prohibited

The Panda base must never be repositioned by directly rewriting:

```text
world → panda_base
```

during normal execution.

Correct motion:

```text
change q_rail
```

Incorrect motion:

```text
set panda base world position directly
```

Direct base repositioning is only acceptable during initial model construction before runtime.

---

## 14. Panda Link Frames

Robot link transforms come from the Panda kinematic model.

They should not be duplicated manually in project code.

The transform chain is generated from:

```text
rail joint
+
Panda arm joints
```

The project must rely on:

- MuJoCo kinematics,
- TF,
- MoveIt robot model,
- and validated URDF / robot description

rather than custom handwritten link transforms.

---

## 15. Hand Frame

Canonical hand frame:

```text
panda1_hand
panda2_hand
```

This represents the Franka Hand body frame.

The hand frame is not automatically the same as the task TCP.

---

## 16. TCP Frame

Canonical tool-center-point frames:

```text
panda1_tcp
panda2_tcp
```

The TCP is the task-space frame used for:

- grasp planning,
- Cartesian approach,
- place planning,
- distance-to-target measurement,
- and pose validation.

The transform:

```text
T_hand_tcp
```

must be fixed and explicitly defined.

It must never be guessed from visual mesh geometry.

---

## 17. TCP Definition Principle

The TCP should be located at a physically meaningful manipulation point.

For the Franka Hand, the preferred TCP is:

```text
center between the two fingertips
at the intended grasp reference plane
```

The exact offset should be validated from the actual gripper model.

Once defined, the TCP offset becomes stable.

---

## 18. Gripper Finger Frames

If exposed, canonical finger frames should preserve Franka naming while remaining robot-specific.

Conceptually:

```text
panda1_leftfinger
panda1_rightfinger

panda2_leftfinger
panda2_rightfinger
```

Exact names may follow the validated Franka model.

Finger contact checks should use actual gripper geometry and contact state, not synthetic frame distances alone.

---

## 19. Planning Frame

The global MoveIt planning frame should be:

```text
world
```

This ensures:

- both rails,
- both robots,
- all work surfaces,
- all objects,
- and all semantic locations

share one common planning reference.

A robot-local frame must not become the global planning frame.

---

## 20. MoveIt Kinematic Root

For Panda 1, the effective kinematic chain begins at:

```text
panda1_rail
```

and includes:

```text
panda1_rail_joint
+
Panda arm joints
```

For Panda 2:

```text
panda2_rail
```

and:

```text
panda2_rail_joint
+
Panda arm joints
```

The exact SRDF planning-group structure will be defined during MoveIt integration.

---

## 21. Planning Group Concept

Expected conceptual groups:

```text
panda1_arm_with_rail
panda1_hand

panda2_arm_with_rail
panda2_hand
```

Potential later convenience groups:

```text
panda1_arm_only
panda2_arm_only
```

The primary manipulation group should include the rail so reachability is solved jointly.

---

## 22. Why Rail + Arm Must Share Planning

For an object far along the table row, a fixed-base arm may have no valid IK solution.

The planner should be able to solve:

```text
rail displacement
+
arm configuration
```

together.

The intended reasoning is:

```text
target pose
   ↓
joint solution over 8 DOF
   ↓
rail + Panda trajectory
```

not:

```text
manually move rail to arbitrary position
   ↓
plan arm separately
```

unless a deliberately staged motion is required.

---

## 23. Object Frames

Every functional object has a canonical object frame.

Examples:

```text
cube
apple
purple_ball
bowl
pan
```

The object frame should be attached to the object's body origin.

For dynamic objects, this frame moves with the object.

---

## 24. Object Pose Representation

Runtime object poses should be reported as:

```text
T_world_object
```

Example:

```text
T_world_apple
```

The Scene State Provider should expose object state in the `world` frame by default.

This prevents consumers from needing to know which table currently supports the object.

---

## 25. Object Local Frames

Objects may define additional semantic local frames.

Examples:

```text
apple_grasp_center
cube_center
pan_handle
bowl_inner
bowl_rim
```

These frames are rigidly attached to the parent object.

Example:

```text
world
└── bowl
    ├── bowl_inner
    └── bowl_rim
```

These are semantic frames, not separate physical bodies unless required.

---

## 26. Bowl Frames

The bowl should expose at minimum:

```text
bowl
bowl_inner
```

Optional:

```text
bowl_rim
```

Meaning:

```text
bowl
→ physical body origin

bowl_inner
→ reference for valid placement region

bowl_rim
→ reference for approach / clearance logic
```

The placement target must not rely only on the bowl body origin.

---

## 27. Pan Frames

The pan may eventually expose:

```text
pan
pan_handle
pan_inner
```

For early phases only `pan` may be required.

Future grasping should use:

```text
pan_handle
```

rather than a hard-coded offset from the pan body.

---

## 28. Surface Frames

Each work surface has a canonical frame.

Examples:

```text
surface_left_1
surface_left_2
surface_left_3

surface_right_1
surface_right_2
surface_right_3
```

The frame origin should be defined consistently.

Recommended convention:

```text
center of tabletop top surface
```

This gives:

```text
surface frame Z = tabletop height
```

and simplifies:

- spawn regions,
- placement regions,
- support checks.

---

## 29. Surface Frame Orientation

Surface frames should align with the world axes whenever practical.

Preferred:

```text
surface X → world X
surface Y → world Y
surface Z → world Z
```

Do not rotate a surface frame simply because its visual mesh has an arbitrary mesh coordinate system.

Visual mesh transforms and semantic frame transforms are separate concerns.

---

## 30. Support-Surface Relationship

If an object is resting on a table:

```text
support_surface = surface_left_1
```

the runtime object pose remains expressed as:

```text
T_world_object
```

The support relation is semantic metadata.

Do not change the object's parent TF to the table merely because it is resting there.

---

## 31. Held Object Relationship

When Panda 1 grasps an object:

```text
held_by = panda1
```

The physical object continues to exist in MuJoCo.

Its runtime pose remains available as:

```text
T_world_object
```

A temporary physical stabilization constraint does not change the semantic frame naming.

---

## 32. Attached Object in MoveIt

When an object becomes held, MoveIt may represent it as an attached collision object.

Conceptually:

```text
world object
   ↓ grasp
attached to panda1_tcp / hand link
```

The exact attachment link will be selected based on MoveIt requirements.

The physical MuJoCo state remains authoritative.

---

## 33. Semantic Workspace Frames

The following semantic regions exist:

```text
panda1_primary_workspace
panda2_primary_workspace
shared_workspace
handover_zone
```

These are spatial regions rather than necessarily physical TF frames.

A region may be represented by:

```text
reference frame
+
center
+
size
```

Example:

```text
frame: world
center: [...]
size: [...]
```

Do not publish unnecessary TF frames for every abstract region unless runtime tooling benefits from them.

---

## 34. Named Location Frames

Task destinations may reference:

```text
world
surface frame
object frame
```

Examples:

```text
surface_left_1_drop_zone
→ relative to surface_left_1

bowl
→ resolved through bowl_inner

handover_zone
→ world-relative region
```

The owning reference frame must be explicit in `locations.yaml`.

---

## 35. Config Pose Rule

Every pose in configuration must declare or inherit a known reference frame.

Bad:

```yaml
position: [0.5, 0.2, 0.8]
```

with no known frame.

Good:

```yaml
frame: world
position: [0.5, 0.2, 0.8]
```

or a schema where the file guarantees:

```text
all top-level scene poses are world-relative
```

The convention must be documented.

---

## 36. Preferred Configuration Convention

Recommended rule:

```text
static scene entity poses → world frame

robot carriage-relative geometry → carriage frame

object semantic subframes → object-local frame

surface regions → surface-local frame

container interior regions → container-local frame
```

This reduces unnecessary world-coordinate duplication.

---

## 37. Transform Lookup Rule

Any module that needs a pose in another frame should use the transform system.

Conceptually:

```text
pose_in_target_frame
=
T_target_source
×
pose_in_source_frame
```

The implementation should use TF / validated transform libraries rather than custom matrix multiplication scattered through task code.

---

## 38. Raw MuJoCo Pose Conversion

MuJoCo body poses must be converted into the project's ROS / TF convention exactly once in the integration layer.

The integration layer owns:

```text
MuJoCo body pose
→ normalized ROS pose
```

Higher layers must not each implement their own conversion.

---

## 39. Quaternion Convention

ROS geometry messages use quaternion ordering:

```text
x, y, z, w
```

Any MuJoCo API ordering must be checked explicitly during integration.

The project must never assume quaternion element order.

A dedicated tested conversion function is required.

---

## 40. Quaternion Normalization

Every quaternion entering ROS / TF from external conversion code should be normalized or validated.

Invalid quaternions must trigger a clear error rather than silently propagating.

---

## 41. RPY Usage

Roll-pitch-yaw values are acceptable in configuration for human readability.

However:

- runtime transforms should use quaternions or matrices,
- repeated Euler conversion inside motion code should be avoided,
- and angle units must be radians.

---

## 42. Right-Handed Coordinate System

The complete project uses a right-handed coordinate system.

No package may introduce a left-handed frame.

Any imported mesh with different axis conventions must be corrected at asset-import level.

---

## 43. Mesh Frame vs Semantic Frame

A mesh's native origin is not automatically the semantic frame.

Example:

```text
bowl_mesh_origin
```

may be inconvenient.

The project may define:

```text
bowl
```

at a useful physical center and apply a fixed mesh offset.

This prevents model-authoring quirks from leaking into task code.

---

## 44. Visual Frame vs Collision Frame

Visual and collision meshes may require different local transforms.

Both must remain attached to the same semantic body frame.

Example:

```text
bowl
├── visual geometry transform
└── collision geometry transform
```

The semantic `bowl` frame remains unchanged.

---

## 45. Rail Collision Frame

Each rail must define collision geometry relative to:

```text
pandaN_rail
```

The carriage collision geometry must be relative to:

```text
pandaN_carriage
```

This ensures collision geometry moves correctly with the prismatic joint.

---

## 46. Robot Mount Frame

If the physical model requires an intermediate mount frame, it may be introduced:

```text
panda1_carriage
└── panda1_mount
    └── panda1_base
```

However, unnecessary frames should be avoided.

If added, they must be documented here before broad use.

---

## 47. Rail Limits and Frames

Rail joint limits are expressed in the rail joint coordinate.

Example:

```text
q_rail ∈ [q_min, q_max]
```

These limits are not world X coordinates.

They are displacement values relative to the rail joint's zero reference.

This distinction must remain clear.

---

## 48. Rail Zero Reference

Each rail must define:

```text
q_rail = 0
```

at a mechanically meaningful reference point.

Recommended:

```text
carriage aligned with rail-center reference
```

or another clearly documented model origin.

The exact zero point must not change after Phase 1.

---

## 49. Home Position vs Zero Position

The rail home position may differ from zero:

```text
q_home ≠ 0
```

This is allowed.

The configuration must distinguish:

```text
zero reference
home position
```

Do not use these concepts interchangeably.

---

## 50. Robot Joint Frames

Panda joint frames should follow the validated Franka model.

The project must not rename internal joints casually.

Robot namespace isolation should occur through prefixes or model names rather than destroying known Franka semantics.

---

## 51. TF Prefix Strategy

For two identical robots, all robot frames must remain unique.

Preferred strategy:

```text
panda1_*
panda2_*
```

This applies to:

- links,
- hand,
- TCP,
- rail,
- carriage,
- optional mount frames.

---

## 52. Static Transforms

Static transforms include:

```text
world → panda1_rail
world → panda2_rail

panda1_carriage → panda1_base
panda2_carriage → panda2_base

panda1_hand → panda1_tcp
panda2_hand → panda2_tcp
```

These should not be continuously recomputed unless required by the runtime representation.

---

## 53. Dynamic Transforms

Dynamic transforms include:

```text
panda1_rail → panda1_carriage
panda2_rail → panda2_carriage

robot joint transforms
dynamic object transforms
```

These must update with simulation state.

---

## 54. Frame Ownership

Each frame has one authoritative owner.

Suggested ownership:

```text
world
→ scene system

rail/carriage
→ robot model + simulation state

Panda links
→ robot state publisher / robot model

TCP
→ robot description / static transform

dynamic objects
→ scene state provider

semantic regions
→ configuration
```

Avoid multiple publishers for the same TF.

---

## 55. TF Publication Rule

There must be exactly one authoritative transform publisher for each dynamic frame relationship.

Duplicate TF publication is prohibited.

Example prohibited:

```text
MuJoCo node publishes panda1_base
AND
custom task node also publishes panda1_base
```

---

## 56. Simulation State to TF Pipeline

Recommended data flow:

```text
MuJoCo state
    ↓
robot / scene integration layer
    ↓
joint states + object state
    ↓
robot_state_publisher / scene TF publisher
    ↓
TF tree
```

The exact implementation may vary, but ownership must stay unambiguous.

---

## 57. Object TF Publication Policy

Not every object necessarily needs continuous TF publication.

Required objects for manipulation should expose a reliable pose through the Scene State Provider.

TF may be published for active objects if useful.

The project should avoid flooding TF with decorative object frames.

---

## 58. Pose Timestamp Rule

Runtime poses should use simulation time.

ROS nodes should follow:

```text
use_sim_time = true
```

when running against MuJoCo simulation.

Mixing wall-clock and simulation-time transforms is prohibited.

---

## 59. Reset and TF

After reset:

- rail positions,
- robot joint states,
- object poses,
- and all dynamic transforms

must return to the configured reset state.

Stale transforms from pre-reset state must not remain logically active.

---

## 60. Pick Pose Frame

A grasp pose should always carry an explicit frame.

Recommended canonical representation:

```text
PoseStamped
frame_id: world
```

for planner-facing final targets.

Internally, grasp candidates may be constructed in an object-local frame.

Example:

```text
apple
  ↓ grasp transform
candidate_tcp_pose_in_apple
```

then converted:

```text
T_world_tcp_target
=
T_world_apple
×
T_apple_tcp_target
```

This is preferred over manually offsetting world coordinates.

---

## 61. Pre-Grasp Pose Frame

Pre-grasp should be derived from the grasp pose using a documented approach direction.

Example:

```text
T_world_pregrasp
=
T_world_grasp
×
T_grasp_pregrasp
```

The offset belongs to the grasp profile.

It must not be an unexplained world-space Z offset.

---

## 62. Place Pose Frame

Place poses should be derived from semantic target frames.

Example:

```text
bowl_inner
```

rather than raw world coordinates.

Conceptually:

```text
T_world_place
=
T_world_bowl_inner
×
T_bowl_inner_place
```

This makes the task robust if the bowl later moves.

---

## 63. Container Placement Frame

If a container becomes movable in later phases, its interior frame must move with it.

This is why placement targets should use:

```text
bowl_inner
```

instead of:

```text
hard-coded world XYZ
```

---

## 64. Handover Frame

The future handover zone may use:

```text
frame: world
```

during early planning.

Later a dedicated semantic frame may be introduced if required.

Any introduction must be documented here.

---

## 65. Camera Frames — Future

Perception is not part of early phases.

When introduced, cameras should use conventional frame naming such as:

```text
camera_link
camera_optical_frame
```

and must respect ROS optical-frame conventions.

Camera-frame design will be added only when the perception phase begins.

---

## 66. Real Hardware Compatibility

The frame structure should remain conceptually compatible with a future physical rail-mounted Panda system.

Simulator-only assumptions should not leak into semantic frame names.

For example:

```text
panda1_tcp
```

is preferable to:

```text
mujoco_panda1_tip
```

---

## 67. Frame Validation Tests

Automated tests should verify:

```text
world exists
rail frames exist
carriage frames exist
Panda base frames exist
TCP frames exist
no duplicate frames
rail axis is correct
static transforms are stable
dynamic transforms update
object transforms are valid
quaternions are normalized
```

---

## 68. Rail Transform Test

For a controlled rail displacement:

```text
Δq
```

the expected Panda base translation should satisfy:

```text
Δx ≈ Δq
Δy ≈ 0
Δz ≈ 0
```

in the rail frame.

No unexpected base rotation should occur.

---

## 69. TCP Transform Test

At a known robot joint configuration:

```text
MuJoCo FK
```

and:

```text
TF / MoveIt FK
```

for `pandaN_tcp` should agree within a defined tolerance.

This is a critical integration test.

---

## 70. Object Transform Test

For a known object reset pose:

```text
MuJoCo body pose
```

and:

```text
Scene State Provider world pose
```

must agree within tolerance.

This validates simulator-to-ROS pose conversion.

---

## 71. Surface Transform Test

Each configured work surface should satisfy:

```text
surface frame Z
=
physical top height
```

within tolerance.

This is important for object spawn and placement logic.

---

## 72. Bowl Frame Test

The project should verify that:

```text
bowl_inner
```

is physically inside the bowl and below the rim.

A placement target outside the actual interior invalidates the scene configuration.

---

## 73. Codex Frame Rules

Codex must:

1. read this document before editing transform-related code,
2. preserve the `world` root frame,
3. preserve X-forward, Y-left, Z-up convention,
4. treat rails as prismatic robot joints,
5. never teleport Panda bases,
6. use TF or transform utilities for frame conversion,
7. keep object poses world-relative at runtime,
8. use object-local frames for grasp semantics,
9. use container-local frames for place semantics,
10. avoid duplicate TF publishers,
11. use explicit frame IDs,
12. validate quaternion ordering,
13. use simulation time,
14. keep Panda 1 and Panda 2 frame namespaces unique,
15. update this document if a new persistent semantic frame is introduced.

---

## 74. Prohibited Patterns

The following patterns are explicitly prohibited.

### Hard-Coded World Offset

```python
target.x = apple.x + 0.04
target.z = apple.z + 0.12
```

without semantic justification.

### Manual Panda Base Translation

```python
panda_base.x = desired_table_x
```

### Duplicate Frame Ownership

```text
two nodes publishing the same panda1_base transform
```

### Anonymous Pose

```text
Pose()
```

passed across layers without a known reference frame.

### Mesh-Origin Dependency

Using a mesh's arbitrary local origin as a task semantic frame without validation.

---

## 75. Recommended Frame-Aware Manipulation Pattern

Correct pattern:

```text
Object Registry
   ↓
apple frame
   ↓
grasp profile
   ↓
T_apple_tcp_grasp
   ↓
TF lookup: T_world_apple
   ↓
T_world_tcp_grasp
   ↓
MoveIt planning
```

For place:

```text
Location Registry
   ↓
bowl_inner
   ↓
T_bowl_inner_place
   ↓
TF lookup: T_world_bowl_inner
   ↓
T_world_tcp_place
   ↓
MoveIt planning
```

---

## 76. Relationship to Configuration Files

### `scene.yaml`

Defines:

```text
world-relative static scene transforms
surface frames
workspace region references
```

### `robots.yaml`

Defines:

```text
rail world transforms
rail zero references
rail limits
rail home positions
carriage-to-base transforms
robot home joint states
```

### `objects.yaml`

Defines:

```text
initial object world poses
object-local semantic frames
```

### `locations.yaml`

Defines:

```text
semantic location reference frames
local region geometry
container-relative placement regions
```

### `grasp_profiles.yaml`

Defines:

```text
object-to-grasp transforms
pre-grasp offsets
approach directions
```

---

## 77. Coordinate Frame Freeze Policy

Once Phase 1 is accepted, the following become stable:

```text
world definition
world axis convention
rail frame names
rail world transforms
rail zero references
carriage frame names
Panda base frame names
surface frame conventions
object canonical frame names
TCP frame names
```

Changing any of these requires a documented architectural migration.

---

## 78. Future Perception Transition

When vision is introduced, perception should estimate object poses into the same canonical frame system.

Expected flow:

```text
camera observation
   ↓
object pose in camera frame
   ↓
TF
   ↓
object pose in world
   ↓
Scene State Interface
```

The task and manipulation layers should continue consuming the same object-state abstraction.

---

## 79. Final Frame Principle

Every spatial value in Home Robotics must answer:

```text
What frame is this expressed in?
Who owns that frame?
Is the transform static or dynamic?
How is it obtained?
Is it configuration-driven or state-driven?
```

If any pose cannot answer those questions, it is not valid project data.

The coordinate-frame system exists to guarantee that rail motion, robot motion, object motion, planning, and later perception all describe the same physical world consistently.
