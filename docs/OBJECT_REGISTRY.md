# Home Robotics — Object Registry

## 1. Purpose

This document defines the canonical object registry for the Home Robotics project.

The object registry is the single semantic source of truth for all manipulable, container, and task-relevant scene objects.

Its purpose is to ensure that:

- every object has one stable identity,
- every object has a defined physical role,
- every object has a known geometry model,
- every object has a known support relation,
- every object has a defined grasp policy,
- runtime state is represented consistently,
- MoveIt and MuJoCo use matching semantic IDs,
- the LLM does not invent object names,
- and manipulation code does not hard-code object-specific coordinates.

Primary runtime configuration:

```text
config/objects.yaml
```

Related configuration:

```text
config/grasp_profiles.yaml
config/locations.yaml
config/scene.yaml
config/physics.yaml
```

---

## 2. Registry Principle

Every functional object must exist in the Object Registry before it can be used by:

- MuJoCo,
- Scene State Provider,
- MoveIt Planning Scene,
- manipulation logic,
- Task Executor,
- benchmarking,
- or the LLM tool layer.

Canonical flow:

```text
objects.yaml
    ↓
Object Registry
    ↓
Simulation / Scene State
    ↓
Manipulation
    ↓
Task Executor
    ↓
LLM
```

No parallel object database may be created.

---

## 3. Canonical Object IDs

Initial canonical IDs:

```text
cube
apple
purple_ball
bowl
pan
```

These IDs are stable.

They must not be renamed casually.

Good:

```text
apple
purple_ball
```

Bad:

```text
red_apple_01
ball_final
object_4
purpleSphere
```

---

## 4. Object Categories

Every object belongs to one primary category.

Allowed initial categories:

```text
manipulable
container
environment
future_manipulable
```

Potential future categories:

```text
tool
fragile
articulated
deformable
```

The category must describe the object's role in the robotics system, not merely its appearance.

---

## 5. Initial Object Classification

| Object | Category | Dynamic | Pickable | Place Target | Initial Role |
|---|---|---:|---:|---:|---|
| `cube` | manipulable | yes | yes | no | baseline manipulation object |
| `apple` | manipulable | yes | yes | no | semantic household object |
| `purple_ball` | manipulable | yes | yes | no | spherical grasp/contact object |
| `bowl` | container | initially no | no | yes | placement target |
| `pan` | future_manipulable | initially no | later | later | kitchen context / future grasp target |

Phase 1 physical values are configuration-owned: cube `0.12 kg`, apple `0.15 kg`, purple ball `0.06 kg`, fixed bowl metadata `0.35 kg`, and fixed pan metadata `0.80 kg`. All visuals use project-authored MuJoCo primitives; no external object asset is used.

---

## 6. Registry Data Model

Every registered object should define at minimum:

```text
id
category
dynamic
pickable
place_target
visual_asset
collision_geometry
mass
inertia_strategy
friction_profile
initial_pose
initial_support_surface
grasp_profile
semantic_aliases
planning_scene_enabled
benchmark_enabled
```

Optional fields:

```text
container_region
semantic_subframes
preferred_orientation
fragility
max_grasp_force
placement_constraints
contact_policy
future_capabilities
```

---

## 7. Identity Consistency

The same object should use the same semantic ID across all layers.

Preferred mapping:

```text
MuJoCo body ID       → apple
MoveIt object ID     → apple
Registry ID          → apple
Task Executor ID     → apple
Benchmark log ID     → apple
```

Avoid unnecessary translation tables such as:

```text
MuJoCo: red_sphere_01
MoveIt: fruit_collision
Task API: apple
```

One semantic identity should follow the object through the entire system.

---

## 8. Semantic Aliases

Natural-language aliases may exist.

Example:

```yaml
apple:
  aliases:
    - apple
    - red apple
```

Aliases belong to semantic resolution.

The LLM layer may resolve:

```text
"red apple"
```

to:

```text
apple
```

before issuing a robot action.

The Task Executor should receive only canonical IDs.

---

## 9. Ambiguous Alias Rule

Ambiguous aliases should not resolve automatically.

Example:

If multiple spherical objects exist later:

```text
ball
```

may become ambiguous.

The language layer should request clarification or use additional scene context.

The registry must never silently choose an arbitrary object.

---

## 10. Metadata vs Runtime State

The Object Registry owns relatively stable metadata.

Example:

```text
mass
geometry
grasp profile
semantic role
aliases
```

MuJoCo owns physical runtime state.

Example:

```text
pose
velocity
contact
```

The Scene State Provider creates a unified runtime object view.

Therefore:

```text
Registry
=
what the object is

Runtime State
=
where the object is and what it is doing
```

These responsibilities must remain separate.

---

## 11. Runtime Object State

Each runtime object state should expose:

```text
id
pose_world
linear_velocity
angular_velocity
support_surface
held_by
container
contact_state
state_valid
```

Example:

```yaml
id: apple
support_surface: surface_left_1
held_by: null
container: null
state_valid: true
```

---

## 12. Pose Representation

Runtime object poses should be expressed in:

```text
world
```

by default.

Each pose must include:

```text
position
orientation
frame_id
timestamp
```

Object-local subframes may also exist, but the main runtime state should remain easy to consume from the global frame.

---

## 13. Initial Pose Ownership

Initial object poses belong in:

```text
config/objects.yaml
```

They must not be duplicated inside:

```text
MuJoCo loader code
manipulation nodes
Task Executor
LLM prompts
benchmark scripts
```

Tests may load expected values from the configuration.

---

## 14. Initial Support Surface

Every initial object should declare:

```text
initial_support_surface
```

Example:

```yaml
initial_support_surface: surface_left_1
```

This information supports:

- deterministic reset,
- state validation,
- scene summaries,
- and placement reasoning.

---

## 15. Runtime Support State

At runtime:

```text
support_surface
```

may change.

Possible examples:

```text
surface_left_1
surface_left_2
bowl
pan
null
```

While an object is securely held:

```text
support_surface = null
```

---

## 16. Held State

An object may be held by:

```text
panda1
panda2
null
```

Example:

```yaml
held_by: panda1
```

A normal single-object grasp may have only one owner.

Future cooperative dual-arm grasping may extend this model deliberately.

---

## 17. Container State

An object may have:

```text
container
```

Example:

```yaml
container: bowl
```

Container state must be based on actual geometric / physical validation.

It must not be inferred only from distance to the container body origin.

---

## 18. Example State Transitions

Before pick:

```text
apple
  support_surface: surface_left_1
  held_by: null
  container: null
```

During transport:

```text
apple
  support_surface: null
  held_by: panda1
  container: null
```

After successful placement:

```text
apple
  support_surface: bowl
  held_by: null
  container: bowl
```

---

## 19. Object Geometry

Every object may define:

```text
visual geometry
collision geometry
```

These are intentionally separate.

Visual geometry may be detailed.

Collision geometry must prioritize stable simulation and predictable planning.

---

## 20. Collision Geometry Types

Preferred collision representations:

```text
box
sphere
capsule
cylinder
simple convex geometry
small primitive composition
```

Triangle-mesh collision should be avoided by default.

If mesh collision is required, it must be justified and validated.

---

## 21. Collision Geometry Ownership

The registry should reference the intended collision model.

The actual MuJoCo model may implement it through primitive geoms.

MoveIt should use a planning representation that preserves the same effective physical volume closely enough for safe motion planning.

---

## 22. Visual Asset Ownership

Objects may reference visual assets such as:

```text
meshes/apple.obj
meshes/pan.glb
```

The visual mesh origin must not define task semantics accidentally.

If needed, the object body frame should include a fixed mesh transform.

---

## 23. Mass

Every dynamic object must define a physically plausible mass in kilograms.

The mass must:

- be realistic,
- remain stable across repeated tests,
- avoid numerical extremes,
- and be documented.

Mass should not be changed simply to force successful grasping.

---

## 24. Inertia Strategy

Simple primitive objects may derive inertia from:

```text
geometry
+
mass
```

Mesh-based objects must use a documented inertia strategy.

Arbitrary inertia tensors without explanation are prohibited.

---

## 25. Friction

Every dynamic object should use a documented friction profile.

Friction may be:

```text
global default
```

or:

```text
object-specific override
```

Object-specific overrides require a physical reason.

Friction tuning belongs in:

```text
config/physics.yaml
```

or a clearly defined object override.

---

## 26. Rest Stability

An object at reset must remain physically stable.

A valid reset state requires:

```text
no unexplained sliding
no spontaneous rolling
no immediate tipping
no contact explosion
no table penetration
```

The purple ball may physically roll when disturbed, but should remain stable at rest.

---

## 27. Pickability

Every object explicitly declares:

```text
pickable: true | false
```

Example:

```text
cube → true
apple → true
purple_ball → true
bowl → false initially
pan → false initially
```

Unsupported pick requests must fail before motion planning.

---

## 28. Place-Target Capability

Objects may define:

```text
place_target: true | false
```

Initial:

```text
bowl → true
```

Future examples:

```text
pan → true
basket → true
bin → true
```

A place target must have a valid location / region definition.

---

## 29. Grasp Profile Reference

Every pickable object references a grasp profile.

Examples:

```text
cube → cube_standard
apple → apple_standard
purple_ball → sphere_standard
```

Detailed grasp parameters belong in:

```text
config/grasp_profiles.yaml
```

The registry stores only the relationship.

---

## 30. Grasp Profile Responsibilities

A grasp profile may define:

```text
candidate grasp orientations
approach direction
pre-grasp distance
finger opening target
closure policy
verification thresholds
lift clearance
```

These values must not be duplicated into the object registry.

---

## 31. Semantic Subframes

Objects may expose stable semantic subframes.

Examples:

```text
apple_grasp_center
cube_center
bowl_inner
bowl_rim
pan_handle
```

These frames are object-local.

They must have explicit definitions.

---

## 32. Cube Specification

Canonical ID:

```text
cube
```

Category:

```text
manipulable
```

Primary purpose:

```text
baseline manipulation benchmark
```

Recommended collision:

```text
box
```

Recommended grasp profile:

```text
cube_standard
```

The cube is the first end-to-end pick-and-place validation object.

---

## 33. Why Cube Comes First

The cube provides:

- planar grasp surfaces,
- predictable contact normals,
- simple collision geometry,
- simple placement behavior,
- and easy pose validation.

This reduces uncertainty while validating the manipulation stack.

Development order should begin:

```text
cube
```

before:

```text
apple
purple_ball
```

---

## 34. Apple Specification

Canonical ID:

```text
apple
```

Category:

```text
manipulable
```

Primary role:

```text
semantic household pick/place object
```

Preferred collision:

```text
sphere
```

or a validated simple convex approximation.

Preferred grasp profile:

```text
apple_standard
```

The apple should visually resemble a real apple while retaining simple stable collision geometry.

---

## 35. Apple Grasping Considerations

Potential grasp styles:

```text
side grasp
slightly top-biased side grasp
```

The apple should not require scene-specific XYZ offsets.

Any geometry-specific adjustment belongs to:

```text
apple_standard
```

grasp profile.

---

## 36. Purple Ball Specification

Canonical ID:

```text
purple_ball
```

Category:

```text
manipulable
```

Collision:

```text
sphere
```

Purpose:

```text
symmetric grasp test
friction test
rolling-object robustness test
```

Preferred grasp profile:

```text
sphere_standard
```

---

## 37. Purple Ball Stability Rule

The ball must spawn in a safe region with sufficient table-edge clearance.

At reset:

```text
linear velocity = 0
angular velocity = 0
```

The physics model should allow realistic rolling after interaction without causing idle drift.

---

## 38. Bowl Specification

Canonical ID:

```text
bowl
```

Category:

```text
container
```

Initial dynamics:

```text
static or fixed
```

Primary role:

```text
place destination
```

The bowl must expose:

```text
bowl
bowl_inner
```

Optional:

```text
bowl_rim
```

---

## 39. Bowl Collision Rule

The bowl must not be represented as a fully solid primitive if objects are expected to be placed inside it.

Its collision geometry should approximate:

```text
bottom
+
walls
```

while leaving the interior physically accessible.

Possible implementation:

```text
multiple simple collision primitives
```

or a carefully validated convex decomposition.

---

## 40. Bowl Interior Region

The valid target is not:

```text
bowl body origin
```

The valid target is:

```text
bowl_inner
```

plus a defined valid placement volume.

`place("bowl")` should resolve to this interior region.

The Phase 1 `bowl_inner` region is cylindrical: radial range `0.0–0.05 m`, vertical range `0.0–0.035 m`, safe radius `0.045 m`, and rim clearance `0.02 m`. The collision model uses a bottom cylinder and eight wall boxes, leaving the center physically open.

---

## 41. Bowl Placement Verification

Successful placement into the bowl should eventually verify:

- the object is no longer held,
- the object center is within the valid interior region,
- the object is physically supported,
- the object is below the allowed rim threshold,
- and the object remains stable for a short validation window.

---

## 42. Pan Specification

Canonical ID:

```text
pan
```

Initial category:

```text
future_manipulable
```

Initial role:

```text
kitchen context
future destination
future handle-grasp task
```

Initial dynamics:

```text
static or fixed
```

---

## 43. Pan Geometry

Recommended visual geometry:

```text
detailed pan mesh
```

Recommended collision:

```text
shallow cylinder / primitive composition
+
handle capsule or box
```

The handle should eventually expose:

```text
pan_handle
```

The Phase 1 `pan_handle` site is object-local at `[0.09, 0.0, 0.045] m`.

semantic frame.

---

## 44. Pan Future Manipulation

Future pick behavior should use:

```text
pan_handle
```

rather than grasping an arbitrary body location.

Its future grasp profile may be:

```text
pan_handle_grasp
```

This behavior is not required in the first reliable pick-and-place milestone.

---

## 45. Initial Object Placement Strategy

Objects should be placed to satisfy:

```text
safe reachability
clear approach path
minimum object separation
edge clearance
visual realism
stable reset
```

The object layout should still look natural.

However, visual composition must not override manipulation safety.

---

## 46. Initial Object-to-Surface Mapping

The final exact mapping is configuration-driven.

A reasonable initial semantic distribution should ensure Panda 1 can test:

```text
cube
apple
purple_ball
bowl
```

without clutter.

The final object coordinates must be validated only after:

```text
rail reachability
+
arm IK
+
pre-grasp clearance
+
collision checks
```

pass.

---

## 47. Rail-Aware Reachability

Because each Panda is mounted on a linear rail, object reachability must consider:

```text
rail joint
+
7 Panda arm joints
```

A target should not be marked unreachable based only on the arm at rail home position.

Correct validation asks:

```text
Does a valid rail + arm configuration exist?
```

---

## 48. Object Workspace Metadata

The registry may define:

```text
preferred_workspace
```

Example:

```yaml
cube:
  preferred_workspace: panda1_primary_workspace
```

This does not permanently bind the object to one robot.

It describes its initial or preferred operational region.

---

## 49. Reachability Metadata

Optional runtime / precomputed metadata may include:

```text
reachable_by:
  - panda1
```

Later:

```text
reachable_by:
  - panda1
  - panda2
```

Such metadata must come from actual reachability analysis, not assumption.

---

## 50. Planning Scene Participation

Each object should declare:

```text
planning_scene_enabled
```

Initial functional objects should generally be:

```text
true
```

Decorative objects outside the manipulation region may be omitted from MoveIt if physically irrelevant.

---

## 51. Held Object Planning State

Before pick:

```text
apple → world collision object
```

After successful grasp:

```text
apple → attached collision object
```

After release:

```text
apple → world collision object
```

MoveIt representation must remain synchronized with MuJoCo state.

---

## 52. Object Contact State

The Scene State Provider should eventually summarize relevant contacts.

Example:

```yaml
contact_state:
  touching_left_finger: true
  touching_right_finger: true
  touching_support_surface: false
```

This supports hybrid grasp verification.

---

## 53. Grasp Verification Metadata

The registry may reference object-specific verification requirements through the grasp profile.

Example conditions:

```text
both-finger contact
object between fingers
finger separation in valid range
object velocity below threshold
```

The registry should not implement verification itself.

---

## 54. Object Reset Contract

Reset must restore:

```text
initial pose
initial orientation
zero linear velocity
zero angular velocity
initial support relation
held_by = null
container = initial value
temporary constraint = removed
```

Every dynamic object must satisfy this.

---

## 55. Object Reset Order

Recommended reset sequence:

```text
1. stop task execution
2. remove grasp constraints
3. reset robot / rail state
4. reset object poses
5. clear object velocities
6. step simulation for stabilization
7. validate support contacts
8. publish valid scene state
```

This avoids stale physical state.

---

## 56. Object Validity

Each object state includes:

```text
state_valid
```

Invalid examples:

```text
pose contains NaN
object penetrates furniture
object missing from MuJoCo
object outside expected world bounds
unresolved registry entry
```

Manipulation must refuse invalid object state.

---

## 57. Scene Integrity Monitoring

Task evaluation should track all relevant objects.

For a task:

```text
apple → bowl
```

unrelated objects include:

```text
cube
purple_ball
pan
```

If these move beyond configured tolerance without being part of the task, scene integrity may fail.

---

## 58. Object Disturbance Metrics

Benchmarking may record:

```text
translation disturbance
rotation disturbance
object fall count
unintended contact count
```

This supports a stronger definition of manipulation success.

---

## 59. Semantic Query Interface

The registry should eventually support operations conceptually similar to:

```text
get_object("apple")
list_objects()
list_pickable_objects()
list_containers()
resolve_alias("red apple")
```

The exact implementation belongs to the software layer.

---

## 60. Task Validation

Before executing:

```text
pick("apple")
```

the system should validate:

```text
object exists
object state valid
pickable = true
not already held
reachable candidate exists
```

Before executing:

```text
place("bowl")
```

the system should validate:

```text
target exists
place_target = true
valid placement region exists
robot currently holds an object
```

---

## 61. LLM Exposure

The LLM should receive a reduced semantic scene description.

Example:

```text
pickable_objects:
- cube
- apple
- purple_ball

place_targets:
- bowl
```

It does not need:

```text
mass tensors
collision geometry
raw MuJoCo IDs
```

High-level planning should consume only relevant semantic data.

---

## 62. New Object Addition Procedure

To add a new functional object:

```text
1. choose canonical ID
2. define category
3. define visual geometry
4. define collision geometry
5. define mass / inertia
6. define physics profile
7. define initial pose
8. define support surface
9. define semantic subframes
10. define grasp profile if pickable
11. define destination region if place target
12. register in MoveIt mapping
13. add reset test
14. add state validation
15. add benchmark if relevant
```

No object should be introduced only by dropping a mesh into the world.

---

## 63. Object Removal Procedure

Removing a registered object requires checking:

- scene configuration,
- location definitions,
- grasp profiles,
- tests,
- benchmarks,
- LLM examples,
- and documentation.

Canonical IDs should not be reused for unrelated future objects.

---

## 64. Object Registry Validation Test

An automated test should verify:

```text
all canonical IDs are unique
all configured objects exist
all visual assets resolve
all collision definitions resolve
all support surfaces exist
all grasp profiles resolve
all semantic subframes are valid
all place targets have valid locations
all dynamic objects have mass
all runtime objects map to registry entries
```

---

## 65. MuJoCo Consistency Test

For each registered physical object:

```text
registry ID
↕
MuJoCo body / model entity
```

must resolve successfully.

Missing or duplicate physical objects should block startup.

---

## 66. MoveIt Consistency Test

For each planning-relevant object:

```text
registry ID
↕
MoveIt collision object ID
```

should match.

Pose synchronization should be validated against MuJoCo state.

---

## 67. Config Schema Example

Illustrative structure:

```yaml
objects:

  cube:
    category: manipulable
    dynamic: true
    pickable: true
    place_target: false

    visual_asset: meshes/cube.obj

    collision:
      type: box
      size: [...]

    mass: ...

    initial:
      frame: world
      pose: ...
      support_surface: surface_left_1

    grasp_profile: cube_standard

    aliases:
      - cube
      - block

    planning_scene_enabled: true
    benchmark_enabled: true
```

Exact numeric values must be validated during implementation.

---

## 68. Bowl Schema Example

Illustrative:

```yaml
bowl:
  category: container
  dynamic: false
  pickable: false
  place_target: true

  visual_asset: meshes/bowl.obj

  collision:
    type: primitive_compound

  initial:
    frame: world
    pose: ...
    support_surface: surface_left_2

  semantic_frames:
    bowl_inner: ...
    bowl_rim: ...

  planning_scene_enabled: true
```

---

## 69. Apple Schema Example

Illustrative:

```yaml
apple:
  category: manipulable
  dynamic: true
  pickable: true
  place_target: false

  visual_asset: meshes/apple.obj

  collision:
    type: sphere
    radius: ...

  mass: ...

  initial:
    frame: world
    pose: ...
    support_surface: surface_left_1

  grasp_profile: apple_standard

  aliases:
    - apple
    - red apple

  planning_scene_enabled: true
  benchmark_enabled: true
```

---

## 70. Source-of-Truth Rule

If implementation code and `objects.yaml` disagree:

```text
objects.yaml wins
```

unless the configuration itself has been formally revised.

Code must not silently override registry values.

---

## 71. No Object-Specific Coordinate Hacks

Forbidden:

```python
if object_id == "apple":
    grasp_z += 0.031
```

Correct:

```text
apple
 ↓
apple_standard grasp profile
 ↓
semantic grasp transform
```

Object-specific behavior is allowed only when represented explicitly through metadata, geometry, or grasp profiles.

---

## 72. No Automatic Attachment Rule

Being registered as:

```text
pickable: true
```

does not permit instant attachment.

Correct flow:

```text
object lookup
→ physical approach
→ gripper close
→ contact verification
→ grasp validation
→ temporary stabilization
```

The registry describes capability, not execution shortcuts.

---

## 73. Versioning

The object registry should eventually expose:

```text
registry_version
```

Example:

```text
1.0
```

Changes should be classified as:

```text
semantic
physics
visual
benchmark
```

Renaming canonical IDs is a breaking semantic change.

---

## 74. Codex Rules

When Codex edits object-related code or configuration:

1. Read `OBJECT_REGISTRY.md`.
2. Read `SCENE_MAP.md`.
3. Read `COORDINATE_FRAMES.md`.
4. Do not invent canonical object IDs.
5. Do not duplicate object coordinates.
6. Do not add object-specific magic offsets.
7. Keep visual and collision geometry separate.
8. Keep runtime physical state owned by MuJoCo.
9. Resolve grasp behavior through grasp profiles.
10. Resolve destinations through the location registry.
11. Validate all references.
12. Preserve deterministic reset.
13. Update tests when a functional object changes.
14. Do not make static objects dynamic without documented reason.
15. Do not bypass grasp verification.

---

## 75. Initial Acceptance Criteria

The initial registry is accepted when:

- `cube` resolves correctly,
- `apple` resolves correctly,
- `purple_ball` resolves correctly,
- `bowl` resolves correctly,
- `pan` resolves correctly,
- all canonical IDs are unique,
- every object maps to a valid physical entity,
- all initial support surfaces resolve,
- pickable objects have grasp profiles,
- bowl exposes a valid interior region,
- MoveIt IDs match semantic IDs,
- reset reproduces initial object states,
- and runtime state queries are consistent.

---

## 76. Final Object Registry Principle

Every task-relevant object in Home Robotics must have a clear answer to:

```text
What is it?
What is its canonical ID?
Where is it?
What supports it?
Can it be picked?
Can it receive another object?
How is it grasped?
Which frame defines its semantics?
How is its runtime state validated?
```

If those answers are scattered across implementation code, the object architecture is incomplete.

The Object Registry exists so that simulation, manipulation, planning, benchmarking, and language reasoning all refer to the same physical object in the same way.
