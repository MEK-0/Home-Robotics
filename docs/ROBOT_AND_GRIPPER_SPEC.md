# Home Robotics — Robot and Gripper Specification

## 1. Purpose

This document defines the mechanical, kinematic, control, and behavioral contract for the robot systems used in Home Robotics.

The project contains two rail-mounted Franka Panda manipulators.

Each robot system is composed of:

```text
linear rail
+
rail carriage
+
7-DOF Franka Panda
+
standard Franka Hand
```

This document defines:

- robot identity,
- linear-rail behavior,
- Panda arm behavior,
- Franka Hand behavior,
- joint naming,
- limits,
- planning expectations,
- control boundaries,
- home states,
- TCP definition,
- grasp behavior,
- safety behavior,
- reset behavior,
- and dual-arm readiness.

Exact numeric parameters belong primarily in:

```text
config/robots.yaml
config/grasp_profiles.yaml
config/physics.yaml
```

---

## 2. Robot Systems

Canonical robot systems:

```text
panda1
panda2
```

Both robots share one fixed physical rail:

```text
shared_rail
```

The shared rail has two independent carriages:

```text
panda1_carriage
panda2_carriage
```

The carriage and arm joints are independently controlled while the physical rail support is shared and fixed.

---

## 3. Initial Activation State

During early project phases:

```text
Panda 1 + carriage 1 → active

Panda 2 + carriage 2 → present but inactive
```

The second robot remains fully modeled in the scene.

It should still contribute to:

- collision geometry,
- visual composition,
- workspace design,
- and future dual-arm planning preparation.

Panda 2 must not be removed simply because it is inactive.

---

## 4. Final Robot Architecture

The final project contains:

```text
Shared rail system
world
└── shared_rail
    ├── panda1_carriage
        └── panda1_base
            └── 7-DOF Panda
                └── Franka Hand
                    └── panda1_tcp
```

and:

```text
    └── panda2_carriage
        └── panda2_base
            └── 7-DOF Panda
                └── Franka Hand
                    └── panda2_tcp
```

---

## 5. Effective Degrees of Freedom

For motion planning, each active rail-mounted Panda has:

```text
1 rail prismatic DOF
+
7 Panda revolute DOF
=
8 planning DOF
```

The Franka Hand adds:

```text
2 finger joints
```

but the gripper is treated as a separate end-effector subsystem.

Therefore:

```text
rail + arm planning group
=
8 DOF

gripper group
=
2 finger joints
```

---

## 6. Linear Rail Purpose

The rail exists to extend the usable workspace of each Panda.

A fixed-base Panda cannot reliably cover all near, middle, and far table sections.

The rail allows the robot to reposition longitudinally while preserving the final scene layout.

The rail is therefore part of the robot, not part of the scenery.

---

## 7. Linear Rail Axis

Nominal rail direction:

```text
world X
```

With the project convention:

```text
X → forward
Y → left
Z → up
```

rail travel is approximately:

```text
±X
```

The exact rail frame is defined in `COORDINATE_FRAMES.md`.

---

## 8. Rail Joint

Canonical joint names:

```text
panda1_rail_joint
panda2_rail_joint
```

Joint type:

```text
prismatic
```

Each carriage prismatic joint must expose:

```text
position
velocity
effort / actuator state where applicable
```

to the control stack.

---

## 9. Rail Limits

Each carriage must define:

```text
lower position limit
upper position limit
home position
maximum velocity
maximum acceleration
actuation limit
```

These values belong in:

```text
config/robots.yaml
```

or the appropriate controller configuration.

No planner may command outside the valid rail range.

---

## 10. Rail Home Position

Each carriage has a distinct deterministic home position on the shared rail.

Conceptually:

```text
panda1_rail_home
panda2_rail_home
```

The home value does not have to equal:

```text
q = 0
```

The configuration must distinguish:

```text
mechanical zero
```

from:

```text
robot home position
```

---

## 11. Rail Motion Behavior

Rail motion must be:

- continuous,
- bounded,
- physically simulated,
- controlled through the robot-control layer,
- and visible in joint state.

The rail must never behave like:

```text
instant base teleportation
```

---

## 12. Base Translation Rule

The Panda base world pose is derived from the rail carriage.

Correct:

```text
rail joint moves
→ carriage moves
→ Panda base moves
```

Incorrect:

```text
task node writes Panda world X directly
```

Direct base teleportation is prohibited during runtime.

---

## 13. Coordinated Rail + Arm Planning

Normal manipulation planning should be capable of using:

```text
rail motion
+
arm motion
```

together.

Example:

```text
far object
   ↓
MoveIt searches 8-DOF configuration
   ↓
rail moves toward target region
+
arm assumes efficient grasp posture
```

The planner should not require a manually selected rail coordinate for every task.

---

## 14. Rail-Only Motion

Rail-only motions may be used intentionally for:

- staging,
- workspace transition,
- returning home,
- recovery,
- debugging,
- and future dual-arm scheduling.

Such motions must still use the normal robot-control interfaces.

---

## 15. Rail Collision Safety

Rail motion must account for:

```text
carriage ↔ furniture
carriage ↔ table structure
Panda ↔ furniture
Panda ↔ second Panda
held object ↔ scene
```

The rail cannot be planned independently of robot geometry when collision safety matters.

---

## 16. Franka Panda Model

The arm model is:

```text
Franka Emika Panda
```

with:

```text
7 revolute arm joints
```

The project should use a validated robot description rather than a hand-authored approximation.

Where practical, official or widely validated model geometry and joint limits should be preserved.

---

## 17. Panda Joint Naming

Internal Panda joint names should follow the validated Franka model.

Conceptually:

```text
panda_joint1
panda_joint2
panda_joint3
panda_joint4
panda_joint5
panda_joint6
panda_joint7
```

For the dual-robot system, namespace or prefixing must keep the two robots unambiguous.

Example conceptual identities:

```text
panda1_joint1
...
panda1_joint7

panda2_joint1
...
panda2_joint7
```

Exact naming should be determined consistently by the robot model.

---

## 18. Panda Joint Limits

The project must preserve validated:

```text
position limits
velocity limits
effort limits
```

for every arm joint.

Limits must not be expanded to make a target reachable.

If a target is unreachable:

```text
use the rail
change the task location
or report failure
```

Do not violate Panda limits.

---

## 19. Panda Home Configuration

Each robot defines one canonical home joint configuration.

Example:

```text
panda1_home
panda2_home
```

The home posture must be:

- collision-free,
- comfortably inside joint limits,
- clear of work surfaces,
- clear of the other robot,
- compatible with rail home state,
- and visually neutral.

The exact joint values belong in:

```text
config/robots.yaml
```

---

## 20. Home State Definition

A full robot home state includes:

```text
rail position
+
7 Panda arm joints
+
gripper state
```

Therefore:

```text
home
```

is not only an arm configuration.

---

## 21. Full Home State Example

Conceptually:

```yaml
panda1:
  rail:
    position: home
  arm:
    joints: [...]
  gripper:
    state: open
```

The real schema may differ.

---

## 22. Robot Reset

Reset must restore:

```text
rail position
rail velocity
arm joint positions
arm joint velocities
gripper opening
controller state
temporary grasp state
```

Reset should result in the same state every time.

---

## 23. Franka Hand

The gripper is:

```text
standard Franka Hand
```

Type:

```text
two-finger parallel gripper
```

The project intentionally avoids adding a third-party gripper during the core phases.

---

## 24. Gripper Joint Model

The hand contains two finger joints.

Conceptually:

```text
left finger
right finger
```

The validated Franka model should determine exact names and mechanical coupling behavior.

---

## 25. Gripper Opening

The gripper must define:

```text
maximum opening
minimum opening
```

using realistic Franka Hand values.

The project must not enlarge the gripper opening simply to accommodate an oversized object.

---

## 26. Gripper Commands

The control layer should expose semantic operations such as:

```text
open
close
move_to_width
```

where appropriate.

Higher-level task code should not directly command individual finger actuators unless required internally.

---

## 27. Gripper Control Boundary

Allowed flow:

```text
Manipulation Layer
      ↓
Gripper Controller
      ↓
ros2_control / MuJoCo
      ↓
finger joints
```

Prohibited:

```text
LLM
↓
MuJoCo finger actuator
```

---

## 28. Gripper Open State

The project should define one canonical open state.

This state should provide enough clearance for the initial object set:

```text
cube
apple
purple_ball
```

while respecting real gripper limits.

---

## 29. Gripper Close State

A close command does not necessarily mean:

```text
finger position = fully closed
```

The fingers should close until:

```text
target width reached
```

or:

```text
physical object contact prevents further closure
```

depending on controller implementation.

---

## 30. TCP

Each robot has a canonical tool-center-point:

```text
panda1_tcp
panda2_tcp
```

The TCP is used for:

- grasp targets,
- pre-grasp targets,
- Cartesian approach,
- place targets,
- lift planning,
- retreat planning,
- and pose validation.

---

## 31. TCP Location

Preferred physical meaning:

```text
center between fingertips
at the intended grasp reference plane
```

The exact hand-to-TCP transform must be validated against the actual Franka Hand geometry.

It must not be estimated visually.

---

## 32. Hand Frame vs TCP

The following are different concepts:

```text
panda1_hand
```

and:

```text
panda1_tcp
```

The hand frame belongs to the mechanical model.

The TCP is the manipulation reference.

---

## 33. Grasp Strategy

The project uses a hybrid grasp strategy.

Approved sequence:

```text
pre-grasp
↓
approach
↓
physical finger closure
↓
contact verification
↓
grasp verification
↓
temporary stabilization
↓
lift
```

The gripper must physically participate in every normal grasp.

---

## 34. Grasp Verification

A grasp should not be declared successful solely because:

```text
close command completed
```

Verification should consider evidence such as:

```text
left finger contact
right finger contact
finger separation
object position relative to gripper
relative object velocity
```

Exact thresholds belong in:

```text
config/grasp_profiles.yaml
```

---

## 35. Two-Finger Contact Preference

For standard objects, valid grasp should normally require meaningful contact from both fingers.

A one-sided incidental contact is generally not enough.

Exceptions must be documented in an object-specific grasp profile.

---

## 36. Finger Separation Verification

After closure, finger separation may help infer whether an object is actually between the fingers.

Example logic:

```text
fully closed with no object
→ likely failed grasp

non-zero stable separation
+
valid contacts
→ possible valid grasp
```

This is only one part of verification.

---

## 37. Object Between Fingers

The object center or grasp region should lie within an expected gripper volume.

This prevents false positives caused by:

```text
finger touching object from outside
```

without actually holding it.

---

## 38. Relative Velocity Check

Before stabilization, the object should not be rapidly moving relative to the hand.

A high relative velocity suggests an unstable or invalid grasp.

The threshold must be tuned experimentally.

---

## 39. Stabilization Activation

Only after verification passes may the system activate the temporary grasp stabilization constraint.

The constraint must preserve the actual grasp-relative transform.

No large object snap is allowed.

---

## 40. Gripper Does Not Own Stabilization

Conceptually:

```text
Franka Hand
→ creates physical grasp

Constraint Manager
→ stabilizes verified grasp
```

These responsibilities must remain separate.

The gripper controller itself should not silently weld objects.

---

## 41. Grasp Release

Approved release sequence:

```text
arrive at valid place pose
↓
lower object
↓
establish support
↓
open fingers
↓
remove stabilization
↓
allow settling
↓
verify placement
```

The exact ordering of finger opening and constraint removal may be experimentally tuned, but must avoid object snapping or launching.

---

## 42. Release Verification

Successful release requires:

```text
held_by = null
constraint inactive
finger separation increasing / open
object no longer following TCP rigidly
```

Then placement verification begins.

---

## 43. Gripper Failure Conditions

Possible failure states include:

```text
GRIPPER_OPEN_FAILED
GRIPPER_CLOSE_FAILED
NO_FINGER_CONTACT
INVALID_FINGER_SEPARATION
GRASP_VERIFICATION_FAILED
OBJECT_SLIPPED
OBJECT_DROPPED
RELEASE_FAILED
```

These should later map into the project failure taxonomy.

---

## 44. Collision Behavior

The robot must physically collide with:

```text
tables
objects
containers
second robot
rail structures
```

when geometry overlaps.

The robot model must not depend entirely on MoveIt for physical collision avoidance.

---

## 45. Planning Collision Geometry

The Panda, rail, carriage, hand, and held object must have planning collision representations consistent with the physical simulation.

Large mismatches between MuJoCo and MoveIt geometry are prohibited.

---

## 46. Arm-to-Rail Self-Collision

The valid robot configuration should not cause the Panda to collide with its own rail or carriage.

If this occurs repeatedly:

```text
model geometry
mount transform
joint limits
planning collision model
```

must be inspected.

Do not globally disable these collisions.

---

## 47. Rail-to-Furniture Collision

The carriage path must remain clear across its configured range.

A valid rail range must not drive the robot mount through:

```text
tables
counters
walls
other fixed assets
```

The rail range itself is therefore part of scene validation.

---

## 48. Robot-to-Robot Collision

Even before Panda 2 is active, Panda 1 must avoid it physically.

Later dual-arm operation must include:

```text
panda1 carriage joint
panda2 carriage joint
Panda 1
Panda 2
held objects
```

in cross-robot collision reasoning.

---

## 49. Motion Planning Group

The primary Panda 1 manipulation group should conceptually be:

```text
panda1_arm_with_rail
```

containing:

```text
panda1_rail_joint
+
panda1 arm joints 1–7
```

The same architecture applies later to:

```text
panda2_arm_with_rail
```

---

## 50. Arm-Only Planning Group

An optional arm-only group may also exist:

```text
panda1_arm_only
```

This is useful for:

- small local motions,
- debugging,
- controlled Cartesian approach,
- tests where rail must stay fixed.

The primary reachability logic should still understand the full rail + arm system.

---

## 51. Gripper Planning Group

The hand should have a separate group:

```text
panda1_hand
```

and later:

```text
panda2_hand
```

The exact MoveIt group definition is decided during MoveIt integration.

---

## 52. Reachability Definition

An object is considered reachable by a robot when a valid configuration exists for:

```text
rail position
+
arm joint configuration
+
required gripper pose
```

within:

```text
joint limits
rail limits
collision constraints
```

Reachability must not be evaluated only from rail home.

---

## 53. Preferred Rail Positioning

When several valid rail positions exist, the system should later prefer configurations that provide:

- comfortable Panda posture,
- joint-limit margin,
- collision margin,
- short total motion,
- good grasp orientation,
- and future task efficiency.

This may become an optimization objective.

---

## 54. Rail Position Should Not Be Object Metadata

Do not define:

```text
apple_rail_position = 0.72
```

as a permanent object property.

The same object may move.

Rail position is a planning result, not object identity.

---

## 55. Workspace Transition

For distant tasks, execution may intentionally contain:

```text
rail reposition
↓
arm positioning
↓
grasp
```

or a coordinated rail + arm trajectory.

The choice should be determined by planning strategy, not hard-coded per object.

---

## 56. Robot State Interface

Each active robot system should expose:

```text
rail position
rail velocity
arm joint positions
arm joint velocities
gripper state
TCP pose
controller state
held object
```

This forms the robot-level runtime state.

---

## 57. Gripper State Interface

The gripper state should support at least:

```text
opening width
finger positions
command state
contact status
grasp verified
```

Potential later fields:

```text
estimated force
slip state
```

---

## 58. Held Object State

The robot state may expose:

```text
held_object: apple
```

or:

```text
held_object: null
```

The canonical object state remains owned by the Scene State Provider.

---

## 59. Robot Readiness

A robot is ready for a task only if:

```text
rail controller ready
arm controller ready
gripper controller ready
robot state valid
no active fault
no stale grasp constraint
```

---

## 60. Task Start Safety

Before a pick or place starts, the system should validate:

```text
robot ready
rail state valid
arm state valid
gripper state valid
target state valid
planning scene synchronized
```

A task must not begin from unknown robot state.

---

## 61. Control Architecture

Expected control chain:

```text
MoveIt / Manipulation
        ↓
ROS 2 controllers
        ↓
ros2_control
        ↓
MuJoCo interface
        ↓
rail actuator
Panda actuators
gripper actuators
```

No layer should bypass this path during normal operation.

---

## 62. Controller Separation

Conceptually, controllers may include:

```text
rail / arm trajectory control
gripper control
joint state broadcaster
```

The exact controller architecture will be selected during ROS 2 integration.

The design should minimize unnecessary controller fragmentation.

---

## 63. Trajectory Execution

A valid robot trajectory may contain coordinated commands for:

```text
rail joint
+
Panda arm joints
```

The trajectory controller must preserve timing consistency across the complete chain.

---

## 64. Cartesian Motion

Cartesian motion is defined relative to:

```text
pandaN_tcp
```

Examples:

```text
approach
lift
lower
retreat
```

The underlying planner may still move the rail if allowed by the selected planning strategy.

---

## 65. Local Cartesian Approach

For the final short grasp approach, it may be desirable to keep the rail approximately fixed while moving primarily the arm.

This reduces unnecessary base motion close to the object.

Such behavior should be a documented planning policy rather than an uncontrolled assumption.

---

## 66. Lift Behavior

After grasp stabilization, the initial lift should prioritize:

```text
clear vertical separation
stable grasp
no rail jerk
no object-table scraping
```

The lift distance belongs in the grasp profile or manipulation configuration.

---

## 67. Rail Motion While Holding Object

Rail motion while holding an object is allowed.

However:

- acceleration should remain controlled,
- held-object collision must remain active,
- the temporary grasp constraint must remain valid,
- and transport motion should avoid unnecessary rail oscillation.

---

## 68. Safe Stop

The robot-control architecture should eventually support a safe stop behavior.

A safe stop should:

- stop trajectory execution,
- avoid uncontrolled rail motion,
- avoid uncontrolled arm motion,
- preserve or safely handle the held object,
- and report execution failure.

Exact safety semantics depend on the final controller integration.

---

## 69. Cancellation

ROS 2 Action cancellation should later propagate into robot execution.

A cancelled task must not leave:

```text
rail moving
arm moving indefinitely
gripper in unknown command state
stale object constraint
```

Recovery behavior will be defined in `FAILURE_AND_RECOVERY.md`.

---

## 70. Panda 2 Activation Requirements

Panda 2 must not become active simply by enabling its controller.

Before Phase 7 activation, the project must validate:

```text
namespace isolation
panda2_rail_joint control
arm 2 control
gripper 2 control
cross-robot planning scene
shared workspace rules
cross-robot collision
task ownership
```

---

## 71. Dual-Rail Coordination

In the dual-arm phase, both rail positions become shared planning state.

Possible conflicts include:

```text
both robots approaching shared workspace
crossing arm volumes
carriage proximity
held-object interaction
handover coordination
```

A later resource / coordination layer must handle these cases.

---

## 72. Handover Readiness

The robot architecture should support future handover tasks.

Conceptually:

```text
Panda 1 holds object
↓
moves to handover zone
↓
Panda 2 approaches
↓
Panda 2 verifies grasp
↓
ownership transfer
↓
Panda 1 releases
```

This is not implemented in early phases but should remain architecturally possible.

---

## 73. Robot Configuration Ownership

`config/robots.yaml` owns:

```text
robot IDs
rail transforms
rail limits
rail home states
robot activation states
Panda home joints
controller namespaces
TCP configuration references
workspace ownership
```

Do not duplicate these values in source code.

---

## 74. Proposed `robots.yaml` Structure

Illustrative:

```yaml
robots:

  panda1:

    active: true
    namespace: panda1

    rail:
      joint: panda1_rail_joint
      axis: [1.0, 0.0, 0.0]
      lower_limit: ...
      upper_limit: ...
      home_position: ...
      max_velocity: ...
      max_acceleration: ...

    arm:
      model: franka_panda
      home_joints: [...]

    gripper:
      model: franka_hand
      tcp_frame: panda1_tcp

    workspace:
      primary: panda1_primary_workspace
      shared: shared_workspace
```

Panda 2 follows the same schema.

---

## 75. No Magic Rail Positions

Forbidden:

```python
if object == "apple":
    rail_target = 0.63
```

unless this represents an explicit tested waypoint defined in configuration for a specific task design.

Preferred:

```text
target pose
↓
planning
↓
rail + arm solution
```

---

## 76. No Joint Limit Hacks

Forbidden:

```text
increase Panda joint limit
increase rail limit
```

to solve a reach problem without mechanical justification.

Correct responses include:

```text
use valid rail motion
change grasp candidate
change legal object location
report unreachable
```

---

## 77. No Gripper Scaling Hacks

Do not enlarge:

```text
finger length
finger opening
object collision radius
```

simply to produce a successful grasp.

Manipulation should work with realistic geometry.

---

## 78. Model Versioning

The project should eventually record versions for:

```text
Panda model
Franka Hand model
rail model
TCP definition
```

Benchmark results should identify the robot-model version when relevant.

---

## 79. Robot Validation Tests

Before pick-and-place, validate:

```text
rail state
rail range
rail hold
rail trajectory
Panda joint state
Panda home pose
Panda trajectory
gripper open
gripper close
TCP FK
rail + arm coordinated motion
```

---

## 80. Rail Range Test

For each rail:

```text
move near lower limit
move to center
move near upper limit
return home
```

Validate:

```text
correct direction
no collision
no overshoot
no drift
correct joint state
```

---

## 81. Rail Direction Test

Positive command must move the carriage along the documented positive rail axis.

Expected:

```text
Δq_rail > 0
→ carriage moves +X in rail frame
```

If not, the model must be fixed before planning integration.

---

## 82. Panda FK Test

At a known joint configuration:

```text
MuJoCo TCP pose
```

must agree with:

```text
MoveIt / TF TCP pose
```

within tolerance.

This validates the robot model across layers.

---

## 83. Coordinated Planning Test

A target intentionally outside fixed-base reach but inside rail-assisted reach should be used.

Expected:

```text
planner finds valid rail + arm solution
```

This is a core acceptance test for the rail architecture.

---

## 84. Gripper Open/Close Test

Repeated cycles:

```text
open
close
open
close
```

should produce:

```text
consistent finger motion
no oscillation
no asymmetric drift
no invalid contact
```

---

## 85. Empty-Gripper Close Test

When no object is present:

```text
close gripper
```

must not produce a false grasp state.

Expected:

```text
grasp_verified = false
```

---

## 86. Cube Grasp Test

The cube is the first object used for physical grasp validation.

Required sequence:

```text
approach
close
verify two-finger contact
verify object between fingers
activate stabilization
lift
```

---

## 87. False Grasp Test

The robot should intentionally close slightly away from the object.

Expected:

```text
grasp verification fails
constraint remains disabled
lift is not executed
```

This protects against magical attachment.

---

## 88. Release Test

With a verified held cube:

```text
lower to valid surface
open gripper
remove stabilization
retreat
```

Expected:

```text
object remains on support
held state clears
no object launch
```

---

## 89. Robot Acceptance Criteria — Phase 2

The robot-control stack is accepted when:

- panda1_rail_joint can be commanded reliably.
- Panda 1 arm can be commanded reliably.
- Franka Hand can open and close reliably.
- Joint states are correct.
- TF is correct.
- TCP pose is consistent.
- Rail + arm coordinated motion is possible.
- Robot returns to deterministic home.
- Panda 2 remains physically present and inactive.
- No direct base teleportation is used.

---

## 90. Gripper Acceptance Criteria — Phase 4

The gripper/manipulation interface is accepted when:

- physical finger contact is measurable,
- empty closure is not considered a grasp,
- valid cube grasp is correctly detected,
- stabilization activates only after verification,
- object transport is stable,
- release removes stabilization,
- and placement does not launch or drag the object.

---

## 91. Codex Rules

Codex must:

1. Treat each rail as a first-class robot joint.
2. Never teleport Panda bases.
3. Preserve Panda joint limits.
4. Preserve realistic gripper limits.
5. Use the standard Franka Hand.
6. Keep Panda 1 and Panda 2 namespaces isolated.
7. Keep rail state inside normal robot state.
8. Prefer coordinated rail + arm planning.
9. Do not hard-code object-specific rail positions.
10. Use the canonical TCP.
11. Never treat gripper-close completion as grasp success.
12. Require grasp verification before stabilization.
13. Keep collision geometry active.
14. Restore rail, arm, and gripper state on reset.
15. Update tests when robot behavior changes.

---

## 92. Prohibited Patterns

The following are explicitly prohibited:

```text
manual Panda base teleportation
rail motion outside configured limits
expanded Panda joint limits
instantaneous gripper closure
instant object attachment
object-specific rail magic numbers
duplicate robot namespaces
direct LLM actuator commands
collision disabling to force success
unverified TCP offsets
```

---

## 93. Relationship to Other Documents

`COORDINATE_FRAMES.md`

defines:

```text
rail, carriage, base, hand, and TCP frames
```

`MUJOCO_PHYSICS_SPEC.md`

defines:

```text
rail dynamics
joint dynamics
gripper contact
constraint physics
```

`PICK_PLACE_SPEC.md`

defines:

```text
how the robot uses these capabilities to manipulate objects
```

`ROS2_ARCHITECTURE.md`

defines:

```text
controllers, topics, actions, and state interfaces
```

`FAILURE_AND_RECOVERY.md`

defines:

```text
what happens when robot or gripper execution fails
```

---

## 94. Final Robot Principle

The Home Robotics robot is not simply a 7-DOF Panda mounted at a fixed point.

It is a rail-mounted manipulation system:

```text
linear mobility
+
articulated manipulation
+
physical two-finger grasping
```

The complete robot state is therefore:

```text
rail
+
arm
+
gripper
+
TCP
+
held-object state
```

All planning, control, reset, testing, and later dual-arm coordination must respect this full system.

The core rule is:

> Reach should come from valid rail-and-arm motion, not from changing the scene or violating robot limits.
