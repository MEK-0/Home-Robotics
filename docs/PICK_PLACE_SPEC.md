# Home Robotics — Pick and Place Specification

## 1. Purpose

This document defines the complete behavioral contract for reliable pick-and-place execution in the Home Robotics project.

It specifies:

- task preconditions,
- object and target resolution,
- rail-aware reachability,
- grasp candidate generation,
- pre-grasp planning,
- approach motion,
- physical gripper closure,
- grasp verification,
- temporary stabilization,
- lift,
- transport,
- placement,
- release,
- placement verification,
- retreat,
- recovery,
- failure handling,
- scene-integrity checks,
- and success criteria.

This document is a core implementation contract.

The goal is to prevent pick-and-place from becoming a collection of ad-hoc motions or object-specific hacks.

The intended behavior is:

```text
resolve
→ validate
→ plan
→ approach
→ physically grasp
→ verify
→ stabilize
→ lift
→ transport
→ place
→ release
→ verify
→ retreat
```

---

## 2. Scope

This specification applies to:

```text
panda1
```

during the initial reliable-manipulation phase.

The same structure will later be extended to:

```text
panda2
```

and dual-arm execution.

The initial supported pickable objects are:

```text
cube
apple
purple_ball
```

The initial supported destination is:

```text
bowl
```

Additional destinations may later be added through the location registry.

---

## 3. Architectural Boundary

Pick-and-place belongs to the Manipulation Layer.

The manipulation layer may use:

```text
Object Registry
Scene State Provider
Location Registry
Grasp Profiles
MoveIt 2
Robot Control
Gripper Control
Constraint Manager
```

It must not use:

```text
LLM-generated joint values
hard-coded scene coordinates
direct MuJoCo actuator commands
direct Panda base teleportation
```

---

## 4. Public Task Semantics

The task-level operations are conceptually:

```text
pick(object_name)
place(location_name)
```

Example:

```text
pick("apple")
place("bowl")
```

The task caller specifies semantics.

The manipulation layer determines physical execution.

---

## 5. Pick Preconditions

Before starting a pick task, validate:

```text
robot ready
rail controller ready
arm controller ready
gripper controller ready
scene state valid
planning scene synchronized
object exists
object pickable
object state valid
object not already held
no stale grasp constraint
```

If any precondition fails, no motion should begin.

---

## 6. Place Preconditions

Before starting a place task, validate:

```text
robot ready
robot currently holds object
target exists
target supports placement
target state valid
planning scene synchronized
held object state valid
grasp stabilization state consistent
```

If any precondition fails, placement must not begin.

---

## 7. Pick State Machine

Canonical pick state machine:

```text
IDLE
  ↓
PICK_VALIDATE
  ↓
OBJECT_LOOKUP
  ↓
SCENE_SYNC
  ↓
GRASP_CANDIDATE_GENERATION
  ↓
REACHABILITY_EVALUATION
  ↓
PRE_GRASP_PLAN
  ↓
MOVE_PRE_GRASP
  ↓
APPROACH_PLAN
  ↓
APPROACH
  ↓
GRIPPER_CLOSE
  ↓
GRASP_VERIFY
  ↓
GRASP_STABILIZE
  ↓
LIFT_PLAN
  ↓
LIFT
  ↓
HELD_OBJECT_VERIFY
  ↓
PICK_SUCCESS
```

Any state may transition to a failure state when its acceptance condition is not met.

---

## 8. Place State Machine

Canonical place state machine:

```text
OBJECT_HELD
  ↓
PLACE_VALIDATE
  ↓
TARGET_LOOKUP
  ↓
SCENE_SYNC
  ↓
PLACE_CANDIDATE_GENERATION
  ↓
REACHABILITY_EVALUATION
  ↓
PRE_PLACE_PLAN
  ↓
MOVE_PRE_PLACE
  ↓
PLACE_APPROACH_PLAN
  ↓
LOWER
  ↓
SUPPORT_CHECK
  ↓
GRIPPER_OPEN
  ↓
REMOVE_STABILIZATION
  ↓
SETTLING_WINDOW
  ↓
PLACEMENT_VERIFY
  ↓
RETREAT_PLAN
  ↓
RETREAT
  ↓
PLACE_SUCCESS
```

---

## 9. Object Lookup

The object must be resolved through the Object Registry.

Example:

```text
apple
```

The manipulation layer must obtain:

```text
canonical ID
current world pose
object geometry
grasp profile
support surface
held state
contact state
```

No object coordinate should be invented locally.

---

## 10. Target Lookup

A place target must be resolved through the location system.

Example:

```text
bowl
```

should resolve to:

```text
bowl_inner
+
valid placement volume
+
approach constraints
```

The bowl body origin itself is not the placement target.

---

## 11. Scene Synchronization

Before planning, the MoveIt Planning Scene should reflect the current MuJoCo state.

Required synchronization includes:

```text
object pose
static obstacles
container pose
held-object state
second robot geometry
```

Planning against stale object geometry is not allowed.

---

## 12. Rail-Aware Reachability

Reachability is evaluated over:

```text
1 rail joint
+
7 Panda arm joints
```

The system must answer:

```text
Does a valid rail + arm configuration exist?
```

not:

```text
Can the fixed-base arm reach from rail home?
```

---

## 13. Reachability Evaluation

For each grasp or place candidate, evaluate:

```text
rail limits
Panda joint limits
IK validity
self-collision
environment collision
other-robot collision
pre-grasp feasibility
approach feasibility
lift / retreat feasibility
```

A candidate is valid only if the complete manipulation segment is feasible.

---

## 14. Grasp Candidate Generation

Grasp candidates must come from:

```text
object geometry
+
object-local semantic frames
+
grasp profile
```

They must not come from unexplained world-coordinate offsets.

Conceptually:

```text
T_world_grasp
=
T_world_object
×
T_object_grasp
```

---

## 15. Candidate Ranking

When several grasp candidates are valid, ranking may consider:

```text
collision margin
joint-limit margin
rail travel
arm travel
orientation quality
approach clearance
gripper compatibility
task continuation cost
```

The first implementation may use a deterministic ranking policy.

---

## 16. Cube Grasp Candidates

The cube is the baseline object.

Candidate grasps may include:

```text
left-right face grasp
front-back face grasp
top-access side grasp
```

depending on scene orientation and gripper clearance.

The project should prefer simple face-aligned grasps first.

---

## 17. Apple Grasp Candidates

Apple candidates may include:

```text
side grasp
slightly elevated side grasp
```

The grasp profile should respect approximately spherical geometry.

Do not create a special world-space apple offset.

---

## 18. Purple Ball Grasp Candidates

The purple ball is rotationally symmetric.

Candidate generation may therefore focus on:

```text
approach direction
gripper clearance
scene collision
```

rather than object yaw.

The ball should be treated as a harder contact case than the cube.

---

## 19. Grasp Orientation

Gripper orientation must be selected intentionally.

A valid grasp orientation should consider:

```text
finger closing direction
surface normal
approach direction
wrist clearance
table clearance
nearby objects
```

Orientation must not be inherited blindly from the object's visual mesh.

---

## 20. Pre-Grasp Pose

Every grasp has a corresponding pre-grasp pose.

Conceptually:

```text
T_world_pregrasp
=
T_world_grasp
×
T_grasp_pregrasp
```

The pre-grasp offset comes from the grasp profile.

---

## 21. Pre-Grasp Clearance

The pre-grasp pose should provide enough clearance to:

```text
avoid accidental object contact
avoid table contact
allow controlled final approach
```

The distance should be object / grasp-profile dependent.

---

## 22. Rail Policy During Pre-Grasp

Rail motion is allowed during motion to pre-grasp.

MoveIt may use coordinated:

```text
rail
+
arm
```

motion.

The planner should seek a comfortable configuration rather than forcing extreme Panda posture.

---

## 23. Rail Policy During Final Approach

During the short final approach, the preferred policy is:

```text
rail approximately fixed
arm performs local approach
```

unless the target is impossible without small coordinated rail motion.

This reduces moving-base disturbance near the object.

---

## 24. Move to Pre-Grasp

The trajectory to pre-grasp must be:

```text
collision-free
within rail limits
within arm limits
free of unrelated-object contact
```

Execution success must be confirmed before approach begins.

---

## 25. Pre-Grasp Pose Verification

After execution, verify that the TCP reached the expected pose within tolerance.

Do not assume controller success automatically means geometric success.

---

## 26. Approach Motion

The approach is a controlled local motion from:

```text
pre-grasp
```

to:

```text
grasp pose
```

It should follow the grasp profile's approach direction.

---

## 27. Approach Safety

During approach:

```text
gripper open
rail preferably stationary
speed reduced
collision state monitored
```

The robot should not contact unrelated objects.

---

## 28. Approach Completion

Approach completes when:

```text
TCP reaches grasp pose tolerance
```

and:

```text
no unacceptable collision occurred
```

Then gripper closure begins.

---

## 29. Gripper Closure

The Franka Hand should physically close around the object.

Closure must be controlled.

It must not:

```text
teleport fingers
penetrate object
instantly activate attachment
```

---

## 30. Closure Completion Is Not Grasp Success

A completed close command means only:

```text
gripper motion completed
```

It does not mean:

```text
object grasped
```

Grasp verification is mandatory.

---

## 31. Grasp Verification Inputs

Possible verification signals:

```text
left finger contact
right finger contact
finger separation
object relative position
object relative velocity
contact consistency
object inside expected grasp volume
```

The exact rule belongs to the grasp profile and manipulation implementation.

---

## 32. Minimum Verification Policy

For the initial parallel-gripper objects, a valid grasp should generally require:

```text
meaningful left finger contact
+
meaningful right finger contact
+
object between fingers
+
finger separation within expected range
```

---

## 33. False Grasp Prevention

The following must not be considered valid:

```text
finger touches outer side of object
but object is not between fingers
```

or:

```text
gripper closes fully with no object
```

or:

```text
single incidental contact only
```

---

## 34. Grasp Verification Failure

If verification fails:

```text
do not stabilize
do not lift
```

Recommended initial behavior:

```text
open gripper
↓
retreat to pre-grasp
↓
return GRASP_VERIFICATION_FAILED
```

Bounded retry may be added later.

---

## 35. Grasp Stabilization

After valid physical grasp:

```text
temporary stabilization constraint
```

may be enabled.

Purpose:

```text
preserve a real verified grasp during transport
```

Not:

```text
create a grasp artificially
```

---

## 36. Stabilization Relative Transform

Constraint activation must preserve the actual verified object-to-gripper transform.

The object must not visibly snap.

Excessive attachment correction must fail.

---

## 37. Stabilization Verification

After activation, verify:

```text
constraint active
object pose remains continuous
no large position jump
no large orientation jump
```

If not, abort before lift.

---

## 38. Lift Planning

After successful stabilization, generate a lift motion.

Preferred initial behavior:

```text
primarily +Z in world
```

or another environment-safe vertical direction consistent with the scene.

The exact lift clearance belongs to the grasp profile.

---

## 39. Lift Purpose

The lift must:

```text
clear support surface
avoid scraping
confirm object follows gripper
create transport clearance
```

---

## 40. Lift Verification

After lift:

```text
object held_by == panda1
constraint active
object moved with TCP
object clear of original support
no unintended collision
```

must be verified.

---

## 41. Object Drop During Lift

If the object is no longer following the gripper:

```text
OBJECT_DROPPED
```

The system must not continue to transport.

---

## 42. Pick Success

A pick is successful only when:

```text
object physically left support surface
grasp verified
stabilization active
object follows gripper
robot is in safe lifted state
```

Then:

```text
pick(object)
```

returns success.

---

## 43. Transport

Transport moves the held object toward the destination region.

It may use:

```text
rail motion
+
arm motion
```

as needed.

---

## 44. Held-Object Collision

During transport, the held object becomes part of the effective robot collision geometry.

MoveIt must account for it.

MuJoCo physical collision remains active.

---

## 45. Transport Safety

Transport must avoid:

```text
tables
containers before intended approach
unrelated objects
Panda 2
rail structures
scene furniture
```

---

## 46. Rail Motion During Transport

Rail movement while holding an object is permitted.

Preferred behavior:

```text
smooth
bounded acceleration
no unnecessary oscillation
no abrupt reversal
```

This reduces disturbance to the grasp.

---

## 47. Place Candidate Generation

A semantic destination may correspond to many valid object poses.

For a bowl:

```text
candidate object poses inside bowl_inner
```

should be generated.

The target is a region, not one magic XYZ coordinate.

---

## 48. Place Candidate Requirements

A valid candidate should satisfy:

```text
inside target region
collision-safe approach
valid object orientation
sufficient gripper clearance
support stability
reachable rail + arm configuration
```

---

## 49. Bowl Placement Strategy

Initial bowl placement should prioritize:

```text
centered placement
clear rim clearance
moderate descent
stable support
```

The object should not be dropped from an unnecessarily large height.

---

## 50. Pre-Place Pose

A pre-place pose should be generated above or before the final target.

Conceptually:

```text
T_world_pre_place
=
T_world_place
×
T_place_pre_place
```

The offset must be semantically defined.

---

## 51. Move to Pre-Place

This motion may use full:

```text
rail + arm planning
```

The held object must remain included in collision checking.

---

## 52. Final Place Approach

The final lowering / approach should be slow and controlled.

Preferred:

```text
rail stationary
arm performs local descent
```

unless the planner requires a small coordinated adjustment.

---

## 53. Support Check

Before release, the system should determine whether the object is sufficiently close to / contacting a valid support.

For bowl placement:

```text
object within interior
+
valid lower support or stable containment geometry
```

should be present.

---

## 54. No Mid-Air Release

Normal place behavior must not intentionally release an object far above the destination.

A small controlled drop may be acceptable only if explicitly defined by the place profile.

Baseline policy:

```text
lower first
release second
```

---

## 55. Gripper Opening During Place

The gripper should open gradually / normally.

Opening must not generate a large impulse.

---

## 56. Stabilization Removal

The temporary constraint must be removed as part of release.

The sequence should avoid:

```text
object snapping to bowl
object being dragged by opening fingers
object being launched
```

---

## 57. Release Sequence

Recommended baseline:

```text
lower into supported pose
↓
confirm support / containment
↓
begin gripper opening
↓
remove stabilization
↓
finish opening
↓
hold robot briefly
↓
allow object settling
```

Exact sequencing may be tuned experimentally.

---

## 58. Settling Window

After release, the robot should remain temporarily still while the object settles.

During this window:

```text
no immediate retreat through object
no rail motion
no re-grasp
```

---

## 59. Placement Verification

A place is successful only if:

```text
held_by == null
constraint inactive
object in valid destination region
object state valid
object stable
scene integrity preserved
```

---

## 60. Bowl Verification

For:

```text
place("bowl")
```

verify:

```text
object center inside bowl valid volume
object below allowed rim threshold
object not held
object stable
object not penetrating invalid bowl geometry
```

---

## 61. Stability Threshold

After placement, the object should remain within velocity thresholds for a configured verification period.

This prevents declaring success while the object is still bouncing or rolling out.

---

## 62. Retreat

After placement verification:

```text
TCP retreats away from target
```

The retreat direction should avoid:

```text
bowl rim
placed object
nearby objects
```

---

## 63. Rail During Retreat

The short local retreat should preferably use the arm.

Rail repositioning may occur after safe clearance is established.

---

## 64. Place Success

A place is successful only after:

```text
release verified
placement verified
retreat successful
scene integrity valid
```

Then:

```text
place(location)
```

returns success.

---

## 65. Scene Integrity

Every manipulation task must preserve unrelated scene objects.

For task:

```text
apple → bowl
```

unrelated objects may include:

```text
cube
purple_ball
pan
```

Significant unintended displacement counts as task degradation or failure.

---

## 66. Scene Snapshot

Before execution, record relevant object states.

After execution, compare unrelated objects.

Possible checks:

```text
position displacement
orientation displacement
fall state
unexpected contact
```

---

## 67. Unintended Object Contact

Not every incidental light contact must necessarily fail immediately.

The project should distinguish:

```text
minor acceptable contact
```

from:

```text
meaningful unintended disturbance
```

Thresholds should be explicit and benchmarkable.

---

## 68. Forbidden Success Pattern

This is not a valid success:

```text
apple successfully in bowl
+
purple_ball knocked off table
```

Task success requires scene integrity.

---

## 69. Pick Failure Codes

Recommended pick failure categories:

```text
ROBOT_NOT_READY
OBJECT_NOT_FOUND
OBJECT_NOT_PICKABLE
INVALID_OBJECT_STATE
SCENE_SYNC_FAILED
NO_VALID_GRASP
TARGET_UNREACHABLE
PRE_GRASP_PLANNING_FAILED
PRE_GRASP_EXECUTION_FAILED
APPROACH_FAILED
GRIPPER_CLOSE_FAILED
GRASP_VERIFICATION_FAILED
GRASP_STABILIZATION_FAILED
LIFT_FAILED
OBJECT_DROPPED
COLLISION_DETECTED
TIMEOUT
```

---

## 70. Place Failure Codes

Recommended place failure categories:

```text
NO_OBJECT_HELD
LOCATION_NOT_FOUND
INVALID_LOCATION_STATE
SCENE_SYNC_FAILED
NO_VALID_PLACE_POSE
TARGET_UNREACHABLE
PRE_PLACE_PLANNING_FAILED
PRE_PLACE_EXECUTION_FAILED
PLACE_APPROACH_FAILED
SUPPORT_NOT_ESTABLISHED
GRIPPER_OPEN_FAILED
STABILIZATION_RELEASE_FAILED
PLACEMENT_VERIFICATION_FAILED
RETREAT_FAILED
OBJECT_DROPPED
COLLISION_DETECTED
TIMEOUT
```

---

## 71. Failure Does Not Mean Continue

If a critical state fails:

```text
do not continue the nominal state machine
```

Example:

```text
GRASP_VERIFY failed
```

must not transition to:

```text
LIFT
```

---

## 72. Initial Recovery Philosophy

Early recovery should be deterministic and conservative.

Example:

```text
grasp failed
↓
open gripper
↓
retreat
↓
return failure
```

Do not hide systematic bugs with repeated automatic retries.

---

## 73. Bounded Retry

A later version may allow:

```text
max_retries = N
```

for selected recoverable failures.

Retries must:

```text
change something meaningful
```

such as choosing another grasp candidate.

Repeating the identical failed trajectory is not useful recovery.

---

## 74. Rail Recovery

If rail planning or execution fails:

```text
stop safely
validate rail state
return to known safe configuration if possible
```

Do not teleport the carriage.

---

## 75. Collision Recovery

If unexpected collision occurs:

```text
stop execution
do not push through
retreat if safe
report collision
```

Collision recovery should never blindly continue forward.

---

## 76. Dropped Object Recovery

If an object is dropped:

```text
stop task
remove stale constraint if any
update scene state
report OBJECT_DROPPED
```

Automatic re-pick may be added only later.

---

## 77. Constraint Failure Recovery

If the stabilization constraint cannot activate cleanly:

```text
keep object physically grasped if safe
do not lift
open gripper if necessary
retreat
report failure
```

---

## 78. Timeout Policy

Every major execution stage should eventually have a timeout.

Examples:

```text
planning timeout
trajectory timeout
gripper timeout
settling timeout
verification timeout
```

Timeout values must be configuration-driven.

---

## 79. Cancellation

If the task is cancelled:

```text
stop active trajectory
stabilize robot state
handle held object safely
clear / preserve constraint intentionally
return cancelled result
```

Exact cancellation recovery belongs in `FAILURE_AND_RECOVERY.md`.

---

## 80. Speed Policy

Manipulation should use phase-appropriate speed.

Examples:

```text
workspace transit
→ moderate

final approach
→ slow

gripper close
→ controlled

lift
→ controlled

final place descent
→ slow
```

The project should not use one global aggressive speed for all motions.

---

## 81. Acceleration Policy

Abrupt accelerations should be avoided, especially:

```text
while grasping
while moving rail
while transporting round objects
near containers
```

Smooth motion improves physical stability.

---

## 82. Joint-Limit Margin

Candidate selection should prefer solutions away from joint limits when possible.

A barely valid extreme posture is less desirable than a small rail reposition.

---

## 83. Rail-Limit Margin

Likewise, avoid operating exactly at rail limits unless required.

The planner should prefer some travel margin when possible.

---

## 84. Collision Margin

The planner should avoid trajectories that merely skim collision boundaries.

Robust manipulation benefits from meaningful clearance.

---

## 85. Cube Baseline Task

The first reliable pick-and-place task should be:

```text
cube
→ safe table destination
```

or:

```text
cube
→ bowl
```

after bowl placement is validated.

The cube is the baseline object for the complete state machine.

---

## 86. Apple Task

After cube reliability:

```text
apple
→ bowl
```

becomes the primary semantic demonstration task.

This task should validate:

```text
rounder object grasp
semantic target
container placement
scene integrity
```

---

## 87. Purple Ball Task

The purple ball should be introduced after cube and apple.

It tests:

```text
rolling-object handling
symmetric grasping
friction sensitivity
placement stability
```

---

## 88. Reliability Target

A single successful run is not sufficient.

The project should benchmark repeated execution.

Example future targets:

```text
50 trials
100 trials
```

per task configuration.

Exact acceptance thresholds belong in research / benchmarking documentation.

---

## 89. Pick Metrics

Suggested metrics:

```text
pick success rate
grasp candidate success rate
planning success rate
grasp verification failure rate
object drop rate
average pick time
average planning time
rail travel distance
arm path length
```

---

## 90. Place Metrics

Suggested metrics:

```text
place success rate
placement verification rate
release failure rate
container miss rate
average place time
placement position error
settling time
```

---

## 91. Scene Integrity Metrics

Suggested:

```text
unrelated object displacement
unintended contact count
object fall count
collision count
```

---

## 92. End-to-End Task Metric

For:

```text
pick("apple")
place("bowl")
```

end-to-end success requires both operations to succeed within one task episode.

---

## 93. Deterministic Baseline

Baseline pick-place experiments must use:

```text
fixed initial poses
fixed physics
fixed scene version
fixed grasp profiles
fixed planner configuration
```

Randomization belongs to later research phases.

---

## 94. Pick-Place Logging

Each task should eventually log:

```text
task_id
robot
object
target
scene_version
physics_version
grasp_profile
start_time
planning result
rail motion
arm motion
gripper result
grasp verification
constraint activation
placement verification
scene integrity
final result
failure code
```

---

## 95. Task Trace

A useful trace may look like:

```text
task_0042
PICK_VALIDATE        OK
OBJECT_LOOKUP        OK
GRASP_CANDIDATES     4
REACHABLE            2
PRE_GRASP_PLAN       OK
MOVE_PRE_GRASP       OK
APPROACH             OK
GRIPPER_CLOSE        OK
GRASP_VERIFY         OK
GRASP_STABILIZE      OK
LIFT                 OK
PICK_SUCCESS         OK
```

This is important for debugging and academic evaluation.

---

## 96. No Magic Coordinates

Forbidden:

```python
if object == "apple":
    x += 0.03
    z += 0.07
```

Correct:

```text
object pose
+
grasp profile
+
frame transform
```

---

## 97. No Magic Rail Positions

Forbidden:

```python
if target == "surface_left_3":
    rail = 0.82
```

as an undocumented shortcut.

Correct:

```text
target pose
↓
rail-aware planning
↓
valid configuration
```

---

## 98. No Instant Attachment

Forbidden:

```text
pick requested
↓
object weld enabled
```

Correct:

```text
physical grasp
↓
verification
↓
stabilization
```

---

## 99. No Collision Bypass

Forbidden:

```text
disable table collision during approach
```

or:

```text
ignore held-object collision during transport
```

Collision problems must be solved correctly.

---

## 100. No Base Teleportation

Forbidden:

```text
move Panda base near object
```

Correct:

```text
command rail joint
```

The rail is the only valid mechanism for base translation.

---

## 101. No LLM Motion Generation

The LLM may request:

```text
pick("apple")
```

It must never provide:

```text
rail = 0.4
joint2 = -0.8
tcp = [x, y, z]
```

Low-level execution remains deterministic.

---

## 102. Codex Implementation Rules

Codex must:

1. Read this file before implementing pick/place.
2. Use the Object Registry.
3. Use the Location Registry.
4. Use frame transforms.
5. Use rail-aware planning.
6. Never teleport Panda bases.
7. Keep the final approach controlled.
8. Require physical gripper closure.
9. Require grasp verification.
10. Stabilize only after verification.
11. Include held object in collision planning.
12. Verify release.
13. Verify placement.
14. Check scene integrity.
15. Return structured failure codes.
16. Avoid object-specific magic numbers.
17. Avoid infinite retries.
18. Keep all thresholds configuration-driven.
19. Preserve deterministic baseline behavior.
20. Add tests for every new manipulation state.

---

## 103. Pick Acceptance Criteria

Pick is accepted when:

```text
object resolved
valid grasp selected
rail + arm target reachable
pre-grasp reached
approach completed
gripper physically closed
grasp verified
constraint activated without snap
object lifted
object remains held
scene integrity preserved
```

---

## 104. Place Acceptance Criteria

Place is accepted when:

```text
target resolved
valid placement pose selected
rail + arm target reachable
pre-place reached
controlled descent completed
support established
gripper opened
constraint removed
object settles
object inside valid target region
retreat succeeds
scene integrity preserved
```

---

## 105. Phase 4 Exit Criteria

Phase 4 should not be considered complete because one demo works once.

At minimum:

```text
cube pick/place repeatable
apple pick/place repeatable
purple_ball basic grasp validated
bowl placement physically valid
false grasp rejected
object drop detected
constraint failure handled
unrelated-object disturbance monitored
```

Repeated-trial thresholds are defined in the benchmarking documentation.

---

## 106. Relationship to Other Documents

`OBJECT_REGISTRY.md`

defines:

```text
what objects are
```

`COORDINATE_FRAMES.md`

defines:

```text
how poses and transforms are expressed
```

`ROBOT_AND_GRIPPER_SPEC.md`

defines:

```text
how rail, arm, and gripper behave
```

`MUJOCO_PHYSICS_SPEC.md`

defines:

```text
physical contact and stabilization rules
```

`ROS2_ARCHITECTURE.md`

defines:

```text
how execution is exposed through ROS 2
```

`FAILURE_AND_RECOVERY.md`

defines:

```text
detailed recovery behavior
```

---

## 107. Final Pick-and-Place Principle

A valid pick-and-place task is not:

```text
move to object
close gripper
move to target
open gripper
```

It is:

```text
understand current scene
↓
find a valid rail + arm configuration
↓
approach safely
↓
form a real physical grasp
↓
verify it
↓
stabilize it
↓
lift and transport safely
↓
place into a valid semantic region
↓
release physically
↓
verify final state
↓
preserve the rest of the scene
```

The final rule is:

> A task is successful only when the requested object reaches the requested destination through physically valid, verified, collision-aware execution without unacceptable disturbance to the rest of the scene.
