# Home Robotics — Failure and Recovery

## 1. Purpose

This document defines how Home Robotics detects, classifies, reports, and recovers from failures.

The objective is not to pretend failures do not happen.

The objective is to make failures:

```text
visible
structured
reproducible
recoverable when safe
```

---

## 2. Failure Principle

A failed state must never silently transition into nominal execution.

Example:

```text
grasp verification failed
```

must not continue into:

```text
lift
```

---

## 3. Failure Categories

Primary categories:

```text
REQUEST
STATE
SCENE
PLANNING
CONTROL
GRIPPER
GRASP
PHYSICS
COLLISION
PLACEMENT
TIMEOUT
CANCELLATION
SIMULATION
INTERNAL
```

---

## 4. Request Failures

Examples:

```text
INVALID_REQUEST
ROBOT_NOT_FOUND
OBJECT_NOT_FOUND
LOCATION_NOT_FOUND
OBJECT_NOT_PICKABLE
```

Recovery:

```text
reject before motion
```

---

## 5. State Failures

Examples:

```text
ROBOT_NOT_READY
ROBOT_BUSY
INVALID_OBJECT_STATE
NO_OBJECT_HELD
HELD_STATE_INCONSISTENT
```

Recovery:

```text
do not begin task
refresh state
reset if required
```

---

## 6. Scene Failures

Examples:

```text
SCENE_INVALID
SCENE_SYNC_FAILED
MISSING_COLLISION_OBJECT
INVALID_SUPPORT_RELATION
```

Recovery:

```text
block manipulation
resynchronize
or reset
```

---

## 7. Planning Failures

Examples:

```text
NO_VALID_GRASP
NO_VALID_PLACE_POSE
TARGET_UNREACHABLE
IK_FAILED
PLANNING_FAILED
```

Recovery may include:

```text
try another validated candidate
```

but not:

```text
expand limits
teleport base
disable collisions
```

---

## 8. Rail Failures

Examples:

```text
RAIL_CONTROLLER_FAILED
RAIL_LIMIT_REACHED
RAIL_STATE_INVALID
RAIL_EXECUTION_FAILED
```

Recovery:

```text
stop trajectory
validate state
return to safe known position if possible
```

Never teleport the carriage.

---

## 9. Arm Control Failures

Examples:

```text
ARM_CONTROLLER_FAILED
TRAJECTORY_ABORTED
GOAL_TOLERANCE_VIOLATION
```

Recovery:

```text
stop
preserve state
retreat only if safe
report failure
```

---

## 10. Gripper Failures

Examples:

```text
GRIPPER_OPEN_FAILED
GRIPPER_CLOSE_FAILED
INVALID_FINGER_STATE
```

Recovery depends on whether an object is held.

Do not assume safe release.

---

## 11. Grasp Failures

Examples:

```text
NO_FINGER_CONTACT
ONE_SIDED_CONTACT
INVALID_GRASP_VOLUME
GRASP_VERIFICATION_FAILED
GRASP_STABILIZATION_FAILED
OBJECT_SLIPPED
OBJECT_DROPPED
```

---

## 12. Grasp Verification Recovery

Initial policy:

```text
verification fails
↓
do not attach
↓
open gripper
↓
retreat
↓
return failure
```

A later bounded retry may select another grasp candidate.

---

## 13. Stabilization Failure

If temporary stabilization cannot activate cleanly:

```text
do not lift
```

Possible recovery:

```text
maintain physical grasp briefly
open safely
retreat
```

---

## 14. Object Drop

If the held object drops:

```text
stop nominal task
clear stale held state
clear invalid constraint
update scene state
report OBJECT_DROPPED
```

Automatic re-pick is not part of the baseline recovery policy.

---

## 15. Collision Failure

Unexpected collision:

```text
stop execution
do not push through
record involved entities
retreat only if safe
report failure
```

---

## 16. Scene Integrity Failure

Example:

```text
apple placed successfully
but purple_ball falls
```

Result:

```text
task failure
```

Possible code:

```text
SCENE_INTEGRITY_VIOLATION
```

---

## 17. Placement Failures

Examples:

```text
SUPPORT_NOT_ESTABLISHED
STABILIZATION_RELEASE_FAILED
PLACEMENT_VERIFICATION_FAILED
OBJECT_OUTSIDE_TARGET
OBJECT_UNSTABLE
```

---

## 18. Failed Release

If the object remains attached after intended release:

```text
stop retreat
```

Do not move away while dragging the object.

Resolve constraint / gripper state first.

---

## 19. Timeout Failures

Potential timeout sources:

```text
planning
trajectory execution
gripper motion
grasp verification
placement settling
full task
```

Timeout means:

```text
expected state transition did not complete
```

not necessarily a software crash.

---

## 20. Cancellation

Cancellation is an external requested interruption.

It should produce:

```text
CANCELLED
```

not a generic failure.

The system still must reach a known safe state.

---

## 21. Simulation Failures

Examples:

```text
invalid MuJoCo state
NaN
missing body
solver instability
invalid constraint
simulation stopped
```

Recovery:

```text
stop accepting tasks
perform controlled reset if possible
```

---

## 22. Internal Errors

Unexpected software exceptions should produce:

```text
INTERNAL_ERROR
```

with sufficient logs.

Do not convert unknown failures into success or a misleading known code.

---

## 23. Failure Result Structure

Each failure result should include:

```text
task_id
failure_code
layer
message
recoverable
robot_id
object_id if relevant
location_id if relevant
```

---

## 24. Recoverable Flag

Examples likely recoverable:

```text
NO_VALID_GRASP
PLANNING_FAILED
```

depending on alternative candidates.

Examples generally non-recoverable without reset:

```text
SIMULATION_ERROR
INVALID_SCENE_STATE
```

The flag should be conservative.

---

## 25. Retry Policy

Baseline:

```text
0 or very small bounded retries
```

A retry must change a meaningful decision.

Valid:

```text
select next grasp candidate
```

Invalid:

```text
repeat same failed trajectory endlessly
```

---

## 26. Recovery Layers

Recovery should happen at the lowest appropriate layer.

Example:

```text
gripper verification issue
→ Manipulation Layer
```

```text
natural-language ambiguity
→ LLM Layer
```

Do not ask the LLM to solve low-level physics failures.

---

## 27. Safe Retreat

A retreat is allowed only if:

```text
collision-free path known
robot state valid
moving does not worsen the failure
```

Otherwise stop and require reset / operator intervention.

---

## 28. Reset as Recovery

Reset is valid for:

```text
simulation inconsistency
stale constraints
unknown physical state
benchmark episode reset
```

Reset must not be the default response to every minor planning failure.

---

## 29. Recovery State Machine

Conceptual:

```text
FAILURE_DETECTED
↓
STOP_ACTIVE_EXECUTION
↓
CLASSIFY
↓
STABILIZE_STATE
↓
RECOVER_IF_SAFE
↓
VERIFY
↓
RETURN_FAILURE_OR_READY
```

---

## 30. Faulted Robot State

A severe robot/control failure may move robot state to:

```text
FAULTED
```

New task goals are rejected until explicit recovery succeeds.

---

## 31. Failure Logging

Record:

```text
task ID
state before failure
failure code
robot state
rail state
object state
contacts
constraint state
planning result
timestamp
```

This is required for reproducible debugging.

---

## 32. Failure Taxonomy Stability

Failure codes form part of the public Task API.

Once used by the agent layer and benchmarks, they should remain stable.

---

## 33. LLM Recovery Boundary

The LLM may respond to high-level failures.

Example:

```text
NO_VALID_GRASP
```

may trigger:

```text
choose another available object
ask user for clarification
or stop
```

The LLM must not respond by inventing joint commands.

---

## 34. Operator Intervention

Some states may require manual intervention.

Examples:

```text
persistent simulation instability
unrecoverable robot collision
corrupt scene state
```

The system should report this clearly.

---

## 35. Codex Rules

Codex must:

1. Never ignore failure return values.
2. Never continue after critical verification failure.
3. Never add infinite retry loops.
4. Never teleport the rail/base for recovery.
5. Never disable collision to recover.
6. Never auto-attach a dropped object.
7. Clear stale constraints.
8. Preserve structured failure codes.
9. Log enough state for reproduction.
10. Add a regression test when fixing a recurring failure.

---

## 36. Final Principle

The system should fail visibly and safely.

The core rule is:

> A controlled failure is better than an apparently successful but physically invalid task.
