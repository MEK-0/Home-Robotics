# Home Robotics — Task Executor API

## 1. Purpose

This document defines the stable task-level API exposed by Home Robotics.

The Task Executor is the boundary between:

```text
high-level planning
```

and:

```text
robot manipulation execution
```

It exposes semantic robot capabilities such as:

```text
pick("apple")
place("bowl")
move_home("panda1")
get_scene_state()
```

while hiding:

```text
joint values
rail coordinates
IK details
trajectory points
MuJoCo actuator commands
```

The Task Executor is the only interface that the future LLM layer should use for normal robot tasks.

---

## 2. Core Rule

The public API must describe:

```text
WHAT the robot should do
```

not:

```text
HOW the robot should move
```

Allowed:

```text
pick apple
place held object in bowl
move panda1 home
```

Not allowed:

```text
move rail to 0.73 m
set joint 4 to -1.2 rad
move TCP to raw XYZ from LLM
```

---

## 3. Primary Operations

The initial public task API consists of:

```text
PickObject
PlaceObject
MoveHome
GetObjectState
GetSceneState
```

Potential later APIs:

```text
OpenGripper
CloseGripper
CancelTask
GetRobotState
ExecuteTaskPlan
```

Low-level gripper APIs are primarily debugging / internal tools and should not become the main LLM interface.

---

## 4. ROS 2 Interface Types

Use:

```text
Actions
```

for long-running operations:

```text
PickObject
PlaceObject
MoveHome
```

Use:

```text
Services
```

for short state queries:

```text
GetObjectState
GetSceneState
GetRobotState
```

Use Topics for continuous state only where needed.

---

## 5. PickObject Action

Conceptual goal:

```text
robot_id
object_id
```

Example:

```text
robot_id = panda1
object_id = apple
```

The Task Executor must validate canonical IDs before accepting execution.

---

## 6. PickObject Feedback

Feedback should expose meaningful state.

Example:

```text
PICK_VALIDATE
OBJECT_LOOKUP
GRASP_CANDIDATE_GENERATION
PRE_GRASP_PLAN
MOVE_PRE_GRASP
APPROACH
GRIPPER_CLOSE
GRASP_VERIFY
GRASP_STABILIZE
LIFT
```

Optional fields:

```text
progress
selected_grasp_id
message
```

---

## 7. PickObject Result

Result should include:

```text
success
failure_code
message
held_object
robot_id
task_id
```

A successful result must mean that the object is actually held and verified.

---

## 8. PlaceObject Action

Conceptual goal:

```text
robot_id
location_id
```

Example:

```text
robot_id = panda1
location_id = bowl
```

The currently held object should normally be resolved from robot state.

---

## 9. PlaceObject Feedback

Example states:

```text
PLACE_VALIDATE
TARGET_LOOKUP
PLACE_CANDIDATE_GENERATION
PRE_PLACE_PLAN
MOVE_PRE_PLACE
LOWER
SUPPORT_CHECK
GRIPPER_OPEN
REMOVE_STABILIZATION
PLACEMENT_VERIFY
RETREAT
```

---

## 10. PlaceObject Result

Result should include:

```text
success
failure_code
message
task_id
final_object_id
final_location_id
```

A successful place result requires placement verification.

---

## 11. MoveHome Action

Goal:

```text
robot_id
```

Home means:

```text
rail home
+
arm home
+
defined gripper home state
```

It does not mean only Panda arm joints.

---

## 12. GetObjectState

Input:

```text
object_id
```

Output should include:

```text
found
state_valid
pose
support_surface
held_by
container
pickable
place_target
```

The state must come from the Scene State Provider, not duplicate Task Executor state.

---

## 13. GetSceneState

The scene query should return semantic state appropriate for:

```text
task validation
debugging
LLM reasoning
```

It should not expose raw MuJoCo internal arrays.

Possible contents:

```text
scene_valid
objects
robots
available_locations
active_task
```

---

## 14. Robot Selection

Early usage:

```text
robot_id = panda1
```

The API should still include `robot_id` so Panda 2 can later be activated without breaking interfaces.

Future automatic robot selection may allow:

```text
robot_id = auto
```

but this is not required initially.

---

## 15. Canonical IDs

The API accepts only canonical IDs.

Examples:

```text
apple
purple_ball
bowl
panda1
```

Natural-language alias resolution belongs above the Task Executor.

---

## 16. Goal Validation

Before accepting a task, validate:

```text
system ready
robot exists
robot active
robot not busy
scene valid
target object/location exists
requested operation supported
```

Invalid goals should be rejected before motion starts.

---

## 17. Single-Task Ownership

Initially, one robot should execute one manipulation task at a time.

The Task Executor must avoid conflicting commands such as:

```text
pick apple
```

and simultaneously:

```text
move home
```

for the same robot.

---

## 18. Robot Busy State

Possible states:

```text
IDLE
EXECUTING
RECOVERING
FAULTED
```

New normal goals should only begin when the robot is eligible.

---

## 19. Task IDs

Every accepted task receives a unique ID.

Example:

```text
task_000001
```

Task ID should propagate through:

```text
Task Executor
Manipulation Layer
logs
benchmark records
failure records
```

---

## 20. Failure Codes

The API returns standardized failure codes.

Core categories:

```text
INVALID_REQUEST
ROBOT_NOT_FOUND
ROBOT_NOT_READY
ROBOT_BUSY
OBJECT_NOT_FOUND
OBJECT_NOT_PICKABLE
LOCATION_NOT_FOUND
NO_OBJECT_HELD
SCENE_INVALID
SCENE_SYNC_FAILED
NO_VALID_GRASP
NO_VALID_PLACE_POSE
TARGET_UNREACHABLE
PLANNING_FAILED
EXECUTION_FAILED
GRIPPER_FAILED
GRASP_VERIFICATION_FAILED
GRASP_STABILIZATION_FAILED
OBJECT_DROPPED
PLACEMENT_VERIFICATION_FAILED
COLLISION_DETECTED
TIMEOUT
CANCELLED
SIMULATION_ERROR
INTERNAL_ERROR
```

Detailed handling belongs in `FAILURE_AND_RECOVERY.md`.

---

## 21. Fail Closed

If state is ambiguous, the Task Executor should fail rather than guess.

Examples:

```text
unknown object pose
planning scene stale
held state inconsistent
constraint state inconsistent
```

should block new manipulation.

---

## 22. Cancellation

Actions must support cancellation.

Cancellation should propagate:

```text
Task Executor
↓
Manipulation Executor
↓
trajectory execution
```

The system must then enter a known safe state.

---

## 23. Retry Ownership

The Task Executor may permit bounded recovery.

It must not implement infinite blind retries.

Manipulation-specific retries belong primarily in the Manipulation Layer.

LLM-level replanning belongs above the Task Executor.

---

## 24. Timeout Ownership

Timeouts should exist for:

```text
planning
execution
gripper
verification
settling
complete task
```

Values must be configuration-driven.

---

## 25. Idempotence

State query APIs should be idempotent.

Robot-motion Actions are not generally idempotent.

Calling:

```text
pick("apple")
```

twice should not silently create undefined state.

The second call should validate that the object is already held or unavailable.

---

## 26. Precondition Examples

`PickObject` requires:

```text
object exists
object pickable
object not held
robot ready
```

`PlaceObject` requires:

```text
robot holds object
location exists
location is valid target
```

`MoveHome` requires:

```text
robot exists
robot controllable
safe path available
```

---

## 27. Postcondition Examples

Successful pick:

```text
held_by = robot_id
support_surface = null
```

Successful place:

```text
held_by = null
object inside valid location
```

Successful home:

```text
rail within home tolerance
arm within home tolerance
```

---

## 28. Tool-Calling Contract for LLM

The LLM may eventually receive tools conceptually equivalent to:

```text
get_scene_state()
pick(robot_id, object_id)
place(robot_id, location_id)
move_home(robot_id)
```

The LLM should not receive tools such as:

```text
set_joint_position()
set_rail_position()
attach_object()
```

---

## 29. Example LLM Plan

Input:

```text
Put the apple in the bowl.
```

Resolved plan:

```text
1. get_scene_state()
2. pick("panda1", "apple")
3. place("panda1", "bowl")
```

Execution results are returned after each step.

---

## 30. LLM Failure Feedback

The LLM may receive structured results such as:

```text
failure_code: NO_VALID_GRASP
recoverable: true
```

or:

```text
failure_code: SIMULATION_ERROR
recoverable: false
```

The LLM must not reinterpret a failed task as success.

---

## 31. Provider Independence

The API must remain independent of:

```text
Microsoft Foundry Local
OpenAI-compatible API
other local model runtime
```

The robot interface does not change when the model changes.

---

## 32. Proposed PickObject.action

Illustrative:

```text
string robot_id
string object_id
---
bool success
string task_id
string failure_code
string message
string held_object
---
string task_id
string state
float32 progress
string message
```

---

## 33. Proposed PlaceObject.action

Illustrative:

```text
string robot_id
string location_id
---
bool success
string task_id
string failure_code
string message
string object_id
string location_id
---
string task_id
string state
float32 progress
string message
```

---

## 34. Proposed MoveHome.action

Illustrative:

```text
string robot_id
---
bool success
string task_id
string failure_code
string message
---
string task_id
string state
float32 progress
```

---

## 35. API Versioning

Public task APIs should eventually have a documented version.

Example:

```text
task_api_version: 1.0
```

Breaking interface changes must be explicit.

---

## 36. Logging

Each task should log:

```text
task ID
goal
robot
start time
state transitions
result
failure code
duration
```

Logs should support later benchmark analysis.

---

## 37. Codex Rules

Codex must:

1. Keep the Task Executor semantic.
2. Never expose raw MuJoCo IDs.
3. Never expose actuator commands to the LLM layer.
4. Use canonical object and location IDs.
5. Validate goals before motion.
6. Use ROS 2 Actions for long-running tasks.
7. Return structured failure codes.
8. Support cancellation.
9. Preserve robot ID in interfaces.
10. Keep Panda 2 compatibility.
11. Use Scene State Provider for state truth.
12. Never duplicate object state internally.
13. Do not hide failed subtasks.
14. Keep retries bounded.
15. Preserve API compatibility once used by the agent layer.

---

## 38. Acceptance Criteria

The Task Executor API is accepted when:

```text
PickObject can be called directly without an LLM
PlaceObject can be called directly without an LLM
MoveHome can be called directly
GetObjectState returns authoritative state
GetSceneState returns semantic state
failure codes propagate correctly
cancellation works
robot busy state prevents conflicts
task IDs are traceable
```

---

## 39. Final Principle

The Task Executor is the stable contract between intelligence and robotics.

The LLM says:

```text
pick the apple
```

The Task Executor decides whether the request is valid and delegates execution to deterministic robotics software.

The core rule is:

> High-level systems request capabilities; they do not control mechanisms.
