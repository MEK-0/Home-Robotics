# Home Robotics — ROS 2 Architecture

## 1. Purpose

This document defines the ROS 2 software architecture for the Home Robotics project.

Its purpose is to establish:

- node responsibilities,
- ROS 2 package boundaries,
- topic ownership,
- Action and Service interfaces,
- ros2_control integration,
- MoveIt 2 integration,
- scene-state flow,
- task-execution flow,
- robot namespaces,
- linear-rail control,
- gripper control,
- simulation-time behavior,
- failure propagation,
- and dual-arm extensibility.

This file is an architectural contract.

The goal is to keep the ROS 2 graph:

- modular,
- debuggable,
- simulator-aware only where necessary,
- and stable enough that higher layers do not depend on MuJoCo implementation details.

---

## 2. Core ROS 2 Principle

The ROS 2 system follows the dependency direction:

```text
MuJoCo
  ↓
Simulation Integration
  ↓
ros2_control
  ↓
MoveIt 2
  ↓
Manipulation Layer
  ↓
Task Executor
  ↓
LLM / Application
```

Higher layers must not bypass lower layers for convenience.

---

## 3. Initial Runtime Scope

Initial active system:

```text
Panda 1 + Rail 1
```

Physically present but inactive:

```text
Panda 2 + Rail 2
```

The ROS 2 architecture must still reserve clean namespace and package structure for both robots from the beginning.

---

## 4. Recommended ROS 2 Workspace Structure

The repository already contains:

```text
ros2_ws/
```

Recommended source layout:

```text
ros2_ws/
└── src/
    ├── home_robotics_msgs/
    ├── home_robotics_description/
    ├── home_robotics_control/
    ├── home_robotics_sim_bridge/
    ├── home_robotics_moveit_config/
    ├── home_robotics_scene/
    ├── home_robotics_manipulation/
    ├── home_robotics_tasks/
    └── home_robotics_agent/
```

Exact package creation order follows project phases.

---

## 5. Package Responsibility Rules

Each package should own one clear responsibility.

### `home_robotics_msgs`

Owns:

```text
custom Actions
custom Services
custom Messages
error/result enums if represented in interfaces
```

It must not contain robot logic.

---

### `home_robotics_description`

Owns:

```text
robot descriptions
rail description
Panda prefixes
Franka Hand integration
URDF / xacro
SRDF-related source description
meshes needed by ROS representation
```

It should reflect the same kinematic architecture as MuJoCo.

---

### `home_robotics_control`

Owns:

```text
ros2_control configuration
controller configuration
joint interfaces
rail control configuration
gripper controller configuration
```

It should not contain task logic.

---

### `home_robotics_sim_bridge`

Owns simulator-specific integration.

Responsibilities may include:

```text
MuJoCo lifecycle
simulation stepping
simulation reset
joint interface integration
object state extraction
contact extraction
constraint management hooks
simulation clock
```

This is the primary location where direct MuJoCo API usage is allowed.

---

### `home_robotics_moveit_config`

Owns:

```text
MoveIt robot model
SRDF
kinematics configuration
planning pipelines
joint limits
controller mappings
planning-scene setup
```

It must not own scene semantics such as "apple" or "bowl".

---

### `home_robotics_scene`

Owns:

```text
Object Registry runtime
Location Registry runtime
Scene State Provider
Planning Scene synchronization
semantic scene queries
scene integrity tracking
```

This package connects physical simulation state to semantic robotics state.

---

### `home_robotics_manipulation`

Owns:

```text
grasp candidate generation
pre-grasp
approach
grasp verification
stabilization requests
lift
transport
place
release
retreat
```

This package implements the behavior defined in `PICK_PLACE_SPEC.md`.

---

### `home_robotics_tasks`

Owns:

```text
task-level Actions
task lifecycle
pick(object)
place(location)
move_home(robot)
failure propagation
cancellation
high-level sequencing
```

This package is the stable execution interface used by applications and the future LLM layer.

---

### `home_robotics_agent`

Owns:

```text
LLM integration
tool calling
natural-language parsing
task sequencing
execution feedback interpretation
```

This package is introduced only in the LLM phase.

It must never issue raw robot control commands.

---

## 6. Namespace Strategy

Robot-specific ROS interfaces must be namespaced.

Canonical robot namespaces:

```text
/panda1
/panda2
```

Examples:

```text
/panda1/joint_states
/panda1/controller_manager
/panda1/rail_controller
/panda1/arm_controller
/panda1/gripper_controller
```

and later:

```text
/panda2/joint_states
/panda2/controller_manager
/panda2/rail_controller
/panda2/arm_controller
/panda2/gripper_controller
```

---

## 7. Shared System Namespace

Shared project-level interfaces may use:

```text
/home_robotics
```

Examples:

```text
/home_robotics/scene_state
/home_robotics/reset
/home_robotics/task_status
```

The project should avoid placing every node directly under the global root namespace.

---

## 8. Node Naming Rules

Node names must:

- be lowercase,
- use snake_case,
- describe responsibility,
- remain stable,
- avoid temporary suffixes.

Good:

```text
scene_state_provider
planning_scene_sync
task_executor
constraint_manager
```

Bad:

```text
node1
test_node_final
apple_picker
temp_bridge
```

---

## 9. Core Runtime Nodes

A mature single-arm runtime may conceptually contain:

```text
mujoco_simulator
simulation_clock
controller_manager
robot_state_publisher
scene_state_provider
planning_scene_sync
move_group
constraint_manager
manipulation_executor
task_executor
```

Some responsibilities may be combined if there is a strong implementation reason.

The conceptual boundaries should remain intact.

---

## 10. MuJoCo Simulator Node

Conceptual node:

```text
/mujoco_simulator
```

Responsibilities:

```text
load scene
step physics
manage simulation lifecycle
expose simulator readiness
support deterministic reset
publish /clock
```

Direct task semantics do not belong here.

---

## 11. Simulation Time

The system must use ROS simulation time.

Expected:

```text
use_sim_time = true
```

for all nodes that depend on temporal synchronization.

MuJoCo simulation time should drive:

```text
/clock
```

---

## 12. `/clock`

The simulation integration layer should publish:

```text
/clock
```

based on MuJoCo simulation time.

The clock must be monotonic during normal execution.

Reset behavior must be deliberately defined.

---

## 13. Clock Reset Policy

If simulation reset also resets simulation time, all nodes must tolerate the time jump.

Alternatively, simulated time may remain monotonic while physical state resets.

The selected policy must be consistent and tested.

Do not leave time semantics ambiguous.

---

## 14. ros2_control Role

`ros2_control` is the standard control abstraction between ROS 2 and the simulated robot.

It should expose hardware-like interfaces for:

```text
rail joint
Panda arm joints
Franka Hand joints
```

---

## 15. MuJoCo ros2_control Interface

The simulator integration layer maps:

```text
ROS command interfaces
```

to:

```text
MuJoCo actuators
```

and:

```text
MuJoCo joint state
```

to:

```text
ROS state interfaces
```

Higher-level controllers should not know MuJoCo actuator IDs.

---

## 16. State Interfaces

At minimum, the active robot should expose:

```text
position
velocity
```

for:

```text
panda1_rail_joint
panda1 arm joints
gripper joints
```

Effort state may also be exposed when useful and correctly modeled.

---

## 17. Command Interfaces

Command interfaces depend on the selected control mode.

The project should prefer a control mode compatible with:

```text
MoveIt trajectory execution
stable MuJoCo control
future real-hardware reasoning
```

The exact interface is selected during Phase 2 implementation.

---

## 18. Joint State Publication

Joint states should be published through the standard control stack.

Conceptually:

```text
joint_state_broadcaster
```

provides joint state to:

```text
robot_state_publisher
MoveIt
debugging tools
benchmarking
```

---

## 19. Robot State Publisher

`robot_state_publisher` generates the dynamic robot TF tree using:

```text
joint states
+
robot description
```

For Panda 1 the chain includes:

```text
rail
→ carriage
→ Panda
→ hand
→ TCP
```

The exact static / dynamic division follows `COORDINATE_FRAMES.md`.

---

## 20. Rail Controller

The rail must have a proper ROS controller interface.

Conceptually:

```text
/panda1/rail_controller
```

The exact controller may be:

```text
trajectory based
position based
```

or part of a combined rail + arm trajectory controller.

The final decision should favor MoveIt integration and coordinated motion.

---

## 21. Combined Rail + Arm Trajectory

Preferred architecture:

```text
one planning group
```

containing:

```text
rail joint
+
7 Panda arm joints
```

and an execution path capable of maintaining timing consistency across the full trajectory.

This avoids treating rail repositioning as a separate non-robot operation.

---

## 22. Controller Architecture Options

Two main options exist.

### Option A — Combined Trajectory Controller

```text
rail joint
+
7 arm joints
```

controlled through one trajectory controller.

Advantages:

```text
simple synchronized execution
natural MoveIt mapping
```

### Option B — Separate Rail and Arm Controllers

```text
rail controller
arm trajectory controller
```

Advantages:

```text
clear mechanical separation
```

Disadvantage:

```text
coordinated MoveIt execution becomes more complex
```

The final choice should be evidence-driven during Phase 2.

---

## 23. Initial Preference

The initial architectural preference is:

```text
coordinated MoveIt-visible rail + arm execution
```

rather than manually sequencing rail movement outside MoveIt.

The controller implementation should support this principle.

---

## 24. Gripper Controller

The Franka Hand should use a dedicated gripper control interface.

Conceptually:

```text
/panda1/gripper_controller
```

Task-level commands remain semantic:

```text
open
close
move width
```

Manipulation code should not publish arbitrary finger commands directly when a controller abstraction exists.

---

## 25. Panda 2 Controllers

During early phases, Panda 2 controllers remain inactive.

Its ROS description may still exist.

Do not start unused controllers simply because the model is present.

---

## 26. MoveIt 2 Architecture

Primary MoveIt node:

```text
/move_group
```

The planning frame is:

```text
world
```

Primary Panda 1 planning group:

```text
panda1_arm_with_rail
```

containing:

```text
panda1_rail_joint
+
7 Panda arm joints
```

---

## 27. Optional Arm-Only MoveIt Group

A secondary group may exist:

```text
panda1_arm_only
```

for:

```text
local approach
debugging
controlled experiments
```

This must not replace full rail-aware reachability.

---

## 28. Hand MoveIt Group

Conceptually:

```text
panda1_hand
```

The hand may use MoveIt gripper semantics or a dedicated controller interface depending on final integration.

---

## 29. MoveIt Planning Scene

The planning scene must represent:

```text
tables
work surfaces
rail geometry where relevant
Panda 2
containers
manipulable objects
held objects
other collision-relevant furniture
```

It must remain synchronized with MuJoCo physical state.

---

## 30. Planning Scene Synchronizer

Conceptual node:

```text
/planning_scene_sync
```

Responsibilities:

```text
read semantic scene state
update MoveIt object poses
add / remove world collision objects
attach / detach held objects
```

It must not own physical object state.

---

## 31. Scene State Provider

Conceptual node:

```text
/scene_state_provider
```

Responsibilities:

```text
read MuJoCo object state
combine with Object Registry metadata
publish semantic runtime state
resolve support relations
resolve held state
resolve container state
validate object state
```

This is the primary semantic bridge between physics and higher layers.

---

## 32. Scene State Message

A custom message may eventually represent:

```text
scene timestamp
object list
robot summaries
active task relationships
scene validity
```

The exact schema belongs in `home_robotics_msgs`.

---

## 33. Object State Query

The project should expose a structured query for one object.

Conceptually:

```text
GetObjectState
```

Request:

```text
object_id
```

Response:

```text
found
state_valid
pose
support_surface
held_by
container
```

---

## 34. Scene State Query

A full semantic query may expose:

```text
GetSceneState
```

This is useful for:

```text
Task Executor
LLM planner
debugging
benchmarking
```

---

## 35. Constraint Manager

Conceptual node / module:

```text
/constraint_manager
```

Responsibilities:

```text
activate verified grasp stabilization
remove stabilization
query constraint state
clear all temporary constraints on reset
```

Only the manipulation layer should normally request grasp stabilization.

---

## 36. Constraint Interface

Conceptual Service:

```text
SetGraspConstraint
```

or equivalent.

Possible request:

```text
robot_id
object_id
enable
```

The manager must validate:

```text
object exists
robot exists
grasp relationship plausible
```

before changing physical constraint state.

---

## 37. Manipulation Executor

Conceptual node:

```text
/manipulation_executor
```

Responsibilities:

```text
execute pick state machine
execute place state machine
generate grasp candidates
perform grasp verification
coordinate MoveIt
coordinate gripper
coordinate constraint manager
```

It consumes semantic object and location information.

---

## 38. Task Executor

Conceptual node:

```text
/task_executor
```

Responsibilities:

```text
public task lifecycle
pick Action
place Action
move_home Action
validation
cancellation
failure mapping
result reporting
```

The Task Executor should not contain raw motion planning details.

---

## 39. Why ROS 2 Actions

Robot tasks are long-running operations.

Therefore:

```text
pick
place
move_home
```

should use ROS 2 Actions.

Actions support:

```text
goal acceptance
feedback
cancellation
result
```

which matches robotic task semantics.

---

## 40. Pick Action

Conceptual interface:

```text
PickObject.action
```

Goal:

```text
robot_id
object_id
```

Feedback may include:

```text
current_state
progress
active_grasp_candidate
```

Result:

```text
success
failure_code
message
held_object
```

---

## 41. Place Action

Conceptual:

```text
PlaceObject.action
```

Goal:

```text
robot_id
location_id
```

Optional explicit object ID may be included if useful.

Feedback:

```text
current_state
progress
```

Result:

```text
success
failure_code
message
final_object_state
```

---

## 42. Move Home Action

Conceptual:

```text
MoveHome.action
```

Goal:

```text
robot_id
```

The action should restore:

```text
rail home
arm home
gripper desired home state
```

subject to safe execution.

---

## 43. Reset Service

Simulation reset is conceptually exposed as:

```text
/home_robotics/reset
```

or:

```text
ResetSimulation.srv
```

Reset is not a normal manipulation action.

It belongs to simulation / experiment control.

---

## 44. Reset Coordination

Reset must coordinate:

```text
task executor
manipulation state
constraint manager
controllers
MuJoCo state
scene state
planning scene
```

Resetting MuJoCo alone is not enough.

---

## 45. Reset Sequence

Recommended conceptual sequence:

```text
reject / cancel task
↓
stop robot execution
↓
clear temporary constraints
↓
reset rail + arm + gripper
↓
reset objects
↓
settle physics
↓
refresh scene state
↓
refresh planning scene
↓
declare system ready
```

---

## 46. Task Feedback

Action feedback should expose meaningful state.

Example:

```text
state: GRASP_VERIFY
```

not merely:

```text
progress: 62%
```

Percent progress may supplement state but should not replace it.

---

## 47. Failure Codes

Failure codes should use a stable enum-like contract.

Examples:

```text
OBJECT_NOT_FOUND
NO_VALID_GRASP
PLANNING_FAILED
GRIPPER_FAILED
GRASP_VERIFICATION_FAILED
OBJECT_DROPPED
PLACEMENT_VERIFICATION_FAILED
COLLISION_DETECTED
TIMEOUT
SIMULATION_ERROR
```

The exact taxonomy is defined in `FAILURE_AND_RECOVERY.md`.

---

## 48. Failure Propagation

Example:

```text
MoveIt planning failure
        ↓
Manipulation Layer
        ↓
Task Executor
        ↓
Pick Action Result
        ↓
LLM / User
```

Failures must remain traceable to their originating layer.

---

## 49. Topics vs Services vs Actions

Use:

### Topics

for continuously changing state.

Examples:

```text
joint states
scene summaries
diagnostics
clock
```

### Services

for short request/response operations.

Examples:

```text
get object state
reset simulation
constraint toggle
```

### Actions

for long-running operations.

Examples:

```text
pick
place
move home
```

---

## 50. Topic Ownership

Each topic must have one clear owner.

Example:

```text
/panda1/joint_states
```

should not be independently published by both:

```text
MuJoCo bridge
```

and:

```text
custom state node
```

One authoritative publisher is required.

---

## 51. TF Ownership

Dynamic robot TF should be generated through the standard robot-state path.

Object TF, if published, should have a single scene-state owner.

Duplicate transform publishers are prohibited.

---

## 52. Diagnostics

The system should eventually expose diagnostics for:

```text
simulator ready
controller ready
MoveIt ready
scene synchronized
constraint state valid
task executor ready
```

This prevents task execution before system readiness.

---

## 53. Lifecycle Considerations

Where useful, major infrastructure nodes may use ROS 2 lifecycle semantics.

Potential candidates:

```text
simulation bridge
scene state provider
task executor
```

Lifecycle is not mandatory unless it clearly improves startup / reset reliability.

Do not add lifecycle complexity without need.

---

## 54. Startup Order

Recommended startup dependency:

```text
1. robot / scene descriptions
2. MuJoCo simulation
3. ros2_control interfaces
4. controllers
5. robot_state_publisher
6. Scene State Provider
7. MoveIt 2
8. Planning Scene Sync
9. Manipulation Executor
10. Task Executor
11. Agent / UI
```

Higher layers must wait for dependencies.

---

## 55. Readiness Gates

Before Task Executor accepts goals:

```text
simulation ready
controllers ready
TF valid
MoveIt available
scene state valid
planning scene synchronized
```

must all pass.

---

## 56. Shutdown Order

Shutdown should stop higher-level task execution before physics / controllers disappear.

Recommended:

```text
agent
task executor
manipulation
MoveIt
controllers
simulation
```

---

## 57. Launch Architecture

Launch files should compose clear runtime modes.

Examples:

```text
simulation.launch.py
control.launch.py
moveit.launch.py
manipulation.launch.py
full_stack.launch.py
```

Exact names may evolve.

---

## 58. Full Stack Launch

A future main launch should conceptually support:

```bash
ros2 launch home_robotics_bringup full_stack.launch.py
```

If a dedicated bringup package is introduced, it should contain orchestration only.

Do not place business logic in launch files.

---

## 59. Headless Mode

The system should support headless simulation as the default development mode.

Conceptually:

```text
gui:=false
```

GUI should be optional for visual inspection and demos.

---

## 60. Robot Selection Parameter

Launch should eventually support activation state such as:

```text
panda1_active:=true
panda2_active:=false
```

During Phase 7:

```text
panda1_active:=true
panda2_active:=true
```

---

## 61. Configuration Loading

ROS 2 nodes should consume the central project configuration.

Avoid duplicating scene semantics into ROS parameter files when those semantics already belong in:

```text
config/*.yaml
```

ROS-specific settings may live separately.

---

## 62. ROS Parameter Responsibilities

Appropriate ROS parameters include:

```text
topic names
timeouts
controller names
update rates
debug flags
planning pipeline selection
```

Physical scene truth should remain in the central project configuration.

---

## 63. QoS Policy

QoS should be selected according to message semantics.

Examples:

```text
joint state
→ sensor-like state

task result
→ reliable

static / infrequent configuration state
→ reliable / transient local where appropriate
```

QoS should not be changed randomly to hide dropped-state bugs.

---

## 64. Scene State Frequency

Scene state does not necessarily need physics-timestep frequency.

The publication rate should balance:

```text
responsive manipulation
low CPU overhead
planning freshness
```

The exact frequency should be measured.

---

## 65. Joint State Frequency

Joint-state publication must be high enough for:

```text
robot_state_publisher
MoveIt
controller monitoring
```

without flooding the constrained VM unnecessarily.

---

## 66. Logging

Every major node should use structured ROS logging.

Useful fields include:

```text
task ID
robot ID
object ID
state
failure code
```

Avoid unstructured print statements in production nodes.

---

## 67. Task ID

Every Task Executor goal should receive a unique task ID.

Example:

```text
task_000042
```

The ID should propagate through logs where practical.

---

## 68. Benchmark Correlation

Benchmark tools should be able to correlate:

```text
task ID
ROS logs
MuJoCo state logs
planning results
final result
```

This is important for academic reproducibility.

---

## 69. rosbag Strategy

Selected ROS topics may be recorded during debugging / experiments.

Potential topics:

```text
joint states
task feedback
scene state
TF
diagnostics
```

Avoid recording unnecessary high-bandwidth data by default.

---

## 70. No Camera Topics in Early Phases

Perception is not part of the initial stack.

Do not introduce:

```text
RGB camera pipeline
depth image pipeline
object detector
```

before the designated phase.

Ground-truth Scene State Provider remains the source of object pose.

---

## 71. LLM Interface Boundary

The LLM / agent layer must interact through task-level interfaces.

Allowed:

```text
PickObject
PlaceObject
MoveHome
GetSceneState
```

Prohibited:

```text
joint trajectory publication
rail controller command
gripper actuator command
MoveIt raw trajectory generation
```

---

## 72. Agent Provider Independence

`home_robotics_agent` should expose model-independent planning logic where practical.

Potential backends:

```text
Microsoft Foundry Local
local OpenAI-compatible model
remote tool-calling model
```

The ROS Task Executor API remains unchanged.

---

## 73. Panda 2 Namespace Reservation

Even before activation, do not create Panda 1 interfaces that assume:

```text
there will only ever be one robot
```

For example, prefer:

```text
robot_id
```

fields in task Actions where reasonable.

---

## 74. Dual-Arm Runtime

Future dual-arm runtime may contain:

```text
/panda1/...
/panda2/...
shared scene state
shared planning world
shared task coordinator
```

The architecture should not require renaming existing Panda 1 interfaces when Panda 2 becomes active.

---

## 75. Dual-Arm Task Coordinator

Phase 7 may introduce a coordinator responsible for:

```text
robot selection
workspace locking
handover sequencing
shared resource ownership
collision-aware task allocation
```

This should sit above individual robot manipulation executors.

---

## 76. Shared Workspace Locking

A future shared-workspace mechanism may expose logical resource state such as:

```text
shared_workspace owner
handover_zone owner
```

This is not required in early phases but should fit naturally above the Task Executor / robot selection layer.

---

## 77. Controller Failure

If a controller becomes unavailable during task execution:

```text
task must fail safely
```

The Task Executor should not silently switch to direct MuJoCo control.

---

## 78. MoveIt Failure

If MoveIt planning fails:

```text
return planning failure
```

Do not bypass planning with a hard-coded joint trajectory except in an explicitly defined debugging tool.

---

## 79. Simulator Failure

If MuJoCo state becomes invalid:

```text
stop accepting new tasks
mark system not ready
return simulation error
```

Higher layers must not continue using stale state.

---

## 80. Planning Scene Sync Failure

If planning scene cannot be synchronized with physical state:

```text
do not plan manipulation
```

Planning with stale world geometry is unsafe.

---

## 81. Constraint State Failure

If grasp stabilization state and semantic held state disagree:

```text
system enters failure condition
```

Example invalid state:

```text
held_by = null
but weld constraint still active
```

This must be detected.

---

## 82. ROS 2 Interface Versioning

Custom interfaces should remain stable once consumed by higher layers.

Breaking changes to:

```text
PickObject.action
PlaceObject.action
GetSceneState
```

should be explicit and documented.

---

## 83. Proposed `PickObject.action`

Illustrative:

```text
string robot_id
string object_id
---
bool success
string failure_code
string message
string held_object
---
string state
float32 progress
```

Exact schema should be finalized when implementation begins.

---

## 84. Proposed `PlaceObject.action`

Illustrative:

```text
string robot_id
string location_id
---
bool success
string failure_code
string message
---
string state
float32 progress
```

---

## 85. Proposed `MoveHome.action`

Illustrative:

```text
string robot_id
---
bool success
string failure_code
string message
---
string state
```

---

## 86. Proposed `GetObjectState.srv`

Illustrative:

```text
string object_id
---
bool found
bool state_valid
geometry_msgs/PoseStamped pose
string support_surface
string held_by
string container
```

---

## 87. Proposed Scene Summary

A future semantic scene message might contain:

```text
Header
ObjectState[]
RobotState[]
bool scene_valid
```

The exact message should remain minimal.

Do not reproduce full MuJoCo state in a semantic ROS message.

---

## 88. Controller Naming Stability

Once controller names are used by MoveIt configuration, they should remain stable.

Renaming controllers is a cross-layer change.

---

## 89. Planning Group Naming Stability

Canonical conceptual names:

```text
panda1_arm_with_rail
panda1_arm_only
panda1_hand

panda2_arm_with_rail
panda2_arm_only
panda2_hand
```

If implementation requires different names, the mapping must be documented before use.

---

## 90. No Simulator IDs Above Bridge

Higher layers must not use:

```text
MuJoCo body numeric ID
MuJoCo joint numeric ID
MuJoCo geom numeric ID
```

They use semantic IDs.

The bridge owns simulator-specific lookup.

---

## 91. No Duplicate Scene State

Do not create separate truth in:

```text
Task Executor
MoveIt helper
LLM agent
```

The Scene State Provider is the semantic runtime source.

---

## 92. No Direct Rail Topic Hacks

Manipulation code should not casually publish to a raw rail command topic.

Preferred:

```text
MoveIt / controller abstraction
```

or a clearly defined robot-control API.

---

## 93. No Direct Gripper Physics Hack

Manipulation must not modify finger joint positions directly in MuJoCo.

Use the controller path.

---

## 94. No Direct Object Teleport During Task

Task execution must not modify object pose directly.

Direct object pose reset belongs only to:

```text
simulation reset
experiment setup
```

not successful manipulation.

---

## 95. Startup Validation Test

Automated integration test should verify:

```text
/clock active
controller manager reachable
joint states active
TF valid
MoveIt active
scene state active
planning scene synchronized
Task Executor ready
```

---

## 96. Rail State Integration Test

Command Rail 1 through the control stack.

Verify:

```text
MuJoCo carriage moves
joint state changes
TF base transform changes
MoveIt current state changes
```

All four must agree.

---

## 97. Rail + Arm Trajectory Test

Send a coordinated trajectory through the selected MoveIt/controller path.

Verify:

```text
rail and arm execute synchronized motion
no controller desynchronization
final joint state within tolerance
```

---

## 98. Gripper Integration Test

Command:

```text
open
close
```

through ROS.

Verify:

```text
finger joints move in MuJoCo
ROS state reflects motion
no direct simulator intervention required
```

---

## 99. Scene State Integration Test

Move a test object physically in MuJoCo.

Verify:

```text
Scene State Provider updates
MoveIt Planning Scene updates
Task query returns latest pose
```

---

## 100. Reset Integration Test

Call reset.

Verify:

```text
task state cleared
constraint cleared
rail reset
arm reset
gripper reset
objects reset
scene state updated
planning scene updated
system returns ready
```

---

## 101. Action Cancellation Test

Start a long move / manipulation task.

Cancel it.

Verify:

```text
Action acknowledges cancellation
robot stops safely
task state clears
no stale command remains
```

---

## 102. Headless Acceptance

The full robotics stack must run without graphical MuJoCo visualization.

GUI must not be a required ROS dependency.

---

## 103. Codex ROS 2 Rules

Codex must:

1. Preserve package responsibilities.
2. Keep simulator-specific code inside the simulation bridge.
3. Use ROS 2 Actions for long-running robot tasks.
4. Use Services for short request/response operations.
5. Keep continuous state on Topics.
6. Preserve `/panda1` and `/panda2` namespace strategy.
7. Treat the rail as a robot joint.
8. Never teleport Panda bases.
9. Use ros2_control for normal actuator interfaces.
10. Use MoveIt for normal planned arm/rail motion.
11. Preserve simulation time.
12. Avoid duplicate topic / TF publishers.
13. Keep Task Executor independent of MuJoCo APIs.
14. Keep LLM layer above Task Executor.
15. Add integration tests when cross-package interfaces change.
16. Never add raw simulator numeric IDs to public interfaces.
17. Preserve canonical object IDs.
18. Fail closed when scene synchronization is invalid.
19. Keep Panda 2 inactive until its phase.
20. Document deliberate deviations from this architecture.

---

## 104. Phase Mapping

### Phase 1

ROS involvement should remain minimal.

Main focus:

```text
MuJoCo scene
simulation readiness
basic state extraction foundation
```

### Phase 2

Implement:

```text
simulation bridge
ros2_control
rail control
Panda control
gripper control
joint states
TF
```

### Phase 3

Implement:

```text
MoveIt 2
planning groups
planning scene
controller mapping
```

### Phase 4

Implement:

```text
manipulation executor
constraint manager
grasp verification integration
```

### Phase 5

Implement:

```text
Task Executor
public Actions
state Services
```

### Phase 6

Implement:

```text
agent package
LLM tool calling
```

### Phase 7

Activate:

```text
Panda 2
Rail 2
dual-arm coordinator
```

---

## 105. ROS 2 Architecture Acceptance Criteria

The ROS 2 architecture is considered healthy when:

- MuJoCo can run independently of the LLM.
- ros2_control reflects real MuJoCo robot state.
- Rail movement propagates correctly through joint state and TF.
- Panda arm movement propagates correctly through joint state and TF.
- Gripper control uses the ROS control path.
- MoveIt sees the complete rail + arm chain.
- Scene state has one semantic authority.
- Planning Scene matches physical object state.
- Pick/place can be called without the LLM.
- Failures propagate through structured results.
- Panda 2 can later be enabled without renaming Panda 1 interfaces.
- The entire stack runs headless.

---

## 106. Final ROS 2 Principle

ROS 2 is the communication and control backbone of Home Robotics.

It should make each layer observable and replaceable without mixing responsibilities.

The target architecture is:

```text
MuJoCo
↓
clean simulation bridge
↓
standard ROS 2 control
↓
MoveIt 2
↓
reliable manipulation
↓
stable task Actions
↓
LLM / applications
```

The final rule is:

> Higher-level intelligence should interact with stable robot capabilities, not with simulator internals.
