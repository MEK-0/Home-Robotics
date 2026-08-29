# Home Robotics — System Architecture

## 1. Purpose

This document defines the system architecture for the Home Robotics project.

Its purpose is to establish:

- component boundaries,
- data flow,
- control ownership,
- simulation responsibilities,
- ROS 2 responsibilities,
- MoveIt 2 responsibilities,
- task-execution responsibilities,
- LLM responsibilities,
- simulator abstraction rules,
- and the architectural constraints that all later implementation must follow.

This document is a system contract.

Implementation may evolve internally, but the separation of responsibilities defined here should remain stable unless a deliberate architectural revision is documented.

---

## 2. Architectural Goal

The Home Robotics system is designed as a layered manipulation platform.

The central architectural principle is:

> High-level intent must remain separated from low-level robot execution.

The final system should allow a user to issue a natural-language command while ensuring that:

- the LLM does not control joints,
- the LLM does not generate trajectories,
- the task layer does not solve inverse kinematics,
- MoveIt 2 does not interpret natural language,
- MuJoCo does not contain task logic,
- and scene geometry is not duplicated across unrelated code.

The intended top-level architecture is:

```text
User
  ↓
Natural-Language Interface
  ↓
LLM Planner
  ↓
Task Plan
  ↓
Task Executor
  ↓
Manipulation Layer
  ↓
MoveIt 2
  ↓
ROS 2 Control
  ↓
MuJoCo Integration
  ↓
MuJoCo Physics Simulation
  ↓
Franka Panda + Scene Objects
```

---

## 3. Architectural Layers

The system is divided into eight major layers.

```text
Layer 8 — User / Application Layer
Layer 7 — LLM Planning Layer
Layer 6 — Task Execution Layer
Layer 5 — Manipulation Layer
Layer 4 — Motion Planning Layer
Layer 3 — Robot Control Layer
Layer 2 — Simulation Integration Layer
Layer 1 — Physics / Scene Layer
```

Each layer may depend on lower layers.

Lower layers must not depend on higher layers.

For example:

```text
LLM → Task Executor → MoveIt → ROS 2 Control → MuJoCo
```

is valid.

The following direction is invalid:

```text
MuJoCo → LLM
```

The simulator must never contain logic such as:

```text
if user_requested_apple:
    move_robot_to_apple()
```

---

## 4. Layer 1 — Physics and Scene Layer

### 4.1 Responsibilities

The Physics / Scene layer contains the actual MuJoCo world.

It is responsible for:

- rigid-body dynamics,
- contact simulation,
- gravity,
- friction,
- object mass and inertia,
- collision geometry,
- visual geometry,
- robot bodies and joints,
- gripper finger dynamics,
- scene furniture,
- object spawning,
- deterministic initial state,
- and simulation stepping.

The main technology is:

```text
MuJoCo
```

### 4.2 This Layer Owns

This layer is the authoritative runtime owner of:

```text
robot physical state
object physical state
contacts
forces
joint positions
joint velocities
object positions
object orientations
constraint state
```

### 4.3 This Layer Must Not Own

This layer must not implement:

- natural-language parsing,
- task planning,
- grasp selection policy,
- MoveIt planning,
- ROS action semantics,
- high-level task state machines,
- or LLM logic.

---

## 5. Layer 2 — Simulation Integration Layer

### 5.1 Purpose

This layer connects MuJoCo to ROS 2.

It provides a clean integration boundary so higher layers do not directly manipulate MuJoCo internals.

Expected responsibilities include:

- exposing robot joint state,
- receiving actuator commands,
- synchronizing ROS time with simulation time,
- publishing simulation state,
- exposing object ground-truth state,
- managing reset commands,
- exposing contact information where needed,
- and supporting grasp-stabilization constraints.

### 5.2 Primary Interface Direction

```text
MuJoCo
  ↕
MuJoCo Integration Layer
  ↕
ROS 2
```

### 5.3 Implementation Rule

Higher-level ROS 2 nodes should not directly access arbitrary MuJoCo memory structures.

Instead, simulator-specific details must be isolated behind clearly defined interfaces.

For example:

```text
SceneStateProvider
```

may internally use MuJoCo data, but a task node should only see:

```text
get_object_pose("apple")
```

rather than:

```text
mjData.xpos[body_id]
```

---

## 6. Layer 3 — Robot Control Layer

### 6.1 Purpose

This layer provides standard ROS 2 robot-control semantics.

Expected technologies:

```text
ROS 2 Jazzy
ros2_control
controller_manager
joint_state_broadcaster
joint trajectory controller
gripper controller
```

### 6.2 Responsibilities

The Robot Control layer is responsible for:

- exposing joint state,
- receiving joint trajectory commands,
- enforcing controller interfaces,
- controlling the Franka arm joints,
- controlling the Franka Hand,
- mapping ROS controller commands to MuJoCo actuators,
- and reporting controller execution state.

### 6.3 Control Ownership

The following chain should remain clear:

```text
MoveIt 2
   ↓
trajectory command
   ↓
ROS 2 controller
   ↓
ros2_control hardware interface
   ↓
MuJoCo actuator
```

MoveIt should not directly command MuJoCo actuators.

The LLM should not directly command ROS controllers.

---

## 7. Layer 4 — Motion Planning Layer

### 7.1 Technology

The motion-planning layer is based on:

```text
MoveIt 2
```

### 7.2 Responsibilities

MoveIt 2 is responsible for:

- robot model representation,
- kinematic chains,
- inverse kinematics,
- joint-space planning,
- Cartesian planning,
- self-collision checking,
- world collision checking,
- trajectory generation,
- held-object collision handling,
- planning-scene management,
- and motion execution through ROS 2 controllers.

### 7.3 Planning Scene

The MoveIt Planning Scene should contain:

- both Panda robots,
- tables,
- worktops,
- fixed furniture,
- relevant scene objects,
- held objects,
- and target containers.

The MoveIt world representation must remain synchronized with the physical MuJoCo world.

### 7.4 Source of Truth Rule

MuJoCo owns physical object state.

MoveIt owns planning representation.

Therefore synchronization flows as:

```text
MuJoCo object state
        ↓
Scene State Bridge
        ↓
MoveIt Planning Scene
```

MoveIt must not silently assume an object's pose is unchanged after a manipulation event.

---

## 8. Layer 5 — Manipulation Layer

### 8.1 Purpose

The Manipulation layer converts task-level actions into reliable robot behaviors.

It contains the domain logic required for:

```text
pick
place
grasp
release
approach
retreat
lift
recovery
```

### 8.2 Responsibilities

This layer owns:

- grasp candidate generation,
- pre-grasp pose computation,
- approach planning,
- gripper closure logic,
- grasp verification,
- hybrid grasp stabilization,
- lifting,
- object transport,
- place-pose generation,
- release sequencing,
- retreat motion,
- and low-level manipulation recovery.

### 8.3 Example Pick Flow

```text
object lookup
      ↓
validate object state
      ↓
generate grasp candidates
      ↓
select valid grasp
      ↓
plan pre-grasp
      ↓
move to pre-grasp
      ↓
approach
      ↓
close gripper
      ↓
verify contact
      ↓
verify grasp
      ↓
enable stabilization constraint
      ↓
lift
      ↓
verify held object
      ↓
success
```

### 8.4 Example Place Flow

```text
validate held object
      ↓
resolve target location
      ↓
generate place pose
      ↓
plan pre-place
      ↓
move to pre-place
      ↓
lower object
      ↓
open gripper
      ↓
disable stabilization constraint
      ↓
verify placement
      ↓
retreat
      ↓
success
```

### 8.5 Critical Rule

Manipulation logic must not be implemented inside the LLM layer.

For example, the LLM may request:

```text
pick("apple")
```

but it must not decide:

```text
approach 8 cm above apple
rotate wrist 17 degrees
close finger to 0.021 m
```

Those are execution-layer responsibilities.

---

## 9. Layer 6 — Task Execution Layer

### 9.1 Purpose

The Task Executor provides stable task-level robot operations.

It is the main interface consumed by later high-level planners.

### 9.2 Expected Public Actions

The initial interface should eventually expose operations such as:

```text
pick(object_name)
place(location_name)
move_home(robot_name)
open_gripper(robot_name)
close_gripper(robot_name)
get_scene_state()
get_object_state(object_name)
```

### 9.3 ROS 2 Interface Style

Long-running robot operations should use ROS 2 Actions rather than simple services.

Example:

```text
PickObject.action
PlaceObject.action
MoveHome.action
```

Queries may use services or topics.

Example:

```text
GetObjectState.srv
GetSceneState.srv
```

### 9.4 Task Executor Responsibilities

The Task Executor owns:

- action validation,
- action lifecycle,
- manipulation sequencing,
- high-level timeout handling,
- retry policy where explicitly allowed,
- cancellation handling,
- result reporting,
- and mapping failures to standardized error codes.

### 9.5 Task Executor Must Not

The Task Executor should not:

- directly manipulate MuJoCo joint state,
- bypass MoveIt for normal arm motion,
- invent object coordinates,
- parse natural-language commands,
- or contain provider-specific LLM logic.

---

## 10. Layer 7 — LLM Planning Layer

### 10.1 Purpose

The LLM layer translates human intent into validated symbolic actions.

Example:

```text
"Put the apple into the bowl."
```

becomes:

```text
pick("apple")
place("bowl")
```

### 10.2 Responsibilities

The LLM layer may perform:

- natural-language understanding,
- task decomposition,
- tool selection,
- action sequencing,
- simple reasoning over scene state,
- failure-aware replanning,
- and clarification generation.

### 10.3 Strict Safety Boundary

The LLM is not allowed to directly issue:

```text
joint angles
joint velocities
joint torques
Cartesian actuator commands
raw controller commands
MuJoCo actuator values
trajectory points
```

### 10.4 Provider Independence

The architecture should allow multiple backends, including:

```text
Microsoft Foundry Local
local OpenAI-compatible models
remote OpenAI-compatible APIs
other tool-calling LLM providers
```

The task-execution API must remain independent of the selected model provider.

---

## 11. Layer 8 — User / Application Layer

This layer represents external users or applications.

Possible clients include:

- command-line tools,
- a web UI,
- a desktop UI,
- a research experiment runner,
- an LLM agent,
- or later a voice interface.

The user-facing layer should only interact with stable public APIs.

---

## 12. Core Runtime Components

A target runtime decomposition is:

```text
home_robotics_sim
home_robotics_scene_state
home_robotics_control
home_robotics_moveit
home_robotics_manipulation
home_robotics_task_executor
home_robotics_agent
```

These names represent conceptual modules.

Final package names may be refined during implementation, but responsibilities should remain aligned with these boundaries.

---

## 13. Proposed Runtime Node Graph

A future ROS 2 runtime may resemble:

```text
                        ┌────────────────────┐
                        │      User / UI     │
                        └─────────┬──────────┘
                                  │
                                  ▼
                        ┌────────────────────┐
                        │     LLM Planner    │
                        └─────────┬──────────┘
                                  │
                                  ▼
                        ┌────────────────────┐
                        │   Task Executor    │
                        └─────────┬──────────┘
                                  │
                                  ▼
                        ┌────────────────────┐
                        │ Manipulation Node  │
                        └─────────┬──────────┘
                                  │
                                  ▼
                        ┌────────────────────┐
                        │      MoveIt 2      │
                        │     move_group     │
                        └─────────┬──────────┘
                                  │
                                  ▼
                    ┌────────────────────────────┐
                    │       controller_manager   │
                    └─────────────┬──────────────┘
                                  │
                                  ▼
                    ┌────────────────────────────┐
                    │  MuJoCo ros2_control I/F   │
                    └─────────────┬──────────────┘
                                  │
                                  ▼
                        ┌────────────────────┐
                        │       MuJoCo       │
                        │  physics + scene   │
                        └─────────┬──────────┘
                                  │
                ┌─────────────────┴─────────────────┐
                │                                   │
                ▼                                   ▼
       ┌──────────────────┐                ┌──────────────────┐
       │   Panda Robot 1  │                │   Panda Robot 2  │
       └──────────────────┘                └──────────────────┘
```

Parallel state flow:

```text
MuJoCo
   ↓
Scene State Provider
   ├──→ Planning Scene Synchronizer
   ├──→ Task Executor
   └──→ LLM Scene Summary
```

The LLM receives structured scene state rather than direct simulator memory access.

---

## 14. Scene State Architecture

### 14.1 Ground-Truth Source

During early phases, MuJoCo provides ground-truth scene state.

The state provider should expose structured data such as:

```text
object:
  name
  pose
  velocity
  contact state
  held state
  support surface
```

### 14.2 Future Perception Compatibility

The task layer must not depend on the state being generated specifically by MuJoCo.

The future architecture should allow:

```text
MuJoCo Ground Truth
        │
        ▼
Scene State Interface
```

to later become:

```text
Camera / Perception
        │
        ▼
Scene State Interface
```

without rewriting the manipulation and task layers.

---

## 15. Dual-Robot Architecture

### 15.1 Initial State

Early phases:

```text
Panda 1 → active
Panda 2 → present but inactive
```

The complete scene still includes both robots.

### 15.2 Future State

Later:

```text
Panda 1 → active
Panda 2 → active
```

### 15.3 Required Namespace Isolation

Robot-specific interfaces must use explicit names.

Example:

```text
/panda1/joint_states
/panda1/controller_manager
/panda1/gripper
/panda1/follow_joint_trajectory

/panda2/joint_states
/panda2/controller_manager
/panda2/gripper
/panda2/follow_joint_trajectory
```

Exact names may depend on the final ROS 2 control setup, but robot identity must never be ambiguous.

### 15.4 Shared Resources

The two robots will eventually share:

```text
world
planning scene
object registry
task executor
scene state
workspace map
```

but maintain separate:

```text
joint state
controllers
gripper state
execution state
```

---

## 16. Workspace Ownership

Dual-arm support must be considered from the beginning even though it is implemented later.

The scene should support logical workspace definitions such as:

```text
panda1_primary_workspace
panda2_primary_workspace
shared_workspace
handover_zone
```

Early phases may only use:

```text
panda1_primary_workspace
```

The naming and scene structure should still leave room for future shared manipulation.

---

## 17. Configuration Architecture

Runtime constants must be configuration-driven.

Authoritative configuration files:

```text
config/scene.yaml
config/robots.yaml
config/objects.yaml
config/locations.yaml
config/grasp_profiles.yaml
config/physics.yaml
```

### 17.1 Example Ownership

```text
scene.yaml
  → furniture layout
  → support surfaces
  → workspace regions

robots.yaml
  → robot base poses
  → active/inactive state
  → default home configurations

objects.yaml
  → object dimensions
  → mass
  → semantic roles
  → initial poses

locations.yaml
  → named task destinations
  → placement zones
  → container definitions

grasp_profiles.yaml
  → grasp clearances
  → approach distances
  → gripper parameters

physics.yaml
  → timestep
  → friction
  → solver parameters
  → contact settings
```

### 17.2 Hard-Code Rule

Code should not duplicate these values unless there is a strong documented reason.

This is prohibited:

```python
APPLE_X = 0.61
TABLE_HEIGHT = 0.82
```

when those values already belong to configuration.

---

## 18. Simulator Abstraction

Although MuJoCo is the selected simulator, the architecture should avoid unnecessary simulator lock-in above the integration layer.

Higher layers should consume abstractions such as:

```text
RobotStateProvider
SceneStateProvider
ConstraintManager
SimulationResetInterface
```

rather than direct MuJoCo API calls.

This does not mean building a generic simulator framework.

It means simulator-specific code should remain localized.

---

## 19. Hybrid Grasp Architecture

The hybrid grasp system requires coordination between several layers.

```text
Manipulation Layer
        ↓
close gripper
        ↓
Robot Control
        ↓
MuJoCo contact
        ↓
Scene / Contact State
        ↓
Manipulation Layer verifies grasp
        ↓
Constraint Manager
        ↓
MuJoCo temporary stabilization
```

The stabilization constraint is not a substitute for grasp verification.

The sequence must remain:

```text
physical contact first
verification second
constraint third
```

Never:

```text
pick requested
      ↓
instant weld
```

---

## 20. Failure Propagation

Failures must propagate upward with structured information.

Example:

```text
MuJoCo collision instability
        ↓
Simulation Integration Error
        ↓
Manipulation Failure
        ↓
Task Result
        ↓
LLM / User
```

An error should not be hidden by the lower layer.

Possible standardized categories include:

```text
OBJECT_NOT_FOUND
INVALID_OBJECT_STATE
NO_VALID_GRASP
PLANNING_FAILED
CONTROLLER_FAILED
GRIPPER_FAILED
GRASP_VERIFICATION_FAILED
COLLISION_DETECTED
OBJECT_DROPPED
PLACE_VERIFICATION_FAILED
TIMEOUT
SIMULATION_ERROR
```

The detailed taxonomy is defined in `FAILURE_AND_RECOVERY.md`.

---

## 21. Reset Architecture

Deterministic reset is a first-class system capability.

Reset should restore:

```text
robot joint state
gripper state
object position
object orientation
object velocity
temporary constraints
controller state
task state
```

A reset must not leave stale grasp constraints active.

The simulation reset system should be callable independently from the LLM.

---

## 22. Logging and Observability

Every major layer should provide useful observability.

At minimum:

```text
simulation state
controller status
planning result
task state
grasp verification result
failure reason
execution duration
```

Task execution should use identifiable task IDs.

Example:

```text
task_id: task_000042
action: pick
object: apple
robot: panda1
result: success
duration: 4.83 s
```

This becomes important for both debugging and later academic benchmarking.

---

## 23. Dependency Direction

The required dependency direction is:

```text
Application
   ↓
LLM
   ↓
Task Executor
   ↓
Manipulation
   ↓
MoveIt
   ↓
ROS 2 Control
   ↓
Simulation Integration
   ↓
MuJoCo
```

Shared configuration and utility packages may be used horizontally.

Circular architectural dependencies must be avoided.

---

## 24. Architecture Rules for Codex

Codex implementation must obey the following rules:

1. Do not bypass an architectural layer for convenience.
2. Do not place task logic inside MuJoCo model files.
3. Do not place LLM logic inside manipulation nodes.
4. Do not directly access simulator internals from unrelated high-level packages.
5. Do not duplicate authoritative configuration values.
6. Do not silently introduce simulator-specific assumptions into the Task Executor.
7. Do not add a second implementation of object-state tracking.
8. Do not bypass MoveIt for normal planned arm motion.
9. Do not use an attachment constraint before grasp verification.
10. Do not activate Panda 2 before its designated project phase.
11. Preserve robot namespaces.
12. Preserve the world-frame convention.
13. Keep APIs narrow and explicit.
14. Add tests when introducing new cross-layer interfaces.
15. Document deliberate deviations from this architecture.

---

## 25. Phase-to-Architecture Mapping

### Phase 0

Defines all architecture contracts.

### Phase 1

Implements primarily:

```text
Layer 1
Layer 2 foundation
```

### Phase 2

Implements:

```text
Layer 2
Layer 3
```

### Phase 3

Implements:

```text
Layer 4
```

### Phase 4

Implements:

```text
Layer 5
```

### Phase 5

Implements:

```text
Layer 6
```

### Phase 6

Implements:

```text
Layer 7
Layer 8 integration
```

### Phase 7

Extends Layers 3–6 for dual-arm execution.

### Phase 8

Adds research extensions without violating existing layer boundaries.

---

## 26. Architecture Acceptance Criteria

The architecture is considered correctly implemented when:

- MuJoCo can run without the LLM layer,
- robot control can run without the LLM layer,
- MoveIt can be tested without the Task Executor,
- pick-and-place can be tested without natural language,
- the Task Executor can be called directly through ROS 2,
- the LLM communicates only through task-level interfaces,
- physical object state comes from a single runtime source,
- MoveIt receives synchronized scene state,
- simulator-specific APIs are localized,
- Panda 1 and Panda 2 remain unambiguously separated,
- and every failure can be traced to the layer where it originated.

---

## 27. Final Architectural Principle

The Home Robotics architecture is designed around one rule:

> Each layer should solve one class of problem well.

MuJoCo solves physics.

ROS 2 provides communication and control infrastructure.

ros2_control manages robot control interfaces.

MoveIt 2 solves robot motion planning.

The manipulation layer solves reliable pick-and-place behavior.

The Task Executor exposes stable symbolic robot actions.

The LLM decides which symbolic actions should happen and in what order.

Keeping these boundaries clean is essential to making the project reliable, debuggable, research-ready, and extensible.
