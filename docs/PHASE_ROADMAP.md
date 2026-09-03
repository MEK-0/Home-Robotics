# Home Robotics — Phase Roadmap

## 1. Purpose

This document defines the implementation roadmap for Home Robotics.

The roadmap is intentionally phase-gated.

A later phase must not be used to hide unresolved failures in an earlier phase.

Core dependency:

```text
Foundation
↓
Simulation
↓
Control
↓
MoveIt
↓
Manipulation
↓
Task API
↓
LLM
↓
Dual Arm
↓
Research Extensions
```

---

## 2. Phase 0 — Foundation

### Goal

Establish the project contract before implementation.

### Deliverables

```text
repository structure
project brief
system architecture
design decisions
scene specification
scene map
coordinate frames
object registry
MuJoCo physics specification
robot/gripper specification
pick/place specification
ROS 2 architecture
task API
test strategy
failure taxonomy
Codex rules
research plan
```

### Exit Criteria

- Documentation is internally consistent.
- Linear rails are part of the architecture.
- Scene naming is stable.
- Frame conventions are stable.
- Phase boundaries are documented.
- No major unresolved architecture question blocks Phase 1.

---

## 3. Phase 1 — MuJoCo World

### Goal

Build the final-layout simulation environment.

### Deliverables

```text
world
floor
one shared physical linear rail
two carriages
two Franka Panda models
two Franka Hands
six work surfaces
initial objects
bowl
pan
major kitchen assets
collision models
physics configuration
deterministic reset
```

### Key Tests

```text
scene load
asset validation
initial collision validation
rail geometry validation
idle stability
100 deterministic resets
```

### Exit Criteria

- Scene loads headless.
- Both rail-mounted robots are present.
- Panda 1 + carriage 1 are active-ready.
- Panda 2 + carriage 2 are present and low-level-control active.
- No initial penetrations.
- No unexplained object drift.
- Bowl interior is physically accessible.
- Scene layout is locked as baseline version 1.0.
- Six work surfaces each contain four physical legs.
- All five canonical functional objects exist and reset deterministically.
- System workspace coverage is 100% by at least one robot; per-robot 8/9 coverage remains diagnostic.

---

## 4. Phase 2 — ROS 2 Robot Control

### Goal

Expose the simulated robot through standard ROS 2 control interfaces.

### Deliverables

```text
MuJoCo ROS 2 integration
ros2_control
rail joint state
rail control
Panda joint state
Panda control
Franka Hand control
TF
simulation clock
robot reset
```

### Key Tests

```text
rail direction test
rail range test
rail hold test
Panda home test
joint trajectory test
gripper open/close test
TF consistency
reset integration test
```

### Exit Criteria

- panda1_rail_joint moves through ROS.
- Panda 1 moves through ROS.
- Franka Hand operates through ROS.
- Joint states match MuJoCo.
- TF matches kinematics.
- Full robot home state is deterministic.
- No direct base teleportation exists.

---

## 5. Phase 3 — MoveIt 2

### Goal

Create reliable rail-aware motion planning.

### Deliverables

```text
MoveIt config
SRDF
panda1_arm_with_rail group
panda1_arm_only optional group
panda1_hand group
IK
joint planning
Cartesian planning
planning scene synchronization
controller mapping
held-object planning support
```

### Key Tests

```text
home planning
near target planning
far target requiring rail
collision rejection
planning scene update
TCP consistency
rail + arm trajectory execution
```

### Exit Criteria

- MoveIt sees 1 rail + 7 arm DOF.
- Far-table targets can be solved using rail motion.
- Planning avoids scene geometry.
- Physical and planning geometry are sufficiently consistent.
- Planning scene reflects current object state.

---

## 6. Phase 4 — Reliable Pick and Place

### Goal

Implement physically validated manipulation.

### Development Order

```text
cube
↓
apple
↓
purple_ball
↓
bowl placement
```

### Deliverables

```text
Object Registry runtime
grasp profiles
grasp candidate generation
pre-grasp
approach
gripper closure
grasp verification
constraint stabilization
lift
transport
place
release
placement verification
scene integrity checks
basic deterministic recovery
```

### Key Tests

```text
false grasp rejection
cube grasp
cube transport
cube release
apple grasp
apple → bowl
ball grasp
object drop detection
constraint snap rejection
unrelated-object disturbance
```

### Exit Criteria

- Cube pick/place is repeatable.
- Apple → bowl is repeatable.
- False grasps are rejected.
- Stabilization only follows verification.
- Placement is physically verified.
- Scene integrity is monitored.
- Failure states are structured.

---

## 7. Phase 5 — Task API

### Goal

Expose stable task-level ROS 2 interfaces.

### Deliverables

```text
home_robotics_msgs
PickObject.action
PlaceObject.action
MoveHome.action
GetObjectState
GetSceneState
Task Executor
task IDs
feedback
cancellation
failure codes
```

### Key Tests

```text
direct CLI pick
direct CLI place
move home
invalid object rejection
invalid location rejection
cancellation
busy robot rejection
failure propagation
```

### Exit Criteria

A user can execute:

```text
pick("panda1", "apple")
place("panda1", "bowl")
```

without knowing any joint or Cartesian values.

---

## 8. Phase 6 — LLM Orchestration

### Goal

Convert natural language into validated symbolic robot tasks.

### Preferred Direction

Provider-independent architecture with a local-first path.

Microsoft Foundry Local is a preferred candidate.

### Deliverables

```text
agent package
tool schemas
scene summarization
natural-language parsing
task sequencing
execution feedback
bounded replanning
provider abstraction
```

### Allowed Tools

```text
get_scene_state
pick
place
move_home
```

### Forbidden Agent Capabilities

```text
joint control
rail control
trajectory generation
MuJoCo actuator access
direct attachment
```

### Exit Criteria

A command such as:

```text
Put the apple in the bowl.
```

is converted into valid task calls and executed successfully.

---

## 9. Phase 7 — Dual-Arm Operation

### Goal

Add high-level dual-arm coordination and shared task allocation.

### Deliverables

```text
Panda 2 control
panda2_rail_joint control
MoveIt groups
cross-robot collision
robot selection
workspace ownership
shared workspace locking
task allocation
handover groundwork
```

### Key Tests

```text
independent robot motion
simultaneous safe rail motion
shared workspace exclusion
cross-robot collision rejection
robot-specific task execution
```

### Optional Advanced Milestone

```text
Panda 1 → Panda 2 object handover
```

### Exit Criteria

Both robots can operate without namespace ambiguity or unsafe shared-workspace behavior.

---

## 10. Phase 8 — Research Extensions

Possible directions:

```text
camera perception
object detection
pose estimation
domain randomization
RL
imitation learning
sim-to-real
grasp robustness
failure-aware agents
multi-robot planning
language-conditioned manipulation
```

This phase is intentionally open-ended.

---

## 11. Phase Gate Rule

A phase is complete only when its exit criteria pass.

Examples:

```text
scene looks good
```

does not complete Phase 1.

```text
robot moved once
```

does not complete Phase 2.

```text
apple placed once
```

does not complete Phase 4.

---

## 12. No Forward-Layer Patching

Forbidden:

```text
physics problem
→ fix with LLM retry
```

```text
wrong frame
→ fix with object-specific offset
```

```text
unreachable table
→ move Panda base manually
```

Problems must be fixed at their originating layer.

---

## 13. Major Milestones

### M0 — Architecture Locked

Phase 0 complete.

### M1 — Deterministic World

Phase 1 complete at scene/physics baseline 1.0 after model load, 10-second idle stability, 100/100 full-scene reset, bowl-access, ball-physics, collision, and system-level workspace-coverage acceptance.

### M2 — Controlled Rail-Mounted Panda

Phase 2 complete.

### M3 — Rail-Aware Motion Planning

Phase 3 complete.

### M4 — Reliable Manipulation

Phase 4 complete.

### M5 — Stable Task API

Phase 5 complete.

### M6 — Natural-Language Task Execution

Phase 6 complete.

### M7 — Dual-Arm Home Robotics

Phase 7 complete.

---

## 14. Repository Evidence per Phase

Each phase should leave evidence.

Examples:

```text
docs
tests
benchmark logs
screenshots / demo recordings
configuration versions
result summaries
```

Implementation without validation evidence is incomplete.

---

## 15. Commit Discipline

Prefer phase-focused commits.

Examples:

```text
feat(sim): add panda1 linear rail
test(sim): validate deterministic scene reset
feat(control): expose rail joint through ros2_control
feat(moveit): add rail-aware Panda planning group
feat(manipulation): add verified cube grasp
feat(tasks): add PickObject action
feat(agent): add task tool adapter
```

---

## 16. Final Roadmap Principle

The project should grow vertically through validated layers rather than horizontally through unfinished features.

The central rule is:

> Do not add intelligence on top of unreliable robotics.
