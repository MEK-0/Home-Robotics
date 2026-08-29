# Home Robotics — Project Brief

## 1. Project Overview

Home Robotics is a simulation-first robotic manipulation platform built around two Franka Panda robot arms operating inside a structured home / kitchen environment.

The project is designed as both:

- an academic robotics development platform,
- and a portfolio-quality end-to-end robotics system.

The primary goal is to build a reliable manipulation stack in which a robot can understand a high-level task, identify the relevant object and destination, plan a safe motion, execute a stable grasp, transport the object, and place it at the requested location.

The system will be developed incrementally.

The first stages focus purely on deterministic robotic manipulation.

Later stages introduce a Large Language Model (LLM) layer capable of translating natural-language instructions such as:

> "Put the apple on the table into the bowl."

into a sequence of structured robot actions such as:

```text
pick("apple")
place("bowl")
```

The LLM will never directly control robot joints, Cartesian coordinates, torques, or actuator commands.

High-level reasoning and low-level robot execution must remain strictly separated.

---

## 2. Core Project Objective

The final system should support the following execution pipeline:

```text
Natural Language Command
        ↓
LLM / Task Planner
        ↓
Structured Robot Actions
        ↓
Task Executor
        ↓
MoveIt 2
        ↓
ROS 2 Control Layer
        ↓
MuJoCo
        ↓
Franka Panda Robot
        ↓
Physical Interaction with Scene
```

For example:

```text
User:
"Put the apple into the bowl."

        ↓

Planner:

pick("apple")
place("bowl")

        ↓

Robot:

detect object state
move to pre-grasp
approach object
close gripper
verify grasp
stabilize grasp
lift object
move to bowl
lower object
release object
verify placement
retreat
return success
```

The project is therefore not intended to be only a visual simulation.

The main engineering goal is reliable task execution.

---

## 3. Simulation Platform

The primary simulation engine is:

**MuJoCo**

MuJoCo is selected because the project requires:

- efficient CPU execution,
- low memory usage,
- ARM64 compatibility,
- reliable rigid-body physics,
- stable contact simulation,
- manipulation-oriented physics,
- realistic object interaction,
- and future compatibility with robotics research workflows.

The simulation will run inside:

```text
Ubuntu 24.04
ARM64
ROS 2 Jazzy
```

with approximately:

```text
CPU: 4 cores
RAM: 8 GB
Disk: 80 GB
GPU: no NVIDIA GPU
```

The project must therefore avoid unnecessary GPU-dependent tooling and excessive simulation overhead.

Isaac Sim is not part of the runtime architecture.

Visual references from Isaac Sim / Omniverse may be used only as scene-design references.

---

## 4. Robot Configuration

The final system will contain:

```text
2 × Franka Panda
```

robot arms.

Each robot will use the standard:

```text
Franka Hand
two-finger parallel gripper
```

configuration.

The project will initially activate only one Panda robot for control and manipulation.

The second Panda will already exist in the final scene layout but will not participate in manipulation during the early phases.

This avoids future redesign of the simulation environment while keeping the initial control system simple.

The second robot will later become an active participant during the dual-arm phase.

---

## 5. Scene Philosophy

The simulation environment represents a structured home / kitchen workspace.

The scene should be designed correctly from the beginning.

The project should not rely on a temporary simplified layout that is later replaced by an entirely different environment.

Instead:

- the global layout remains stable,
- robot mounting positions remain stable,
- tables and work surfaces remain stable,
- coordinate frames remain stable,
- destination zones remain stable,
- and major scene dimensions remain stable.

Individual assets may later gain improved meshes or textures, but their functional geometry and coordinate definitions should remain compatible.

The scene therefore follows the principle:

```text
Stable Layout
+
Realistic Scale
+
Simplified Collision Geometry
+
High-Quality Visual Geometry
```

---

## 6. Unit Convention

All simulation dimensions use SI units.

The primary distance convention is:

```text
1 MuJoCo unit = 1 meter
```

Other quantities must also follow SI conventions where applicable:

```text
distance     → meter
mass         → kilogram
time         → second
force        → Newton
torque       → Newton-meter
velocity     → meter/second or radian/second
```

Artificial scene scaling should not be introduced to solve grasping or physics problems.

If an interaction fails, the physical parameters or manipulation strategy must be corrected instead.

---

## 7. Global Coordinate Convention

The world coordinate system follows:

```text
X → forward
Y → left
Z → up
```

The root frame is:

```text
world
```

Robot frame structures will follow explicit namespaces.

Example:

```text
world
│
├── panda1_base
│   └── panda1_link0
│       └── ...
│           └── panda1_hand
│               └── panda1_tcp
│
└── panda2_base
    └── panda2_link0
        └── ...
            └── panda2_hand
                └── panda2_tcp
```

Hard-coded transformations must be avoided whenever a frame-based transformation can be used instead.

---

## 8. Initial Scene Objects

The first supported manipulation objects are:

```text
apple
purple_ball
cube
bowl
pan
```

Initial roles:

| Object | Primary Role |
|---|---|
| apple | pickable object |
| purple_ball | pickable object |
| cube | pickable calibration / test object |
| bowl | destination / container |
| pan | destination / environment object |

These roles may later expand.

However, every object must exist in a central object registry rather than being defined independently inside manipulation code.

---

## 9. Object State

During the initial phases there is no camera-based perception system.

Object state is read directly from MuJoCo ground-truth simulation data.

For example:

```text
object name
object position
object orientation
linear velocity
angular velocity
contact state
```

may be obtained directly from the simulation.

This is a deliberate architectural decision.

The initial objective is to solve manipulation and task execution independently from perception.

Perception will be introduced only after reliable manipulation has been demonstrated.

---

## 10. Grasping Strategy

The project uses a hybrid grasping strategy.

The robot must first perform a physically plausible grasp.

The expected sequence is:

```text
move to pre-grasp
        ↓
approach object
        ↓
close Franka Hand
        ↓
detect finger/object contact
        ↓
verify grasp conditions
        ↓
activate temporary grasp stabilization
        ↓
lift
```

The object must not be attached automatically simply because the robot requested a pick action.

A grasp must first satisfy defined validity conditions.

Once the grasp has been physically validated, MuJoCo constraints may be used to stabilize the object during transportation.

This approach provides a compromise between:

- physical realism,
- deterministic behavior,
- reproducibility,
- and engineering reliability.

The stabilization constraint must be removed during object release.

---

## 11. Manipulation Philosophy

A valid pick-and-place operation is not simply:

```text
move
close gripper
move
open gripper
```

Instead, manipulation must be treated as a state machine.

A typical pick operation is:

```text
IDLE
 ↓
OBJECT_LOOKUP
 ↓
PRE_GRASP
 ↓
APPROACH
 ↓
GRIPPER_CLOSE
 ↓
GRASP_VERIFY
 ↓
GRASP_STABILIZE
 ↓
LIFT
 ↓
PICK_SUCCESS
```

A place operation is:

```text
OBJECT_HELD
 ↓
PLACE_LOOKUP
 ↓
PRE_PLACE
 ↓
APPROACH_PLACE
 ↓
LOWER
 ↓
GRIPPER_OPEN
 ↓
REMOVE_STABILIZATION
 ↓
PLACEMENT_VERIFY
 ↓
RETREAT
 ↓
PLACE_SUCCESS
```

Failures must produce explicit failure states instead of silently continuing.

---

## 12. Collision Safety

Collision avoidance is a core requirement.

The robot must not achieve a task by:

- pushing unrelated objects away,
- knocking over nearby objects,
- passing through tables,
- clipping through scene geometry,
- penetrating the target object,
- or using physically impossible trajectories.

The motion planning system must account for:

```text
robot self-collision
tables
worktops
static furniture
other robot
scene objects
held object
target containers
```

Manipulation success therefore includes both:

```text
task completion
```

and:

```text
scene integrity
```

A task that places the apple correctly but knocks over the pan is considered a failure.

---

## 13. ROS 2 Integration

ROS 2 Jazzy is the primary robotics middleware.

The intended software architecture includes:

```text
MuJoCo
    ↓
ROS 2 integration layer
    ↓
ros2_control
    ↓
robot controllers
    ↓
MoveIt 2
    ↓
task executor
```

The exact implementation of the MuJoCo ↔ ROS 2 control bridge will be documented separately.

The architecture must minimize simulator-specific assumptions in higher-level nodes.

Task planning logic should not know whether the physical backend is MuJoCo, another simulator, or eventually real hardware.

---

## 14. MoveIt 2 Responsibility

MoveIt 2 is responsible for robot motion-level planning.

Its responsibilities include:

```text
inverse kinematics
motion planning
collision checking
planning scene
Cartesian motion
trajectory generation
robot state management
```

MoveIt must not be responsible for interpreting natural language.

Likewise, the LLM must not perform inverse kinematics or generate joint trajectories.

---

## 15. Task Executor

A dedicated task execution layer separates high-level planning from robotic motion.

The eventual public task API should expose operations such as:

```text
pick(object_name)

place(location_name)

move_home(robot_name)

get_scene_state()

get_object_state(object_name)

open_gripper(robot_name)

close_gripper(robot_name)
```

Higher-level systems interact with these operations instead of directly commanding robot joints.

---

## 16. LLM Layer

The LLM layer will be introduced only after deterministic manipulation is reliable.

The LLM architecture should remain provider-independent.

Possible runtime providers may include:

```text
local models
Microsoft Foundry Local
OpenAI-compatible APIs
other tool-calling LLM systems
```

The expected architecture is:

```text
Natural Language
        ↓
LLM
        ↓
Task Planner
        ↓
Validated Tool Calls
        ↓
Task Executor
```

Example:

```text
"Put the apple in the bowl."
```

becomes:

```json
{
  "actions": [
    {
      "tool": "pick",
      "object": "apple"
    },
    {
      "tool": "place",
      "location": "bowl"
    }
  ]
}
```

The LLM is never allowed to directly output executable joint or actuator commands.

---

## 17. Dual-Arm Goal

Although early manipulation uses a single active Panda, the architecture must support the future activation of both robots.

The later dual-arm system may require:

```text
workspace ownership
shared planning scene
cross-robot collision checking
task allocation
resource locking
synchronized execution
handover operations
multi-arm motion planning
```

Dual-arm coordination must not be implemented prematurely.

However, early design decisions must avoid making dual-arm support impossible later.

---

## 18. Research Direction

The project should be suitable for future research extensions.

Potential directions include:

```text
contact-rich manipulation
grasp robustness
domain randomization
sim-to-real transfer
robot learning
reinforcement learning
imitation learning
visual perception
pose estimation
language-conditioned manipulation
multi-agent robotic planning
dual-arm coordination
LLM-based task planning
failure-aware robotic agents
```

The architecture should therefore emphasize modularity and reproducibility.

---

## 19. Academic and Portfolio Standard

The repository is intended to demonstrate more than functioning code.

It should document:

- system architecture,
- design decisions,
- assumptions,
- physical parameters,
- test methodology,
- failure cases,
- benchmarking,
- reproducibility,
- experimental results,
- known limitations,
- and future research directions.

Experiments should be repeatable.

Claims should be supported by measured results whenever possible.

Example metrics may include:

```text
pick success rate
place success rate
task completion rate
collision rate
grasp failure rate
average execution time
planning time
recovery success rate
simulation stability
reset determinism
```

---

## 20. Project Phases

The project is divided into the following phases.

### Phase 0 — Foundation

Establish:

```text
repository structure
documentation
architecture
scene specification
coordinate conventions
configuration schemas
development rules
```

No manipulation implementation begins before the fundamental contracts are defined.

### Phase 1 — MuJoCo World

Build the complete simulation environment.

Requirements include:

```text
final scene layout
two Franka Panda models
Franka Hand models
tables
work surfaces
scene objects
collision geometry
physics parameters
deterministic reset
```

The layout established here should remain stable throughout later phases.

### Phase 2 — Robot Control

Integrate robot control with ROS 2.

Target capabilities:

```text
joint state publication
robot state
gripper control
trajectory controller
ros2_control integration
MuJoCo control interface
TF tree
```

### Phase 3 — MoveIt 2

Integrate MoveIt 2.

Target capabilities:

```text
IK
motion planning
collision checking
planning scene
joint-space motion
Cartesian motion
trajectory execution
```

### Phase 4 — Reliable Pick and Place

Build the complete manipulation pipeline.

Target capabilities:

```text
object lookup
grasp generation
pre-grasp
approach
gripper control
grasp verification
grasp stabilization
lift
transport
place
release
retreat
failure recovery
```

Reliability is more important than adding new features during this phase.

### Phase 5 — Task API

Expose manipulation through stable task-level interfaces.

Example:

```text
pick("apple")
place("bowl")
move_home("panda1")
get_scene_state()
```

This phase creates the interface later consumed by the LLM layer.

### Phase 6 — LLM Orchestration

Add natural-language task planning.

Responsibilities include:

```text
command interpretation
tool selection
task sequencing
execution feedback
failure-aware planning
```

LLM control remains strictly above the execution layer.

### Phase 7 — Dual-Arm Manipulation

Activate the second Franka Panda.

Research and implementation topics include:

```text
workspace partitioning
shared collision environment
task allocation
mutual exclusion
dual-arm planning
coordinated tasks
object handover
```

### Phase 8 — Research Extensions

Possible extensions include:

```text
camera perception
object detection
pose estimation
domain randomization
robot learning
reinforcement learning
sim-to-real experiments
advanced LLM agents
benchmark datasets
```

Phase 8 is intentionally open-ended.

---

## 21. First Major Milestone

The first major milestone is:

### M1 — Deterministic Simulation Environment

The complete MuJoCo environment must:

1. load successfully,
2. contain both Panda robots,
3. contain the defined furniture and objects,
4. use real metric dimensions,
5. contain valid collision geometry,
6. initialize objects at defined poses,
7. remain physically stable while idle,
8. reset deterministically,
9. avoid spontaneous object movement,
10. reproduce the same initial state across repeated runs.

A target validation test is:

```text
100 simulation resets
```

with:

```text
0 unintended object falls
0 invalid robot initial states
0 scene penetrations
0 spontaneous collisions
consistent object poses
consistent robot poses
```

Manipulation development should not begin until the scene passes its stability tests.

---

## 22. Development Principle

The project follows one central engineering rule:

> Reliability before complexity.

New layers should only be introduced after lower layers are validated.

The intended dependency direction is:

```text
Scene
 ↓
Physics
 ↓
Robot Control
 ↓
Motion Planning
 ↓
Manipulation
 ↓
Task API
 ↓
LLM
```

If a lower layer is unstable, higher layers must not be used to hide the problem.

For example:

```text
bad grasp physics
```

must not be "fixed" by:

```text
LLM retries
```

Likewise:

```text
incorrect scene coordinates
```

must not be fixed by hard-coded offsets inside manipulation code.

Problems must be corrected at the layer where they originate.

---

## 23. Source-of-Truth Principle

Important project parameters must have a single authoritative definition.

Examples:

```text
scene layout       → config/scene.yaml
robot poses        → config/robots.yaml
objects            → config/objects.yaml
target locations   → config/locations.yaml
grasp parameters   → config/grasp_profiles.yaml
physics parameters → config/physics.yaml
```

Implementation code must consume these configurations.

The same value should not be independently duplicated across multiple source files.

---

## 24. Non-Goals for Early Phases

The following are explicitly outside the initial project scope:

```text
camera perception
object detection
pose estimation from images
reinforcement learning
dual-arm manipulation
sim-to-real deployment
LLM-generated trajectories
autonomous scene redesign
```

These features may be introduced only in their assigned later phases.

---

## 25. Definition of Project Success

The core project can be considered successful when a user can issue a command such as:

> "Put the apple into the bowl."

and the system can reliably perform:

```text
interpret task
        ↓
resolve apple
        ↓
resolve bowl
        ↓
select robot
        ↓
plan safe grasp
        ↓
pick apple
        ↓
verify grasp
        ↓
transport apple
        ↓
place apple in bowl
        ↓
verify final state
        ↓
report success
```

without:

```text
manual coordinate entry
unintended collisions
unrelated object displacement
invalid grasp attachment
direct LLM motor control
scene-specific hard-coded hacks
```

That behavior represents the central objective of Home Robotics.
