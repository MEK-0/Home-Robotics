# Home Robotics

A research-oriented home robotics platform built around **dual Franka Emika Panda manipulators**, **MuJoCo**, **ROS 2 Jazzy**, **ros2_control**, and **MoveIt 2**.

The project explores reliable household manipulation in a simulated environment with two Panda robots mounted on independent carriages over a shared linear rail.

The long-term goal is to build a modular system capable of:

- collision-aware motion planning,
- reliable object manipulation,
- dual-arm coordination,
- task-level execution,
- and high-level VLA / LLM-based interaction.

---

## Overview

Home Robotics is designed as an academic and research-oriented robotics project rather than a single scripted demo.

The system is developed incrementally through clearly separated phases:

```text
Simulation
    ↓
ROS 2 Control
    ↓
MoveIt 2 Motion Planning
    ↓
Reliable Manipulation
    ↓
Task Execution
    ↓
VLA / LLM Integration
    ↓
Dual-Arm Coordination
    ↓
Research & Benchmarking
```

The current platform uses:

- two Franka Emika Panda robots,
- two Franka Hands,
- one shared physical linear rail,
- two independently controlled rail carriages,
- six work surfaces,
- static environment geometry,
- and multiple household manipulation objects.

---

## System Architecture

```text
                    High-Level Task Layer
                            │
                            ▼
                 Manipulation Architecture
                            │
                            ▼
                        MoveIt 2
                ┌───────────┴───────────┐
                │                       │
        panda1_manipulator      panda2_manipulator
          rail + 7 DOF            rail + 7 DOF
                │                       │
                └───────────┬───────────┘
                            ▼
                       ros2_control
                            │
                  controller_manager
                            │
          ┌─────────────────┴─────────────────┐
          │                                   │
 Panda trajectory/gripper          Panda trajectory/gripper
       controllers                       controllers
          │                                   │
          └─────────────────┬─────────────────┘
                            ▼
               MuJoCo Hardware Interface
                            │
                            ▼
                         MuJoCo
```

MuJoCo is the authoritative physics simulator.

MoveIt does not command MuJoCo directly. Motion execution follows:

```text
MoveIt
  ↓
FollowJointTrajectory / GripperCommand
  ↓
ros2_control
  ↓
MuJoCo hardware interface
  ↓
MuJoCo
```

---

## Robots

The platform contains two full Franka Panda manipulators.

Each manipulator planning group contains:

```text
1 prismatic rail joint
+
7 Franka Panda revolute joints
=
8 controlled DOF
```

### Panda 1

```text
panda1_rail_joint
panda1_joint1
panda1_joint2
panda1_joint3
panda1_joint4
panda1_joint5
panda1_joint6
panda1_joint7
```

Planning group:

```text
panda1_manipulator
```

TCP:

```text
panda1_tcp
```

### Panda 2

```text
panda2_rail_joint
panda2_joint1
panda2_joint2
panda2_joint3
panda2_joint4
panda2_joint5
panda2_joint6
panda2_joint7
```

Planning group:

```text
panda2_manipulator
```

TCP:

```text
panda2_tcp
```

---

## Shared Rail

Both robots move over the same physical rail but use independent carriages.

A minimum rail separation constraint is enforced:

```text
minimum separation = 0.70 m
```

The rail safety layer is kept separate from MoveIt geometric collision checking.

This means the system uses both:

```text
rail separation safety
+
full robot collision checking
```

---

## Simulation Environment

The MuJoCo environment currently includes:

- two Franka Panda robots,
- one shared linear rail,
- three rail supports,
- six four-legged work tables,
- arena boundaries,
- floor geometry,
- cube,
- apple,
- purple ball,
- bowl,
- pan.

The static environment is also represented inside the MoveIt PlanningScene.

---

## ROS 2 Control

The project uses **ROS 2 Jazzy** and **ros2_control**.

Active controllers include:

```text
joint_state_broadcaster

panda1_trajectory_controller
panda2_trajectory_controller

panda1_gripper_controller
panda2_gripper_controller
```

The trajectory controllers command:

```text
rail + 7 arm joints
```

for each Panda.

The gripper controllers command the primary finger joint while the second finger follows through mimic behavior.

---

## MoveIt 2

MoveIt 2 provides:

- inverse kinematics,
- joint-space planning,
- Cartesian pose target planning,
- collision checking,
- PlanningScene integration,
- controller integration,
- trajectory execution.

The initial planning pipeline uses:

```text
OMPL
└── RRTConnect
```

Kinematics:

```text
KDLKinematicsPlugin
```

Both manipulator groups are treated as 8-DOF chains.

---

## Collision Model

The robot description contains dedicated collision geometry for:

- Panda links,
- hands,
- fingers,
- rail,
- carriages.

MoveIt checks:

```text
self collision
Panda1 ↔ Panda2
Panda ↔ rail
Panda ↔ environment
Panda ↔ dynamic objects
```

The Allowed Collision Matrix only disables selected adjacent or permanently connected link pairs.

Cross-robot collision checking remains enabled.

---

## PlanningScene

Static scene geometry is loaded from the same authoritative scene configuration used by the MuJoCo environment.

The static PlanningScene currently contains:

```text
floor
4 arena boundaries
6 work surfaces
24 table-leg primitives
3 rail supports
```

These are represented using deterministic collision-object IDs.

Dynamic household objects are synchronized separately.

---

## Dynamic Objects

The manipulation architecture currently tracks:

```text
cube
apple
purple_ball
bowl
pan
```

MuJoCo remains the authoritative source for dynamic object poses.

The object state pipeline is:

```text
MuJoCo
  ↓
Object State Interface
  ↓
Object Registry
  ↓
Dynamic Scene Sync
  ↓
MoveIt PlanningScene
```

This allows MoveIt to reason about object positions during motion planning.

---

## Manipulation Architecture

Phase 4 introduces a structured manipulation state machine.

Conceptually:

```text
IDLE
  ↓
OBJECT_SELECTED
  ↓
PREGRASP_PLANNED
  ↓
APPROACHING
  ↓
GRIPPER_CLOSING
  ↓
GRASP_VERIFYING
  ↓
GRASP_VERIFIED
  ↓
OBJECT_ATTACHED
  ↓
LIFTING
  ↓
TRANSPORTING
  ↓
PLACING
  ↓
RELEASING
  ↓
PLACE_VERIFYING
  ↓
DONE
```

The first manipulation baseline focuses on:

```text
Panda1
  ↓
cube
  ↓
top-down grasp
```

The Phase 4 baseline now includes contact-verified cube grasp, stabilized lift, transport to `surface_left_2`, supported release and final placement verification. See [the demo](docs/DEMO.md) and [measured reliability](docs/PHASE4_RELIABILITY.md) for acceptance status and limitations.

---

## Grasp Strategy

The project uses a hybrid grasping strategy.

The intended flow is:

```text
physical contact
+
gripper closure
+
grasp verification
        ↓
temporary MuJoCo stabilization constraint
        ↓
lift / transport
        ↓
release
        ↓
constraint removal
```

Objects are not teleported during normal manipulation.

Temporary stabilization is only intended after a grasp has been physically validated.

---

## Validated Capabilities

The following components have been implemented and tested during development:

### Simulation

- deterministic MuJoCo reset,
- dual Panda integration,
- shared rail,
- Franka Hand integration,
- object and environment scene construction.

### ROS 2

- ros2_control hardware abstraction,
- trajectory controllers,
- gripper controllers,
- joint state publication,
- TF,
- simulation clock,
- reset integration.

### MoveIt 2

- dual 8-DOF planning groups,
- joint-space planning,
- Cartesian pose target planning,
- collision-aware planning,
- controller execution,
- static PlanningScene integration,
- dynamic object integration,
- Panda1/Panda2 collision checking,
- table collision rejection.

### Manipulation

- manipulation architecture,
- object registry,
- dynamic object scene synchronization,
- grasp-pose generation infrastructure currently under development.

---

## Project Roadmap

### Phase 0 — Foundation & Architecture

**Complete**

### Phase 1 — MuJoCo World

**Complete**

### Phase 2 — ROS 2 Robot Control

**Complete**

### Phase 3 — MoveIt 2

**Complete**

### Phase 4 — Reliable Pick & Place

**Complete — simulated Panda1/cube baseline (9/10 acceptance trials).**

Scope and evidence: [Phase 4 reliability](docs/PHASE4_RELIABILITY.md).

- dynamic object synchronization,
- manipulation state machine,
- grasp-pose generation,
- pre-grasp planning,
- contact validation,
- hybrid grasp stabilization,
- lift,
- transport,
- place,
- recovery.

### Phase 5 — Task Executor API

Planned.

```text
pick(object)
place(location)
move_to(...)
```

### Phase 6 — VLA / LLM Integration

Planned.

High-level natural-language instructions will be converted into structured manipulation tasks.

The LLM will not directly command robot joints.

### Phase 7 — Dual-Arm Coordination

Planned.

- task allocation,
- shared workspace management,
- collision-aware coordination,
- synchronized manipulation,
- handover,
- concurrent tasks.

### Phase 8 — Research & Benchmarking

Planned.

- robustness evaluation,
- manipulation success rates,
- planning benchmarks,
- failure analysis,
- domain randomization,
- ablation studies.

---

## Repository Structure

```text
Home-Robotics/
├── config/
│   └── scene.yaml
├── docs/
├── simulation/
├── tests/
└── ros2_ws/
    └── src/
        ├── home_robotics_control/
        ├── home_robotics_description/
        ├── home_robotics_mujoco_hardware/
        ├── home_robotics_bringup/
        ├── home_robotics_moveit_config/
        └── home_robotics_manipulation/
```

---

## Development Environment

Primary development environment:

```text
Ubuntu 24.04
ROS 2 Jazzy
MuJoCo
MoveIt 2
Python
C++
```

The project is also developed and tested on Apple Silicon through an Ubuntu ARM64 virtual machine.

---

## Build

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
python -m colcon build
source install/setup.bash
```

---

## Start the Control Stack

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch home_robotics_bringup phase2_control.launch.py use_viewer:=true
```

---

## Start MoveIt

```bash
ros2 launch home_robotics_moveit_config move_group.launch.py
```

---

## Load the Static PlanningScene

```bash
ros2 launch home_robotics_moveit_config planning_scene_environment.launch.py
```

---

## RViz

```bash
ros2 launch home_robotics_moveit_config moveit_rviz.launch.py
```

---

## Testing

Simulation tests:

```bash
cd ~/Home-Robotics
source .venv/bin/activate
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/simulation
```

ROS tests:

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
python -m colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
```

---

## Research Direction

This repository is intended to evolve into a research platform for:

- home robotics,
- manipulation,
- motion planning,
- simulation-to-real concepts,
- VLA systems,
- dual-arm robotics,
- robust robot control,
- intelligent task execution.

A key design principle is to keep:

```text
physics
control
planning
manipulation
reasoning
```

as clearly separated layers.

---

## Project Status

```text
Phase 0  Foundation                  COMPLETE
Phase 1  MuJoCo World               COMPLETE
Phase 2  ROS 2 Control              COMPLETE
Phase 3  MoveIt 2                   COMPLETE
Phase 4  Reliable Pick & Place      COMPLETE (Panda1/cube baseline)
Phase 5  Task Executor              PLANNED
Phase 6  VLA / LLM                  PLANNED
Phase 7  Dual-Arm Coordination      PLANNED
Phase 8  Research / Benchmarking    PLANNED
```

---

## Author

**Mahmut Esat Kolay**

Computer Engineering  
AI · Robotics · Intelligent Systems

GitHub: [MEK-0](https://github.com/MEK-0)

---

## License

This project is currently developed as an academic and research-oriented robotics project.

Third-party robot models and mesh assets retain their original licenses and attribution.