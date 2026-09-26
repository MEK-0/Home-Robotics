# Home Robotics

## Project summary

Home Robotics is a simulation-based manipulation project built with MuJoCo,
ROS 2 Jazzy, ros2_control, and MoveIt 2. It models two Franka Panda robots on
independent carriages sharing one physical rail. The current work establishes a
validated Panda1 pick-and-place baseline; it is not a general household-robot
system.

The repository preserves the implementation, configuration, tests, and measured
validation records for Phases 0–5. Phase 6 and later are future work.

## Current validated capability

- Simulation only, with MuJoCo as the physics environment.
- Panda1 is the validated manipulation baseline. Panda2 is present and
  independently controllable, but is not used for coordinated manipulation.
- The cube is the primary end-to-end validated object. It can be grasped,
  physically verified, temporarily stabilized, lifted, transported to a named
  surface, placed, released, and verified.
- `surface_left_2` is an example validated destination.
- Phase 5 exposes the cube path through a `PickAndPlace` ROS Action.
- Only one manipulation task may be active at a time.

`purple_ball` has one additional grasp/lift/return baseline trial. It is not a
named-placement reliability result and is not accepted by the Phase 5 Action.
Apple, bowl, and pan manipulation are not validated capabilities.

## Architecture

```text
Task client
    ↓
Task Executor (PickAndPlace Action)
    ↓
Panda1 manipulation pipeline
    ↓
MoveIt 2
    ↓
ros2_control
    ↓
MuJoCo runtime
```

MuJoCo owns physics, object pose, and contact state. MoveIt owns planning and
collision reasoning. ros2_control is the command/state boundary between them.
The static environment is loaded into MoveIt's PlanningScene; dynamic object
poses are synchronized from MuJoCo. Shared-rail separation is enforced outside
of geometric collision checking, so the two safety layers remain distinct.

See [Architecture](docs/ARCHITECTURE.md) for boundaries and runtime ownership.

## Implemented components

- MuJoCo scene with two Panda robots, one shared rail, static work surfaces,
  and configured scene objects.
- ROS 2 Jazzy runtime, a MuJoCo-backed `ros2_control` hardware interface,
  trajectory controllers, gripper controllers, simulation time, and reset.
- MoveIt 2 configuration for both Panda chains, static PlanningScene loading,
  and dynamic object synchronization.
- Panda1 cube pick/place pipeline with contact-based grasp verification and
  placement verification.
- `home_robotics_interfaces/action/PickAndPlace.action` and a single-task
  Phase 5 executor.

## Quick start

The following is the normal development workflow. It assumes the ROS and Python
environments described below are available.

```bash
cd ~/Home-Robotics
source .ros_venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ros2_ws
python -m colcon build
source install/setup.bash
```

Start the runtime in separate terminals after sourcing the same environment:

```bash
ros2 launch home_robotics_bringup phase2_control.launch.py use_viewer:=false
ros2 launch home_robotics_moveit_config move_group.launch.py
ros2 launch home_robotics_moveit_config planning_scene_environment.launch.py
ros2 launch home_robotics_manipulation dynamic_object_scene_sync.launch.py
```

The detailed demo and reset sequence is in [docs/DEMO.md](docs/DEMO.md).

## Demo

Run planning-only first. This performs the Phase 4 preflight without issuing
physical motion commands:

```bash
ros2 launch home_robotics_manipulation cube_pick_place_demo.launch.py \
  robot:=panda1 object:=cube target:=surface_left_2 execute:=false
```

After a clean reset and stack restart, use `execute:=true` for the physical
simulation run. The same path is available through the Phase 5 Action:

```bash
ros2 launch home_robotics_task_executor task_executor.launch.py execute:=false
ros2 action send_goal --feedback /home_robotics/pick_and_place \
  home_robotics_interfaces/action/PickAndPlace \
  "{object_id: cube, target_id: surface_left_2}"
```

## Validation

The concise evidence summary is [docs/VALIDATION.md](docs/VALIDATION.md).

- 92 deterministic simulation tests passed.
- 56 ROS package tests completed with 0 errors, 0 failures, and 1 existing
  skipped test.
- 10 focused Task Executor tests cover the Phase 5 Action contract.
- A 10-trial full-stack simulated cube benchmark recorded 9 successes and one
  planning/retiming failure. The failed trial is retained in the evidence.

These are different forms of validation, not one combined score. Runtime and
RViz observations are documented separately from automated tests.

## Repository structure

```text
Home-Robotics/
├── config/                       # Authoritative scene and object configuration
├── simulation/                   # MuJoCo scene and deterministic simulation code
├── tests/                        # Simulation tests
├── ros2_ws/src/
│   ├── home_robotics_control
│   ├── home_robotics_description
│   ├── home_robotics_mujoco_hardware
│   ├── home_robotics_bringup
│   ├── home_robotics_moveit_config
│   ├── home_robotics_manipulation
│   ├── home_robotics_interfaces
│   └── home_robotics_task_executor
└── docs/                         # Supervisor-facing technical documentation
```

Historical specifications and raw validation outputs are retained under
`docs/archive/` rather than being removed.

## Development environment

Primary development used an Apple MacBook with an Apple M4 chip, 16 GB unified
memory, and a 512 GB SSD. Robotics development ran primarily in an Ubuntu 24.04
ARM64 virtual machine with ROS 2 Jazzy, MuJoCo, MoveIt 2, and ros2_control.

This is development context, not a performance benchmark. Apple Silicon and the
available local memory influenced the scope: VLA/LLM-heavy work is deferred.

## Limitations

- Simulation only; there is no real-robot deployment.
- Panda1 is the primary validated manipulation robot.
- Object state is ground truth from simulation; no perception pipeline exists.
- There is no coordinated dual-arm manipulation or multi-task scheduler.
- The validated named-placement set is limited to the cube.
- The Task Executor delegates Phase 4 through a process-level ROS launch
  subprocess; it is a prototype boundary, not a reusable in-process API.
- Execution-mode cancellation is limited to supported safe boundaries.
- VLA/LLM and semantic vision are not implemented.

## Future work

Not implemented: Panda2 joystick teleoperation with a Logitech F710, human
demonstration recording, extraction of tasks from demonstrations, Panda1
reproduction of demonstrated tasks, perception, dual-arm coordination, VLA/LLM
integration, and sim-to-real work.

## License and attribution

Project license has not yet been finalized. Existing package license fields are
unchanged. Panda mesh assets are intentionally duplicated for MuJoCo and
ROS/URDF packaging; their source and license material remain beside each copy.
No asset deduplication was performed for this review.
