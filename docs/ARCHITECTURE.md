# Architecture

```text
MuJoCo
  ↓ state, object pose, contact
ROS 2 runtime / MuJoCo hardware interface
  ↓
ros2_control
  ↓
MoveIt 2
  ↓
Panda1 manipulation layer
  ↓
Task Executor
```

The direction above describes runtime responsibility, not a claim that every
layer calls the next one directly. A task client calls the Task Executor; the
executor delegates the current Phase 4 pipeline, which uses MoveIt and standard
controller actions through ros2_control.

## Authority boundaries

- **MuJoCo** is authoritative for physics, object poses, contacts, and simulation
  time. The runtime publishes those observations.
- **MoveIt 2** is authoritative for motion planning and geometric collision
  reasoning. It receives static environment geometry and synchronized dynamic
  object poses in its PlanningScene.
- **ros2_control** is the execution boundary. Controllers expose standard
  trajectory/gripper interfaces and exchange samples with the MuJoCo-backed
  hardware interface.
- **Manipulation** implements the Panda1 cube sequence: preflight, grasp
  verification, temporary stabilization, lift, transport, placement, release,
  and verification.
- **Task Executor** owns the Phase 5 Action boundary, request validation,
  feedback/results, and one-active-task policy. It does not own MuJoCo or raw
  MoveIt internals.

## Safety and scene model

The static PlanningScene represents tables, rail geometry, and other fixed
environment elements. Dynamic objects are synchronized from MuJoCo. During a
verified grasp, the selected object is moved between WORLD and ATTACHED state
in the PlanningScene, while MuJoCo remains the physical authority.

Shared-rail separation and geometric collision checking are separate layers.
The hardware/control boundary enforces a 0.70 m carriage separation; MoveIt
checks robot and environment geometry. Neither layer replaces the other.

## Current prototype boundary

The `PickAndPlace` Action is a real ROS interface. Its Phase 4 execution is
currently invoked through a process-level `ros2 launch` subprocess so the
validated pipeline can be reused without changing it. A future refactor may
expose that pipeline as a direct reusable library/API. That refactor is not part
of the current validated scope.
