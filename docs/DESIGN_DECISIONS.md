# Home Robotics — Design Decisions

## 1. Purpose

This document records the major architectural and engineering decisions for the Home Robotics project.

Its purpose is to prevent the project from drifting into inconsistent implementations during development.

Each decision includes:

- the decision itself,
- the reason for the decision,
- alternatives that were considered,
- the consequences of the decision,
- and whether the decision is considered stable or revisitable.

This file should be treated as an architectural decision record for the project.

---

## 2. Decision Summary

The current project decisions are:

| Area | Decision |
|---|---|
| Repository | `home-robotics` |
| Primary simulator | MuJoCo |
| Middleware | ROS 2 Jazzy |
| Motion planning | MoveIt 2 |
| Robot | 2 × Franka Panda on independent linear rails |
| Early active robot count | Panda 1 + Rail 1 |
| Gripper | Standard Franka Hand |
| Grasp strategy | Hybrid physical grasp + temporary stabilization constraint |
| Units | SI units, meters for distance |
| World axes | X forward, Y left, Z up |
| Scene philosophy | Final-layout-first |
| Perception in early phases | MuJoCo ground-truth state |
| LLM placement | High-level task planning only |
| LLM provider | Provider-independent, local-first preferred |
| Academic goal | Research-ready + portfolio-quality |
| Configuration strategy | Central source-of-truth YAML files |
| Dual-arm activation | Later phase |
| Physics priority | Stability and reproducibility over visual spectacle |

---

## 3. DD-001 — Use MuJoCo as the Primary Simulator

### Decision

MuJoCo is the primary simulation and physics engine for Home Robotics.

### Rationale

The project runs in an ARM64 Ubuntu virtual machine with constrained resources.

Expected environment:

```text
CPU: 4 cores
RAM: 8 GB
GPU: no NVIDIA GPU
```

The project needs:

- low memory usage,
- efficient CPU simulation,
- stable rigid-body dynamics,
- reliable contact simulation,
- good manipulation physics,
- ARM64 compatibility,
- deterministic experiments,
- and future suitability for robotics research.

MuJoCo matches these requirements well.

### Alternatives Considered

#### Gazebo Harmonic

Advantages:

- stronger native ROS ecosystem,
- mature ROS 2 integration,
- mature ros2_control workflows,
- convenient robotics visualization,
- well-established use with MoveIt.

Disadvantages for this project:

- heavier runtime,
- higher memory pressure,
- greater risk of exceeding the VM's resource budget,
- visual capabilities are not the primary objective,
- less attractive for future manipulation-learning experiments.

#### Isaac Sim

Rejected because:

- NVIDIA GPU is unavailable,
- runtime requirements do not fit the development environment,
- the current project does not need high-end photorealistic simulation.

### Consequences

The project must explicitly solve:

- MuJoCo ↔ ROS 2 integration,
- ros2_control integration,
- MoveIt planning-scene synchronization,
- gripper/contact state bridging,
- and simulator-side constraint management.

These are accepted engineering costs.

### Stability

**Stable.**

Changing the main simulator should require a formal architectural revision.

---

## 4. DD-002 — Use ROS 2 Jazzy as the Robotics Middleware

### Decision

The project uses ROS 2 Jazzy.

### Rationale

The development environment already uses:

```text
Ubuntu 24.04
ROS 2 Jazzy
```

The project requires:

- modular ROS 2 nodes,
- Actions and Services,
- ros2_control,
- MoveIt 2,
- TF,
- launch systems,
- and future compatibility with real robot interfaces.

### Consequences

All core robotics interfaces should use ROS 2-native concepts.

The project should avoid introducing unnecessary custom IPC mechanisms where ROS 2 already provides a suitable abstraction.

### Stability

**Stable.**

---

## 5. DD-003 — Use Two Franka Panda Robots in the Final Scene

### Decision

The final scene contains two Franka Panda robot arms, each mounted on an independent linear side rail.

### Rationale

The project is intended to evolve toward dual-arm home manipulation.

The physical layout should therefore include both robots from the beginning so that:

- furniture does not need to be redesigned later,
- rail mounting positions remain stable,
- workspace boundaries can be defined early,
- future shared-workspace planning remains possible,
- and visual identity remains consistent.

### Early-Phase Behavior

Panda 1 and Rail 1 are active initially.

Panda 2 and Rail 2 are present but inactive.

### Why Not Start With Two Active Robots?

Two active robots would immediately introduce:

- controller namespaces,
- rail coordination,
- shared planning complexity,
- mutual collision checking,
- task allocation,
- synchronization,
- resource locking,
- and larger debugging surfaces.

These are not required for proving the basic manipulation stack.

### Stability

**Stable.**

---

## DD-RAIL — Mount Each Panda on an Independent Linear Rail

### Decision

Each Franka Panda is mounted on its own independently actuated linear rail.

The nominal rail axis is aligned with world X so the carriage can move along the near / middle / far table sequence.

### Rationale

A fixed-base Panda cannot reliably cover the complete longitudinal workspace.

The rail increases usable workspace without forcing:

- extreme joint configurations,
- unrealistic arm reach,
- repeated scene redesign,
- or moving objects closer only to satisfy reachability.

### Kinematic Consequence

The normal arm-planning chain is:

```text
1 prismatic rail DOF
+
7 Panda arm DOF
```

The Franka Hand remains the end effector.

### Planning Rule

The rail is part of the robot and should participate in motion planning and collision checking.

It must not be used as an out-of-band teleport mechanism.

### Dual-Arm Consequence

Both rails are independent.

Future dual-arm execution must consider:

- rail travel limits,
- carriage positions,
- shared workspace occupancy,
- cross-robot collision,
- and simultaneous rail motion.

### Stability

**Stable.**

---

## 6. DD-004 — Use the Standard Franka Hand

### Decision

Each Panda uses the standard Franka Hand two-finger gripper.

### Rationale

Using the standard gripper avoids unnecessary model and integration complexity.

It provides:

- realistic Franka configuration,
- known geometry,
- known finger kinematics,
- a simple parallel-gripper abstraction,
- and direct compatibility with later real-hardware reasoning.

### Alternatives Rejected

Additional grippers such as Robotiq models are intentionally excluded from the initial design.

### Stability

**Stable.**

---

## 7. DD-005 — Use a Hybrid Grasping Strategy

### Decision

Grasping uses a hybrid strategy:

```text
physical approach
    ↓
real finger closure
    ↓
contact detection
    ↓
grasp verification
    ↓
temporary stabilization constraint
    ↓
transport
```

### Rationale

Purely physical grasping is realistic but can become fragile due to:

- small contact-model differences,
- solver tuning,
- friction sensitivity,
- object geometry,
- small numerical errors,
- and simulation timestep choices.

Pure attachment-based grasping is deterministic but unrealistic.

For example, the following is prohibited:

```text
pick("apple")
    ↓
instant attach
```

The hybrid strategy provides a balance between realism and reproducibility.

### Required Rule

A stabilization constraint can only be enabled after a valid physical grasp has been verified.

### Release

During placement:

```text
lower object
    ↓
open gripper
    ↓
remove stabilization constraint
    ↓
verify support/contact
```

### Stability

**Stable for the core project.**

Advanced research may later compare this with fully physical grasping.

---

## 8. DD-006 — Use Ground-Truth Object State Before Perception

### Decision

Early project phases use MuJoCo ground-truth state for object pose and physical state.

### Rationale

The primary early research problem is robotic manipulation, not visual perception.

Adding perception too early would create simultaneous uncertainty in:

- object detection,
- pose estimation,
- coordinate transforms,
- grasp planning,
- motion planning,
- and physical execution.

This would make debugging unnecessarily difficult.

### Early State Source

The simulation may provide:

```text
object pose
object orientation
object velocity
contact state
support state
held state
```

### Future Architecture

The task and manipulation layers should depend on an abstract scene-state interface.

This allows the source to change later from:

```text
MuJoCo Ground Truth
```

to:

```text
Camera + Perception
```

without rewriting task logic.

### Stability

**Stable for early phases.**

Perception is intentionally deferred.

---

## 9. DD-007 — Use Final-Layout-First Scene Design

### Decision

The major scene layout should be designed correctly in Phase 1 and remain stable.

### Rationale

The project should not begin with a throwaway scene and later replace it with a completely different environment.

Changing major geometry later would invalidate:

- object positions,
- workspace regions,
- reachability assumptions,
- collision tests,
- benchmark results,
- grasp clearances,
- and robot base placements.

### Stable Elements

The following should remain stable after Phase 1 acceptance:

```text
world origin
robot base positions
major tables
worktop dimensions
main destination regions
shared workspace location
major furniture placement
```

### Allowed Later Improvements

The following may change without architectural redesign:

```text
textures
visual meshes
decorative assets
non-functional scene details
minor rendering improvements
```

### Stability

**Stable.**

---

## 10. DD-008 — Use Simplified Collision Geometry

### Decision

Visual geometry and collision geometry may differ.

### Principle

```text
visual geometry → detailed
collision geometry → simple and stable
```

### Rationale

Using complex visual meshes directly as collision geometry can cause:

- unstable contacts,
- excessive solver cost,
- unexpected collision normals,
- penetration artifacts,
- and difficult debugging.

Collision geometry should prefer:

- boxes,
- cylinders,
- spheres,
- capsules,
- convex shapes,
- or carefully simplified meshes.

### Example

A detailed pan may use a high-quality mesh for rendering, while its collision model may use:

```text
thin cylinder
+
handle capsule
```

### Stability

**Stable.**

---

## 11. DD-009 — Use Real Metric Scale

### Decision

All physical scene dimensions use real-world metric scale.

```text
1 MuJoCo unit = 1 meter
```

### Rationale

Artificially scaling the world to make planning or grasping easier creates hidden errors.

Real scale improves:

- physical realism,
- model reuse,
- grasp parameter interpretation,
- robot reach reasoning,
- sim-to-real compatibility,
- and academic reproducibility.

### Rule

If an apple is difficult to grasp, do not make the apple artificially large.

Fix:

- grasp pose,
- finger geometry,
- collision geometry,
- controller tuning,
- or friction.

### Stability

**Stable.**

---

## 12. DD-010 — Use X Forward, Y Left, Z Up

### Decision

The global world frame follows:

```text
X → forward
Y → left
Z → up
```

### Rationale

A single clear frame convention reduces ambiguity across:

- MuJoCo,
- TF,
- MoveIt,
- scene mapping,
- object configuration,
- and debugging.

### Root Frame

```text
world
```

### Robot Naming

```text
panda1_base
panda1_link0
...
panda1_hand
panda1_tcp

panda2_base
panda2_link0
...
panda2_hand
panda2_tcp
```

### Stability

**Stable.**

---

## 13. DD-011 — Use Central Configuration as Source of Truth

### Decision

Scene and manipulation parameters are stored in centralized configuration files.

Primary files:

```text
config/scene.yaml
config/robots.yaml
config/objects.yaml
config/locations.yaml
config/grasp_profiles.yaml
config/physics.yaml
```

### Rationale

Scattered hard-coded values create:

- duplicated truth,
- configuration drift,
- fragile experiments,
- difficult debugging,
- and inconsistent documentation.

### Example

Bad:

```python
APPLE_POSITION = [0.51, 0.22, 0.84]
```

inside a manipulation node.

Good:

```text
objects.yaml
    ↓
Object Registry
    ↓
Manipulation Layer
```

### Stability

**Stable.**

---

## 14. DD-012 — Keep LLM Planning Above Robot Execution

### Decision

The LLM is a symbolic task planner, not a motion controller.

### Allowed LLM Output

Examples:

```text
pick("apple")
place("bowl")
move_home("panda1")
```

### Prohibited LLM Output

The LLM must not directly control:

```text
joint positions
joint velocities
torques
trajectory points
raw TCP poses for execution
MuJoCo actuator values
controller commands
```

### Rationale

Allowing the LLM to control low-level motion would:

- reduce determinism,
- weaken safety boundaries,
- make failures difficult to diagnose,
- mix semantic reasoning with kinematics,
- and reduce reproducibility.

### Stability

**Stable and non-negotiable.**

---

## 15. DD-013 — Keep the LLM Provider Independent

### Decision

The LLM interface must not depend on one vendor or runtime.

### Preferred Direction

Local execution is preferred when practical.

Microsoft Foundry Local is a strong candidate for later integration.

### Possible Backends

```text
Microsoft Foundry Local
local OpenAI-compatible servers
remote OpenAI-compatible APIs
other tool-calling models
```

### Architectural Rule

The Task Executor API must remain identical regardless of LLM provider.

### Stability

**Stable.**

The specific model may change freely.

---

## 16. DD-014 — Use MoveIt 2 for Normal Arm Motion

### Decision

MoveIt 2 is the standard motion-planning path for arm movement.

### Responsibilities

MoveIt handles:

```text
IK
joint planning
Cartesian planning
collision checking
planning scene
trajectory execution
```

### Rationale

The project should not maintain a separate parallel motion-planning implementation unless a research experiment explicitly requires one.

### Exceptions

Direct control may later be used for narrowly defined behaviors such as:

- low-level gripper commands,
- controlled research experiments,
- emergency recovery,
- specialized local servoing.

Such exceptions must be explicit.

### Stability

**Stable.**

---

## 17. DD-015 — Treat Pick and Place as State Machines

### Decision

Pick and place are explicit stateful behaviors.

### Pick

```text
OBJECT_LOOKUP
PRE_GRASP
APPROACH
GRIPPER_CLOSE
GRASP_VERIFY
GRASP_STABILIZE
LIFT
VERIFY_HELD
SUCCESS
```

### Place

```text
TARGET_LOOKUP
PRE_PLACE
APPROACH_PLACE
LOWER
GRIPPER_OPEN
REMOVE_STABILIZATION
PLACEMENT_VERIFY
RETREAT
SUCCESS
```

### Rationale

A sequential script with no state validation tends to hide failures.

State machines allow:

- precise failure reporting,
- retries,
- cancellation,
- recovery,
- benchmarking,
- and observability.

### Stability

**Stable.**

---

## 18. DD-016 — A Successful Task Must Preserve Scene Integrity

### Decision

Task success requires both:

```text
goal achieved
+
no unacceptable collateral interaction
```

### Example

This is not a valid success:

```text
apple placed in bowl
BUT
pan knocked off table
```

### Failure Conditions May Include

```text
unrelated object displacement
unintended object fall
table collision
robot collision
object penetration
object drop
workspace violation
```

### Rationale

Home manipulation must be evaluated as a scene-level task, not only an end-position task.

### Stability

**Stable.**

---

## 19. DD-017 — Deterministic Reset Is a Core Feature

### Decision

Reset is treated as part of the simulation architecture, not as a debugging convenience.

### Reset Must Restore

```text
robot configuration
gripper state
object poses
object velocities
constraint state
controller state
task state
```

### Rationale

Research experiments require repeatable initial conditions.

Without deterministic reset:

- benchmarks are unreliable,
- regression testing becomes weak,
- failures become difficult to reproduce.

### Initial Milestone

A scene should survive repeated reset testing before manipulation development proceeds.

### Stability

**Stable.**

---

## 20. DD-018 — Do Not Activate Panda 2 Prematurely

### Decision

Panda 2 remains inactive until the dual-arm phase.

### Rationale

The second robot should not increase early complexity without providing immediate value.

The architecture will prepare:

```text
namespaces
workspace definitions
collision geometry
base pose
future controller configuration
```

but actual controller activation is deferred.

### Stability

**Stable until Phase 7.**

---

## 21. DD-019 — Use Explicit Robot Namespaces

### Decision

Robot-specific interfaces must include robot identity.

Conceptually:

```text
/panda1/...
/panda2/...
```

### Rationale

Ambiguous topics and controllers become a major source of failure in multi-robot systems.

Robot namespace isolation is required from the beginning.

### Stability

**Stable.**

---

## 22. DD-020 — Build for Academic Reproducibility

### Decision

The repository is not treated only as a demo.

It must support measurable experiments.

### Required Characteristics

The project should eventually report:

```text
success rates
failure rates
planning time
execution time
collision rate
grasp failure rate
placement accuracy
reset stability
recovery rate
```

### Rationale

A portfolio project becomes substantially stronger when engineering claims are backed by reproducible evidence.

This also keeps the project compatible with future academic work.

### Stability

**Stable.**

---

## 23. DD-021 — Delay Reinforcement Learning

### Decision

Reinforcement learning is not part of the core manipulation pipeline.

### Rationale

The project should first establish a deterministic classical robotics baseline.

This baseline provides:

- a reliable comparison point,
- a working system,
- known failure categories,
- and a stable research platform.

Only later should learning-based methods be introduced.

### Stability

**Stable for the core phases.**

---

## 24. DD-022 — Delay Camera-Based Perception

### Decision

Camera-based perception is deferred to the research-extension phase.

### Rationale

The project should first isolate manipulation failures from perception failures.

This allows the team to answer:

```text
Can the robot manipulate correctly when object pose is known?
```

before asking:

```text
Can the robot infer the object pose from vision?
```

### Stability

**Stable for early phases.**

---

## 25. DD-023 — Avoid Scene-Specific Hacks

### Decision

A one-off coordinate correction should not be added simply because one object is difficult to grasp.

### Prohibited Pattern

```python
if object_name == "apple":
    target_x += 0.037
    target_z += 0.019
```

unless the adjustment represents a documented semantic property or grasp profile.

### Correct Alternatives

Use:

```text
object geometry
grasp profile
frame transform
object registry
grasp candidate generation
```

### Rationale

Scene-specific hacks destroy generality and become impossible to maintain.

### Stability

**Stable.**

---

## 26. DD-024 — Use Object-Specific Grasp Profiles Where Necessary

### Decision

Objects may have semantic grasp profiles.

### Example

```text
apple
  → side grasp / top grasp candidates

cube
  → face-aligned grasps

purple_ball
  → symmetric grasp candidates

pan
  → handle grasp in future
```

### Rationale

Avoiding hacks does not mean every object must use an identical grasp.

Object geometry legitimately affects grasp strategy.

These differences belong in:

```text
config/grasp_profiles.yaml
```

or the grasp-generation layer.

### Stability

**Stable.**

---

## 27. DD-025 — Prioritize Stability Over Rendering Quality

### Decision

Physics stability and manipulation reliability have priority over rendering quality.

### Rationale

The project is a robotics platform, not a graphics benchmark.

Visual quality is still important for portfolio presentation, but should not compromise:

- stable collisions,
- CPU budget,
- memory budget,
- deterministic simulation,
- or manipulation behavior.

### Stability

**Stable.**

---

## 28. DD-026 — Keep Simulator-Specific Code Localized

### Decision

MuJoCo-specific API calls must be concentrated in simulator integration modules.

### Rationale

The project is committed to MuJoCo, but higher-level code should remain conceptually reusable.

For example:

Bad:

```text
Task Executor
   ↓
mjData.xpos
```

Good:

```text
Task Executor
   ↓
SceneStateProvider
   ↓
MuJoCo implementation
```

### Stability

**Stable.**

---

## 29. DD-027 — Use Named Semantic Locations

### Decision

Task-level placement targets should use names rather than raw coordinates.

Example:

```text
place("bowl")
place("left_bin")
place("counter_2")
```

instead of:

```text
place([0.62, -0.41, 0.83])
```

### Rationale

Semantic locations:

- are easier for LLM planning,
- reduce hard-coded coordinates,
- support future perception,
- and make tasks interpretable.

Coordinates belong inside the location registry.

### Stability

**Stable.**

---

## 30. DD-028 — Separate Functional Scene Data from Visual Assets

### Decision

A visual asset is not itself the authoritative scene definition.

### Rationale

Meshes may change.

Functional data must remain available independently.

For example:

```text
table visual mesh
```

may change while:

```text
table pose
table work surface
table collision bounds
```

remain stable in configuration.

### Stability

**Stable.**

---

## 31. DD-029 — Use Failure-Aware Execution

### Decision

Failures must return structured results rather than silently retry forever.

### Example

```text
NO_VALID_GRASP
PLANNING_FAILED
GRIPPER_FAILED
OBJECT_DROPPED
PLACE_VERIFICATION_FAILED
```

### Rationale

Structured failure reporting is necessary for:

- debugging,
- recovery,
- benchmarking,
- LLM replanning,
- and research analysis.

### Stability

**Stable.**

---

## 32. DD-030 — Keep Recovery Deterministic

### Decision

Recovery behavior should initially use explicit deterministic rules.

Example:

```text
grasp verification failed
        ↓
open gripper
        ↓
retreat
        ↓
return failure
```

Later versions may support bounded retries.

### Rationale

Unbounded autonomous retries can hide systematic errors.

### Stability

**Stable for early phases.**

---

## 33. Decision Governance

A stable decision should not be changed casually during implementation.

If a stable decision must change:

1. explain the problem,
2. identify the affected modules,
3. document the alternative,
4. update this file,
5. update dependent specifications,
6. add migration notes if required,
7. and commit the architectural change separately.

Example commit:

```text
docs(architecture): revise grasp stabilization strategy
```

Architectural changes should not be hidden inside unrelated implementation commits.

---

## 34. Decisions Intentionally Left Open

The following details remain intentionally unresolved until their implementation phase:

- exact MuJoCo ROS 2 integration package,
- exact controller configuration,
- exact MoveIt kinematics plugin,
- exact motion-planning pipeline,
- exact LLM model,
- exact Foundry Local model integration,
- camera model for future perception,
- dual-arm task-allocation policy,
- object-handover strategy,
- learning framework for future RL experiments.

These are implementation decisions, not core project identity decisions.

They should be selected when enough evidence exists.

---

## 35. Final Principle

The project should optimize for:

```text
clarity
stability
reproducibility
modularity
physical plausibility
research value
```

rather than for the fastest possible demo.

When two implementation choices are available, prefer the one that makes the system easier to:

- reason about,
- test,
- reproduce,
- benchmark,
- and extend.

That principle governs the design decisions recorded in this document.
