# Home Robotics — Codex Implementation Rules

## 1. Purpose

This document defines the mandatory implementation rules for Codex and any other coding agent working on Home Robotics.

Codex should treat the repository documentation as a project contract, not optional background reading.

---

## 2. Required Reading Order

Before modifying implementation, read the relevant documents.

General order:

```text
PROJECT_BRIEF.md
SYSTEM_ARCHITECTURE.md
DESIGN_DECISIONS.md
SCENE_SPECIFICATION.md
SCENE_MAP.md
COORDINATE_FRAMES.md
OBJECT_REGISTRY.md
MUJOCO_PHYSICS_SPEC.md
ROBOT_AND_GRIPPER_SPEC.md
PICK_PLACE_SPEC.md
ROS2_ARCHITECTURE.md
TASK_EXECUTOR_API.md
TEST_STRATEGY.md
FAILURE_AND_RECOVERY.md
```

For a narrow change, still inspect all directly affected contracts.

---

## 3. Project Source of Truth

The primary configuration files are:

```text
config/scene.yaml
config/robots.yaml
config/objects.yaml
config/locations.yaml
config/grasp_profiles.yaml
config/physics.yaml
```

Do not duplicate their values into source code.

---

## 4. Architectural Rule

Preserve dependency direction:

```text
MuJoCo
↓
Simulation Integration
↓
ros2_control
↓
MoveIt
↓
Manipulation
↓
Task Executor
↓
Agent
```

Do not create shortcuts across layers.

---

## 5. Rail Rule

Exactly one fixed physical shared rail supports two independently controlled carriages, one per Panda.

Each carriage joint is:

```text
a prismatic robot joint
```

not:

```text
a visual prop
a teleport mechanism
a hard-coded table selector
```

Never directly rewrite Panda world pose during runtime. Panda bases may move only through their own carriage prismatic joints.

The configured minimum carriage separation must always be preserved, panda1 must remain before panda2 along +X, and carriage crossing is prohibited.

---

## 6. Coordinate Rule

Preserve:

```text
world
X forward
Y left
Z up
```

Do not introduce a second world convention.

---

## 7. Frame Rule

Every pose must have a known frame.

Use transform utilities.

Do not manually combine world-space offsets when a semantic frame exists.

---

## 8. Object Rule

Use canonical object IDs:

```text
cube
apple
purple_ball
bowl
pan
```

Do not invent aliases inside execution code.

Natural-language aliases belong to the agent layer.

---

## 9. No Magic Numbers

Do not write unexplained values such as:

```python
target_z += 0.037
```

If a value has physical meaning, place it in the appropriate configuration.

Examples:

```text
grasp clearance
approach distance
placement tolerance
rail limit
```

---

## 10. No Object-Specific Hacks

Forbidden:

```python
if object_id == "apple":
    rail = 0.64
```

Correct:

```text
object pose
→ grasp profile
→ rail-aware planning
```

---

## 11. Physics Rule

Do not fix manipulation by cheating physics.

Forbidden:

```text
reduce gravity
freeze object
disable collision
make object mass nearly zero
attach before contact
```

---

## 12. Collision Rule

Keep physical collision active.

Do not globally disable:

```text
robot-table
robot-object
held object-environment
robot-robot
```

---

## 13. Grasp Rule

Required order:

```text
physical closure
↓
contact
↓
verification
↓
stabilization
```

Never:

```text
pick requested
↓
instant weld
```

---

## 14. Scene Rule

Do not move:

```text
shared rail assembly
major work surfaces
world origin
```

to solve a local manipulation bug.

Scene-layout changes require explicit documentation.

---

## 15. Panda 2 Rule

Keep Panda 2 and its carriage active at the low-level control layer from Phase 2; high-level coordination remains reserved for Phase 7.

Keep the architecture compatible with future activation.

---

## 16. ROS 2 Rule

Use:

```text
Topics for continuous state
Services for short queries
Actions for long-running tasks
```

Do not expose raw MuJoCo IDs publicly.

---

## 17. MoveIt Rule

Use MoveIt for normal rail + arm motion planning.

Do not bypass planning with hard-coded trajectories in production code.

Debug utilities must be clearly separated.

---

## 18. Task API Rule

The future LLM uses:

```text
pick
place
move_home
get_scene_state
```

It does not use:

```text
joint commands
rail commands
MuJoCo actuators
```

---

## 19. Error Handling Rule

Never swallow critical errors.

Return or propagate structured failure states.

Do not log an error and continue nominal execution when state is invalid.

---

## 20. Retry Rule

Retries must be bounded.

A retry should change a meaningful choice.

Do not create loops that repeatedly execute the same failed action.

---

## 21. Reset Rule

Reset must clear:

```text
task state
temporary constraints
rail state
arm state
gripper state
object velocity
object state
```

Never leave stale attachment constraints.

---

## 22. Test Rule

Every critical new behavior needs a test.

Every important bug fix should gain a regression test where practical.

Never delete or weaken a test merely to pass a phase.

---

## 23. Phase Rule

Do not implement future-phase features prematurely.

Examples:

```text
no LLM in Phase 1
no perception in Phase 4
no dual-arm control before Phase 7
```

---

## 24. Documentation Rule

If a change modifies a documented architectural contract:

```text
update the document
```

in the same logical change.

Do not let code silently diverge from docs.

---

## 25. File Placement Rule

Put code in the package / directory that owns its responsibility.

Do not create random helper files at repository root.

Keep simulator-specific code localized.

---

## 26. Dependency Rule

Avoid circular dependencies.

Lower layers must not depend on higher layers.

---

## 27. Naming Rule

Use descriptive stable names.

Avoid:

```text
temp
final2
new
test123
object1
```

for production entities.

---

## 28. Commit Scope Rule

Prefer small, coherent commits.

Examples:

```text
feat(sim): add panda1 rail model
fix(scene): correct bowl collision geometry
test(control): add rail limit regression test
docs(frames): clarify rail carriage transform
```

---

## 29. Benchmark Integrity Rule

Do not change:

```text
physics
scene
planner
grasp thresholds
```

mid-benchmark without versioning the experiment.

---

## 30. Debugging Order

When manipulation fails, investigate:

```text
1. frame
2. scene geometry
3. collision geometry
4. state synchronization
5. reachability
6. controller behavior
7. grasp geometry
8. physics parameters
9. solver tuning
```

Do not randomly tune several layers at once.

---

## 31. Do Not Guess Missing Architecture

If documentation does not define a critical behavior:

```text
stop
report the ambiguity
propose options
```

Do not silently invent a project-wide convention.

---

## 32. Preserve Academic Quality

Implementation should favor:

```text
reproducibility
clarity
measurement
modularity
```

over a quick one-off demo.

---

## 33. Prohibited Patterns Summary

Never introduce:

```text
Panda base teleportation
instant object attachment
global collision disabling
gravity hacks
object freezing during grasp
unexplained coordinate offsets
duplicate scene truth
duplicate TF publishers
raw MuJoCo IDs in public APIs
LLM motor control
unbounded retry loops
silent architecture changes
```

---

## 34. Completion Report

After a significant Codex implementation task, report:

```text
files changed
architecture affected
tests added
tests run
results
known issues
phase status
```

Do not report success without execution evidence.

---

## 35. Final Rule

The governing principle for Codex is:

> Implement the documented system, not the fastest workaround that makes one demo pass.
