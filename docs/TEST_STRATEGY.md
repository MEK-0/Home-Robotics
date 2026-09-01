# Home Robotics — Test Strategy

## 1. Purpose

This document defines the test strategy for Home Robotics.

Testing is a core project feature because the system combines:

```text
physics
robot control
motion planning
manipulation
task execution
LLM orchestration
```

Failures must be isolated to the correct layer.

---

## 2. Test Pyramid

The project uses multiple test levels:

```text
configuration validation
unit tests
component tests
integration tests
simulation tests
manipulation tests
regression tests
benchmark tests
end-to-end tests
```

No single demo replaces these layers.

---

## 3. Test Directories

Repository-level tests live under:

```text
tests/
```

ROS package tests may also live inside package-specific test directories.

Suggested structure:

```text
tests/
├── config/
├── simulation/
├── control/
├── moveit/
├── manipulation/
├── integration/
├── regression/
└── benchmarks/
```

---

## 4. Configuration Tests

Validate:

```text
unique canonical IDs
valid object references
valid support surfaces
valid workspace references
valid grasp profiles
valid robot references
valid locations
valid rail limits
valid physics schema
```

Configuration errors should fail before simulation starts.

---

## 5. Scene Mapping Tests

Verify:

```text
all documented scene IDs exist
six work surfaces exist
one shared rail and two carriages exist
two carriages exist
two Pandas exist
initial objects exist
bowl interior exists
shared workspace exists
handover zone exists
```

---

## 6. Frame Tests

Verify:

```text
world root exists
rail frames exist
carriage frames exist
Panda frames exist
TCP frames exist
no duplicate TF publishers
quaternions valid
rail axis correct
```

---

## 7. Rail Transform Test

Command a known rail displacement.

Expected:

```text
carriage translation matches rail joint displacement
Panda base transform changes accordingly
no unexpected rotation
```

---

## 8. FK Consistency Test

Compare:

```text
MuJoCo TCP pose
TF TCP pose
MoveIt FK TCP pose
```

at known robot configurations.

Differences must remain within documented tolerance.

---

## 9. Physics Tests

Core physics tests:

```text
idle stability
drop test
ball roll test
bowl containment
rail hold
joint hold
gripper contact
constraint activation
constraint release
```

---

## 10. 100-Reset Test

Phase 1 requires:

```text
100 reset cycles
```

Each cycle verifies:

```text
same rail reset states
same arm reset states
same object poses
zero stale constraints
zero invalid collision
zero unexplained drift
```

---

## 11. Initial Collision Test

At reset:

```text
0 illegal robot self-collision
0 robot-table penetration
0 rail-furniture penetration
0 object-object penetration
0 object-furniture penetration
```

Expected support contacts are allowed.

---

## 12. Control Tests

Validate:

```text
rail command
rail limit enforcement
rail hold
arm trajectory
arm hold
gripper open
gripper close
controller readiness
```

---

## 13. Rail Limit Test

Commands beyond:

```text
q_min
q_max
```

must be rejected or safely bounded according to the control design.

No physical geometry should cross mechanical limits.

---

## 14. MoveIt Tests

Validate:

```text
current robot state
home planning
near target
far target requiring rail
collision rejection
planning-scene sync
attached object
Cartesian approach
```

---

## 15. Far-Reach Test

Create a target that:

```text
cannot be reached from fixed rail home
```

but:

```text
can be reached with valid rail motion
```

Expected:

```text
MoveIt finds rail + arm solution
```

This is a mandatory rail-architecture test.

---

## 16. False Reach Test

Create an actually unreachable target.

Expected:

```text
planner fails cleanly
```

The system must not:

```text
expand limits
teleport base
ignore collisions
```

---

## 17. Gripper Tests

Core gripper tests:

```text
open/close repeatability
empty close
cube contact
two-finger contact
false grasp
release
```

---

## 18. False Grasp Test

Close the gripper near but not around the cube.

Expected:

```text
grasp_verified = false
constraint remains disabled
lift does not begin
```

---

## 19. Constraint Snap Test

Attempt constraint activation from an invalid relative pose.

Expected:

```text
activation rejected
```

No object should snap into the gripper.

---

## 20. Pick Tests

Object progression:

```text
cube
apple
purple_ball
```

Each object must have repeated pick trials.

---

## 21. Place Tests

Initial destination:

```text
bowl
```

Verify:

```text
object inside valid region
constraint removed
object stable
object not held
```

---

## 22. Scene Integrity Tests

For each task, measure unrelated-object movement.

Example:

```text
apple → bowl
```

Track:

```text
cube
purple_ball
pan
```

Significant movement should be flagged.

---

## 23. Task API Tests

Validate:

```text
PickObject success
PickObject invalid object
PlaceObject without held object
PlaceObject invalid location
MoveHome
cancellation
robot busy
structured failure result
```

---

## 24. Reset During Fault Test

After a failed grasp or collision:

```text
reset
```

must restore a clean baseline.

This specifically checks stale constraints and task state.

---

## 25. Regression Tests

Any previously fixed bug should gain a regression test when practical.

Examples:

```text
bowl collision was solid
rail direction reversed
object constraint persisted after reset
TCP offset mismatch
planning scene stale after place
```

---

## 26. Test Naming

Use descriptive names.

Good:

```text
test_reset_clears_grasp_constraint
test_far_target_requires_rail_motion
test_false_grasp_does_not_attach_object
```

Bad:

```text
test1
test_robot
new_test
```

---

## 27. Determinism

Baseline tests should use:

```text
fixed scene
fixed physics
fixed initial object poses
fixed planner config
```

Randomized tests belong to later research stages.

---

## 28. Tolerances

Every numeric test tolerance should have semantic meaning.

Examples:

```text
TCP position tolerance
rail position tolerance
reset object pose tolerance
placement containment tolerance
```

Do not scatter arbitrary tolerances across source files.

---

## 29. Test Evidence

Failed integration tests should capture enough evidence to reproduce the problem.

Useful data:

```text
scene version
physics version
task ID
robot state
object state
failure code
contact summary
```

---

## 30. Benchmark vs Test

A test answers:

```text
Does the system satisfy a required condition?
```

A benchmark answers:

```text
How well does the system perform repeatedly?
```

Do not mix these concepts.

---

## 31. CI Strategy

As the project matures, lightweight tests should run automatically.

Potential CI-safe tests:

```text
configuration validation
schema validation
pure unit tests
static launch/config checks
```

Full MuJoCo / MoveIt integration may require dedicated CI setup.

---

## 32. Headless Test Requirement

Simulation and integration tests should be runnable headless.

GUI interaction must not be required for pass/fail evaluation.

---

## 33. Phase Gates

### Phase 1 Gate

```text
100 resets pass
scene stable
```

### Phase 2 Gate

```text
rail + arm + gripper controlled reliably
```

### Phase 3 Gate

```text
rail-aware planning passes
```

### Phase 4 Gate

```text
verified repeated pick/place
```

### Phase 5 Gate

```text
task Actions validated
```

### Phase 6 Gate

```text
LLM invokes only task-level tools
```

---

## 34. Codex Testing Rules

Codex must:

1. Add tests with new critical behavior.
2. Never delete a failing test to make a phase pass.
3. Never loosen tolerances without justification.
4. Preserve deterministic baselines.
5. Keep tests headless where practical.
6. Add regression tests for fixed bugs.
7. Validate scene/config references.
8. Test rail-specific behavior explicitly.
9. Test false grasp cases.
10. Test reset after failures.

---

## 35. Final Principle

The project does not trust successful-looking animation.

It trusts:

```text
state
verification
repeatability
tests
measured results
```

The core rule is:

> If a behavior cannot be tested, it is not yet a reliable project capability.
