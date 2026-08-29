# Home Robotics — MuJoCo Physics Specification

## 1. Purpose

This document defines the physics contract for the Home Robotics MuJoCo simulation.

Its purpose is to make the simulation:

- stable,
- reproducible,
- physically plausible,
- suitable for robotic manipulation,
- compatible with the dual linear-rail Franka architecture,
- and resistant to the common failure modes of contact-rich pick-and-place tasks.

This file governs:

- simulation timestep,
- solver strategy,
- gravity,
- damping,
- friction,
- contact parameters,
- object dynamics,
- rail dynamics,
- robot joint dynamics,
- gripper contact,
- temporary grasp stabilization,
- collision policy,
- reset physics,
- and deterministic validation.

Exact numeric values belong in:

```text
config/physics.yaml
```

This document defines the rules and meaning of those values.

---

## 2. Physics Priorities

The project follows this priority order:

```text
1. stability
2. reproducibility
3. physical plausibility
4. manipulation reliability
5. computational efficiency
6. visual realism
```

Rendering quality must never be improved at the cost of unstable physics.

---

## 3. Runtime Constraints

The target development environment is:

```text
Ubuntu 24.04
ARM64
4 CPU cores
8 GB RAM
No NVIDIA GPU
ROS 2 Jazzy
MuJoCo
```

Physics configuration must remain efficient enough for this environment.

The project should avoid:

- unnecessarily high solver iterations,
- excessive dynamic-body counts,
- overly complex collision meshes,
- and tiny timesteps without measured need.

---

## 4. SI Units

All physical quantities use SI units.

```text
distance     → meter
mass         → kilogram
time         → second
velocity     → meter/second
angular rate → radian/second
force        → Newton
torque       → Newton-meter
```

No hidden unit scaling is allowed.

---

## 5. Gravity

Default gravity direction:

```text
-Z
```

with Earth-like magnitude.

Conceptually:

```text
gravity ≈ [0, 0, -9.81]
```

The exact value should remain physically realistic.

Gravity must not be reduced simply to make grasping easier.

---

## 6. Simulation Timestep

The timestep must be chosen for:

- contact stability,
- Franka joint control,
- gripper contact,
- rail motion,
- and acceptable CPU usage.

The project should begin with a conservative robotics-oriented timestep.

Recommended initial design target:

```text
0.001 s to 0.002 s
```

The final value must be validated experimentally.

---

## 7. Timestep Change Rule

The timestep must not be changed casually to fix one failing object.

Any timestep change requires revalidation of:

```text
idle stability
rail control
arm control
gripper behavior
contact stability
grasp success
reset determinism
```

The chosen value is global simulation infrastructure.

---

## 8. Control Frequency

Controller update rates must be compatible with the physics timestep.

The implementation should maintain a clear relationship between:

```text
physics update frequency
ROS controller update frequency
trajectory sampling frequency
scene-state publication frequency
```

These rates do not need to be identical.

They must not create unstable control aliasing.

---

## 9. Solver Strategy

The solver should prioritize stable contact and manipulation.

The project must document:

```text
solver type
iteration count
tolerance
contact settings
```

in `physics.yaml`.

The solver configuration should be tuned globally before object-specific hacks are introduced.

---

## 10. Solver Tuning Principle

If grasp contact is unstable, investigate in this order:

```text
collision geometry
mass / inertia
controller behavior
friction
contact parameters
timestep
solver settings
```

Do not immediately:

```text
increase friction to extreme values
reduce gravity
make objects nearly massless
```

---

## 11. Contact Model

MuJoCo contact is the physical basis of:

```text
object support
finger contact
table contact
container contact
collision detection
```

Contact behavior must remain physically meaningful.

The project must not disable contact globally to simplify motion.

---

## 12. Contact Parameters

The project should explicitly configure or validate:

```text
friction
solref
solimp
contact margin
gap
```

where applicable.

These values must be interpreted as part of a global contact model.

Object-specific overrides should be rare.

---

## 13. Friction Model

Friction must support:

```text
stable tabletop objects
realistic gripper contact
controlled ball rolling
container support
```

The system should avoid both extremes:

```text
too little friction
→ objects constantly slide

too much friction
→ unrealistic sticking
```

---

## 14. Friction Profiles

Recommended semantic friction profiles:

```text
default_tabletop
robot_gripper
round_object
container
rail_mechanism
```

The exact implementation may use shared defaults plus object overrides.

Profiles should be defined in:

```text
config/physics.yaml
```

---

## 15. Tabletop Friction

Tabletop friction should allow objects to remain stationary under gravity and small numerical perturbations.

It must not be so high that objects behave as if glued to the table.

The validation criteria should be behavior-based, not visual.

---

## 16. Gripper Friction

Finger-object friction must support physically plausible grasp formation.

However, successful transport must not depend only on extremely high friction values.

The hybrid grasp architecture provides stabilization only after physical grasp verification.

---

## 17. Purple Ball Friction

The purple ball requires special attention.

It should:

```text
remain stable at reset
roll when physically pushed
not spontaneously drift
not behave as glued to the surface
```

This object is a physics validation case.

---

## 18. Object Mass Policy

Every dynamic object must have a plausible mass.

Initial object classes:

```text
cube
apple
purple_ball
```

should use masses representative of small household objects.

Mass must not be artificially reduced to avoid gripper effort.

---

## 19. Container Mass Policy

Initially fixed containers such as:

```text
bowl
pan
```

may be modeled as static.

If made dynamic later, realistic mass and inertia must be introduced before manipulation.

---

## 20. Inertia Policy

Inertia should come from:

```text
validated primitive geometry
```

or:

```text
documented mesh-based approximation
```

Arbitrary inertia tensors are prohibited.

Incorrect inertia can produce:

- unrealistic tipping,
- unstable rotation,
- bad contact response,
- and solver artifacts.

---

## 21. Center of Mass

The center of mass should be physically sensible.

Examples:

```text
cube
→ geometric center

ball
→ geometric center

apple
→ near geometric center

pan
→ body + handle dependent
```

Incorrect COM definitions must not be used to make an object easier to grasp.

---

## 22. Damping

Joint and body damping should suppress numerical oscillation without hiding real dynamics.

Damping must be defined intentionally.

Too much damping can make:

```text
robot motion sluggish
object motion unrealistic
rail response artificial
```

Too little damping can cause:

```text
oscillation
contact chatter
control instability
```

---

## 23. Panda Joint Dynamics

The Franka Panda should use validated joint limits and dynamics.

The simulation must respect:

```text
joint position limits
joint velocity limits
effort limits
damping
```

The project should reuse known Franka model parameters wherever possible.

---

## 24. Linear Rail Dynamics

Each Panda has one rail prismatic joint.

The rail must define:

```text
position limits
velocity limits
acceleration limits
actuation limits
damping
```

The rail must behave as a mechanical axis.

It must not act as instantaneous translation.

---

## 25. Rail Travel Limits

Each rail has:

```text
q_min
q_max
```

defined in:

```text
config/robots.yaml
```

Physics and controller layers must enforce these limits.

No motion request may exceed them.

---

## 26. Rail Collision Geometry

The rail and carriage must have physical collision geometry.

Required concerns:

```text
carriage ↔ table
carriage ↔ fixed structure
rail ↔ environment
Panda ↔ rail
cross-robot geometry
```

Collision geometry should be simplified but conservative.

---

## 27. Rail Home Stability

At the configured home position:

```text
q_rail_home
```

the carriage should remain stable without drift.

A stationary rail command must not produce:

```text
creep
oscillation
numerical translation
```

---

## 28. Rail + Arm Coordination Physics

A valid coordinated motion may include:

```text
rail translation
+
arm joint motion
```

The physics engine must remain stable when both occur simultaneously.

Validation should include:

```text
slow coordinated motion
moderate coordinated motion
stop-and-hold
trajectory reversal
```

---

## 29. Gripper Dynamics

The Franka Hand should model:

```text
finger joints
finger velocity
contact with objects
closing motion
opening motion
```

The gripper must not teleport between open and closed states.

---

## 30. Gripper Closing Behavior

Closing behavior should be controlled.

The gripper must not:

```text
close infinitely fast
penetrate the object
apply unrealistic impulse
```

The closing profile should allow MuJoCo contacts to form naturally.

---

## 31. Gripper Opening Behavior

Opening must:

```text
remove finger pressure
allow object release
avoid launching the object
```

The release sequence must coordinate with grasp-stabilization removal.

---

## 32. Contact Verification

Before grasp stabilization, the system should evaluate physical contact.

Typical evidence may include:

```text
left finger contact
right finger contact
object located between fingers
valid finger separation
low relative object motion
```

The exact thresholds belong in:

```text
config/grasp_profiles.yaml
```

---

## 33. Hybrid Grasp Physics

The approved grasp sequence is:

```text
approach
↓
finger closure
↓
physical contact
↓
grasp verification
↓
temporary stabilization constraint
↓
transport
```

This sequence is mandatory.

---

## 34. Stabilization Constraint Purpose

The temporary stabilization constraint exists to reduce simulation-specific grasp loss during transport.

It should:

```text
preserve an already valid grasp
```

It must not:

```text
create a grasp that never physically occurred
```

---

## 35. Stabilization Constraint Type

The implementation may use a MuJoCo equality / weld-style constraint or another validated equivalent.

The constraint must be:

```text
disabled by default
activated only after grasp verification
removed during release
cleared during reset
```

---

## 36. Constraint Transform

When stabilization activates, the relative transform between:

```text
gripper / TCP
```

and:

```text
object
```

should preserve the actual verified grasp configuration.

The object must not snap to a predefined pose.

Large snap displacement is a failure.

---

## 37. Maximum Attachment Error

The project should define a maximum allowed position and orientation correction when enabling stabilization.

If activating the constraint would produce an excessive jump:

```text
grasp verification must fail
```

rather than forcing the object into place.

Exact thresholds belong in the grasp configuration.

---

## 38. Constraint Removal

During placement:

```text
object lowered
↓
support contact established
↓
gripper opens
↓
constraint removed
↓
object settles
```

The exact ordering may be refined experimentally, but release must avoid:

```text
object launch
object snap
gripper pulling object after release
```

---

## 39. Constraint Reset Rule

Every reset must explicitly clear all temporary constraints before restoring object state.

A stale grasp constraint after reset is a critical simulation failure.

---

## 40. Collision Policy

The simulation must preserve real collisions.

Do not globally disable collision between:

```text
robot and tables
robot and objects
held object and environment
robot and second robot
```

Collision filtering is allowed only where mechanically justified.

---

## 41. Self Collision

Panda self-collision should remain physically and planning-wise valid.

The simulator must not permit links to pass through each other because MoveIt is expected to avoid them.

Simulation remains the final physical layer.

---

## 42. Panda-to-Rail Collision

Robot links should not collide with their own rail under valid joint configurations.

If model geometry produces false self-contact, geometry should be fixed deliberately.

Do not broadly disable rail collision.

---

## 43. Panda-to-Panda Collision

Even while Panda 2 is inactive, Panda 1 must not physically pass through it.

Both robots remain part of the physical world.

Later dual-arm operation requires full cross-robot collision handling.

---

## 44. Object-to-Object Collision

Manipulable objects should collide physically.

A robot that pushes:

```text
apple into cube
```

should produce real interaction.

The task layer may classify this as unwanted disturbance.

---

## 45. Container Collision

Containers such as the bowl must support actual physical placement.

The object should contact:

```text
bowl bottom
bowl walls when appropriate
```

and should not pass through the container.

---

## 46. Bowl Physics

The bowl collision model must provide:

```text
accessible interior
stable bottom support
realistic wall contact
```

A visually correct but physically solid bowl is invalid.

---

## 47. Pan Physics

Initially static pan physics should still include collision.

Its handle must not create unexpected penetration or collision traps.

When made dynamic later, its inertia and COM must be reevaluated.

---

## 48. Static Structures

The following should generally be fixed:

```text
floor
tables
counters
sink
stove
fixed cabinets
rail support structures
```

Static furniture should not react dynamically to accidental robot contact unless a research phase explicitly requires it.

---

## 49. Decorative Assets

Decorative assets should be static unless interaction is required.

Examples:

```text
chair
laptop
controller
decorative kitchen objects
```

Avoid adding unnecessary dynamic bodies.

---

## 50. Collision Mesh Rule

Visual meshes are not automatically valid collision meshes.

Preferred:

```text
visual → detailed mesh
collision → simple primitives
```

This is especially important for:

```text
tables
bowl
pan
chairs
rail assembly
```

---

## 51. Contact Chatter

Persistent high-frequency contact chatter should be treated as a physics bug.

Investigate:

```text
penetration
bad collision geometry
timestep
solver parameters
mass ratio
damping
contact stiffness
```

Do not simply ignore it.

---

## 52. Penetration Rule

No object should begin a simulation in visible or meaningful penetration.

Initial geometry should satisfy:

```text
object bottom
≈
support surface top
```

with stable contact.

---

## 53. Spawn Settling

After reset, the simulation may step for a short controlled settling window before the scene is declared ready.

The settling window should be deterministic.

During settling:

```text
no task commands
no rail commands
no arm commands
```

---

## 54. Scene Ready Condition

The scene should only report ready after:

```text
robot state valid
rail state valid
object state valid
velocities within threshold
no illegal contact
no stale constraint
```

---

## 55. Deterministic Reset Physics

Reset must restore:

```text
rail positions
arm joints
gripper state
object poses
object velocities
object angular velocities
temporary constraints
controller state
```

Reset must not rely only on visual repositioning.

---

## 56. Velocity Reset

All dynamic objects must have:

```text
linear velocity = 0
angular velocity = 0
```

after reset initialization unless a specific experiment states otherwise.

The rail and robot must also begin from consistent velocity state.

---

## 57. Reset Validation

After reset and settling:

```text
position tolerance
orientation tolerance
velocity tolerance
contact validity
```

must be checked.

A reset that visually looks correct but retains non-zero unstable dynamics is invalid.

---

## 58. 100-Reset Requirement

Phase 1 requires:

```text
100 deterministic resets
```

with target:

```text
0 unintended object falls
0 stale constraints
0 invalid robot states
0 invalid rail states
0 unexplained drift
0 initial penetration
```

---

## 59. Idle Stability Test

The complete scene should remain idle for a defined simulated duration.

During the test:

```text
rails hold
robot arms hold
grippers hold
objects remain stable
```

Any unexplained motion should be investigated.

---

## 60. Rail Hold Test

At multiple rail positions:

```text
home
near limit
middle
far limit
```

the carriage should hold position stably.

Test for:

```text
drift
oscillation
controller saturation
collision
```

---

## 61. Joint Hold Test

At representative Panda configurations, the arm should hold state without:

```text
large oscillation
joint drift
contact instability
```

This validates controller / dynamics integration.

---

## 62. Gripper Hold Test

For a verified grasp, the fingers should maintain the intended closure state before stabilization.

This test helps ensure the physical grasp is plausible before the hybrid constraint is activated.

---

## 63. Drop Test

A controlled object drop test should validate gravity and table contact.

Example:

```text
cube released a few centimeters above tabletop
```

Expected:

```text
falls
contacts surface
settles
does not explode or tunnel
```

---

## 64. Ball Roll Test

The purple ball should be used to validate friction.

A controlled push should produce:

```text
rolling motion
energy loss
eventual settling
```

without extreme sticking or endless motion.

---

## 65. Bowl Placement Physics Test

A cube or apple placed inside the bowl should:

```text
enter the interior
contact bowl geometry
settle
remain contained
```

without tunneling through the bowl.

---

## 66. Grasp Stabilization Test

Test sequence:

```text
form valid grasp
record object-relative transform
enable stabilization
lift
transport
stop
```

Check:

```text
no object snap
no visible constraint jump
no excessive oscillation
no object drift
```

---

## 67. Constraint Failure Test

Attempting stabilization without a valid grasp must fail.

Example:

```text
gripper 10 cm away from cube
```

must never produce:

```text
cube attaches magically
```

---

## 68. Unintended Collision Test

During benchmark motions, record unintended contacts with:

```text
tables
unrelated objects
Panda 2
rails
decorative physics objects
```

The presence of a collision should be observable.

---

## 69. Physics Logging

For debugging and research, the project should be able to log:

```text
simulation timestep
solver configuration
object mass
friction profile
rail state
joint state
contact events
constraint state
task result
```

This allows failed runs to be reproduced.

---

## 70. Physics Configuration Version

`physics.yaml` should eventually contain:

```text
physics_version
```

Example:

```text
1.0
```

Benchmark results must record the version.

---

## 71. Physics Change Classification

Physics changes should be classified as:

```text
solver
contact
object dynamics
robot dynamics
rail dynamics
gripper dynamics
constraint
```

A physics change may invalidate previous benchmark results.

---

## 72. Benchmark Freeze

Once a benchmark set begins, the following should be frozen for that benchmark version:

```text
timestep
solver parameters
friction
object masses
rail dynamics
gripper contact parameters
stabilization behavior
```

Do not tune physics between benchmark runs without changing the benchmark version.

---

## 73. Proposed `physics.yaml` Structure

Illustrative only:

```yaml
physics:

  version: "1.0"

  timestep: ...

  gravity:
    x: 0.0
    y: 0.0
    z: -9.81

  solver:
    type: ...
    iterations: ...
    tolerance: ...

  defaults:
    friction: ...
    damping: ...

  profiles:

    tabletop:
      friction: ...

    gripper:
      friction: ...

    rolling_object:
      friction: ...

  grasp_stabilization:
    enabled: true
    max_position_error: ...
    max_orientation_error: ...
```

Exact values must be established experimentally.

---

## 74. Rail Physics Configuration

Rail-related configuration should include:

```yaml
rail:
  damping: ...
  max_velocity: ...
  max_acceleration: ...
  actuation_limit: ...
```

Travel limits belong primarily in:

```text
config/robots.yaml
```

The physics file owns generic rail dynamics.

---

## 75. No Per-Object Solver Hacks

Forbidden pattern:

```text
if object == apple:
    increase solver iterations
```

The solver is a global system parameter.

Object-specific physical differences should be represented through legitimate:

```text
geometry
mass
friction
grasp profile
```

---

## 76. No Gravity Hacks

Forbidden:

```text
lower gravity during pick
restore gravity after place
```

This would invalidate manipulation realism and benchmarks.

Gravity remains stable throughout normal operation.

---

## 77. No Object Freezing During Pick

Manipulable objects must not be silently changed from dynamic to static during grasping.

The approved stabilization mechanism is an explicit temporary constraint after verification.

---

## 78. No Collision Disabling During Transport

Held objects must continue interacting physically with the environment.

Do not disable:

```text
held object ↔ table
held object ↔ bowl
held object ↔ other robot
```

simply to avoid planning failures.

---

## 79. MuJoCo–MoveIt Consistency

Physical collision geometry and planning collision geometry should be aligned closely enough that:

```text
MoveIt says collision-free
```

usually means:

```text
MuJoCo does not physically collide
```

Large geometry mismatches are unacceptable.

---

## 80. Simulation Time

ROS 2 nodes interacting with MuJoCo must use simulation time.

Expected:

```text
use_sim_time = true
```

Physics stepping is the authoritative source of simulated time.

---

## 81. Real-Time Requirement

The simulation does not need to run exactly in real time during all experiments.

Correctness has priority.

Possible modes:

```text
real-time-ish interactive mode
faster-than-real-time headless benchmark
slower-than-real-time debugging
```

Task timing metrics must distinguish simulated time from wall-clock time.

---

## 82. Determinism

Given the same:

```text
scene version
physics version
initial state
command sequence
```

the simulation should produce sufficiently repeatable behavior for research evaluation.

Perfect bitwise determinism is not required unless practical.

Behavioral reproducibility is required.

---

## 83. Randomization Policy

Randomization is disabled during baseline development unless an experiment explicitly enables it.

Baseline:

```text
fixed mass
fixed friction
fixed initial pose
fixed solver
fixed timestep
```

Domain randomization belongs to later research phases.

---

## 84. Phase 1 Physics Acceptance

Phase 1 physics is accepted when:

- both rails remain stable,
- both Panda models initialize correctly,
- no object drifts at reset,
- collision geometry behaves predictably,
- the bowl interior is physically accessible,
- the ball rolls only when disturbed,
- gripper contact is stable,
- reset clears all velocities and constraints,
- and the scene passes 100 reset cycles.

---

## 85. Phase 4 Physics Acceptance

Reliable pick-and-place physics is accepted when:

```text
physical contact occurs before stabilization
grasp verification is meaningful
constraint activation does not snap object
held object remains stable
release is physically clean
placed object settles correctly
unrelated objects remain undisturbed
```

---

## 86. Codex Physics Rules

Codex must:

1. Read this document before physics changes.
2. Keep SI units.
3. Preserve gravity.
4. Preserve rail-as-prismatic-joint architecture.
5. Never teleport robot bases.
6. Never freeze dynamic objects during normal grasp.
7. Never attach objects before grasp verification.
8. Never disable collisions globally.
9. Prefer simple collision geometry.
10. Keep physics values configuration-driven.
11. Document object-specific overrides.
12. Clear constraints during reset.
13. Clear velocities during reset.
14. Re-run relevant physics tests after tuning.
15. Never hide instability with arbitrary magic values.

---

## 87. Physics Debugging Order

When an interaction fails, debug in this order:

```text
1. frame correctness
2. geometry correctness
3. collision geometry
4. initial penetration
5. object mass / inertia
6. controller behavior
7. friction
8. contact parameters
9. timestep
10. solver tuning
11. grasp stabilization
```

Do not start by randomly changing several physics values at once.

---

## 88. Change Isolation Rule

A physics experiment should change one conceptual variable at a time where practical.

Example:

```text
change friction
run test
record result
```

rather than simultaneously changing:

```text
friction
mass
timestep
solver
controller gain
```

This is essential for reproducible debugging.

---

## 89. Failure Evidence

Physics-related failures should capture:

```text
task ID
scene version
physics version
object state
rail state
robot state
contact summary
constraint state
failure timestamp
```

Screenshots or videos may supplement this evidence but are not enough by themselves.

---

## 90. Final Physics Principle

The MuJoCo world must behave as a physical system, not as an animation.

The project should never need to cheat physics simply to produce a successful demo.

The intended manipulation chain is:

```text
valid geometry
↓
valid dynamics
↓
valid contact
↓
valid grasp
↓
verified stabilization
↓
safe transport
↓
physical release
↓
stable placement
```

Physics is the foundation beneath control, motion planning, manipulation, and later LLM orchestration.

If the physics layer is unstable, higher layers must not be used to hide the problem.
