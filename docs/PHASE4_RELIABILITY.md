# Phase 4.9: reliability benchmark

Cube acceptance on 2026-09-21: **9/10 complete successes (90%)**. The full series,
including its failure, is in [benchmark.json](validation/phase4/benchmark.json) and
[per-trial summary](validation/phase4/benchmark.md).
**FULL PHASE 4 COMPLETE: YES**, for the reliable simulated Panda1/cube baseline.
Phase 4.0 through 4.9 are COMPLETE. Phase 5 was not started.

The ten-trial acceptance series preceded the sphere-only extension. After that
extension and final guards, both the [original cube lift-return regression](validation/phase4/lift-return-regression.json)
and a separate [final-source cube pick-place check](validation/phase4/final-source-pick-place.json)
passed physically. They are reported separately and were not substituted for the
failed acceptance trial. Lift-return final position error: **0.150 mm**. Final-source
pick-place error: **1.469 mm**, angular error **0.000133 rad**, maximum relative drift
**0.109275 mm**, one WORLD cube and no attachment. All required closure checks passed;
the GUI observation checklist remains for the user to inspect manually.

Successful trials: final position error min/mean/max **1.416 / 1.598 / 1.923 mm**;
maximum angular error **0.000174 rad**; maximum cube/gripper translation drift
**0.109333 mm**; maximum Panda2 drift **3.057e-11 rad/m**. Mean recorded planning
time was **1.055 s**; mean task duration **100.644 s**.

Trial 2 failed in PREGRASP_PLANNED before any motion: MoveIt TOTG reported negative
path velocity on the hypothetical transport trajectory. The original detailed
failure and its then-generic INVALID_OBJECT_STATE code are preserved. Diagnostics
now identify retiming failure as PLANNING_FAILED. This is a known path/timing
limitation, not a physical grasp/drop failure. No autonomous retry hides it.
The runner returns a nonzero exit when ANY trial fails, even when the ≥8/10 phase
acceptance threshold is satisfied.

Build and source the ROS environment as in [DEMO](DEMO.md), then run from the repo root:

```bash
ros2 run home_robotics_manipulation benchmark_pick_place.py \
  --trials 10 --domain 64 --target surface_left_2 \
  --output docs/validation/phase4/benchmark.json
```

The domain must be empty. The runner owns only the processes it starts. Each trial
starts the existing Phase 2 stack from authoritative reset configuration, waits for
controller readiness, starts MoveIt/static scene/dynamic sync, and runs the demo.
It then stops those processes completely before the next trial. Thus there is only
one MuJoCo instance at a time, no parallel simulator and no normal-task teleport.
A full stack restart also resets controller targets and attached-object state.

Default trial count is 10; accepted range is 1–100. Logs stay in `/tmp/phase4-benchmark`;
compact JSON and Markdown summaries go to the evidence directory. Every trial is
counted, including startup failures, timeout, missing result files and failed stages.
Planning-only uses `--planning-only`; it never counts as physical success.
The original demo can be selected with `--regression`.

Per-trial results include success, stage/reason/detail on failure, recorded planning
time (including Cartesian IK and preflight), grasp verification, lift, transport,
placement, final position/angular error, target/final poses, drift, ownership counts
and task duration. Planning time is a pipeline measure, not just OMPL solver time.
The aggregate records success/failure count, success rate and failures by stage.

The development trial is preserved separately from acceptance trials. Failed or
interrupted development attempts must not be silently replaced by successful evidence.


## Second object: purple_ball

The configured sphere has radius 0.035 m (diameter 0.070 m) and mass 0.06 kg.
It leaves 0.010 m opening margin in the Panda's 0.080 m gripper. The apple's configured
0.080 m diameter leaves no comparable margin, so it was not chosen.
The existing top-down generator and contact observer were extended for this one
known sphere. Both observers share the same runtime model/data. A second inactive
weld in that model is not a second MuJoCo instance. The runtime still requires live
bilateral verification before activating either weld.

[Planning](validation/phase4/ball-planning.json) passed candidate generation, IK,
collision-aware pre-grasp, approach and hypothetical attached lift/return.
[Physical execution](validation/phase4/ball-execution.json) passed bilateral contact
(1.04 s continuous, measured width 0.068992 m), lift **0.099675 m**, supported return,
stabilization removal, WORLD ownership, gripper opening and retreat. Maximum relative
translation drift was **0.000108637 m**. Final error relative to its original position
was **0.012670 m**. Panda2 maximum drift was **1.38e-11 rad/m**. Final ownership: one
WORLD sphere and zero attached sphere.

Bonus baseline: **PASS (one physical grasp/lift/return trial)**. Named-target sphere
placement is deliberately unsupported; the geometry policy remains box placement.
The sphere rolled after release (angular change 0.388 rad). This is not a sphere
placement reliability benchmark or an orientation-constrained spherical placement
claim. The legacy `cube_lift` / `cube_gripper_max_drift` result keys refer to the
selected `object` for this reused demo.

## Regression and scope

Simulation: **92 passed**. ROS: **44 tests, 0 errors, 0 failures, 1 skipped**; all six
packages build. Evidence: [simulation](validation/phase4/simulation-tests.txt),
[ROS results](validation/phase4/ros-results.txt), [build](validation/phase4/build.txt).
The skip is the pre-existing generated-source copyright check.

One development test fixture initially omitted the world frame; it was corrected.
An existing Phase 3 negative test timed out while shutting down in the occupied
default ROS domain; it passed in isolated domain 65 without Phase 3 code changes.
The regression commands in DEMO.md use that isolated domain to avoid user-owned nodes.

Remaining warnings/limitations: mixed prismatic/revolute TOTG warning and occasional
retiming rejection; non-realtime scheduling privileges; existing deprecated ROS
interfaces/dependencies. No physical fault-injection reliability rate, GUI observation,
unknown-object manipulation or real-robot capability is claimed. Pre-release support
uses bounded geometry proximity when there is no contact; post-release verification
requires actual destination contact.

Planning time includes initial OMPL reported planning time plus Cartesian IK,
retiming and path checks. The short OMPL replan after gripper opening is not included
in this recorded metric; total task wall duration includes it.
