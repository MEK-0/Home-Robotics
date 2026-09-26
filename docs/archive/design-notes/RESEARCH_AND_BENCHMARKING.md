# Home Robotics — Research and Benchmarking

## 1. Purpose

This document defines how Home Robotics will be evaluated as an academic and portfolio-quality robotics system.

The project should produce:

```text
working demonstrations
+
measured evidence
+
reproducible experiments
```

The repository should be able to answer not only:

```text
Does it work?
```

but also:

```text
How reliably does it work?
Why does it fail?
How expensive is execution?
What changes improve or degrade performance?
```

---

## 2. Baseline Research Questions

Initial research questions include:

1. How reliably can a rail-mounted Panda perform deterministic tabletop pick-and-place?
2. How much does the linear rail improve reachable workspace?
3. How reliable is hybrid grasp stabilization compared with raw physical grasp transport?
4. Which failure modes dominate cube, apple, and spherical-object manipulation?
5. How much scene disturbance occurs during successful tasks?
6. How much planning and execution time does rail motion add?
7. Can a local LLM planner successfully orchestrate symbolic robot tools without controlling motion?

---

## 3. Baseline Objects

Benchmark progression:

```text
cube
apple
purple_ball
```

The cube is the geometric baseline.

The apple introduces household semantics and curved geometry.

The ball introduces rolling/contact difficulty.

---

## 4. Baseline Destination

Initial container benchmark:

```text
bowl
```

Primary semantic task:

```text
apple → bowl
```

---

## 5. Baseline Environment

Baseline experiments must freeze:

```text
scene version
physics version
robot model version
grasp profile version
planner configuration
initial object poses
```

This makes results comparable.

---

## 6. Core Success Metrics

Track:

```text
pick success rate
place success rate
end-to-end task success rate
```

For N trials:

```text
success_rate = successful_trials / total_trials
```

---

## 7. Planning Metrics

Track:

```text
planning success rate
planning time
number of grasp candidates
number of reachable candidates
```

Potential later metrics:

```text
path length
joint-space cost
collision margin
```

---

## 8. Rail Metrics

Because rail motion is central to this project, record:

```text
rail travel distance
rail planning contribution
rail execution time
rail final position
rail-limit proximity
```

---

## 9. Arm Metrics

Potential arm metrics:

```text
joint path length
joint-limit margin
maximum velocity
execution duration
```

These may help compare fixed-rail-local motion versus rail-assisted motion.

---

## 10. Grasp Metrics

Track:

```text
grasp verification success
false-grasp rejection
stabilization activation success
object slip
object drop
```

---

## 11. Placement Metrics

Track:

```text
placement success
container miss
object final position
object final orientation if relevant
settling time
```

---

## 12. Scene Integrity Metrics

Track collateral effects.

Examples:

```text
unrelated object translation
unrelated object rotation
unintended contact count
unintended object fall
collision count
```

---

## 13. Time Metrics

Separate:

```text
planning time
execution simulated time
wall-clock time
grasp verification time
settling time
total task time
```

Do not mix simulation time and wall-clock time.

---

## 14. Failure Metrics

Count failure categories.

Example:

```text
NO_VALID_GRASP
PLANNING_FAILED
GRASP_VERIFICATION_FAILED
OBJECT_DROPPED
PLACEMENT_VERIFICATION_FAILED
COLLISION_DETECTED
```

Failure distribution is a research result.

---

## 15. Repeat Counts

Development tests may use small counts.

Formal baseline benchmark should eventually use repeated trials such as:

```text
N = 50
```

or:

```text
N = 100
```

per selected task condition where computationally practical.

---

## 16. Deterministic Benchmark

The first benchmark is deterministic.

Use:

```text
fixed spawn
fixed physics
fixed task
fixed planner parameters
```

Purpose:

```text
measure implementation reliability
```

before adding randomized variation.

---

## 17. Randomized Benchmark — Future

Later research may randomize:

```text
object position
object orientation
mass
friction
grasp candidate order
```

This becomes a robustness benchmark.

It must remain separate from baseline deterministic results.

---

## 18. Rail Ablation Study

A valuable research comparison:

### Condition A

```text
rail fixed at home
```

### Condition B

```text
rail available to planner
```

Compare:

```text
reachable targets
planning success
joint-limit margin
execution time
```

This directly demonstrates the value of the side rails.

---

## 19. Grasp Stabilization Ablation

Possible study:

### Condition A

```text
physical grasp only
```

### Condition B

```text
physical verification + temporary stabilization
```

Compare:

```text
drop rate
transport success
placement success
```

This should only be done after the baseline system is stable.

---

## 20. Object Difficulty Comparison

Compare:

```text
cube
apple
purple_ball
```

Expected differences may appear in:

```text
grasp verification
contact stability
drop rate
planning success
```

Do not assume the result in advance.

Measure it.

---

## 21. Distance / Rail Position Study

Create object targets at:

```text
near
middle
far
```

sections of the work surface.

Measure:

```text
rail travel
planning success
task time
arm posture
```

This is particularly relevant to the scene design.

---

## 22. Local LLM Evaluation

Phase 6 may evaluate:

```text
natural-language interpretation accuracy
tool selection accuracy
task sequence accuracy
invalid tool-call rate
recovery decision quality
```

---

## 23. Foundry Local Integration

Microsoft Foundry Local is a candidate local runtime.

The robot benchmark must remain independent of the chosen LLM provider.

Possible comparison later:

```text
local model A
local model B
remote model
```

using the same task API.

---

## 24. LLM Safety Boundary Metric

One useful architectural metric:

```text
raw motion command violations = 0
```

The LLM should produce only approved symbolic tools.

Any attempt to access prohibited low-level interfaces is an architecture failure.

---

## 25. Experiment Metadata

Every formal experiment should record:

```text
experiment_id
date
git commit
scene version
physics version
robot version
grasp profile version
planner configuration
task definition
trial count
```

---

## 26. Result Storage

Suggested repository structure:

```text
experiments/
├── phase1/
├── phase2/
├── phase3/
├── phase4/
├── phase6/
└── benchmarks/
```

Large raw data may eventually require external storage.

---

## 27. Reproducibility

A result is reproducible when another developer can determine:

```text
which code
which config
which scene
which physics
which command
which trial procedure
```

produced it.

---

## 28. Git Commit Recording

Formal experiments should record:

```text
git rev-parse HEAD
```

or equivalent commit identifier.

Benchmark data without code version is weak evidence.

---

## 29. Benchmark Freeze

Once a formal benchmark begins, do not change configuration mid-series.

If a change is needed:

```text
end benchmark version
make change
start new benchmark version
```

---

## 30. Visualization

Useful result plots may include:

```text
success rate by object
failure-code distribution
planning time distribution
rail travel by target region
task duration distribution
scene disturbance
```

Visualizations should be generated from stored data rather than manually estimated.

---

## 31. Portfolio Evidence

The repository should eventually include concise evidence such as:

```text
architecture diagram
scene screenshot
pick/place demo
benchmark table
failure analysis
dual-arm roadmap
LLM tool-call example
```

Avoid turning README into a raw development log.

---

## 32. Academic Reporting

A technical report should clearly distinguish:

```text
method
experimental setup
metrics
results
limitations
future work
```

Claims must be tied to measured evidence.

---

## 33. Limitations

The project should explicitly document limitations.

Expected early limitations include:

```text
ground-truth object pose
no vision
single active arm
hybrid grasp stabilization
simulation-only execution
```

These are design-stage limitations, not facts to hide.

---

## 34. Future Research Directions

Possible directions:

```text
camera-based perception
6D pose estimation
domain randomization
sim-to-real
RL grasping
imitation learning
failure-aware agents
multi-arm task allocation
object handover
language-conditioned manipulation
```

---

## 35. Benchmark Acceptance Principle

A benchmark result should never be improved by silently changing the success definition.

Success criteria must be frozen before trials begin.

---

## 36. Negative Results

Failed approaches should be documented when informative.

Examples:

```text
unstable friction setting
bad bowl collision model
grasp profile with high drop rate
planner setup with poor rail use
```

Negative evidence improves the academic quality of the project.

---

## 37. Codex Research Rules

Codex must:

1. Record configuration versions in benchmark code.
2. Never change success criteria mid-run.
3. Keep benchmark data machine-readable.
4. Separate deterministic and randomized results.
5. Preserve failure codes.
6. Record git commit where practical.
7. Avoid hand-edited result numbers.
8. Do not claim improvements without comparison data.
9. Keep experimental code separate from core runtime.
10. Document limitations.

---

## 38. Final Principle

Home Robotics should be demonstrable, measurable, and reproducible.

The central research rule is:

> A visually successful demo is an example; repeated measured performance is evidence.
