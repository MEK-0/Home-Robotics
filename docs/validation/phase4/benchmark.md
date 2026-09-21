# Phase 4 cube acceptance — 2026-09-21

Physical cube pick-place: 9 / 10 (90%).

| Trial | Result | Failure stage / detail | Position error (m) |
|---|---|---|---|
| 1 | PASS | — / — | 0.0018855165330413492 |
| 2 | FAIL | PREGRASP_PLANNED / Retiming failed | None |
| 3 | PASS | — / — | 0.0014346901653117525 |
| 4 | PASS | — / — | 0.0018132476688642665 |
| 5 | PASS | — / — | 0.0014161209212505627 |
| 6 | PASS | — / — | 0.0015077533849740983 |
| 7 | PASS | — / — | 0.0014569406571012685 |
| 8 | PASS | — / — | 0.001922680021403934 |
| 9 | PASS | — / — | 0.0014430485471998685 |
| 10 | PASS | — / — | 0.0014993909345842854 |

| Successful-trial metric | Minimum | Mean | Maximum |
|---|---|---|---|
| final_position_error | 0.00141612092 | 0.00159770987 | 0.00192268002 |
| final_orientation_error | 9.8402565e-05 | 0.000121397202 | 0.000174025282 |
| cube_gripper_max_drift | 0.000109257045 | 0.000109290973 | 0.000109333047 |
| panda2_max_drift | 1.66699363e-11 | 2.46382475e-11 | 3.05692971e-11 |
| planning_time | 0.996159353 | 1.05452742 | 1.10770372 |
| total_duration | 97.7775357 | 100.643558 | 104.010693 |

Trial 2 failed before motion while TOTG retimed the transport preflight. MoveIt reported negative path velocity. No grasp, stabilization, attachment or lift occurred. The raw report classified the uncoded exception as INVALID_OBJECT_STATE; subsequent diagnostics now explicitly classify retiming as PLANNING_FAILED. No failed trial was removed.

This acceptance series preceded the purple_ball-only extension. Final-source cube regression/smoke results are recorded separately. Raw logs: /tmp/phase4-acceptance (not committed).
