# Phase 4 reliability

The Phase 4 acceptance benchmark is a 10-trial full-stack simulated Panda1/cube
series to `surface_left_2`. Nine trials completed successfully. Trial 2 failed
before motion when MoveIt TOTG reported negative path velocity while retiming a
hypothetical transport trajectory.

The failure is retained in the per-trial record and is not a grasp, lift, or
placement failure. No autonomous retry hides it.

| Successful-trial measure | Recorded range / result |
| --- | --- |
| Final position error | 1.416–1.923 mm; mean 1.598 mm |
| Maximum orientation error | 0.000174 rad |
| Mean recorded planning time | 1.055 s |
| Mean task duration | 100.644 s |

The cube sequence includes live grasp verification before temporary stabilization,
WORLD/ATTACHED scene ownership checks, lift and placement verification, and final
release. It remains a simulation result; no real-robot reliability claim is made.

A separate `purple_ball` grasp/lift/return baseline passed once. It is not a
named-placement reliability benchmark. Raw benchmark reports, including the
retiming failure, are preserved in
[archive/evidence/validation/phase4](archive/evidence/validation/phase4).
