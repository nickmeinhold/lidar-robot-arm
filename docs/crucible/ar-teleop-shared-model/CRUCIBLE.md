# CRUCIBLE — AR See/Grab/Stay-safe teleoperation via one shared kinematic model

## The ore (selected 2026-07-31, given by Nick)
Build the AR teleoperation vision around its **spine**: ONE shared kinematic model of the SO-100 arm serving three consumers:
- **See** — the phone renders the robot's actual pose in AR space.
- **Grab** — you move your real arm to match the shown pose; on alignment it locks in and hands over control (a deliberate handshake, not the current accidental auto-lock-on-first-frame).
- **Stay inside the lines** — the arm is only driveable where it won't self-collide, across the full joint tuple.

## Why this thrills me AND what it changes
- **Elegance = impact, same move.** The spine collapses three features into one model + three consumers. Build the model once (link geometry → forward kinematics → capsule set), get render + collision-check + alignment-target for free.
- **Removes a real, months-old blocker.** Task #4 (safe full-range shoulder motion) has been open since May because static per-joint limits can't express configuration-space self-collision. This is the thing that makes teleop *trustworthy* instead of a supervised demo.
- **A genuine conceptual surprise.** The "Grab handshake = absolute registration" reframe could DELETE the servo-zero calibration subsystem entirely (the human eye supplies the absolute reference by aligning body to shown-pose). Design-for-subtraction.
- **The instruments already exist on the bench.** Link dims are in `viewer.html` (verified: UPPER_ARM_LEN 0.1126, FOREARM_LEN 0.1349, WRIST_LEN 0.0601, radii). A second independent collision detector exists — `load_monitor.py` reads PRESENT_LOAD spikes. We can cross-validate a geometric predictor against physical ground truth.

## The spark (if true, drop everything)
A capsule-per-link model driven by the arm's live joint angles can flag "these two links are about to intersect" BEFORE contact — and the load monitor confirms it at the boundary. If that cross-validation holds, "Stay inside the lines" is real preventive safety, not reactive damage-control, and it generalizes to any articulated arm.

## The falsifier (what proves this ore is slag)
**PRIMARY:** If a geometric capsule model can't predict SO-100 self-collision to within the safety margin's tolerance — because the real collision geometry (cables, servo horns, gripper soft parts, mount brackets) is too irregular for capsule approximation — then the model's "safe" verdict diverges from real load-spike ground truth and "Stay-safe" becomes a lie dressed as safety. Test cheaply: build the capsule model, predict a boundary config, provoke it by hand, watch the load monitor. Model says safe + load spikes (or vice versa) past tolerance → slag.

**SECONDARY (crux):** If ARKit body-tracking noise ≫ the collision safety margin, the Grab handshake supplies a FALSE absolute registration and Stay-safe fails even with a perfect model. The reframe (handshake = registration) is the open question Temper must strike, not something to pre-litigate.

## Recommended first slice (all three retro perspectives endorsed)
The geometric self-collision model, lit up in the Mac web viewer, cross-validated against real load spikes from `load_monitor.py` (built last session, NEVER yet run against provoked contacts — a cheap high-value experiment that gates everything downstream). Build the model as a STANDALONE module (not embedded in one consumer) because all three moves depend on it.

## Output artifacts
- CRUCIBLE.md (this) · RESEARCH.md (Heat) · DESIGN.md (Cast) — all in docs/crucible/ar-teleop-shared-model/
