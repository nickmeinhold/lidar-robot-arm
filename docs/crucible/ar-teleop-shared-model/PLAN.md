# PLAN — Collision-safe teleoperation via a shared kinematic model

Tempered blade from `/crucible` (2026-07-31). North star (Nick-confirmed): the AR
See/Grab vision. This phase ships the **crux** — safe full-range teleop — and builds the
reusable spine the AR layer plugs into later. Survived a cross-family Temper (Carnot
NEEDS-REVISION + Kelvin FATAL, both folded).

## Safety architecture (from Temper — the load-bearing revision)
- **PRIMARY safety = reactive PRESENT_LOAD current-cutoff.** Simple, always-on, dumb,
  robust. Catches everything physical: cables, world, operator, unmodeled geometry.
- **PREVENTIVE layer + spine = geometric capsule model.** Smart, fallible; prevents
  *entering* modeled self-collision; is the shared FK core for See/Grab. NOT the last line.
- Defense-in-depth: the simple thing guards, the smart thing prevents.

## Ordered slices (each independently useful; Slice 0 ∥ Slice 1, then 2, then 3)

### Slice 0 — Load baseline model + reactive cutoff (the primary safety layer)
- Run `load_monitor.py` against real provoked self-contacts (built last session, NEVER
  run against contact). Capture ≥3 events with magnitudes + which joints couple.
- Build a load baseline: load as f(pose, speed) from no-contact trajectories → threshold on
  *residual above expected*, not raw magnitude (Temper: Carnot#2 + Kelvin#3 convergent).
- Wire reactive current-cutoff as the primary safety reflex in the controller.
- **Done:** cutoff trips on real contact, no false-trip on gravity / fast moves.

### Slice 1 — Standalone FK core + capsule model (`server/arm_kinematics.py`)
- FK: joint angles → link frames, reusing the VIEWER's existing angle convention (Fold#5);
  harvest SO-101 URDF for link dims + limits only, not its frame convention.
- Capsule per link + **gripper-jaw capsules + base/table ground-plane** as first-class
  bodies (Temper: Carnot#3 + Kelvin#4). Shared FK core; collision volumes SEPARATE from
  render meshes (Carnot#6).
- Unit-test against 2-3 hand-computed poses.
- **Done:** given a joint tuple, returns capsules; tests pass.

### Slice 2 — Collision gate at the single mutator door
- `collides(angles) -> (bool, pair)` via Ericson `ClosestPtSegmentSegment` − radii + ACM
  (exclude the 5 adjacent pairs).
- Gate at FeetechArmController's ONE write point. Checks commanded goal + swept-path
  interpolation on the delta + **encoder-vs-command divergence stop** (Carnot#1).
- On reject: **nearest-safe-projection** (clamp toward closest safe config), not bare
  reject; "disable drive until operator backs out" if projection fails (Carnot#4 recovery).
- **Connect-time check on current encoder pose**; boots-colliding → warn + limp before
  enabling drive (Kelvin#1 bootstrap).
- Fail-safe: unreadable pose → fall back to firmware-clamp + reactive mode w/ loud warning
  (Fold#1), never brick, never pass-through.
- Cross-validate vs Slice 0: model says "collide" where load actually spiked.
- **Done:** commanded self-collision refused with a recovery path; validated vs load data;
  **task #4 solved** (safe full-range shoulder motion unblocked).

### Slice 3 — Light it up in the web viewer (`viewer.html`)
- Render capsules; glow red + name the offending link-pair when `collides()` fires; driven
  by live encoder state over the existing WS.
- The tuning instrument: adjust capsule radii by eye + iPhone camera + load cross-check.
- **Done:** capsules turn red exactly when the model predicts contact, matching the monitor.

### Deferred — Slice 4+ (iOS, the AR vision the model is the spine for)
- **See:** render encoder-pose arm in ARKit world space (device VIO sub-cm; only body
  tracking is noisy).
- **Grab:** clutch-based engagement — coarse AR aim enters "armed" zone → deliberate
  confirm (hand-in-volume or button) commits the lock; zero-velocity-at-lock + first-delta
  magnitude cap (Temper: Carnot#5 + Kelvin#5). Borrow trzy's clutch; AR pose-match is the
  aiming aid, not the sole trigger.
- Safety stays on encoders; control stays relative-delta; only the visual overlay uses ARKit.
- FUTURE: iPhone LiDAR → world-collision via depth (Kelvin#4 extension).

## Blast-radius & consent
Driving under torque. Layered guard: reactive cutoff (primary) + geometric prevention +
ID2 firmware clamp + 4A hardware limit + hand-on-power. Slice-2 validation (driving the
shoulder through real range) is supervised until the model is trusted vs ≥N matched events.

## Relationship to existing tasks
- **Absorbs task #4** (reactive collision) — now Slice 0 + the reactive-primary architecture.
- **Builds on task #5** (the uncommitted virtual→real drive) — the gate wires into that
  controller path; commit #5's drive changes first (keeping iOS/mac-app WIP out).

## First concrete step
Slice 0: run `load_monitor.py` against real provoked contacts. Cheap, gates everything,
and it's now the primary safety layer.
