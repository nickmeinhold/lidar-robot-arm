# DESIGN — AR teleoperation via one shared kinematic model

Status: CAST (pre-Temper). Open variables enumerated at the bottom; do not read as final.

## Problem
Drive the SO-100 follower arm by moving your own arm, safely through its FULL range —
including the shoulder poses that self-collide. Today: (1) safety is firmware angle-limit
clamps that can't express configuration-space self-collision (task #4 open since May);
(2) teleop works but is relative-only with no spatial feedback; (3) no way to see where
the arm is or engage control deliberately. The vision: **See** (phone shows the arm's pose
in space), **Grab** (align your arm to it → lock in), **Stay-safe** (only driveable where
it won't hit itself).

## The shape — one model, three consumers, THREE DIFFERENT STATE SOURCES

The elegance is a shared **kinematic model** (link geometry → forward kinematics → a
capsule per link). The correction Heat forced: the three consumers do NOT share a state
source — and that separation is what makes the whole thing safe.

```
                    ┌─────────────────────────────────────┐
                    │   Shared kinematic model (module)    │
                    │   link dims → FK → capsule per link  │
                    └─────────────────────────────────────┘
                       ▲             ▲              ▲
        encoder state  │             │ encoder      │ ARKit BODY pose (human)
        (exact,0.088°) │             │ + ARKit VIO  │ (coarse, several cm)
                       │             │ (sub-cm dev) │
                　     │             │              │
              ┌────────┴───┐   ┌─────┴──────┐  ┌────┴─────────┐
              │ STAY-SAFE  │   │    SEE     │  │    GRAB      │
              │ capsule    │   │ AR overlay │  │ align human  │
              │ check gates│   │ of real    │  │ arm to shown │
              │ every write│   │ arm pose   │  │ pose → lock  │
              └────────────┘   └────────────┘  └──────────────┘
```

- **STAY-SAFE** checks the **commanded goal configuration** (the joint tuple we're about to
  write), NOT a fresh per-tick encoder read (Fold #2 — this is prevention: refuse to
  *command* a colliding pose, and it removes bus contention, since reading 6 encoders +
  writing 6 goals every tick would hit the ~80-160Hz 1Mbaud ceiling). Per-tick: FK on the
  goal tuple → capsules → closed-form capsule-capsule distance (Ericson
  `ClosestPtSegmentSegment` − radii), excluding adjacent pairs (Allowed Collision Matrix).
  Any commanded pose whose capsules violate `margin` is REJECTED/CLAMPED before the write.
  ~1500 FLOP/tick, free. Encoders are read at connect (home) and for the SEE display, not
  for the gate. **Path safety (Fold #2):** small per-tick deltas make endpoint-checking ≈
  path-checking; for large jumps (a preset, a tracking glitch) interpolate a few points
  between last-commanded and new goal and check each. **Independent backstop:** PRESENT_LOAD
  current-reflex for cables/hands/world that geometry can't see. Safety NEVER touches ARKit.
- **SEE** renders the real arm (encoder pose) in AR, placed in world via ARKit *device*
  pose (VIO, sub-cm — good). Coarse body noise doesn't affect this; the phone knows where
  IT is well.
- **GRAB** compares ARKit *body* pose (noisy, several cm) to the shown target. On coarse
  alignment (within a generous angular tolerance) it LOCKS the relative reference (the
  existing reference-lock) and hands over relative-delta control. Coarse is fine: it only
  disambiguates gross pose + picks the engagement moment; precise control is relative, and
  safety is on encoders.

**Resolution of the crux:** the handshake supplies COARSE absolute alignment — enough for
engagement + visual overlay, NOT relied on for safety (encoder) or fine control (relative
delta). The reframe "handshake dissolves calibration" is TRUE for the parts that can
tolerate cm error and IRRELEVANT for the parts that can't (they never needed ARKit).

## Build order (core-first, each step independently useful)

**Slice 0 — gather ground truth (cheap, gates everything).** Run `load_monitor.py`
against real provoked contacts (it was built but never run against contact). Hand-drive
the arm into a known self-collision (wrist down + shoulder rotate) and capture the load
spike magnitude + which joints couple. Output: measured spike threshold + confirmation
the load channel sees self-contact. *Done = ≥3 captured self-collision events with
magnitudes.* Independently useful: unblocks the reactive backstop regardless of the rest.

**Slice 1 — standalone kinematic model module** (`server/arm_kinematics.py`, pure Python +
NumPy). Input: 6 joint angles. Output: capsule endpoints per link (FK from harvested URDF
link origins + hand-authored capsule radii measured off the printed arm). NO consumer
embedded. *Done = given a joint tuple, returns 6 capsules; unit-tested against 2-3
hand-computed poses.* Independently useful: it's the shared spine.

**Slice 2 — collision check + gate** (`arm_kinematics.collides(angles) -> (bool, pair)`
using Ericson closed-form + ACM). Wire it as a SINGLE pre-write gate in
`FeetechArmController` (single-door-in-the-mutator: the one place GOAL_POSITION is written).
Reject/clamp a commanded pose that would self-collide. *Done = a commanded self-collision
pose is refused; a safe pose passes; validated against Slice 0 — model says "collide" at
the same configs the load monitor spiked.* THIS is the safety layer that unblocks driving
the shoulder. Independently useful: task #4 solved.

**Slice 3 — light it up in the web viewer** (`viewer.html`): render the capsules, glow
red + name the offending link-pair when `collides()` fires, driven by live encoder state
over the existing WS. Fast web iteration, no iOS. *Done = pose the arm (real or slider),
capsules turn red exactly when the model predicts contact, matching the load monitor.*
Independently useful: the design/debug instrument for tuning capsule radii by eye+camera.

**Slice 4+ (LATER, bigger, iOS) — See/Grab in AR.** Render the encoder-pose arm in ARKit
world space; detect body-to-target alignment; lock + hand over. Deferred: it's the native
lift, and Slices 0-3 deliver the crux (safe full-range teleop) without it.

## Tradeoffs (named, with owner)
- **Capsules over-approximate** wide/bracketed parts → conservative (lost workspace, not
  unsafe). Accepted: size to bounding radius incl. brackets. Owner: whoever tunes radii in
  Slice 1-3, by eye + camera + load-monitor cross-check.
- **Cables unmodeled** by any capsule → the PRESENT_LOAD reflex is the *required* backstop,
  not optional. Accepted named tradeoff: geometry prevents modeled collisions, current
  catches the rest. Owner: Slice 0 delivers the threshold.
- **ARKit body noise (several cm)** → Grab is coarse-only. Accepted: engagement + display
  tolerate it; safety/control don't use it. This is the crux resolution, not a fudge.
- **URDF is under-defined** (issue #54: missing limits, inverted axes, removed base meshes)
  → harvest limits + link origins only; hand-verify axes against the physical arm; hand-
  author capsule radii. Accepted.

## Blast-radius & consent spine (cage before monster)
- **Owner:** Nick, at the bench, hand on power, ~4A current limit as hardware backstop.
- **Injection surface:** commanded GOAL_POSITION (from sliders, then body tracking). The
  collision gate sits at the single mutator door, so EVERY command path (slider, in-proc,
  body, test) passes through one check — no bypass.
- **The gate is fail-safe:** if the kinematics module can't evaluate a pose (NaN, missing
  encoder read), it must REFUSE the write (fail closed), not pass it. Uncertainty removes
  authority.
- **Validation is supervised:** Slice 2's "drive the shoulder through its real range" step
  happens with a human watching and the load-reflex armed, never unattended, until the
  model is trusted against ≥N matched load events.
- **ID2 firmware clamp stays** as the outermost backstop during bring-up (defense in depth;
  remove only after the geometric gate is validated).

## Claims to falsify (for the adversary — strike these)
1. **A capsule model is trustworthy enough.** Claim: hand-tuned capsules cross-validated
   against load spikes agree with reality to ~1-2 cm, good enough to gate. Falsify: find a
   self-collision geometry (gripper-into-forearm? cable snag? a bracket overhang) the
   capsule set systematically mis-predicts past the margin.
2. **Encoder-state safety fully decouples from ARKit.** Claim: safety never needs body
   tracking. Falsify: find a case where the thing that must be prevented depends on human
   pose, not follower joint state (e.g. arm colliding with the OPERATOR — that IS ARKit-
   dependent and the current reflex is the only guard).
3. **Single-door gating has no bypass.** Claim: all command paths go through one pre-write
   check. Falsify: find a write path (reconnect, calibration script, direct register write)
   that reaches GOAL_POSITION without the gate.
4. **Slice ordering is safe.** Claim: each slice is independently useful and safe to ship
   alone. Falsify: a slice that requires driving the shoulder before the gate exists, or
   that can't be validated without a later slice.
5. **Coarse Grab is enough.** Claim: cm-level alignment suffices for engagement + relative
   handover. Falsify: a scenario where coarse alignment locks a reference so wrong that the
   subsequent relative drive is unsafe or unusable.

## Rejected alternatives
- **Precomputed forbidden-config map (b).** Rejected: heavyweight for 6-DOF, brittle to any
  bracket/reprint change, and its only edge (runtime speed) is moot when the online check is
  already free. (Research §2.)
- **Full FCL / mesh-mesh collision.** Rejected for now: capsules are ~a few dozen lines, no
  dependency, and adequate; FCL only earns its weight if capsule precision proves
  insufficient (a Slice-2 validation outcome, not an upfront choice).
- **trzy-style relative end-effector IK, drop the AR vision.** Rejected: it structurally
  dodges the exact thing that makes this project novel (pose-matching + self-collision
  envelope). It's the safe boring pick; the whole point of the crucible was to NOT pick it.
  BUT its clutched-relative control is exactly what we adopt post-Grab — we borrow its
  mechanism without abandoning the vision.
- **Reactive-load-only (the original task #4 framing).** Rejected as primary: reacts after
  contact = damage-before-stop at speed. Kept as the backstop layer, not the main guard.

## Fold — author self-strike results (folded back before Temper)
1. **Missing/silent encoder → FAIL to firmware-clamp mode, don't brick.** If a joint the
   model needs is unreadable (a servo drops), the geometric gate can't evaluate. Fold: fall
   back to the PRE-EXISTING safety floor (firmware angle-limit clamps + current reflex) with
   a LOUD warning, not refuse-all-writes (which bricks the arm) and not pass-through (unsafe).
   Degrade to the old behavior, don't fail dangerous or fail useless.
2. **Gate checks the COMMANDED GOAL, not per-tick encoder reads.** (Folded into STAY-SAFE
   above.) Removes bus contention AND is more correct — it's prevention (don't command a bad
   pose). Path safety via small-delta streaming + interpolation for large jumps.
3. **Named bypass scope.** "Single-door in the mutator" holds for the CONTROLLER/teleop path
   (all of slider, in-proc, body tracking go through `FeetechArmController`'s one write
   point). The manual calibration scripts (`calibrate_limits.py`, `eeprom_calibrate.py`) open
   their OWN port and write GOAL_POSITION directly — they are a SEPARATE supervised context,
   deliberately outside the gate (manual bring-up tools, human watching). Named, not hidden.
4. **Slice 0 is a validation dep, not a build dep of Slice 2.** The geometric gate can be
   built + unit-tested with zero load data; load data is needed to TRUST it before driving
   the shoulder for real. Build order isn't falsely serialized — Slice 1/2 code can proceed
   in parallel with Slice 0; only the "drive the real shoulder" validation waits on both.
5. **Build the model on the VIEWER's existing convention, NOT a fresh URDF import.** The
   biggest open-variable rabbit hole was reconciling three angle conventions (URDF vs
   viewer vs controller ticks). Fold: the viewer's hierarchy ALREADY renders a plausible arm
   and the controller's tick→angle mapping already drives it correctly — reuse that ONE
   validated convention for the capsule model; harvest the URDF only for link DIMENSIONS +
   joint LIMITS, not its frame convention. Three mappings → one. De-risks the biggest variable.
6. **The geometric model is justified by the SPINE, not purely by safety** (the strongest
   self-attack). "Just use reactive load, it's simpler" is TRUE if collision-avoidance were
   the only goal — at slow teleop speeds on a light 4A-limited arm, reactive-stop might
   suffice. But we need the kinematic model ANYWAY for See (render) and Grab (alignment
   target); making it also do collision is a near-free third use. The model earns its keep as
   the shared spine; collision-prevention is the bonus, not the sole justification. This is
   the honest defense against the simpler alternative — and it means IF the AR vision were
   dropped, reactive-only would be the right call. Keep this framing explicit.

## Temper — cross-family strike results (folded back, round 1)
Verdicts: **Carnot (Codex) NEEDS-REVISION**, **Kelvin (Gemini) FATAL-FLAW**. Both fatals
assessed foldable — neither dissolves the ore. All findings folded here.

**SAFETY ARCHITECTURE INVERSION (Kelvin #2, fatal — the load-bearing fold).** Reassign
safety-criticality: **reactive PRESENT_LOAD current-cutoff is now the PRIMARY, simple,
always-on safety reflex** (the layer that must not fail — dumb, robust, catches everything
physical incl. cables/world/operator). The **geometric capsule model is the PREVENTIVE
layer + the See/Grab spine** (smart, fallible, prevents *entering* modeled self-collision;
NOT the last line of defense). Defense-in-depth with the simple thing load-bearing. This
resolves "geometric model over-trusted as safety-critical." NOTE — this hands ONE decision
to Nick (see below): whether the AR vision is a hard-enough requirement to build the
geometric model at all, or whether reactive-only safety + the existing relative teleop is
enough. If the AR vision is dropped, reactive-only is the right call and the model isn't
built.

**Startup pose safety (Kelvin #1, fatal).** The gate checks *commanded goals*, so the
power-up pose is never checked. Fold: at connect, run the collision check on the CURRENT
encoder pose. If the arm boots already self-colliding, WARN + require manual back-out
(limp mode) before enabling drive. The "zero-th state" gets its own safety case.

**Recovery from a rejected command (Carnot #4).** A naive reject traps the arm at the
boundary — teleop goes discontinuous, operator can't escape. Fold: the gate does
**nearest-safe-projection** (clamp toward the closest non-colliding config, don't just
drop the command) + a "disable drive until operator backs out" state if projection fails.
Rejecting is not enough; there must be an exit.

**Trajectory between writes (Carnot #1).** Goal-tuple checking ignores overshoot/lag/
compliance/packet-loss. Fold: keep velocity/accel limits (GOAL_SPEED/ACC, already present),
add dense swept-path interpolation on the commanded delta, and an **encoder-vs-command
divergence stop** (if actual encoder pose diverges from commanded beyond a threshold, halt
— catches a servo not tracking, a snagged link).

**Load is pose/speed-dependent (Carnot #2 + Kelvin #3, CONVERGENT — 2 votes).** "Spike =
collision" is naive — gravity/posture/accel load mimics contact; slow contact may not
spike. Fold: **Slice 0 expands from spike-capture to building a load BASELINE MODEL** —
load as f(pose, speed) from no-contact trajectories — so the reactive reflex thresholds on
*residual above expected*, not raw magnitude. (This directly upgrades the now-primary
safety layer, so it matters more than before.)

**World/non-link collision (Carnot #3 + Kelvin #4, CONVERGENT).** The model is self-link
only; misses gripper jaws, servo housings, base/table, world objects, the operator. Fold:
(a) add gripper-jaw capsules + a base/table ground-plane as first-class collision bodies;
(b) NARROW the geometric claim honestly to "self + fixed-base collision"; (c) everything
else (world objects, operator, cables) is the reactive reflex's job — now primary, so this
is coherent. FUTURE: the iPhone has LiDAR — world-collision via depth is a real later
extension (noted, not in scope).

**Grab UX risk (Carnot #5 + Kelvin #5, CONVERGENT).** Full-body-pose-matching via noisy AR
risks proprioceptive dissonance + engagement oscillation — possibly unusable. Fold: the
engagement gesture is a **clutch** (coarse alignment enters an "armed" zone, then a
deliberate confirm — hand-in-volume or a button — commits the lock), with zero-velocity-at-
lock + a first-delta magnitude cap (also Carnot #5's unsafe-first-delta). Borrow trzy's
clutch; keep the AR pose-match as the *aiming* aid, not the sole trigger.

**Shared-model coupling (Carnot #6).** See wants accurate visual geometry; Stay-safe wants
inflated-conservative volumes. Fold: the shared thing is the **FK core** (joint angles →
link frames); render meshes and collision capsules are SEPARATE volumes hanging off that
core. "One shared model" = one shared FK, not one shared geometry.

## Open variables (no silent TODOs)
- Capsule radii per link — UNKNOWN until measured off the physical arm (Slice 1).
- Safety margin (keep-out buffer) — UNKNOWN until Slice 0 gives spike thresholds + Slice 2
  cross-validation; starts conservative, tightens with data.
- Exact FK convention mapping the SO-100 URDF joint frames to the existing viewer's angle
  convention — needs reconciliation (the viewer already has a hierarchy; verify it matches
  the URDF chain).
- Whether the gripper needs its own extra capsule(s) beyond the 6 links.
- Whether Grab's alignment tolerance can be made tight enough to feel deliberate without
  being unachievable given ARKit noise (a Slice-4 question, deferred).
