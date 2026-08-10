# CRUCIBLE — the spatial spine + LLM motion IR

**Selected:** 2026-08-10 22:24, Nick-confirmed straight off the AMR meetup demo
(consent gate crossed by explicit invocation — the human chose).

## The ore

Two fused halves, deliberately one forge:

**A. The spatial spine.** Today the arm has perfect joint-sense and zero
spatial-sense: three stacked conventional zeros (encoder tick 0, wherever-it-
woke-up server home, chat seed), no live forward kinematics, and a collision
story that is a *fence* (EEPROM boxes + a demo cage) rather than a
*computation*. Meanwhile the real SO-101 URDF sits in the repo rendering
beautifully in the browser — unconnected to the Python that actually moves
servos, and disagreeing with the hand-coded `arm_kinematics.py` convention
(the 3-sources-of-truth crux, claude-tasks #2485). The spine = calibrate
tick→link-angle offsets against a known pose, crown the URDF as the ONE FK
source, wire a computed self-collision gate into the drive path
(claude-tasks #2303).

**B. The motion IR.** Tonight's meetup proved chat-driven motion delights a
room — and that one-shot flat-command translation makes any LLM choreograph
blind ("the LLM sucked" — Nick). The fix is representational: LLM authors
keyframes in the ROUTINES IR (already the safety-fixture-checkable format),
sees the current pose + envelope, composes motion primitives, and — the
capstone — every generated trajectory is FK-simulated through the spine's
gate before a servo moves.

## Why this thrills AND what it changes

The two halves close a single loop: **generate → simulate → verify → move.**
That's the difference between a puppet with a rulebook and a machine that
knows where its hand is. Concretely it changes: the shoulder comes out of the
cage (safety computed, not fenced); "collision-safe" stops being an
untrustworthy claim (Kelvin's 2026-08-01 verdict dissolves); LLM choreography
becomes verifiable instead of clamped-and-hoped; and the AR "See/Grab/
Stay-safe" vision + the LeRobot demo-rig arc (#2454) both stand on this spine.
Every future arc needs this. It is the load-bearing wall.

**The spark:** the arm goes from "knows how bent its joints are" to "knows
where it is in space" — proprioception to self-model — and chat-choreography
inherits that self-model for free.

## The falsifier (what would prove this ore is slag)

- If tick→URDF-angle calibration cannot be made trustworthy without hardware
  we don't have (e.g. requires a jig / absolute reference pose we can't
  produce by hand+eye), the spine's foundation is sand and the gate inherits
  garbage-in.
- If URDF-based self-collision checking can't run comfortably inside the
  30 Hz drive loop on this Mac, the gate can't live in the hot path and the
  design must retreat to command-time-only checking — weaker, but possibly
  still sufficient. (This bounds, not dissolves.)
- If LeRobot's existing SO-101 calibration flow already solves A end-to-end,
  the build shrinks to adoption — that's a *win*, not slag, but the design
  must check before building.

## Verified artifacts (the ore is real)

- `server/static/models/SO101/so101_new_calib.urdf` + baked `so101_urdf.json` ✓
- `server/arm_kinematics.py` (357 lines, OLD hand-coded convention) +
  `server/static/arm_collision.js` (180 lines, same old convention) ✓
- `server/zero_calibration.json` + capture_zero flow in `server.py` ✓
  (captured ticks exist, never grounded against a measured pose)
- `ArmCommandEngine.ROUTINES` keyframe IR + structural safety fixtures ✓
- Live teleop substrate through commit `0a76f4a` (meetup-battle-tested) ✓
- NOT found: `server/collision_fixtures.json` (named in #2485) — open variable.

## Scoring (rubric, for the record)

- Aliveness **3** — evidence: three retro poles converged on the crux
  2026-08-01; Nick asked "does it know where it is in space?" unprompted
  tonight, then said "let's build the hard part."
- Impact **3** — evidence: unlocks shoulder (visible capability), makes the
  safety claim true (trust), and is the prerequisite for BOTH named future
  arcs (#2454 LeRobot rig, AR vision).
- Product **9** — peak of the board; no competing candidate was close.
