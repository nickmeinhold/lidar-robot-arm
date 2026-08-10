# DESIGN — Spatial Spine + Motion IR

Status: RE-CAST after Temper round 1 (4-family strike: Maxwell/Kelvin/Carnot/
Tesla; Wu dark — K3 quota) · 2026-08-10
Inputs: `CRUCIBLE.md`, `RESEARCH.md` (5 questions, measured numbers), the
Temper ledger (§8).

## 1. Problem

The arm has joint-sense but no spatial-sense, and its geometry has three
non-converged representations:

1. `server/arm_kinematics.py` + `arm_collision.js` — a hand-coded "viewer
   convention" FK that disagrees with the real robot's model.
2. The real SO-101 URDF (`so101_new_calib.urdf` → baked `so101_urdf.json`) —
   renders and articulates in the browser, drives nothing.
3. The drive path — a *relative* zero ("wherever the arm was at server start"),
   soft boxes around it, EEPROM registers as absolute backstop, and a demo cage
   bolted on because none of the above can compute a collision.

Consequences: "collision-safe" is a fence, not a fact; the shoulder is locked;
LLM choreography is clamped-and-hoped; every future arc (AR overlay, LeRobot
demo rig #2454) is blocked on trustworthy geometry.

**Scope truth (Temper):** this design computes **self-collision** safety. It
does NOT model the environment (table, people, cables, props). Uncaging the
shoulder for *public chat* is therefore NOT a deliverable of this design — see
§3 step 6 and §6. What the gate buys directly: trustworthy geometry, safe
*supervised* full-range use, verified LLM choreography, and the spine every
later arc (including an environment model) stands on.

## 2. Shape

### 2.1 One absolute convention (the keystone)

Adopt LeRobot's calibration semantics: after homing, **tick 2047 ≡ the
mid-range pose ≡ the `new_calib` URDF's zero** — designed to coincide
(RESEARCH Q1/Q5). The relative wherever-it-woke-up convention is **retired**,
not reconciled. Internally the controller works in absolute URDF degrees;
every input becomes an adapter at the edge:

- **Chat / keyframes** speak absolute URDF degrees natively (five arm joints).
- **Gripper is dual-space by design** (Temper — all four reviewers): authored
  and wired as LeRobot-style **0..1 linear fraction** everywhere humans and
  LLMs touch it; mapped to the URDF revolute angle **only inside the gate/FK**
  via a measured two-endpoint affine map (calibrated closed-tick ↔ URDF lower,
  open-tick ↔ URDF upper). One mapping function, one home
  (`so101_kinematics.py`), fixtures pin the round-trip.
- **iPhone teleop** keeps its relative UX: the reference-lock maps phone-zero
  onto the arm's *current absolute pose* at lock time; deltas ride on top.

**Epistemic honesty (Tesla):** this is **convention-absolute** — a hand-homed
LeRobot convention chain, not a metrology chain. Precise enough for
self-consistent FK + self-collision (its actual consumers here); NOT certified
for AR ground-truth or dataset "absolute pose" claims. The contract name in
code is `urdf_calib_frame`. Post-home sanity checks (known fixture poses
compared against FK) bound the error; anything needing better gets its own
metrology story later.

Name bridge (ours ↔ URDF/LeRobot), one table, used everywhere:
`shoulder_yaw↔shoulder_pan · shoulder_pitch↔shoulder_lift ·
elbow_pitch↔elbow_flex · wrist_pitch↔wrist_flex · wrist_roll↔wrist_roll ·
gripper↔gripper(dual-space)`.

**wrist_roll is a distinct joint class** (Temper — Tesla): near-continuous
(EEPROM full-turn, URDF ±~157°). The gate and interpolator carry a per-joint
`wrap` flag: wrist_roll interpolates shortest-path in angle space with
explicit unwrap, and its velocity/sweep math uses wrapped deltas. Fixtures
cover the wrap seam BEFORE step-2 hardening.

### 2.2 Calibration layer (adopt, don't invent)

Re-implement LeRobot's two-action flow on our existing `scservo_sdk` stack
(~150 lines, no lerobot runtime dependency), emitting a **byte-compatible
LeRobot `MotorCalibration` JSON** so the LeRobot demo-rig arc gets calibration
for free:

1. Human moves arm (torque OFF) to mid-range pose → write
   `Homing_Offset = raw − 2047` (sign-magnitude encoding — never int16-cast).
2. Human sweeps each joint (torque OFF) through its full range → record
   min/max → write EEPROM Min/Max Angle Limits = recorded range.

**Calibration is a durable transaction, not a script (Temper — Carnot FATAL,
Tesla, Maxwell):** EEPROM mutation is immortal; abort handlers don't run on
kill -9 / sleep / power loss, and LeRobot's flow transiently leaves limits at
`[0,4095]`. So:

- **Intent log on disk before the first write**: the full intended EEPROM
  image + a stage flag, updated at each stage boundary
  (`backup-taken → limits-opened → homing-written → ranges-written →
  committed`).
- **Timestamped EEPROM backup** taken first; `restore` subcommand replays it.
- **Server startup refuses to drive** (torque stays off, loud error) whenever
  the intent log shows an uncommitted transaction OR any joint's EEPROM range
  is degenerate/full-turn — the wide-open state can never meet powered motion.
- Torque OFF for the entire session; **bench-local only by mechanism, not
  policy** (Carnot): the calibration entrypoint is a CLI on the host that
  requires the serial device path as an argument and refuses to run while the
  server process holds the port — it is not reachable through any
  chat/network surface.
- Refuse to commit while any range is degenerate (today's gripper
  `[2046,2047]` signature — which this flow finally heals) or full-turn.

**Sign-vector commissioning is a measurement procedure with pass/fail, not a
jog (Temper — Kelvin, Carnot, Tesla):** a checked-in commissioning script
runs a **multi-pose suite** — per-joint ±10° solo moves PLUS ≥3 multi-joint
known keyframes (home / stretch / fold) — and for each, the human confirms
the physical arm against the browser URDF render of the same angles (the
overlay is the instrument; tape-measure on tool height as the tie-breaker).
Output: `sign: [±1 ×6]` + a recorded pass/fail per pose, stored in the
calibration JSON. Server refuses absolute mode unless the commissioning
record is present AND all-pass. The suite is re-runnable in minutes after any
reassembly.

Server startup contract: calibration JSON + commissioning record present and
sane → absolute mode; absent → explicit LEGACY relative mode (current
behaviour) with a loud warning. EEPROM limits are a *cache* of the
calibration (one truth, two homes); startup cross-checks and warns on drift.

### 2.3 FK core (roll-your-own + an oracle in CI)

`server/so101_kinematics.py`: load the **same baked `so101_urdf.json` the
browser renders** (single FK source as a filesystem fact), precompute the six
fixed origin matrices, per-pose chain `origin @ Rz(θ)` (all axes local-Z —
RESEARCH Q2, measured 0.94 µs/pose batched). Includes the name bridge + sign
vector + URDF joint limits + the gripper dual-space map.

**Oracle, not vibes (Temper — Carnot, Tesla):** correctness of this 40-line
FK is load-bearing for safety, so it is verified against an independent
implementation: a **CI-only test dependency** on `placo` (LeRobot's own FK,
wired to this exact URDF) or `pin`, asserting tool-frame agreement over a
sampled joint grid to <1 mm. Not a runtime dependency; a truth instrument.
A bake-verification test parses the URDF directly and asserts the JSON
matches. `arm_kinematics.py`'s convention FK is deleted; its
segment-distance/sphere math is retained where it survives review.

### 2.4 Collision model + gate (rebuilt on the evidence — Temper round 1)

The first casting contradicted the research it cited (single capsules are
**measured unsound**; 3° sampling violates the derived bound). Rebuilt:

- **Geometry: six k-means spheres per link, derived offline from the meshes**
  (RESEARCH Q2's measured-sound model: 0.00% disagreement vs mesh checks,
  +6.6 mm mean clearance, 20.5 µs all-pairs/pose). The generator script is
  checked in; its OUTPUT (`so101_spheres.json`) is a reviewed, committed
  artifact (a safety judgment with provenance), regenerated only deliberately.
  Hand-authoring is demoted to reviewing/adjusting the generated set.
- **Pair policy with receipts (Carnot):** start from all link pairs; the
  ignore list is produced by the MoveIt-style offline sampler (adjacent →
  default-pose → always → 10k-sample never, with the fixpoint loop) and
  **every ignored pair carries its reason** in the committed file. No bare
  "adjacent" hand-waves for a machine with brackets and horns.
- **Sampling bound is DERIVED, not chosen (Tesla FATAL):** worst-case
  Cartesian motion is 7.79 mm/deg (shoulder_pan, from RESEARCH's lever-arm
  table). Step size satisfies `step_deg ≤ margin_mm / (2 × 7.79)` — for the
  10 mm hard margin that is **≤ 0.64°**, and RESEARCH's own stricter table
  value (0.376° at 10 mm) is adopted as the default. Cost at the measured
  20.5 µs/pose: a full-workspace 478-sample sweep ≈ 10 ms — admission-time
  only, and typical chat motions sample far less. Streaming checks one pose
  per frame (33 ms budget; trivially inside). If admission cost ever bites,
  adaptive bisection with the same bound is the named successor — never a
  coarser fixed step.
- **Margin is an error budget, not a guess (Carnot, Tesla):** the hard margin
  is the SUM of named terms — encoder quantization (0.09°→~0.7 mm), backlash
  (±0.5°→~4 mm), servo tracking/overshoot at GOAL_SPEED (measured in shadow
  mode), calibration/homing error (bounded by commissioning), sphere-fit
  residual (from the generator report), sampling term (½ step sweep). Initial
  numbers land in `so101_gate_config.json` with provenance comments; shadow
  mode's job is to replace estimates with measurements.
- **Command-space vs tracking-space (Tesla):** the gate certifies COMMANDED
  trajectories; physics can lag. Two defenses: the tracking/overshoot term in
  the margin budget above, plus a low-rate (10 Hz) feedback monitor that
  reads Present_Position and HALTS on measured intrusion past the hard
  margin — the MoveIt collision-monitor pattern, watching reality rather
  than intent.
- **Two-tier semantics on the wire (Tesla, Carnot):** distances map to a
  closed status enum — `CLEAR` / `SOFT` (inside soft band: exponential
  velocity scale, streaming only) / `HARD` (inside hard margin: hold) — plus
  machine-readable reason codes (a small closed taxonomy:
  `SELF_COLLISION_PAIR(a,b)`, `JOINT_LIMIT(j)`, `VELOCITY(j)`,
  `WRAP_SEAM(j)`, `STALE_START`, `PREEMPTED_BY_TELEOP`, `NOT_CALIBRATED`).
  The taxonomy is the contract for chat replies, the LLM repair loop, AND the
  viewer UI.
- **Streaming semantics** (iPhone path): MoveIt Servo's law — exponential
  scale in the soft band applied as a lerp of the delta toward current pose;
  `HARD` = hold-in-place (target := current).
- **Escape from a violating pose — v1 is deliberately dumb (Temper — Kelvin,
  Carnot):** the direction-aware escape is DESCOPED from v1 (its clearance-
  gradient math is genuinely non-trivial: non-smooth min-distance, pair
  switching, local minima — all three reviewers converged on "asserted, not
  proven"). v1 escape mechanisms are explicit and already exist: `release`
  (torque off — hand the arm to the human) and a `guided home` that admits
  only clearance-increasing interpolated steps computed by exhaustive
  small-step search (check all single-joint ±1° candidates, take the best
  min-distance improvement; provably terminates or reports stuck). The
  gradient-based escape is a v2 research item with its own falsifiers.
- **Sequence semantics** (chat/keyframe path): admission all-or-nothing over
  the fully-interpolated sequence at the derived step; rejection carries the
  offending step + reason code. v1 environment is static; the 10 Hz feedback
  monitor is the mid-execution backstop. **Shadow mode has an exit criterion
  (Carnot):** ≥ N sessions (default 3) and ≥ M sequences (default 50)
  logged with: zero false-negatives (measured intrusion the gate passed) and
  false-positive rate < 10% (gate rejections a human review judges
  spurious); the log schema (commanded pose, verdict, min-distance, nearest
  pair, Present_Position trace) is part of step 4, and flipping enforcement
  is a reviewed decision on that evidence, not a vibe.

### 2.5 Motion IR v2 (the LLM interface)

Consensus architecture (RESEARCH Q4): *LLM proposes symbolically; the
deterministic gate disposes; rejections return as repair instructions.*

- **Schema:** keyframe sequence with **named joint keys, units in the name**
  (`wrist_flex_deg`, `gripper_frac`) — never positional arrays;
  `additionalProperties: false`; omitted joint = hold. Sequence-level
  `tempo_bpm` + per-step duration in integer beats (≥1); velocity check =
  wrapped Δangle × tempo against the servo ceiling with headroom. Entry/exit
  contract: `start: current|<named-pose>`, must end at a named pose (default
  `home`); stale-start rejected with `STALE_START`.
- **Primitives first-class, raw keyframes the escape hatch (Temper —
  Carnot/RESEARCH Q4, adopted):** the tool schema's primary vocabulary is
  named parameterized moves — the routine library + `oscillate(joint,
  amp, cycles)`, `sweep`, `pulse_grip`, `hold` — compiled server-side to
  keyframes; a `raw_steps` field exists but the prompt steers toward
  primitives. This shrinks the LLM's search space AND the repair churn.
- **Validation split:** strict structured outputs enforce *shape only* (no
  numeric ranges — RESEARCH Q4 hard limit); the **gate is the only
  range/collision authority**. Limits and units also live in every field
  `description`. Fallback if strict mode is unavailable on OAuth-Bearer:
  same schema via prompt + server-side parse (probe before building — §4).
- **Repair loop:** instructive errors keyed by reason code, `is_error` tool
  results, 2 retries, repair-rate logged.
- **Clamp mode is opt-in and never silent (Temper — Carnot, Tesla):**
  default is reject-with-reason. Clamp-with-warning exists for supervised
  play only, is per-channel opt-in (server config, not LLM-selectable), and
  every clamp posts the warning INTO the channel before motion starts.
- **Multi-commander policy (Temper — Carnot, Tesla):** the busy/backpressure
  queue survives; on top: per-user rate limit (config), sequence length caps,
  **skill promotion requires host-side confirmation** (a saved skill is a
  named config change, not a chat side effect — chat proposes, the operator
  accepts on the server console), and any future uncaged-DOF command class
  requires per-command confirmation as policy. Prompt-injection stance: the
  LLM can only emit gate-checked motion inside the same envelope typed
  commands have — the gate, not the model, is the security boundary (and the
  envelope itself stays cage-limited in public channels; §6).
- **Prompt state:** enumerate per-turn the current pose, envelope, named
  poses, routine/primitive library, and the two-tier status of the arm.
  2–3 few-shot exemplar sequences from the hand-authored routines.
- Single `execute_sequence` tool, `disable_parallel_tool_use: true`.

### 2.6 Seams the Fold exposed (kept from round 0, refined)

- **Wire-space versioning, BOTH directions (Fold + Maxwell):** every
  `arm_state` frame — client→server AND server→viewer broadcast — carries
  `space: "legacy_rel_rad" | "urdf_calib_frame"`. The server adapts legacy
  clients via the reference-lock path; a client/viewer speaking an unknown
  space gets an explicit error frame, never silence.
- **Single-writer arbitration lives in the SERVER (Fold + Maxwell):** the
  motion lease and sequence playback are server-side concepts (the single
  mutator). The chat bot SUBMITS a validated sequence; the server owns its
  execution clock. A live iPhone body-tracking stream preempts
  (`PREEMPTED_BY_TELEOP` posted to chat) — **with a liveness rule (Carnot):**
  the teleop stream holds preemption only while frames are fresher than a
  deadman window (500 ms); a stale stream releases the lease. Physical
  E-stop (power/torque-off) outranks all software writers by construction.
- **Legacy raw-tick tooling sweep** (unchanged from round 0).
- **One direction truth:** sign vector supersedes `JointConfig.direction`.

### 2.7 What this unlocks (out of scope here, enabled by this)

Viewer arc re-coloring from the same gate + status taxonomy (task #2486), AR
See/Grab/Stay-safe (needs the metrology upgrade named in §2.1), LeRobot
dataset recording (#2454), and — behind its own gate — the environment model
that could one day justify public uncaging.

## 3. Build order (each step lands value alone)

0. **Commit the SO101 model assets** + baked JSON + bake-verification test.
1. **`so101_kinematics.py`** — FK + name bridge + gripper dual-space map +
   wrap flag + tests (URDF-vs-bake, known poses, **placo/pin oracle in CI**,
   browser cross-check).
2. **Sphere model + gate library** — sphere-cluster generator + reviewed
   `so101_spheres.json`, pair sampler with per-pair reasons,
   `motion_gate.py` (derived sampling bound, error-budget margin, two-tier
   status + reason taxonomy) + **re-run the timing benchmark with REAL pair
   semantics as the step's acceptance gate (Kelvin FATAL)** + fixtures incl.
   degenerate states (empty sequence, single frame, limit-grazing, wrap
   seam, violating start).
3. **Calibration session** (bench, Nick + arm): transaction machinery first
   (intent log, backup, startup refusal), then the two-action flow, then the
   multi-pose sign commissioning suite; gripper range finally calibrated
   (heals task #1). **Transition safety (Maxwell): the server retains
   today's conservative software clamps on shoulder/elbow (current
   soft-window values) until step 4's gate is enforcing** — calibration
   widening EEPROM must not widen the *commanded* envelope early.
4. **Wire the gate — shadow first** with the §2.4 log schema and exit
   criteria; then flip to enforcing on reviewed evidence. Feedback monitor
   (10 Hz Present_Position) lands here.
5. **Motion IR v2** — schema (primitives-first) + validator-on-gate + repair
   loop + multi-commander policy + prompt state. Replace
   `translate_to_commands`.
6. **Envelope policy review — NOT "uncage" (Temper, all reviewers):** with
   the gate live-verified, Nick reviews what changes where. Honest menu:
   supervised sessions get full gated range; **public chat keeps a cage**
   (self-collision safety ≠ environment safety — people and tables are not
   in the model). Public shoulder access waits for an environment model
   (future arc). The CRUCIBLE's "shoulder comes out of the cage" claim is
   hereby scoped to *supervised* use.

## 4. Claims to falsify (expanded by the Temper)

1. Baked JSON ↔ URDF fidelity (step 1 test exists because this could be wrong).
2. Sign vector cleanly measurable — now backed by the multi-pose suite with
   pass/fail (§2.2), not one jog.
3. Sphere-cluster model leaves a usable workspace (generator report + shadow
   FP rate are the instruments).
4. Gate timing survives REAL pair semantics (step 2 acceptance gate).
5. Calibration transaction is crash-safe (kill it mid-write on the bench and
   verify the startup refusal — an explicit step-3 test).
6. Absolute-anchored reference lock preserves iPhone teleop feel.
7. Strict structured outputs available on OAuth-Bearer (probe first).
8. The derived sampling bound + margin budget actually prevent tunneling
   (numeric worst-case fixture: max-lever pose, thinnest corridor).
9. Tracking error stays inside its budgeted margin term (shadow mode
   measures; the 10 Hz monitor is the backstop if not).
10. Multi-commander policy holds under a meetup pile-on (burst test exists
    from the 2026-08-10 session; rerun against sequences).
11. The v1 escape (`release` + guided-home search) suffices in practice —
    if shadow mode shows frequent stuck-states, the v2 gradient escape gets
    pulled forward WITH its own proofs.

## 5. Rejected alternatives

- **FK libraries at runtime** (pinocchio ✓ arm64, placo, yourdfpy) — 40
  lines beats 15 packages; **placo/pin adopted as CI oracle** (Temper).
- **Single capsule per link** — REJECTED BY MEASUREMENT (RESEARCH Q2): 46%
  false rate or workspace-swallowing radii. Six-sphere clusters are the
  evidence-backed model. (Round-0 casting wrongly chose capsules; the Temper
  caught it.)
- **Mesh collision (python-fcl)** — blocked on convex decomposition the URDF
  doesn't ship; sphere clusters measured sufficient.
- **Keeping the relative convention** — root of the divergence.
- **Full lerobot dependency for calibration** — format compatibility without
  the stack.
- **End-to-end VLA** — no validator insertion point.
- **Per-step seconds in the IR** — loses retiming + velocity check.
- **A hand-taught coupled-limit table instead of FK** — solves only the
  collision half, no spine. **Elevated to a NAMED FALLBACK with a kill
  criterion (Tesla):** if shadow-mode false-positive rate stays >10% after
  two tuning rounds, ship the taught table for shoulder policy and let the
  spine mature decoupled from the demo cadence.
- **Direction-aware gradient escape in v1** — descoped (§2.4); the honest v1
  is torque-off + guided-home search.

## 6. Blast radius & consent spine

- **Calibration mutates EEPROM** — bench-local by *mechanism* (CLI + serial
  port exclusivity, unreachable from chat), transactional by *construction*
  (intent log + startup refusal), Nick present, torque off.
- **Gate wiring touches the live drive path** — shadow mode with exit
  criteria; `--legacy-relative` + `--no-gate` escape hatches; conservative
  software clamps persist through the transition window (step 3→4).
- **Chat exposure: the cage does NOT lift for public channels in this
  design.** Self-collision computation ≠ environment safety. Supervised
  envelope changes are Nick's explicit per-session call; public envelope
  changes are out of scope until an environment model exists. Skill
  promotion requires host-side confirmation; clamp mode is per-channel
  opt-in and channel-visible.
- No new network surfaces; LLM calls stay on the existing OAuth-Bearer path.

## 7. Open variables (enumerated, not rounded away)

- Sphere-generator parameters (k per link, fit tolerance) + the reviewed
  first `so101_spheres.json`.
- Margin budget's measured terms (tracking, backlash on THIS arm) — shadow
  mode fills them in.
- Strict-tool-use availability on OAuth (probe before step 5).
- Deadman window value for teleop lease (500 ms starting point).
- Shadow-mode thresholds (3 sessions / 50 sequences / <10% FP) — starting
  points, Nick may tune.
- `collision_fixtures.json` from the old plan — superseded by step 2.

## 8. Temper ledger (round 1 — 2026-08-10)

Panel: Maxwell (author-instance, domain-local only) + Kelvin (Gemini 3 Pro) +
Carnot (GPT-5.5) + Tesla (Grok). **Wu dark — K3 + fallback both
billing-cycle quota-exhausted; noted as a coverage gap, not an approval.**
All four seated verdicts: REQUEST_CHANGES. Gate (Maxwell + ≥2 adversaries):
satisfied.

Verified-real findings and dispositions (every one checked against
RESEARCH.md / repo before folding — none rejected as hallucination this
round; the two research-citation attacks were CONFIRMED against the file):

| # | Finding (convergence) | Class | Disposition |
|---|---|---|---|
| 1 | Self-collision gate ≠ uncage license; environment unmodeled (Carnot F, Tesla F) | FATAL-scope | Folded: §1 scope truth, §3 step 6 reframed, §6 — public cage stays; CRUCIBLE claim scoped to supervised use |
| 2 | Capsule model contradicts RESEARCH's measured result (Tesla F, Carnot MF) | FATAL-geometry | Folded: §2.4 rebuilt on six-sphere clusters (measured sound) |
| 3 | 3° sampling violates derived 0.376° bound (Tesla F, Carnot MF) | FATAL-sampling | Folded: derived bound adopted; adaptive bisection named successor |
| 4 | 0.371 ms benchmark used proxy pair semantics (Kelvin F) | MUST-FOLD | Folded: real-semantics re-benchmark = step-2 acceptance gate |
| 5 | Calibration not crash-safe; EEPROM wide-open window (Carnot F, Tesla MF, Maxwell MF) | MUST-FOLD | Folded: durable transaction + startup refusal + falsifier #5 |
| 6 | Gripper dual representation undesigned (ALL FOUR) | MUST-FOLD | Folded: §2.1 dual-space + measured affine map |
| 7 | Sign commissioning not a metrology process (Kelvin T, Carnot MF, Tesla MF) | MUST-FOLD | Folded: multi-pose suite with pass/fail record |
| 8 | Direction-aware escape asserted, unproven (Kelvin MF, Carnot MF, Tesla) | MUST-FOLD | Folded: descoped to v2; v1 = release + guided-home search |
| 9 | Command-space gate vs tracking-space physics (Tesla MF) | MUST-FOLD | Folded: margin error-budget term + 10 Hz feedback monitor |
| 10 | Shadow mode had no exit criterion (Carnot MF) | MUST-FOLD | Folded: §2.4 log schema + thresholds |
| 11 | Margin was a guess (Carnot MF, Tesla) | MUST-FOLD | Folded: named error budget |
| 12 | Multi-user chat policy under-modeled (Carnot MF, Tesla MF) | MUST-FOLD | Folded: §2.5 multi-commander + skill-save confirmation + clamp visibility |
| 13 | wrist_roll wrap topology unhandled (Tesla MF) | MUST-FOLD | Folded: §2.1 joint class + step-2 fixtures |
| 14 | "Absolute" overclaims epistemically (Tesla MF) | MUST-FOLD | Folded: convention-absolute naming + §2.1 honesty |
| 15 | Two-tier thresholds + reason taxonomy missing (Tesla MF, Carnot T) | MUST-FOLD | Folded: §2.4 status enum + closed reason codes |
| 16 | FK needs an oracle, not just a fallback (Carnot MF, Tesla T) | MUST-FOLD | Folded: placo/pin CI oracle |
| 17 | Wire-space covers only client→server (Maxwell MF) | MUST-FOLD | Folded: §2.6 both directions |
| 18 | Motion lease home unstated (Maxwell MF, Carnot T liveness) | MUST-FOLD | Folded: server-side lease + deadman |
| 19 | Calibration→gate transition window (Maxwell MF) | MUST-FOLD | Folded: §3 step 3 retains conservative clamps |
| 20 | Raw keyframes primary vs primitives (Carnot T) | TRADEOFF | Adopted primitives-first (§2.5) — cheap, aligned with research |
| 21 | A+B fusion couples delivery (Tesla T) | TRADEOFF | Named: owner Nick/build-order; mitigation = steps land value alone; kill criterion on fallback table (§5) |
| 22 | Beats/tempo + clamp-mode footguns (Tesla T) | TRADEOFF | Named: clamp opt-in + channel-visible; beats kept (retiming + velocity check earn it) |
| 23 | Sphere/capsule authoring rigor (Kelvin MF) | MUST-FOLD | Folded: generated-then-reviewed artifact with provenance (§2.4) |

Re-cast rounds used: **1 of 3.** Next: re-strike (round 2) on the re-cast
design — the recast is substantial, so it has NOT survived a clean strike yet.

## 9. Temper round 2 — amendments (normative) and ledger

Panel: Maxwell + Carnot + Tesla (Kelvin dark this round — Gemini 429 capacity;
Wu still quota-dark). Gate satisfied. Both seated adversaries:
REQUEST_CHANGES. Character shift: no findings dissolve the architecture; all
are proof-deepening or scope-honesty. Verified dispositions below; the
AMENDMENTS in this section are normative and override earlier sections where
they conflict.

### 9.1 Amendments folded into the design

1. **Table half-space in the gate (from the environment FATALs).** The gate
   gains a static workspace model v0: a table plane (half-space) at a
   measured height + optional keep-out boxes, checked in the same pass as
   self-collision spheres. Near-zero cost, kills the single most likely
   real-world strike. This does NOT change §6's public-cage stance — it
   shrinks supervised residual risk; it is not an environment model.
2. **The 10 Hz feedback monitor is a DAMAGE-LIMITER, not a safety invariant.**
   Worst-case detection-to-stop travel at rated speed over a 100 ms window is
   tens of degrees; no margin term may credit the monitor. The invariants are
   admission-time checking of commanded paths + the margin budget; the
   monitor exists to shorten bad outcomes and feed shadow-mode data.
3. **Calibration transaction: per-joint readback-verify + commit bitmap.**
   Every EEPROM write is read back and compared before the stage advances; the
   intent log carries a per-joint commit bitmap; startup's single predicate is
   "every joint's hardware state matches the intent image" — stage flags alone
   are insufficient (mid-packet bus death).
4. **Legacy relative mode is a time-boxed migration tool, not a peer mode.**
   Mixed-space sessions are refused (one space per server run); the gate and
   the LLM path REQUIRE absolute mode; legacy exists only to keep the iPhone
   demo alive pre-calibration and is scheduled for deletion after step 4.
5. **Public channels get primitives/routines ONLY** — `raw_steps` is
   supervised-mode-only, enforced server-side by channel policy, not prompt.
6. **Sequence velocity certificates expire on interference:** a sequence that
   enters SOFT or is preempted ABORTS (hold + reason); no partial resume, no
   re-scaled continuation in v1. Admission's velocity proof is only valid for
   the uninterfered playback it certified.
7. **Guided-home demoted:** default escape is `release`. Guided-home runs only
   with monotone improvement across ALL pairs (min-distance up, max
   penetration down) and hard-stops to `release` advice after N steps.
8. **Derived constants derive from artifacts, not prose:** lever-arm bound,
   sampling step, and margin terms are computed at load from the current
   sphere set + baked URDF, so regenerating geometry re-derives the bounds
   automatically. RESEARCH's numbers are evidence, not law.
9. **Commissioning produces NUMBERS, not vibes:** the multi-pose suite
   records physical tool-height/reach measurements (tape/fixture) at ≥2
   poses; the residuals become the calibration-error term in the margin
   budget. The browser overlay remains the workflow instrument; the physical
   measurements are the oracle (breaks the circularity Carnot named).
10. **Shadow-mode false-negative instrument is independent of the sphere
    model:** held-out mesh-oracle checks on logged poses (offline, slow,
    exact) + any physical contact events. A verifier sharing the verified
    representation is blind at the shared layer (Tesla) — the mesh oracle is
    the second eye.
11. **Step-2 acceptance includes an adversarial pose corpus:** joint-limit
    extremes, gripper open/closed, wrap seams, folded configurations, every
    ACM-ignored pair exercised near contact. Ignored pairs each carry a
    deterministic geometric justification; the sampler is a check, not the
    certificate.
12. **Envelope-wideners are audit events:** `--no-gate`, legacy mode, clamp
    opt-in, skill promotion, gate-config changes all log loudly AND announce
    on the wire (viewer + chat see "gate disabled"). The gate being the
    security boundary makes its off-switches part of the attack surface.
13. **Spine/IR ship-split:** the gate API (validate(sequence|pose) → verdict
    + reason codes) is the frozen interface; Spine v1 (steps 0–4) and IR v2
    (step 5) ship on their own criteria against it. If the spine slips, the
    IR does NOT ship on clamp-and-hope — it waits; the demo keeps routines.

### 9.2 Named tradeoffs (owner + cost + rationale)

- **Supervised full-range = operational deadman** — owner: Nick. "Supervised"
  means physical kill authority in hand (power/torque switch), not presence.
  Cost: demo ergonomics. Rationale: self-collision gate ≠ environment safety;
  residual tool-vs-world risk is accepted only under deadman conditions.
- **Convention-absolute frame feeds a safety gate** — owner: design (Maxwell).
  Accepted because self-collision needs self-consistency, with homing error
  entering the margin budget as a MEASURED term (amendment 9). Cost: margins
  carry the residual; AR/dataset claims stay embargoed. The metrology-grade
  chain remains future work.
- **CAD-proxy geometry ("measured sound" ≠ as-built)** — owner: Nick at
  shadow-exit review. Spheres model the CAD meshes, not screws/cables/bow.
  Cost: periodic physical validation; mitigation: conservative radii + margin
  + amendment 10's mesh oracle.
- **Host-local calibration threat model** — owner: Nick. The CLI defends
  against chat/network paths, not a compromised host. Accepted for a hobby
  bench; §6 wording softened from "by construction" to "by mechanism, against
  the network surface."
- **CI oracle must block kinematics merges** — owner: pipeline. A skipped
  placo/pin oracle on a flaky runner silently un-anchors FK; red oracle = no
  merge of kinematics changes.
- **Beats/tempo retained** — owner: IR author. Velocity re-validated against
  measured servo response before admission; certificates expire per
  amendment 6.
- **Wu-family blind spot** — owner: next strike chair. Two rounds ran without
  K3's bias; injection/chat and hardware-transaction findings got only
  Carnot/Tesla coverage. Re-seat Wu when quota refreshes if a further strike
  runs.

### 9.3 Rejected-as-rehash (with reasons)

- "Convention-absolute is FATAL" — round-1 finding #14 already folded; round
  2's residue is the measured-residual requirement (amendment 9) + the named
  tradeoff. The claim "gate certifies a simulation" is true at the layer the
  design now states honestly; dissolving the gate on those grounds would
  reject the entire class of model-based safety, which the evidence (margins
  + budgets + oracle) does not require.
- "Runtime FK library instead of 40 lines" — the CI-oracle + conformance
  fixtures + load-time sanity checks give the correctness guarantees a
  library import would, without the 15-package runtime; classified as
  performance-plus-ownership choice behind tests (Carnot's own alternative
  framing, adopted).

Re-cast rounds used: **2 of 3.** Round 3 is the confirming strike.
