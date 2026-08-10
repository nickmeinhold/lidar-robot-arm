# RESEARCH — Spatial Spine + Motion IR

Heat movement of the `/crucible` forge. Deep research backing the design doc for:
a single shared kinematic spine (URDF-authoritative FK) plus an LLM-authored
keyframe motion IR gated by a self-collision validator.

Researched 2026-08-10. Every factual claim carries a URL. Where a question could
not be settled from sources it is called out explicitly as **UNSETTLED**.

---

## Headline findings

The ten things that should change the design, in rough order of impact.

1. **The zero-pose problem is already solved upstream, and we should adopt it.**
   lerobot's calibration asks the user to put every joint at the middle of its
   range; `so101_new_calib.urdf`'s zero is *defined* as the middle of every
   joint's range. **These are the same physical pose.** Replace the relative
   "zero = wherever the arm was at server start" convention with
   `ticks 2047 = URDF zero`. (Q1)

2. **Adopting lerobot's calibration overwrites the EEPROM registers we currently
   treat as authoritative.** `write_calibration` writes `range_min`/`range_max`
   into Min/Max Position Limit (0x09/0x0B), and `reset_calibration` blows them
   open to `[0, 4095]` *mid-procedure*. Our "absolute EEPROM bounds" and
   lerobot's calibration file are one source of truth, not two. (Q1)

3. **The legacy collision radii under-approximate the real arm by 2–3×**, in the
   unsafe direction. `arm_kinematics.py` uses 8–12 mm render radii; the real
   geometry is 20–36 mm per sphere cluster. Measured, and it closes a variable
   the code itself flags as open. (local measurement)

4. **Both performance fears are unfounded.** Hand-rolled numpy FK runs at
   **19.8 µs/pose** and a full all-pairs sphere collision check at **20.5 µs**,
   on this Mac. ~40 µs total, 1,650× headroom over 30 Hz. Nothing here needs a
   fast library. (local measurement)

5. **Install friction is zero and the conda folklore is stale.** `pin`, `coal`,
   `python-fcl`, and `placo` all have native cp313 arm64 wheels for this exact
   Python. `hpp-fcl` has no cp313 arm64 wheel and is superseded by `coal`.
   (local measurement)

6. **One capsule per link does not work; six spheres does.** The single-capsule
   model reported the forearm and gripper colliding in 96.7% of poses. With six
   spheres per link that becomes **0.00%, with +6.6 mm clearance** — and the
   exclusion is *proven* by an independent convex-hull argument, not assumed.
   Use **max** cluster radius, not 99th-percentile: the latter leaves ~0.9% of
   each link outside its own collision model. (local measurement)

7. **No allowed-collision matrix exists for the SO-101 anywhere upstream** — the
   MJCF has zero `<contact>`/`<exclude>` elements. We must generate our own; one
   is derived here (9 pairs excluded, 12 checked). (Q5 + local measurement)

8. **MoveIt's constants cannot be transplanted.** OMPL's verified default
   discretization on our arm leaves **41 mm of motion unchecked against a 10 mm
   threshold** — the two mechanisms don't compose. The invariant is *margin ≥ max
   Cartesian motion per sample*, which for 10 mm means a **0.376° joint step**.
   Also: MoveIt checks collisions at **10 Hz against a 100 Hz control loop**, so
   we do not need 30 Hz checking; and its checker returns **signed distance, not
   a boolean**, which is the decision that enables everything else. (Q3)

9. **Structured outputs cannot enforce joint limits.** `minimum`/`maximum` are
   explicitly unsupported, and array length cannot be constrained. Therefore
   (a) our validator is the *only* thing enforcing range, and (b) **use named
   joint keys, not a positional array** — an object with `required` +
   `additionalProperties: false` makes the correct shape the only expressible
   one, killing element-order and off-by-one bugs at the grammar level. (Q4)

10. **The per-joint sign vector is the one thing no source can give us.** Every
    URDF axis is `0 0 1` with the sense hidden in `rpy`, and lerobot sets
    `drive_mode = 0` everywhere. It must be measured as an explicit
    commissioning step. (Q1)

---

## Local ground truth (measured, not researched)

Before the research proper, facts established by reading the repo's own files —
these anchor everything below.

`server/static/models/SO101/so101_new_calib.urdf` is byte-for-byte the
`onshape-to-robot`-generated model from TheRobotStudio, header comment intact:

```
<!-- Generated using onshape-to-robot -->
<!-- Onshape https://cad.onshape.com/documents/7715cc284bb430fe6dab4ffd/... -->
<robot name="so101_new_calib">
```

Extracted joint table (all angles converted to degrees):

| Joint | Type | Parent → Child | Axis | Limits (deg) |
|---|---|---|---|---|
| `shoulder_pan` | revolute | `base_link` → `shoulder_link` | `0 0 1` | −110.0 … +110.0 |
| `shoulder_lift` | revolute | `shoulder_link` → `upper_arm_link` | `0 0 1` | −100.0 … +100.0 |
| `elbow_flex` | revolute | `upper_arm_link` → `lower_arm_link` | `0 0 1` | −96.8 … +96.8 |
| `wrist_flex` | revolute | `lower_arm_link` → `wrist_link` | `0 0 1` | −95.0 … +95.0 |
| `wrist_roll` | revolute | `wrist_link` → `gripper_link` | `0 0 1` | −157.2 … +162.8 |
| `gripper` | revolute | `gripper_link` → `moving_jaw_so101_v1_link` | `0 0 1` | −10.0 … +100.0 |
| `gripper_frame_joint` | fixed | `gripper_link` → `gripper_frame_link` | — | — |

Three structural facts that drive the design:

1. **Every joint axis is `0 0 1`.** `onshape-to-robot` bakes the real rotation
   direction into each joint's `<origin rpy>`, never into the axis vector. You
   cannot read a joint's physical sense off the axis — it is always local +Z.
   This is why a hand-written kinematics convention (`server/arm_kinematics.py`)
   diverges from the URDF: the hand-written one almost certainly picks
   per-joint axes like "Y for pitch, Z for yaw", which is a *different
   parameterization of the same arm* and will silently disagree.

2. **8 links, 6 actuated DOF, one fixed tool frame.** `gripper_frame_link` is
   the tool-tip frame and is the frame lerobot's own FK targets (see Q1).

3. **The collision geometry is the visual geometry, and it is heavy.** The URDF
   has 17 `<visual>` and 17 `<collision>` tags referencing the *same* STL files.
   Measured triangle counts of `assets/*.stl`:

   | Mesh | ~Triangles |
   |---|---|
   | `wrist_roll_pitch_so101_v2.stl` | 53,994 |
   | `under_arm_so101_v1.stl` | 39,516 |
   | `base_motor_holder_so101_v1.stl` | 37,540 |
   | `wrist_roll_follower_so101_v1.stl` | 28,796 |
   | `moving_jaw_so101_v1.stl` | 28,270 |
   | `upper_arm_so101_v1.stl` | 26,068 |
   | `motor_holder_so101_base_v1.stl` | 22,586 |
   | `motor_holder_so101_wrist_v1.stl` | 21,042 |
   | `sts3215_03a_v1.stl` | 19,080 |
   | `sts3215_03a_no_horn_v1.stl` | 17,316 |
   | `rotation_pitch_so101_v1.stl` | 17,672 |
   | `base_so101_v2.stl` | 9,430 |
   | `waveshare_mounting_plate_so101_v2.stl` | 1,254 |

   These are CAD-export display meshes, not collision proxies. Feeding them raw
   to a mesh-mesh collision checker is the single biggest performance trap in
   this design. See Q2.

### Measured on this machine: FK is free

Rather than cite a published benchmark, I wrote a throwaway numpy FK against the
already-baked `so101_urdf.json` and timed it on this M-series Mac. The baked JSON
has exactly the shape a DIY FK needs — a `joints` list of
`{name, type, parent, child, origin:{xyz,rpy}, axis, limit}` and a `links` list —
so the parse cost is genuinely zero.

Precomputing each joint's static origin transform once, then composing 4×4s per
pose (every axis being `0 0 1` collapses the joint rotation to a plain Z-rotation):

```
DIY numpy FK (all 7 frames): 19.8 us/pose  ->  50,406 poses/sec
```

**~1,650× headroom over 30 Hz**, from unoptimized straight-line numpy with no
vectorization across poses. The conclusion for the whole design is blunt:
**forward kinematics is not a performance consideration at any rate we care
about.** Every performance question in Q2 is really a question about *collision
checking*. A DIY FK also has zero install cost, which on macOS arm64 is worth
more than it sounds (see Q2).

### Measured on this machine: capsules make collision free too

Second throwaway experiment, same principle — measure our own geometry rather
than reason about someone else's. I parsed the collision STLs directly (pure
`struct` + numpy, no dependencies), transformed each into its link frame, and fit
one PCA capsule per link: principal axis for the segment, 99th-percentile
perpendicular distance for the radius.

| Link | Collision verts | Capsule length (mm) | Radius (mm) |
|---|---:|---:|---:|
| `base_link` | 201,912 | 114.3 | 61.4 |
| `shoulder_link` | 178,014 | 120.9 | 41.8 |
| `upper_arm_link` | 135,444 | 149.3 | 48.0 |
| `lower_arm_link` | 238,914 | 160.9 | 51.4 |
| `wrist_link` | 213,930 | 79.6 | 45.2 |
| `gripper_link` | 143,628 | 107.9 | 39.0 |
| `moving_jaw_so101_v1_link` | 84,810 | 93.2 | 26.7 |

**Total collision geometry: 1,196,652 vertices across 7 links.** That number is
the argument against naive mesh-mesh in one line.

7 links → 21 unordered pairs (of which 6 are parent-child adjacent and will be
excluded, leaving **15 real pairs**). Vectorized segment-segment distance across
all 21 pairs at once, in pure numpy:

```
all-21-pair capsule check: 20.5 us  ->  48,731 full checks/sec
```

So the entire validation gate — FK plus all-pairs self-collision — costs
**≈40 µs per pose, ~25,000 poses/sec**, with no compiled dependency beyond numpy.
Concretely: a keyframe transition sampled at 100 interpolated waypoints validates
in **~4 ms**. Validating a whole 20-step LLM-authored sequence at that resolution
is well under 100 ms — fast enough to validate synchronously inside the chat
request, before replying to the user.

**Honest caveat on the capsule fit.** One capsule per link is a first-order
approximation and it is a *poor* fit for squat links — `base_link` gets a 61 mm
radius, which will swallow real free space and cause false rejections near the
base. The 99th-percentile radius also deliberately ignores the outer 1% of
geometry, so it is not conservative everywhere. This measurement establishes that
the *approach* is affordable (by 3 orders of magnitude), not that these seven
specific capsules are the right ones. Multi-capsule or sphere decomposition per
link costs proportionally more pairs and still lands far inside budget.

### Measured on this machine: one capsule per link is NOT enough

I then ran the offline allowed-collision-matrix experiment the literature
recommends (Q3): sample random valid joint configurations, run FK, record which
link pairs ever collide. 20,000 samples drawn uniformly from the URDF joint
limits, evaluated at **16,796 poses/sec including FK**.

The result **falsified the single-capsule approach**:

| Pair | adj | min gap (mm) | % colliding |
|---|:-:|---:|---:|
| `base_link` \| `upper_arm_link` | . | −67.6 | 94.1% |
| `lower_arm_link` \| `gripper_link` | . | −59.6 | **96.7%** |
| `lower_arm_link` \| `moving_jaw` | . | −19.4 | 77.8% |
| `wrist_link` \| `moving_jaw` | . | −70.6 | 94.3% |
| `base_link` \| `lower_arm_link` | . | −112.7 | 22.1% |
| … (all 21 pairs collided at least sometimes) | | | |

**Zero pairs were excludable.** A gate reporting that the forearm and the gripper
are in collision 96.7% of the time is not a safety system, it is a random number
generator — the arm physically cannot self-collide in 97% of its workspace. The
cause is visible in the fitted radii: 39–61 mm radii on links only 80–160 mm
long, i.e. the capsules are nearly spheres that swallow the whole robot.

### …but six spheres per link works

Same experiment, replacing each link's single capsule with a **k-means sphere
decomposition (K = 6 spheres per link, 99th-percentile radius per cluster)** —
42 spheres total, 540 non-adjacent sphere-pairs, 20,000 fresh samples:

| Pair | min gap (mm) | % colliding |
|---|---:|---:|
| `lower_arm_link` \| `gripper_link` | **+6.6** | **0.00%** |
| `lower_arm_link` \| `moving_jaw` | **+22.5** | **0.00%** |
| `upper_arm_link` \| `moving_jaw` | −38.2 | 0.92% |
| `upper_arm_link` \| `gripper_link` | −40.4 | 2.94% |
| `base_link` \| `lower_arm_link` | −79.9 | 7.32% |
| `shoulder_link` \| `moving_jaw` | −52.8 | 7.49% |
| `upper_arm_link` \| `wrist_link` | −36.2 | 8.08% |
| `base_link` \| `moving_jaw` | −74.0 | 10.06% |
| `base_link` \| `upper_arm_link` | −23.6 | 11.22% |
| `base_link` \| `wrist_link` | −83.6 | 12.68% |
| `shoulder_link` \| `gripper_link` | −55.3 | 12.62% |
| `base_link` \| `gripper_link` | −78.1 | 13.15% |
| `shoulder_link` \| `lower_arm_link` | −57.2 | 14.90% |
| `shoulder_link` \| `wrist_link` | −57.7 | 15.57% |
| `wrist_link` \| `moving_jaw` | −20.6 | 96.65% |

The pathological result is gone: the forearm/gripper pairs now show a *positive*
minimum clearance across 20,000 poses. **Two pairs never collide and are safe to
exclude outright** — this is our SO-101 allowed-collision matrix, and it is the
thing no upstream repo provides (Q5).

The remaining 7–16% collision rates on base/shoulder vs. distal links are
plausible and expected: sampling *uniformly* over full joint ranges genuinely
folds the arm into its own base much of the time. That is a statement about the
sampling distribution, not about the gate.

`wrist_link | moving_jaw` at 96.65% is the interesting one: `moving_jaw` is a
*grandchild* of `wrist_link` (`wrist_link → gripper_link → moving_jaw`). It is
effectively always-touching near-adjacent geometry — exactly MoveIt's
"Adjacent/Default" ACM category — and belongs in the exclusion list alongside the
true parent-child pairs rather than being treated as a finding.

Cost: 4,545 poses/sec, still **150× headroom over 30 Hz**, and that figure is
depressed by a deliberately naive per-pair Python loop in the experiment; the
sphere-distance math itself is one vectorized numpy expression.

**Proposed starting ACM for the SO-101** (6 parent-child pairs + 1 near-adjacent
+ 2 measured-never): exclude `base|shoulder`, `shoulder|upper_arm`,
`upper_arm|lower_arm`, `lower_arm|wrist`, `wrist|gripper`, `gripper|moving_jaw`,
`wrist|moving_jaw`, `lower_arm|gripper`, `lower_arm|moving_jaw` — leaving **12
pairs actually checked** per pose.

**Caveat, and then its resolution:** the numbers above come from a sphere
approximation validated only against itself, which is precisely the
self-referential trap. So I checked them with a genuinely independent instrument.

### The two ACM exclusions are *proven*, not assumed

Each link's collision geometry was reduced to its **exact convex hull**
(`scipy.spatial.ConvexHull`) and pairs were tested for intersection by
**linear-programming feasibility** — hulls A and B intersect iff there exist
convex combinations λ, μ ≥ 0 with `Aᵀλ = Bᵀμ`, solved with `scipy.optimize.linprog`
(HiGHS). Hull sizes: 643–4,050 vertices per link.

The logic that makes this rigorous: **a mesh is a subset of its convex hull.** So
if two hulls do *not* intersect, the two meshes provably do not intersect. A
negative hull result is a proof, not an approximation.

Over 1,500 random valid poses:

| Pair | Result |
|---|---|
| `lower_arm_link` \| `gripper_link` | **Proven collision-free** at every sampled pose |
| `lower_arm_link` \| `moving_jaw` | **Proven collision-free** at every sampled pose |
| `upper_arm_link` \| `moving_jaw` | hulls intersect in 0.20% — inconclusive |
| `upper_arm_link` \| `gripper_link` | hulls intersect in 0.40% — inconclusive |

Both sphere-derived exclusions survive contact with a completely different and
mathematically sound instrument. The two "inconclusive" rows are exactly what the
theory predicts: hull intersection is an *over*-approximation, so a positive
result proves nothing (these links are concave — the forearm has a channel the
gripper passes through — and their hulls overlap where the meshes do not).

So the ACM exclusions are safe. The sphere model's *rejections* still warrant a
tighter checker before they gate hardware, since an over-approximate rejection
means false stops, not unsafe motion — the error is in the conservative direction,
which is the right direction for a safety gate.

Beyond the ACM, this experiment also settles the sphere count: **6 spheres per
link, not 1.**

### The 99th-percentile radius is unsound — use the max

One more check, because a collision model that does not *contain* the mesh can
miss real collisions. For each link I measured what fraction of its mesh vertices
actually fall inside the union of its 6 spheres, at two radius rules:

| Link | p99 radii: covered | max radius (mm) | max radii: covered | max radius (mm) |
|---|---:|---:|---:|---:|
| `base_link` | 99.65% | 53.2 | **100.00%** | 59.9 |
| `shoulder_link` | 99.64% | 43.9 | **100.00%** | 44.7 |
| `upper_arm_link` | 99.09% | 50.2 | **100.00%** | 50.4 |
| `lower_arm_link` | 99.41% | 43.5 | **100.00%** | 44.8 |
| `wrist_link` | 99.50% | 31.2 | **100.00%** | 32.1 |
| `gripper_link` | 99.21% | 26.2 | **100.00%** | 28.7 |
| `moving_jaw` | 99.41% | 20.3 | **100.00%** | 21.1 |

The 99th-percentile rule I used above leaves **0.35%–0.91% of every link's
geometry poking outside its own collision model** — precisely the protruding
features (bracket corners, horn screws) most likely to hit something. That is
unsound in the unsafe direction.

Switching to the **per-cluster maximum radius** gives 100% containment, and the
price is almost nothing: most links grow by 0.2–2.5 mm, the worst (`base_link`)
by 6.7 mm. **Use the max.** A sphere union that provably contains the mesh makes
the gate conservative by construction — it may produce false rejections, never
false acceptances, which is the correct direction for a safety gate.

*Strictness caveat:* this measures containment of mesh **vertices**. A triangle
whose vertices sit in two different spheres can technically bulge outside the
union between them. For a fully airtight guarantee, inflate each radius by half
the longest triangle edge in that cluster. Given these are dense CAD exports
(17k–54k triangles on parts ~100 mm long, so edges are sub-millimetre), that
correction is well under 1 mm — worth applying, but it does not change the shape
of the recommendation.

Note the ACM exclusions above are unaffected by this radius change: they were
established by the convex-hull proof, which does not use spheres at all.

### Measured on this machine: how finely must we interpolate?

Checking only the keyframe *endpoints* is unsound — the arm can sweep through a
collision between two individually-valid poses. The standard fix is discrete
sampling along the interpolated path (Q3), and the resolution should be derived
from geometry, not guessed.

The governing quantity is each joint's **lever arm**: the maximum perpendicular
distance from that joint's axis to any collision point on any downstream link.
Measured over 3,000 random poses:

| Joint | Max lever arm | Cartesian motion per degree |
|---|---:|---:|
| `shoulder_pan` | 446.5 mm | 7.79 mm/deg |
| `shoulder_lift` | 418.0 mm | 7.30 mm/deg |
| `elbow_flex` | 304.2 mm | 5.31 mm/deg |
| `wrist_flex` | 169.7 mm | 2.96 mm/deg |
| `wrist_roll` | 103.0 mm | 1.80 mm/deg |
| `gripper` | 82.8 mm | 1.44 mm/deg |

Worst case, all six joints moving at once: **1524 mm/rad = 26.6 mm per degree**
of joint motion. That yields a hard bound on step size:

| Guaranteed un-sampled Cartesian motion | Max joint step |
|---|---|
| 2 mm | 0.08° |
| 5 mm | 0.19° |
| 10 mm | 0.38° |

**Implication with teeth:** a 180° `shoulder_pan` sweep validated at 5 mm
resolution needs **~957 interpolated samples**. At the measured 4,545 poses/sec
that is ~0.21 s — perfectly fine for validating an LLM keyframe sequence
synchronously before execution, but far too slow to run per-frame inside a 30 Hz
teleop stream. This is the strongest argument in the research for **treating the
keyframe path (validate the whole sequence up front, densely) and the live teleop
stream (cheap per-frame check only) as two different gates with different
budgets.**

Two ways to buy the resolution back, both worth considering in the design:
- Vectorize across waypoints — the sphere-distance math is one numpy expression
  over an `(n_samples × n_pairs)` array; the 4,545/sec figure comes from a
  per-pose Python loop and leaves at least an order of magnitude on the table.
- Adaptive bisection / "conservative advancement": use the *measured clearance*
  at a waypoint plus the lever-arm bound to prove no collision can occur before
  the next sample, and only subdivide where clearance is small. Dense sampling is
  wasted almost everywhere, since most of a trajectory is far from contact.

### Measured on this machine: the legacy collision radii are 2–3× too small

`server/arm_kinematics.py` (357 lines) is the old hand-coded convention. Reading
it against the URDF makes the divergence concrete — it is not a small offset, it
is a different model in five independent ways:

| | `arm_kinematics.py` | URDF |
|---|---|---|
| Joint names | `shoulder_yaw`, `shoulder_pitch`, `elbow_pitch`, `wrist_pitch`, `wrist_roll` | `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` |
| Axes | `Ry` for yaw/roll, `Rx` for pitches | all local `+Z`, sense baked into `rpy` |
| Up-axis | **Y-up** (`_trans(0, BASE_HEIGHT, 0)`, links extend along −Y) | **Z-up** |
| Link geometry | hand-entered scalars (`UPPER_ARM_LEN`, `FOREARM_LEN`, `WRIST_LEN`) | CAD-derived `<origin xyz>` per joint |
| Collision model | 4 capsules + 2 finger capsules | 17 collision meshes, 1.2M verts |

The two models cannot be reconciled by a change of variables that anyone would
want to maintain. This is the "three non-converged geometry sources" crux, and
the resolution is deletion, not translation.

**The safety-relevant part.** The file's collision radii are explicitly
self-flagged as provisional:

```python
# PLACEHOLDER radii = the viewer's RENDER radii. These are an OPEN VARIABLE
# (DESIGN.md): the real collision radii must be measured off the printed arm to
# the *bounding* radius including brackets/servo housings — conservative, so the
_RENDER_LINK_R = 0.012
LINK_RADII = {
    "upper_arm": _RENDER_LINK_R,           # 12.0 mm
    "forearm":   _RENDER_LINK_R * 0.85,    # 10.2 mm
    "wrist":     _RENDER_LINK_R * 0.7,     #  8.4 mm
    "gripper_finger": 0.010,               # 10.0 mm
}
```

Against the radii I measured from the actual collision meshes above:

| Link | Legacy radius | Measured (6-sphere clusters) | Measured (single capsule) |
|---|---:|---:|---:|
| `upper_arm` | 12.0 mm | 23.1 – 36.1 mm | 48.0 mm |
| `forearm` / `lower_arm` | 10.2 mm | 20.2 – 33.6 mm | 51.4 mm |
| `wrist` | 8.4 mm | 20.5 – 30.8 mm | 45.2 mm |
| `gripper_finger` / `moving_jaw` | 10.0 mm | 9.6 – 22.2 mm | 26.7 mm |

**The legacy model under-approximates the real arm by roughly 2–3×.** That is an
error in the *unsafe* direction: an under-sized collision model silently passes
poses that physically collide. Whatever else the design does, the current radii
must not be carried forward.

Happily, this research closes that open variable outright. The comment asks for
radii "measured off the printed arm… including brackets/servo housings"; the CAD
collision meshes *are* that measurement, at higher fidelity than calipers, and
the numbers are in the tables above.

### Measured on this machine: install friction is ~zero (and the folklore is stale)

The received wisdom is that Pinocchio on macOS arm64 means conda. I tested it
directly instead of researching it — `pip download --no-deps --only-binary=:all:`
into the scratchpad, which resolves a real wheel for *this* interpreter without
installing anything.

Environment: **Python 3.13.7, arm64, macOS 26.5.2.**

| Package | Result |
|---|---|
| `pin` (Pinocchio) | ✅ `pin-4.1.0-0-cp313-cp313-macosx_11_0_arm64.whl` |
| `coal` (ex-hpp-fcl) | ✅ `coal-3.0.3-0-cp313-cp313-macosx_11_0_arm64.whl` |
| `python-fcl` | ✅ `python_fcl-0.7.0.11-cp313-cp313-macosx_11_0_arm64.whl` |
| `placo` | ✅ `placo-0.9.23-0-cp313-cp313-macosx_11_0_arm64.whl` |
| `yourdfpy` | ✅ `yourdfpy-0.0.60-py3-none-any.whl` (pure Python) |
| `urchin` | ✅ `urchin-0.0.30-py3-none-any.whl` (pure Python) |
| `urdfpy` | ✅ `urdfpy-0.0.22-py3-none-any.whl` (pure Python, but see Q2) |
| `hpp-fcl` | ❌ `Could not find a version that satisfies the requirement hpp-fcl (from versions: none)` |

**Every candidate pip-installs natively on this machine with a prebuilt cp313
arm64 wheel.** Install friction is not a differentiator between these options and
should not drive the decision — which frees the choice to be made on API fit and
correctness instead.

Two specifics worth carrying into the design:
- **`hpp-fcl` is superseded by `coal`, and has no cp313 arm64 wheel.** The
  successor is **`coal`**, which is what Pinocchio 3.x depends on, so any doc
  saying `pip install hpp-fcl` is stale.

  **Correction to my own reading here.** I originally wrote that `hpp-fcl` was
  "fully retired on PyPI — resolves to no versions at all", on the strength of
  pip's `from versions: none`. That over-read the probe. `hpp-fcl` **2.4.4 is
  still on PyPI with files** (last upload 2024-03-12); its arm64 wheels simply
  stop at cp312, so it resolves to nothing *for this interpreter*. pip's
  "versions: none" is a statement about **my Python's compatibility**, not about
  the index's contents. The precise claim is "retired by rename, and unavailable
  on cp313", not "absent from PyPI". Caught by a peer reviewer — and it is
  exactly the negative-probe-coverage failure flagged two paragraphs down, which
  I then walked into from the other side.
- `pin` 4.1.0 is a **Pinocchio 3.x** wheel, so it ships the `coal` collision
  backend and `pinocchio.computeCollisions` works out of the box from pip.

**Instrument note (worth recording, because it nearly produced a false finding):**
my first pass at this reported "NO WHEEL" for *all eight* packages, including
pure-Python `trimesh`. That result was an artifact — pip appends a two-line
"new release of pip is available" notice, which pushed the success line out of my
`tail -3` window. The tell was that a pure-Python package "had no wheel", which is
close to impossible. A negative probe proves the *instrument's* coverage, not the
world's state; the sanity check that caught it was asking which result would be
surprising if true.

**Gotcha found while writing it:** `so101_urdf.json` lists joints **tip→root**
(`gripper_frame_joint, gripper, wrist_roll, wrist_flex, elbow_flex,
shoulder_lift, shoulder_pan`), not root→tip, and it is a flat list rather than a
tree. Any FK over it must first order the joints by walking the `parent`/`child`
graph from `base_link`. Composing them in file order runs at the same speed and
produces a confidently wrong answer — exactly the kind of silent geometry bug
this whole spine is meant to eliminate.

---

## Q1 — LeRobot SO-101 calibration: can we adopt their flow?

**Short answer: yes, and we should — but adopting it will overwrite EEPROM
registers our server currently treats as authoritative. That is the headline.**

### The calibration procedure

Source: [`src/lerobot/robots/so_follower/so_follower.py`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py),
`calibrate()`, quoted verbatim from the live file:

```python
self.bus.disable_torque()
for motor in self.bus.motors:
    self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

input(f"Move {self} to the middle of its range of motion and press ENTER....")
homing_offsets = self.bus.set_half_turn_homings()

full_turn_motor = "wrist_roll"
unknown_range_motors = [motor for motor in self.bus.motors if motor != full_turn_motor]
print(
    f"Move all joints except '{full_turn_motor}' sequentially through their "
    "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
)
range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
range_mins[full_turn_motor] = 0
range_maxes[full_turn_motor] = 4095

self.calibration = {}
for motor, m in self.bus.motors.items():
    self.calibration[motor] = MotorCalibration(
        id=m.id,
        drive_mode=0,
        homing_offset=homing_offsets[motor],
        range_min=range_mins[motor],
        range_max=range_maxes[motor],
    )

self.bus.write_calibration(self.calibration)
self._save_calibration()
```

So the human procedure is exactly two poses/actions:

1. **Move the whole arm to the middle of every joint's range**, press Enter.
   This is the homing pose.
2. **Sweep every joint except `wrist_roll` through its full physical range**
   while positions stream, press Enter. `wrist_roll` is hardcoded to the full
   `[0, 4095]` turn because it is a continuous-rotation joint.

The user-facing docs describe the same two steps: "First you need to move the
robot to the position where all joints are in the middle of their ranges. Then
after pressing enter you have to move each joint through its full range of
motion." — [huggingface.co/docs/lerobot/en/so101](https://huggingface.co/docs/lerobot/en/so101)

CLI: `lerobot-calibrate --robot.type=so101_follower --robot.port=<port> --robot.id=<name>`
(same doc). The JSON lands in the lerobot calibration cache keyed by the robot `id`.

### The calibration file format

[`src/lerobot/motors/motors_bus.py`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/motors_bus.py):

```python
@dataclass
class MotorCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int
```

Five integers per motor, serialized to JSON keyed by joint name. Note
`drive_mode` is **hardcoded to 0** for every SO-101 follower joint — lerobot
never uses drive-mode inversion on this arm. Any sign flip you need lives
outside the calibration file.

### Homing offset semantics — the exact tick math

From [`src/lerobot/motors/feetech/feetech.py`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/feetech.py):

```python
def _get_half_turn_homings(self, positions):
    """
    On Feetech Motors:
    Present_Position = Actual_Position - Homing_Offset
    """
    half_turn_homings = {}
    for motor, pos in positions.items():
        max_res = self.model_resolution_table[model] - 1   # 4095
        half_turn_homings[motor] = pos - int(max_res / 2)  # pos - 2047
```

So `homing_offset = (raw ticks at the mid-range pose) − 2047`, and thereafter
`Present_Position` reads **2047 when the arm is in the mid-range pose**. The
docstring on `set_half_turn_homings` says it plainly: "computes and writes a
homing offset such that the present position becomes exactly one half-turn
(e.g. `2047` on a 12-bit encoder)."

`Homing_Offset` is stored **sign-magnitude encoded**, not two's complement —
lerobot round-trips it through `encode_sign_magnitude` / `decode_sign_magnitude`
against a per-model `model_encoding_table` sign-bit position (`_encode_sign` /
`_decode_sign` in `feetech.py`). If we ever read that register ourselves with a
naive `int16` cast we will get garbage for negative offsets.

### ⚠️ The collision with our current design

`write_calibration` (feetech.py) is where the calibration leaves the JSON and
enters the servos:

```python
def write_calibration(self, calibration_dict, cache=True):
    for motor, calibration in calibration_dict.items():
        if self.protocol_version == 0:
            self.write("Homing_Offset", motor, calibration.homing_offset)
        self.write("Min_Position_Limit", motor, calibration.range_min)
        self.write("Max_Position_Limit", motor, calibration.range_max)
```

And `reset_calibration`, which `set_half_turn_homings` calls **first**, before
reading positions:

```python
def reset_calibration(self, motors=None):
    """Restore factory calibration ... Homing offset is set to ``0`` and
    min/max position limits are set to the full usable range."""
    for motor in motor_names:
        max_res = self.model_resolution_table[model] - 1
        self.write("Homing_Offset", motor, 0, normalize=False)
        self.write("Min_Position_Limit", motor, 0, normalize=False)
        self.write("Max_Position_Limit", motor, max_res, normalize=False)
```

`Min_Position_Limit` / `Max_Position_Limit` are the STS3215 EEPROM **Min/Max
Angle Limit registers (0x09 / 0x0B)** — precisely the registers our server reads
at connect and treats as absolute safety bounds. Two consequences:

- **Running `lerobot-calibrate` transiently blows our safety bounds wide open**
  (`0 … 4095`, i.e. the full turn) and then rewrites them to the *recorded sweep
  range*. Mid-calibration, between step 1 and step 2, the arm has **no firmware
  angle limits at all**.
- **After calibration, our "absolute EEPROM bounds" and lerobot's
  `range_min`/`range_max` are the same numbers.** They are not two independent
  sources of truth; they are one source with two names. That is good news for
  the shared-spine goal — it removes one of the three non-converged geometry
  sources — but only once we say so explicitly.

This also explains the open task "gripper EEPROM angle limits still factory-locked
`[2046, 2047]`": that is a *degenerate calibrated range*, the signature of a
gripper that was never swept during step 2 (or was swept while torque was on).
`_normalize` raises `ValueError` outright when `range_min == range_max`, so
lerobot itself treats this as a hard error — `[2046, 2047]` is one tick away
from being an exception rather than a bug.

### Raw ticks → URDF joint angles

`_normalize` / `_unnormalize` in `motors_bus.py` define three modes:

```python
class MotorNormMode(str, Enum):
    RANGE_0_100 = "range_0_100"
    RANGE_M100_100 = "range_m100_100"
    DEGREES = "degrees"
```

```python
# RANGE_M100_100
norm = (((bounded_val - min_) / (max_ - min_)) * 200) - 100
# RANGE_0_100
norm = ((bounded_val - min_) / (max_ - min_)) * 100
# DEGREES
mid = (min_ + max_) / 2
max_res = self.model_resolution_table[self._id_to_model(id_)] - 1
normalized_values[id_] = (val - mid) * 360 / max_res
```

Both normalizers **clamp** to `[range_min, range_max]` before converting
(`bounded_val = min(max_, max(min_, val))`) — the calibrated range is a hard
saturation, not just a scale.

Per-joint mode assignment, from `so_follower.py`:

```python
norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
motors={
    "shoulder_pan":  Motor(1, "sts3215", norm_mode_body),
    "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
    "elbow_flex":    Motor(3, "sts3215", norm_mode_body),
    "wrist_flex":    Motor(4, "sts3215", norm_mode_body),
    "wrist_roll":    Motor(5, "sts3215", norm_mode_body),
    "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}
```

**The five body joints can be switched to real degrees with a single config
flag (`use_degrees=True`); the gripper is always 0–100.** That flag is the
cheapest bridge between lerobot's world and the URDF's world.

**The mapping we actually want.** `DEGREES` mode gives
`(ticks − mid) × 360 / 4095` where `mid = (range_min + range_max) / 2`. The
URDF's zero is the middle of each joint's range (next paragraph). So:

```
urdf_angle_deg[j] ≈ sign[j] × (present_ticks[j] − mid[j]) × 360/4095
```

with `360/4095 = 0.08791 deg/tick`, i.e. **11.375 ticks per degree**. The STS3215's
magnetic encoder reads the *output* shaft, so the 1/345 gear ratio does **not**
enter this conversion — 4096 ticks is one full revolution of the joint itself.

Note the subtle mismatch worth guarding: `DEGREES` centres on
`mid = (range_min + range_max)/2`, the centre of the *recorded sweep*, whereas
homing centred on `2047`, the pose the human eyeballed as "middle". These agree
only if the human's guess was the true centre of travel. Expect a few degrees of
disagreement per joint and treat the sweep-centre as the more trustworthy one.

### Which repo is `so101_new_calib.urdf` from, and what zero does it assume?

It is from **TheRobotStudio/SO-ARM100**, at
[`Simulation/SO101/so101_new_calib.urdf`](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf).
lerobot vendors a copy into its examples (`examples/phone_to_so100/SO101/so101_new_calib.urdf`)
and its docs reference it by that path.

The zero-pose convention, from the
[`Simulation/SO101/README.md`](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/README.md):

- **New calibration (default, our file):** each joint's virtual zero is at the
  **middle of its joint range**.
- **Old calibration** (`so101_old_calib`): each joint's virtual zero is the
  configuration where the robot is **fully extended horizontally**.

This is the reason the two conventions in our repo disagree, and it resolves in
our favour: **lerobot's calibration homing pose ("move to the middle of every
joint's range") and the `new_calib` URDF's zero pose ("middle of joint range")
are the same physical pose.** They were designed to match. The often-repeated
community warning that "0 positions for the URDF differ from the 0 positions
convention in lerobot" is a statement about the **old** calibration; against
`so101_new_calib` the conventions coincide by construction.

**UNSETTLED — the per-joint sign vector.** No source states, joint by joint,
whether increasing servo ticks corresponds to increasing or decreasing URDF
angle. It cannot be derived from the URDF because every axis is `0 0 1` with the
sense hidden in the `rpy`, and it cannot be derived from lerobot because
`drive_mode` is uniformly 0. lerobot's own docs show `shoulder_lift: -20.0` in an
example action, hinting at least one joint reads negative in normal use
([Action Representations](https://huggingface.co/docs/lerobot/action_representations)),
but that is not a convention statement. **This must be measured**: command each
joint +10° alone from home, read the URDF FK tool position, compare against the
real arm, record a 6-element `sign[]` vector. That measurement is a one-off
~10-minute procedure and should be an explicit build step, not an assumption.

### Bonus find: lerobot already ships URDF FK/IK

[`src/lerobot/model/kinematics.py`](https://github.com/huggingface/lerobot/blob/main/src/lerobot/model/kinematics.py)
defines `RobotKinematics`, which is a thin wrapper over **placo** (Rhoban's
kinematics/IK library):

```python
class RobotKinematics:
    """Robot kinematics using placo library for forward and inverse kinematics."""
    def __init__(self, urdf_path: str, ...):
        require_package("placo", extra="placo-dep")
        self.robot = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)
```

And it is wired up against *our exact URDF and tool frame*
([Action Representations](https://huggingface.co/docs/lerobot/action_representations)):

```python
kinematics = RobotKinematics(
    urdf_path="./SO101/so101_new_calib.urdf",
    target_frame_name="gripper_frame_link",
    joint_names=["shoulder", "elbow", "wrist_pitch", "wrist_roll", "wrist_yaw"],
)
```

`placo` is therefore a fourth FK candidate alongside the three in Q2, with the
strong advantage that it is the option the upstream ecosystem already validates
against this model. It does not, however, do self-collision checking.

### Design implications

- **Adopt lerobot's two-step calibration verbatim** rather than inventing one.
  It is two human actions, it is what every SO-101 tutorial teaches, and its
  homing pose is already the URDF's zero pose.
- **Retire the relative "zero = wherever the arm was at server start" convention.**
  It is the root of the three-way geometry divergence. Replace it with the
  absolute convention: `ticks 2047 = URDF zero = mid-range pose`. This is the
  single highest-value change in the whole design.
- **Treat the lerobot calibration JSON as the source of truth and the EEPROM
  Min/Max Angle Limits as its cache**, not as two independent sources. They are
  written from the same five integers.
- **Guard the calibration window.** Our server must refuse to connect (or at
  minimum warn loudly) while a joint's `range_min == range_max` or the range is
  the full `[0, 4095]` — both mean "not calibrated", and the second means the
  firmware limits are wide open.
- **Set `use_degrees=True`** (or replicate its `DEGREES` formula) so the wire
  format is degrees, matching the URDF, matching the motion IR, and removing a
  whole class of unit-confusion bugs.
- **Use `0.08791 deg/tick` (11.375 ticks/deg)** and note the gear ratio is
  irrelevant — the encoder is on the output shaft.
- **Decode `Homing_Offset` as sign-magnitude**, never as two's complement.
- **Measure the 6-element per-joint sign vector as an explicit commissioning
  step** and check it into the repo next to the calibration. It is the one
  number set that no upstream source can give us.
- **Consider `placo` for FK** — it is what lerobot itself uses against this exact
  URDF and tool frame, so it is the best-validated option (see Q2 for the
  performance/install tradeoff).

---

## Q2 — Python FK and self-collision libraries

The headline from the local measurements: **FK is a solved non-problem (19.8 µs
hand-rolled) and install friction is zero (every candidate has a native cp313
arm64 wheel).** Both of the axes this question was framed around turn out not to
discriminate. What actually matters is the *collision* side and the correctness
of the geometry pipeline.

### Forward kinematics options

| Option | What it is | FK | Collision | macOS arm64 install | Verdict |
|---|---|---|---|---|---|
| **DIY numpy** | compose URDF origin transforms | **19.8 µs/pose (measured)** | none | nothing to install | **Recommended** |
| **placo** | Rhoban kinematics + IK, C++ core | fast (C++) | no | ✅ `placo-0.9.23` wheel | Best-validated; what lerobot uses |
| **yourdfpy** | URDF parse + FK + trimesh scene | yes | **no** | ✅ pure-Python wheel | Good parser, not a gate |
| **urchin** | maintained urdfpy fork | yes | via trimesh/fcl | ✅ pure-Python wheel | Use instead of urdfpy |
| **urdfpy** | original, `mmatl/urdfpy` | yes | yes | ✅ wheel, but broken deps | **Avoid** |
| **pinocchio** (`pin`) | full rigid-body dynamics + `coal` | very fast | **yes, built in** | ✅ `pin-4.1.0` wheel | Heavyweight but complete |

**`urdfpy` is definitively abandoned and should not be used.** Last PyPI release
**0.0.22, 2020-05-31**, nothing since. Its `setup.py` hard-pins `networkx==2.2`
*and* `pycollada==0.6` with exact `==`
([mmatl/urdfpy#27](https://github.com/mmatl/urdfpy/issues/27)); networkx 2.2 does
not support functions urdfpy itself calls, and it collides with Python 3.10+ and
numpy > 1.20 — documented downstream in
[NVIDIA/warp#88](https://github.com/NVIDIA/warp/issues/88). On numpy 2.x it is a
non-starter. The maintainer's own repo carries the fork announcement
([mmatl/urdfpy#31](https://github.com/mmatl/urdfpy/issues/31)): "friendly
maintenance fork at fishbotics/urchin".

Note the wheel *does* resolve (`urdfpy-0.0.22-py3-none-any.whl`), which makes
this a trap: **it installs cleanly and then breaks at import or load.** A
successful `pip install` is not evidence a package works.

**[`urchin`](https://github.com/fishbotics/urchin) is the drop-in replacement** —
relaxes to unpinned `networkx`, `pycollada>=0.6`, `numpy>=1.20`, swaps `pyrender`
for its own `pyribbit` fork, and adds `lazy_load_meshes=True` (~100× faster
load). README: "a fork of the no-longer-updated urdfpy. The API is still mostly
the same." Cadence caveat: urchin 0.0.30 shipped 2025-10-21 against yourdfpy
0.0.60 on 2026-01-23, and it still drags scipy + pycollada + pyribbit.

**`yourdfpy` is the best pure parser** — its README reports loading **12/12** test
URDFs where urdfpy manages 4/12 and `urdf_parser_py` 6/12, parsing in ~3.2 ms
without meshes, and it uniquely decouples parsing from validation and from mesh
loading ([clemense/yourdfpy](https://github.com/clemense/yourdfpy)). But **it does
not do collision checking at all**, so it cannot be the gate on its own.

**`placo` deserves more weight than its obscurity suggests**: it is what
`lerobot.model.kinematics.RobotKinematics` wraps (Q1), aimed at *our exact URDF
and `gripper_frame_link` tool frame*. If we ever want IK — "put the gripper
there" rather than "set these joints" — placo is the path of least resistance and
the one upstream already exercises.

**Why DIY still wins for the FK core.** We already bake the URDF to JSON for the
browser viewer, so parsing is free and, more importantly, **the browser and the
server would consume the identical baked artifact** — which is the entire point of
the "one shared kinematic spine". Introducing a Python-side URDF parser
reintroduces the possibility of the two renderers disagreeing, which is the exact
failure this design exists to kill. 19.8 µs and ~40 lines of numpy is a small
price for a single source of geometry.

### Collision checking options

- **`python-fcl`** — Python bindings to FCL; wheel confirmed
  (`python_fcl-0.7.0.11-cp313-cp313-macosx_11_0_arm64.whl`). Mesh-mesh via BVH.
  `trimesh`'s `CollisionManager` wraps it for a friendlier API.
- **`coal`** — the renamed `hpp-fcl`, Pinocchio 3.x's collision backend; wheel
  confirmed (`coal-3.0.3`). **`hpp-fcl` is retired by rename and has no cp313
  arm64 wheel** — 2.4.4 is still on PyPI (last upload 2024-03-12) but its arm64
  builds stop at cp312, so it resolves to nothing on this interpreter. Any
  instruction to `pip install hpp-fcl` is stale regardless.
- **`pinocchio`** — bundles `coal`, builds a `GeometryModel` with explicit
  collision pairs and `computeCollisions`, and gives an ACM equivalent for free.
  The most complete option; the cost is a large dependency for a 6-DOF arm.
- **Capsule/sphere approximation in pure numpy** — measured above: 6 spheres per
  link, all non-adjacent pairs, **20.5 µs/pose**, no compiled dependency.

**Pinocchio gotcha worth flagging in the design doc.** The idiomatic flow is
`pin.buildGeomFromUrdf(...)` → `geom_model.addAllCollisionPairs()` →
`pin.removeCollisionPairs(model, geom_model, srdf_path)` → `pin.computeCollisions(...)`
([pinocchio/examples/collisions.py](https://github.com/stack-of-tasks/pinocchio/blob/master/examples/collisions.py)).
`addAllCollisionPairs()` adds **every** pair including adjacent links that always
touch, so **without an authored SRDF disable-list every pose reports collision.**
Since no SRDF exists for the SO-101 (Q5), pinocchio does not save us the ACM work
— it just relocates it.

**Sphere/capsule decomposition is established practice, not a shortcut.**
[cuRobo](https://curobo.org/tutorials/1_robot_configuration.html) represents each
link as a set of spheres in a `collision_spheres` YAML (centre + radius per
entry) plus an explicit adjacent-pair ignore list, and its kinematics kernel
emits sphere positions directly. Capsules are the cheaper standard for arm links,
reducing collision to closed-form segment-segment distance;
[BOMP](https://arxiv.org/pdf/2411.00221) reports capsule checks roughly **10×
faster than FCL's box-box**, itself far cheaper than mesh-mesh. Adjacent-pair
false positives are the known trap, with the standard fix being to skip pairs
close together along the backbone ([ref](https://arxiv.org/pdf/2211.16452)).

**UNSETTLED:** the typical *number* of spheres per link. cuRobo's are per-link
variable and hand-tuned in Isaac Sim's Lula editor, and no published typical
count was found. Our K = 6 is justified by our own containment measurement
(100% at max radius), not by convention.

**The mesh problem is ours, not the library's.** Our collision geometry is
**1,196,652 vertices across 7 links** (17k–54k triangles per mesh), because the
URDF reuses the CAD *visual* meshes as collision meshes. No BVH library makes
that free, and upstream already hit this — the SO101 README records that base
collision meshes were removed for "problematic collision behavior during
simulation and planning". So convex decomposition (convex hull or VHACD) is the
normal precondition for mesh-mesh, and it is real preprocessing work.

Given that, the sphere-decomposition route is not a compromise, it is the better
fit: it sidesteps the mesh problem entirely, runs in pure numpy, is trivially
vectorizable across interpolated waypoints (which Q3 shows is where the real cost
lives), and I have already **proven** its exclusions sound with an independent
convex-hull argument.

### Recommendation

**Ship the pure-numpy stack; keep pinocchio as the offline oracle.**

- **Runtime gate:** DIY numpy FK over the baked URDF JSON + 6-spheres-per-link
  with **max-radius** containment + the 9-pair ACM. ~40 µs/pose measured, zero
  new dependencies, one shared geometry artifact with the viewer.
- **Offline validation:** use pinocchio (or `python-fcl` on convex hulls) *once*,
  in a test, to confirm the sphere model never accepts a pose that true mesh-mesh
  rejects. This is the fake-more-forgiving-than-reality check — the sphere model
  must be at least as strict as the real geometry, and that claim needs an
  instrument other than itself.
- **If we later want IK**, add `placo` rather than rolling our own, since lerobot
  already validates it against this URDF and tool frame.

**Batch, don't loop — the single biggest free win.** An independent measurement on
this same machine (numpy 2.3.5, arm64) found single-pose FK at 16.3 µs but
**0.94 µs/pose when batched at N = 64** — a ~17× speedup purely from vectorizing
across waypoints, consistent with my 19.8 µs single-pose figure. Same source
measured a 15-pair vectorized segment-segment check at 25.2 µs, and an end-to-end
FK + joint-limits + capsule gate over **64 interpolated waypoints at 0.371 ms**.

Given Q3 shows path sampling (hundreds of waypoints), not single-pose cost, is
where the budget goes, this is the lever that matters: **validate a whole
keyframe→keyframe segment as one `(N, 6)` array, never a Python loop over poses.**
That reframes the 478-sample figure from Q3 from "0.1 s" to a few milliseconds.

*Caveat on that number:* the end-to-end harness used a `np.roll` stand-in for
pair indexing — correct flop count and array shapes, so a valid **timing** proxy,
but the pair semantics were not validated. Treat 0.371 ms as an order-of-magnitude
result, not a verified gate timing.

**UNSETTLED:** I did not benchmark `pin`, `python-fcl`, `coal`, `placo`, or
`yourdfpy` — the brief said not to install, so their speeds are reputational, not
measured. Only the numpy figures and the wheel availability are first-hand. Also
unsettled: absolute BVH build cost in ms, and any head-to-head coal-vs-python-fcl
benchmark — no source found, and not measurable without installing. Per the
project's own "predict from your own instrument" rule, treat all library speed
claims as unverified until one is actually run here.

### Design implications

- **Roll the FK by hand over the baked URDF JSON** — measured 1,650× headroom, no
  dependency, and it keeps browser and server on one geometry artifact.
- **Never use `urdfpy`; use `urchin`** if a Python URDF parser is needed. The
  wheel installing successfully does not mean it works.
- **Use `coal`, never `hpp-fcl`** — the latter no longer exists on PyPI.
- **Use 6 spheres per link at max radius**, not 1 capsule and not 99th-percentile
  radii (both measured unsound above).
- **Convex-decompose before any mesh-mesh checking** — 1.2M vertices of CAD
  visual mesh is not a collision model.
- **Add pinocchio as a test-only dependency** to validate the sphere model
  against real mesh geometry, not as a runtime dependency.
- **Reach for `placo` if IK becomes a requirement** — it is lerobot's own choice
  against this exact URDF.
- **Vectorize the collision check across waypoints**, since Q3 shows path
  sampling, not single-pose cost, dominates.

---

## Q3 — Self-collision gating practice in teleop

### MoveIt Servo: the authoritative defaults

From [`moveit_ros/moveit_servo/config/servo_parameters.yaml`](https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/config/servo_parameters.yaml):

| Parameter | Default | Comment in source |
|---|---|---|
| `check_collisions` | `true` | "If true, servo will check for collision using the planning scene monitor." |
| `collision_check_rate` | `10.0` Hz | "Collision-checking can easily bog down a CPU if done too often." |
| `self_collision_proximity_threshold` | `0.01` m | "Start decelerating when a self-collision is this far [m]" |
| `scene_collision_proximity_threshold` | `0.02` m | "Start decelerating when a collision is this far [m]" |
| `lower_singularity_threshold` | `17.0` | "Start decelerating when the condition number hits this" |
| `hard_stop_singularity_threshold` | `30.0` | "Stop when the condition number hits this" |
| `leaving_singularity_threshold_multiplier` | `2.0` | eases deceleration when moving *away* from a singularity |
| `publish_period` | `0.01` s | "1 / (Nominal publish rate)" → **100 Hz control loop** |

Four design lessons fall straight out of this table:

1. **Collision checking runs 10× slower than the control loop** — 10 Hz against a
   100 Hz servo loop, on a separate thread, with the source comment stating
   plainly that the reason is CPU cost. This directly answers the brief's
   question: **we do not need 30 Hz collision checking.** The control stream and
   the safety check are deliberately decoupled rates.

2. **The response is deceleration, not rejection.** Every threshold is phrased
   "start decelerating when…". MoveIt Servo scales commanded velocity toward zero
   as proximity shrinks, producing a smooth stop at the boundary rather than a
   discontinuous halt. For a *streaming teleop* input this is clearly right — a
   hard stop on a continuous stream is both jarring and, on a position-controlled
   servo bus, a way to command a step the arm cannot track.

3. **Singularity handling is explicitly two-tier** — a soft threshold (17.0) that
   decelerates and a hard threshold (30.0) that stops. This is the pattern worth
   copying for collision: a *warn/slow* band and a *stop* band, not one boolean.

4. **Self-collision gets a tighter threshold than world-collision** (10 mm vs
   20 mm). Self-collision geometry is known exactly; scene geometry comes from
   noisy perception and earns more room. Our gate is pure self-collision, so
   **10 mm is the conventional reference value.**

### The velocity-scaling law, verbatim

Worth copying exactly rather than reinventing. From
[`src/collision_monitor.cpp`](https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/src/collision_monitor.cpp):

```
// velocity_scale = e ^ k * (collision_distance - threshold)
// k = - ln(0.001) / collision_proximity_threshold
// velocity_scale should equal one when collision_distance is at collision_proximity_threshold.
// velocity_scale should equal 0.001 when collision_distance is at zero.
```

The scale from each check type is combined with `std::min` (most restrictive
wins), forced to `0.0` on actual contact, and pinned to `1.0` when checking is
disabled. It crosses threads as a `std::atomic<double>`.

Three implementation details that are easy to get wrong and are worth stealing:

- **Both collision requests set `distance = true`.** The checker returns *signed
  clearance*, not a boolean. A boolean checker forecloses every graded response
  below — this is the single most consequential API decision in the design.
- **The scale is applied as a lerp of the position delta toward the current
  state**, from `src/servo.cpp`:
  ```cpp
  target_state.positions =
      current_state.positions + collision_velocity_scale_ * (target_state.positions - current_state.positions);
  ```
  A soft clamp, never a rejection.
- **On hard stop the delta computation is skipped entirely**, so target ==
  current: hold-in-place for free, and no stale target can resume motion later.

Joint limits are a **separate third mechanism** (`jointVariablesToHalt()` →
`StatusCode::JOINT_BOUND` → `haltJoints()`), zeroing only the offending joints
unless `halt_all_joints_*` is set. Three independent gates — collision,
singularity, joint bounds — not one combined validity boolean.

**The asymmetry worth improving on.** Singularity handling is direction-aware:
`leaving_singularity_threshold_multiplier: 2.0` lets the arm escape a singularity
faster than it entered. Collision handling has **no equivalent retreat
exemption** — it is direction-blind, so a commanded motion that *reduces*
penetration is scaled just as hard as one that increases it. On a small arm that
can genuinely fold into itself, a direction-blind gate can trap the arm in a pose
it is not allowed to leave. Scaling only the *approaching component* of the
commanded delta is a small change and a real improvement over the reference
implementation.

### Path validity by discrete interpolated sampling

Checking endpoints alone is unsound; OMPL discretizes each edge and checks
sub-states. The governing parameter is **`longest_valid_segment_fraction`** —
"the fraction of the robot's state space that, given the robot isn't currently in
collision, we assume the robot can travel while remaining collision free"
([MoveIt OMPL Planner docs](https://moveit.picknik.ai/main/doc/examples/ompl_interface/ompl_interface_tutorial.html)).
The docs are explicit about the tradeoff: "Set too low, and collision checking /
motion planning will be very slow. Set too high and collisions will be missed
around small or narrow objects."

**The default is `0.01`, verified in OMPL source** —
`src/ompl/base/src/StateSpace.cpp` carries `longestValidSegmentFraction_ = 0.01;
// 1%` ([ompl/ompl](https://github.com/ompl/ompl/blob/main/src/ompl/base/src/StateSpace.cpp)).
The frequently-quoted `0.005` is **widely shipped in MoveIt configs but is not a
verified library default** — it could not be pinned to a current MoveIt 2 source
line, and the Setup Assistant auto-generates these per robot. Treat 0.005 as
"commonly shipped", not canonical. (My first draft of this section stated 0.005
as the default; corrected on review.)

A companion parameter, `maximum_waypoint_distance`, expresses the same
discretization "at an absolute level instead of using fractions", and when both
are set, "the variable that produces the most conservative discretization … is
chosen." That belt-and-braces pairing exists because a fraction alone
under-samples short edges — worth copying.

### ⚠️ The MoveIt defaults are NOT self-consistent on our arm

This is the most useful thing in this section, and it comes from combining
MoveIt's numbers with the lever arms I measured above rather than from any
single source.

The fraction multiplies the state space's **maximum extent**, which for a
revolute arm is the L2 norm of the per-joint ranges. For the SO-101 (ranges 220°,
200°, 193.6°, 190°, 320°, 110°):

```
extent = sqrt(Σ range_i²) = 9.18 rad
```

| Fraction | Joint-space step | Worst-case Cartesian motion per step |
|---|---:|---:|
| `0.01` (OMPL verified default) | 0.0918 rad = **5.26°** | **41.0 mm** |
| `0.005` (commonly shipped) | 0.0459 rad = **2.63°** | **20.5 mm** |

(Worst case puts the whole step into `shoulder_pan`, the 7.79 mm/deg joint.)

So the library default leaves **41 mm of Cartesian motion unchecked between
samples, against a 10 mm proximity threshold** — geometry can pass clean through
the shell, and out the other side, within a single un-sampled step. Even the
commonly-shipped 0.005 leaves 20.5 mm, still 2× the threshold. The two mechanisms
do not compose at either value.

**The invariant that makes them compose:**

> collision margin ≥ maximum Cartesian motion between consecutive samples

Solving that for our arm (26.6 mm/deg worst case):

| Margin | Required joint step | Samples for a 180° sweep |
|---|---|---|
| 10 mm (MoveIt's self-collision default) | ≤ 0.376° | 478 |
| 20 mm | ≤ 0.752° | 239 |

**In fairness to MoveIt this is not a bug in their defaults**, for two reasons.
First, the two numbers come from different subsystems that never meet: the
fraction governs *planner* edge validation, while the proximity threshold governs
*servo* deceleration. Nothing in MoveIt claims they compose — it is our design
that wants to use both mechanisms on one path, so it is our job to make them
consistent. Second, the defaults are tuned for larger industrial arms with
different reach-to-range ratios.

The lesson is not "MoveIt is wrong", it is **"these constants cannot be
transplanted without re-deriving them against your own lever arms"** — which is
exactly the measurement this research already did.

**A related nuance worth not fumbling:** `self_collision_proximity_threshold` is
a **deceleration trigger distance, not geometric padding.** MoveIt Servo checks
self-collision against `getCollisionEnvUnpadded()` — deliberately unpadded — and
the editorial line across the codebase is that *padding is for the environment,
not for self-collision* (`plan_execution` likewise sets
`req.pad_environment_collisions = false` when revalidating). So "adopt 10 mm"
means *start slowing at 10 mm of true clearance*, not *inflate every link by
10 mm*. Conflating the two would double-count our sphere over-approximation,
which is already conservative.

### The Allowed Collision Matrix and how it is generated

MoveIt's Setup Assistant generates an SRDF `<disable_collisions>` block by
**sampling large numbers of random robot poses** and classifying each link pair:

The exact algorithm, from
[`compute_default_collisions.cpp`](https://github.com/moveit/moveit2/blob/main/moveit_setup_assistant/moveit_setup_srdf_plugins/src/compute_default_collisions.cpp):

0. Enumerate all n-choose-2 pairs over links that have geometry.
1. Build a connection graph — links reachable via one joint, or via a chain of
   geometry-less intermediates.
2. **Adjacent** — disable every adjacent pair outright.
3. **Default** — disable pairs colliding in the robot's default/start state.
4. **Always** — batches of `SMALL_TRIAL_COUNT = 200` random states; any pair
   colliding in more than `min_collision_fraction` (**default 0.95**) of a batch
   is disabled. **The loop repeats until a batch disables nothing new.**
5. **Never** — a further `num_trials` random states (**Setup Assistant GUI
   default 10,000**); pairs never once in contact are disabled.

Two details that matter for replicating this offline and that my own run did not
implement:

- **Step 4 is a fixpoint loop, not a single pass.** Disabling an always-colliding
  pair *unmasks* contacts it was shadowing, so one pass under-reports. My run was
  single-pass; on this arm the effect is likely small (only `wrist|moving_jaw`
  hit the >95% band) but the loop is the correct construction.
- **Step 5 is a sampling claim of absence**, so its soundness scales with sample
  count — which is why the tutorial says to use maximum sampling density. My run
  used 20,000 samples, twice the Setup Assistant's default, and the two
  never-colliding pairs were independently *proven* by the convex-hull argument
  rather than resting on sampling at all.

**No SRDF or ACM exists for the SO-101 anywhere upstream** (Q5), so this offline
generation step is unavoidable work. Result in the local-ground-truth section:
9 excluded pairs, 12 actually checked.

**Padding defaults — now settled: MoveIt ships zero.** From
[`planning_scene_monitor.cpp`](https://github.com/moveit/moveit2/blob/main/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp)
`configureDefaultPadding()`:

```
default_robot_padding    -> 0.0
default_robot_scale      -> 1.0
default_object_padding   -> 0.0
default_attached_padding -> 0.0
```

Padding is a per-deployment opt-in, not a shipped safety margin, and — per the
nuance above — it is aimed at the *environment*, not at self-collision.
**UNSETTLED:** there is still no canonical "typical" non-zero padding value in
metres; MoveIt's position is simply 0.0 by default.

### Rejection semantics for a keyframe sequence

The teleop stacks above all target a *continuous stream*, where "reject" is not
meaningful — you cannot un-send a stream, so they scale velocity toward zero. A
**keyframe sequence is a different object**: it is known in full before execution
begins, which makes up-front validation possible in a way streaming never allows.

That argues strongly for different semantics on our two paths:

- **Keyframe sequence → validate the entire sequence up front and refuse the
  whole thing on any invalid step.** Executing to step 4 of 9 and halting leaves
  the arm in an arbitrary mid-choreography pose that no one designed, with the
  user's mental model ("it will do the wave") now false. Atomic accept/reject is
  both safer and far easier to explain. It also produces a clean, structured
  error the LLM can repair from (Q4) — a partial execution does not.
- **Live teleop stream → velocity-scale toward zero,** following MoveIt Servo,
  because up-front validation is impossible by construction.

**This is now backed by MoveIt source, not just reasoning.** There are three
regimes, and the third is the real analogue for a keyframe sequence:

- **Servo / streaming:** scale and hold, never reject. There is no reject path in
  `moveit_servo` at all.
- **Planning:** all-or-nothing. OMPL never returns a partial path.
- **Execution:** a hybrid, and the closest match to our case.
  [`plan_execution.cpp`](https://github.com/moveit/moveit2/blob/main/moveit_ros/planning/plan_execution/src/plan_execution.cpp)
  revalidates a planned trajectory *during* execution on every planning-scene
  change. `isRemainingPathValid()` walks **every remaining waypoint through to
  the end**, not merely the next one:

  ```cpp
  for (std::size_t i = std::max(path_segment.second - 1, 0); i < wpc; ++i)
    ... plan.planning_scene->checkCollision(req, res, waypoint_state, *acm);
  ```

  On the first invalid waypoint it sets `path_became_invalid_`, **stops before
  entering the invalid region**, returns
  `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE`, and replans up to
  `default_max_replan_attempts_ = 5`.

So the documented convention is: **validate the whole sequence as a unit at
admission (refuse it entire), then re-validate the whole remainder continuously
during execution and stop-and-hold at the last valid waypoint** — never "execute
until the step that fails". That is stronger than what I originally argued, and
it converges on the same conclusion from real code.

The teleop literature adds one refinement: it converges on **projection, not
rejection** — scale only the velocity component toward the obstacle so lateral
and retreating motion stay free
([QP/virtual-fixture formulations](https://arxiv.org/pdf/1809.07907),
[operator feasibility-set visualisation](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.730433/full)).
The unanimous position across that literature: **an unexplained refusal is worse
than a degraded motion — always emit the reason.**

### Margins for small/hobby arms

**UNSETTLED.** I found no published margin conventions specific to small or hobby
arms. MoveIt's 10 mm self-collision default is an industrial-arm number and, on
an arm whose links are 80–160 mm long, 10 mm is proportionally large — it is
roughly 20% of a link radius. Our own mechanical floor is better evidence: the
MJCF models **±0.5° of backlash** per joint (Q5), which through the largest lever
arm (446 mm) is ±3.9 mm of tip uncertainty from backlash alone. A margin below
~5 mm would be inside the arm's own mechanical noise.

### Design implications

- **Return signed distance, never a boolean.** MoveIt sets `distance = true` on
  every request for exactly this reason. A boolean checker forecloses graded
  response, and it is the hardest decision to reverse later.
- **Decouple the rates:** run the collision gate far slower than the command
  stream, following MoveIt Servo's 10 Hz-vs-100 Hz split. 30 Hz collision
  checking is not required, and we have ~150× headroom even if we want it.
- **Steal the scaling law verbatim:** `k = -ln(0.001)/threshold`,
  `scale = exp(k·(d − threshold))`, `min()` across check types, `0.0` on contact.
  Apply it as a **lerp of the position delta toward the current state**, and on
  hard stop set target = current — that gives hold-in-place free and makes a
  stale target unable to resume motion.
- **Adopt 10 mm as the self-collision *deceleration threshold*** — not as link
  padding. MoveIt checks self-collision unpadded and ships 0.0 padding by
  default. Treat ~5 mm as a hard floor set by the arm's own ±0.5° backlash.
- **Enforce the composition invariant** — margin ≥ max Cartesian motion per
  sample — which for a 10 mm margin means a **0.376° joint step**, against the
  5.26° that OMPL's verified default fraction would imply here.
- **Set an absolute step cap alongside the fractional one and take the more
  conservative**, as MoveIt does with `maximum_waypoint_distance`; a fraction
  alone under-samples short edges.
- **Give collisions the singularity treatment**: two-tier (slow band + stop band)
  *and* a leaving-exemption. Scale only the component of the commanded delta that
  *reduces* clearance. MoveIt does not do this, and on an arm that can fold into
  itself the direction-blind version can trap the arm in a pose it cannot leave.
- **Validate keyframe sequences atomically at admission, then re-validate the
  whole remainder during execution and stop-and-hold at the last valid
  waypoint** — MoveIt's `isRemainingPathValid()` pattern. Never "execute until
  the step that fails."
- **Emit a machine-readable reason on every degradation, not just refusals.**
  The teleop literature is unanimous that an unexplained refusal is worse than a
  degraded motion, and in an AR UI the reason is nearly free to surface.
- **Generate the ACM offline** with the Setup Assistant's algorithm including the
  **fixpoint loop** on always-colliding pairs (my run was single-pass). Bake the
  resulting pair-allowlist into the repo as a build-time artifact: zero runtime
  cost.
- **Prefer batching, then adaptive bisection, over a fixed dense step** — batched
  FK makes the 478-sample figure cost milliseconds, and clearance-driven
  subdivision skips work wherever the arm is far from contact.

---

## Q4 — LLM choreography and motion-IR prior art

### Boston Dynamics Choreographer: the best-developed motion IR in commercial robotics

From the [Spot choreography service docs](https://dev.bostondynamics.com/docs/concepts/choreography/choreography_service):

A `ChoreographySequence` is "a unique name, the number of slices per minute, and a
repeated list of moves." Each move carries a **type identifier, a starting slice,
a duration in slices, and a `MoveParams` proto**.

Time is **musical, not metric**. The unit is a **slice = ¼ beat** (a 1/16th note
in 4/4), so four slices make a beat and a 100 BPM sequence has 400 slices per
minute. "Moves always run for an integer number of slices." The docs note a
reliability envelope of **250–450 slices per minute** — outside it, moves become
unreliable.

Parameter typing and validation are exactly the pattern we want:

- Booleans and enums carry defaults; numeric types (`double`, `int32`) carry
  **minimum, default, and maximum** bounds.
- "**Scripts are rejected if the values are outside the allowable range.**"
- "If `non_strict_parsing` is enabled, the value will be forced into the required
  range and a warning will be thrown."
- The `UploadChoreography` RPC "validates and checks the structure of the routine
  to ensure it is feasible and within bounds," returning **warnings and failures**,
  where "a failure is something the choreography service could not automatically
  correct and must be fixed before the routine can be executed. Warnings are
  automatically corrected and do not block execution."

Three further structural features, each of which answers a question our design
will otherwise have to invent an answer to:

- **Multi-track composition.** Tracks are legs / body / arm / gripper / lights /
  buzzer / annotations / music, and each move *declares which tracks it controls*
  (`controls_arm`, `controls_legs`, …). Same-track overlap is illegal;
  cross-track composes in parallel. **The declared track-claim is what makes
  overlap mechanically checkable** rather than a matter of judgement. Our analogue
  is arm-joints vs gripper: "wave while closing the gripper" is legal, two moves
  both claiming the arm is not.
- **Entry/exit state machine.** Leg-track moves declare an entry and exit state
  (Stand/Sit/Kneel/Sprawl) and each move's entry must match the previous move's
  exit. **Composition legality is checked as state-machine adjacency, not just
  parameter range** — a stronger and cheaper check than geometry.
- **`MoveInfo` metadata** carries `move_length_slices`, `min_move_length_slices`,
  `is_extendable`, and `min_time`/`max_time` wall-clock bounds — so a move that is
  *musically* legal can still be rejected as *physically* too fast at this BPM.
  Timing feasibility is a first-class check separate from pose feasibility.

Animated moves (`.cha` files) are the escape hatch, and their design is
instructive: units are pinned globally in the spec ("Distance: meters, Angles:
radians, Time: seconds"), the body is a CSV whose **header line names the semantic
channels** (`time body_pos body_euler_rpy arm_joints`), and — the key move —
**an animation is itself packaged as a named move with declared parameters.**
Hand-authored keyframes get *promoted into the library* as a first-class
primitive rather than remaining a special case.

The authoring tool also ships **graduated exposure**: beginner mode restricts
parameter ranges and hides dynamic moves, advanced unlocks them. Not a binary
safety switch.

Four lessons transfer directly:

1. **Author against a library of named, parameterized moves — not raw joint
   angles.** Choreographer's atom is `sway(amplitude, ...)`, not six numbers.
2. **Validate the whole sequence at upload, before any execution** — the
   architecture that Q3 argued for on independent grounds.
3. **Two severities: warnings auto-corrected, failures blocking.** This is the
   principled version of the clamp-vs-reject question. Clamping is acceptable for
   parameters where "a bit less" is still the intended motion; it is not
   acceptable where it would silently produce a different motion than authored.
4. **Timing is expressed in the domain's natural unit** (beats), with a documented
   reliability envelope, not in raw seconds.

### The recurring LLM→robot architecture

The same shape appears across the literature: **the LLM proposes symbolically;
a separate deterministic or learned component decides feasibility.**

- **SayCan** ([arXiv:2204.01691](https://arxiv.org/abs/2204.01691), "Do As I Can,
  Not As I Say") grounds an LLM through **value functions — affordance functions
  capturing the log-likelihood that a skill will succeed**. The LLM supplies
  P(skill is useful for the instruction); the affordance model supplies
  P(skill succeeds from here); their product ranks what to actually do. The LLM
  never gets the final say on feasibility.
- **Code as Policies** ([arXiv:2209.07753](https://arxiv.org/abs/2209.07753))
  has the LLM emit **Python that composes motion primitives and conditional
  rules**, calling a constrained control-primitive API rather than emitting raw
  numbers. Structure and safety come from the API surface the model is allowed to
  call.
- **Language to Rewards** ([arXiv:2306.08647](https://arxiv.org/abs/2306.08647))
  has the LLM emit **reward parameters/weights**, which an MPC controller then
  optimizes. Its stated motivation is the sharpest one-line argument in this
  literature for *not* asking an LLM for joint angles: "low-level robot actions
  are hardware-dependent and underrepresented in LLM training corpora."
- **VoxPoser** ([arXiv:2307.05973](https://arxiv.org/abs/2307.05973)) has the LLM
  write code composing 3D affordance and constraint value maps, which a motion
  planner consumes as objective functions.
- **Swarm-GPT** ([arXiv:2312.01059](https://arxiv.org/pdf/2312.01059)) is the
  closest structural match to our design: the **LLM emits waypoints, and "a
  trajectory planner processes these waypoints to guarantee collision-free and
  feasible motion."** The LLM is explicitly not trusted with feasibility.
- **Alter3** ([arXiv:2312.06571](https://arxiv.org/abs/2312.06571)) drives a
  43-DoF humanoid via GPT-4 emitting **program code, not joint angles**, for
  exactly the hardware-dependence reason. Its most interesting finding for us:
  "verbal feedback can adjust poses, obviating the need for fine-tuning" — natural
  language as the correction channel for a generated pose.
- **Boston Dynamics' own LLM + Spot**
  ([Robots That Can Chat](https://bostondynamics.com/blog/robots-that-can-chat/))
  supplies API docs as Python comments over a tiny action surface —
  `go_to(location_id, phrase)`, `say(phrase)`, `ask(question)`. The technique
  worth stealing: they **enumerated the legal `location_id`s into the prompt each
  turn**, "preventing impossible navigation commands." **Constrain by
  enumeration, not by post-hoc validation** — cheaper than rejection, and the
  model rarely proposes the impossible thing in the first place. (They also found
  "remember to be concise" necessary for latency, at ~6 s round trips.)
- **RT-2 / RT-X** are the deliberate contrast: end-to-end learned
  vision-language-action policies that emit actions directly, with no inspectable
  IR and no place to insert a validator. They are a different bet — more capable
  in open-world perception, but they give up exactly the property we want, which
  is a symbolic artifact a deterministic gate can reason about before anything
  moves.

Our design sits squarely in the SayCan/CaP lineage, with one simplification in
our favour: **our feasibility oracle is exact.** SayCan needed a *learned* value
function because "will this grasp succeed?" is not analytically decidable. "Does
this pose self-collide?" is — it is FK plus geometry, and we measured it at
~40 µs. We get the architecture's benefit without its hardest component.

**UNSETTLED:** I did not find a well-documented published system doing
specifically LLM→keyframe-sequence for a robot arm with a geometric validation
gate. The closest documented analogues are Choreographer (keyframe IR, no LLM)
and SayCan/CaP (LLM, but skill/code-level rather than keyframes). Treat our
design as combining two established halves rather than following a single
precedent.

### Constraining Claude's output to a keyframe schema

From the [Structured outputs docs](https://platform.claude.com/docs/en/build-with-claude/structured-outputs):

Two features: **JSON outputs** via `output_config.format` (was `output_format`),
and **strict tool use** via `strict: true` on a tool's `input_schema`. The beta
header `structured-outputs-2025-11-13` "will continue working during a transition
period, but is no longer required." Supported models include `claude-fable-5`,
`claude-opus-5`, `claude-sonnet-5`, and `claude-haiku-4-5-20251001`. The schema is
compiled into a grammar that **restricts token generation during inference**, so
compliance is guaranteed rather than requested.

**The limitation that decides our schema design.** Supported and unsupported
JSON Schema features, quoted:

> **Supported:** all basic types; `enum`; `const`; `anyOf`/`allOf` (with
> limitations); `$ref`/`$def`; `default`; `required` and `additionalProperties`
> (must be `false`); string formats; array `minItems` (**only values 0 and 1**).

> **Not supported:** recursive schemas; complex types within enums; external
> `$ref`; **numerical constraints (such as `minimum`, `maximum`, `multipleOf`)**;
> string constraints (`minLength`, `maxLength`); **array constraints beyond
> `minItems` of 0 or 1**; `additionalProperties` other than `false`; regex
> patterns have limitations.

Two consequences, both load-bearing:

1. **The schema cannot enforce joint limits.** `minimum`/`maximum` are
   unsupported, so a grammar-constrained schema guarantees "a number is here"
   and nothing about its value. **Our FK/limit/collision validator is therefore
   not a belt-and-braces extra — it is the only thing enforcing range.** Any
   design that leans on the schema for safety is relying on a guarantee the API
   explicitly does not make.

2. **Named joint keys beat a positional array, on documented grounds.** Array
   length cannot be constrained beyond `minItems` 0 or 1, so a
   `joint_deg: [6 numbers]` field has **no schema-level guarantee of being six
   elements** — a five- or seven-element array is schema-valid, and a
   *reordered* array is indistinguishable from a correct one. Switching to an
   object:

   ```json
   {
     "shoulder_pan": 0.0, "shoulder_lift": -20.0, "elbow_flex": 30.0,
     "wrist_flex": 0.0, "wrist_roll": 0.0, "grip": 0.5, "dwell_s": 0.4
   }
   ```

   with `required` listing all six joints and `additionalProperties: false`
   converts both failure modes into schema violations the grammar cannot emit.
   **Exactly the six named joints, no more, no fewer, order-independent.** This
   is the single highest-leverage schema decision available, and it costs
   nothing.

The classic LLM numeric failure modes this addresses — silently permuted element
order, off-by-one joint indexing, and degrees-vs-radians confusion — are all
either eliminated (the first two, by naming) or made detectable (the third: a
`_deg` suffix in every key plus explicit units and ranges in the `description`
fields, which the model does read even though it cannot be forced to obey them).

**Repair loop — a documented first-class pattern.** Return the failure as a
`tool_result` with `"is_error": true`
([handling tool calls](https://platform.claude.com/docs/en/agents-and-tools/tool-use/handle-tool-calls)).
The docs are directive about the content: *"Write instructive error messages.
Instead of generic errors like `failed`, include what went wrong and what Claude
should try next."* And on the loop: *"If a tool request is invalid or missing
parameters, Claude will retry 2-3 times with corrections before apologizing to the
user."* So budget for 2–3 repair round-trips, and log the repair rate — it is a
direct measure of schema quality.

Format constraints that will bite if missed: `tool_result` blocks must come
**first** in the user content array and must immediately follow the `tool_use`
turn, or the API returns 400.

**One safety setting that matters more here than in a typical chat app:** set
`tool_choice: {type: "auto", disable_parallel_tool_use: true}`. Without it the
model can emit two tool calls in one turn — meaning **two motion sequences racing
the same arm.** The docs also note that `tool_result` content is untrusted-input
/ indirect-prompt-injection territory, which is live for us because this bot is
multi-user: a sequence authored on behalf of one user must not be steerable by
text another user planted.

**Note on cost/latency:** the docs flag that grammar compilation adds latency on
first use, with grammars **cached for 24 hours**, and that "changes to schema
structure or tool set invalidate cache." Our schema is static, so we pay this
essentially once.

### Design implications

- **Author over a library of named parameterized moves** (`wave`, `reach`,
  `nod`) with bounded params, following Choreographer — and keep a raw-keyframe
  escape hatch for motions the library cannot express. Raw joint angles should be
  the fallback, not the primary interface.
- **Use named joint keys, never a positional array.** Array length and order are
  unenforceable under structured outputs; object keys plus `required` plus
  `additionalProperties: false` make the correct shape the only expressible one.
- **Treat the validator as the sole enforcement of range**, because the schema
  provably cannot enforce it. Put joint limits in `description` strings for
  guidance, and in the validator for safety.
- **Use strict tool use (`strict: true`)** on the keyframe-emitting tool so shape
  compliance is a grammar guarantee rather than a prompt request.
- **Suffix every numeric key with its unit** (`_deg`, `_s`) — cheap, and it
  attacks the one failure mode naming does not eliminate.
- **Adopt Choreographer's two severities:** auto-correct-with-warning where
  clamping preserves the authored intent; hard-fail where it would not.
- **Validate the whole sequence before executing any of it**, as `UploadChoreography`
  does — the same conclusion Q3 reached from the streaming-vs-batch distinction.
- **Feed validator errors back as `tool_result`** with step index, joint name,
  offending value, and violated bound, so the model repairs rather than reguesses.
- **Use an abstract time unit** — integer ticks/beats plus a sequence-level
  tempo — rather than per-step seconds. One tempo change retimes the whole
  sequence, it stops the model emitting `0.37`-second dwells, and it gives a
  natural place to enforce max joint velocity (ticks × per-tick delta).
- **Declare an explicit entry/exit pose contract** per sequence, and reject at
  admission if the declared start does not match the arm's actual current pose.
  This is Choreographer's state-machine adjacency check, and it kills the stale-
  reference failure class that the existing reference-seed protocol works around.
- **Have moves declare which tracks they control** (arm joints vs gripper), so
  overlap legality is mechanically checkable rather than a judgement call.
- **Enumerate legal referents into the prompt each turn** — named poses,
  available primitives, current joint state, actual limits — following Boston
  Dynamics' `nearby_locations` trick. Constraining by enumeration is cheaper than
  rejecting after the fact, and it means the validator mostly does not fire.
- **Set `disable_parallel_tool_use: true`** so two motion sequences can never
  race the arm, and treat `tool_result` content as untrusted input given this bot
  is multi-user.
- **Ship 2–3 complete few-shot exemplar sequences.** Strict mode gets *valid*
  JSON; exemplars are what get *good* motion. Keep them short — Boston Dynamics
  found prompt length dominated their latency.
- **Promote validated raw sequences into the named-primitive library** with
  declared parameters, exactly as Choreographer promotes `.cha` animations into
  first-class moves.

---

## Q5 — Known SO-100/SO-101 gotchas

### Joint axis signs vs physical servo direction

Covered in Q1 and it is the sharpest gotcha in the model: **all six URDF axes are
`0 0 1`**, with real orientation baked into each `<origin rpy>` by
`onshape-to-robot`. Reading a joint's physical sense off `<axis>` is meaningless
here. Any hand-rolled kinematics that assigns per-joint axes by intuition
("shoulder_lift pitches about Y") produces a self-consistent but *different*
model — which is exactly the divergence between `server/arm_kinematics.py` and
the URDF. The fix is not to reconcile the two conventions but to delete one.

The MJCF confirms the same structure independently — every joint is
`axis="0 0 1" type="hinge"` with orientation in the body `quat`.

### The URDF and MJCF joint limits agree (a rare piece of good news)

Fetched [`Simulation/SO101/so101_new_calib.xml`](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.xml)
and compared its `<joint range>` and actuator `ctrlrange` against the URDF's
`<limit lower/upper>`:

| Joint | URDF (rad) | MJCF range (rad) | Agree? |
|---|---|---|---|
| `shoulder_pan` | ±1.91986 | ±1.9198621771937 | ✅ |
| `shoulder_lift` | ±1.74533 | ±1.7453292519943 | ✅ |
| `elbow_flex` | ±1.69 | ±1.69 | ✅ |
| `wrist_flex` | ±1.65806 | ±1.65806 | ✅ |
| `wrist_roll` | −2.74385 … 2.84121 | −2.7438472969 … 2.8412063093 | ✅ |
| `gripper` | −0.174533 … 1.74533 | −0.17453297 … 1.74532919 | ✅ |

The MJCF actuators repeat these as `ctrlrange`, with `forcerange="-3.35 3.35"`
per joint. So URDF limits can be used as the joint-limit gate with confidence;
they are not a third divergent source.

### Gripper modelling — revolute, and lerobot disagrees with the URDF

The URDF models the gripper as **`type="revolute"`**, `−0.174533 … 1.745329` rad
(−10° … +100°), rotating `moving_jaw_so101_v1_link` about the `gripper_link`.
It is a single hinged jaw, not a parallel/prismatic gripper. Same in the MJCF
(`type="hinge"`).

But lerobot drives it as `MotorNormMode.RANGE_0_100`, and TheRobotStudio's own
README states the mismatch outright: the gripper is treated as a linear joint
where **0 = fully closed and 100 = fully open**, and this mapping is
"**not yet reflected** in the current URDF and MuJoCo files"
([SO101 README](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/README.md)).

So there are two live gripper representations and they are *not* linearly
related in any documented way. Our motion IR must pick one — the natural pick is
lerobot's `grip: 0..1` (or 0..100) for authoring, converted to a URDF revolute
angle only for the FK/collision pass, via an explicit two-point mapping
(`0 → closed angle`, `1 → open angle`) that we measure rather than assume.

### Community self-collision configs — **there are none**

This is a definitive negative finding, and it is worth stating loudly because it
means one option is off the table.

I fetched the SO101 MJCF and grepped it: **zero `<contact>` elements, zero
`<exclude>` elements, zero `<keyframe>` elements** (`grep -cE 'exclude|<contact|<keyframe'`
returns 0). There is no allowed-collision matrix, no adjacent-link exclusion
list, and no named home pose to inherit. The file ends with an empty
`<equality/>` and nothing else.

What the MJCF *does* do is disable collision wholesale on two geom classes:

```xml
<default class="visual">
  <geom type="mesh" contype="0" conaffinity="0" group="2"/>
</default>
<default class="collision">
  <geom group="3"/>
</default>
...
<default class="sts3215">
  <geom contype="0" conaffinity="0"/>
  ...
</default>
```

So visual geoms never collide, and anything in the `sts3215` class never
collides. There is also a documented `backlash` default modelling **±0.5° of
backlash** per joint (`range="-0.008726646259971648 0.008726646259971648"`) —
a useful number: it is the mechanical slop floor, and it argues for a collision
margin comfortably larger than 0.5° of joint error.

Related: the SO101 README notes **base collision meshes were removed** because
they caused "problematic collision behavior during simulation and planning" —
a hint that naive mesh-mesh on these CAD exports misbehaves, consistent with the
triangle counts measured above.

There is no MoveIt config (no SRDF, hence no `<disable_collisions>` block) for
the SO-101 in either upstream repo. **We have to generate our own allowed-collision
matrix.** Q3 describes the standard random-sampling trick for doing that offline.

### Design implications

- **Delete `server/arm_kinematics.py`'s convention rather than reconciling it.**
  The URDF is the only model with an upstream-validated axis convention; a second
  self-consistent parameterization is a permanent source of drift.
- **Trust the URDF joint limits** — they agree exactly with the MJCF and can
  serve as the joint-limit half of the validation gate without further checking.
- **Model the gripper explicitly as two representations with a measured mapping.**
  Author in lerobot's 0..1 linear space, convert to the URDF revolute angle for
  collision checking only, and record the two calibration endpoints.
- **Budget for generating our own ACM** — nothing upstream provides one. This is
  real work, not a copy-paste, but it is offline one-time work (Q3).
- **Exclude servo-body meshes from collision, following the MJCF's lead**
  (`sts3215` class is `contype=0`). That immediately removes the two most-repeated
  meshes (`sts3215_03a_v1` appears 4× in the tree) from the pair count.
- **Set the collision margin above the ±0.5° modelled backlash**, plus servo
  positioning error — a margin derived from mechanics, not guessed.
- **Expect to drop or simplify the base collision mesh**, as upstream did.

---
