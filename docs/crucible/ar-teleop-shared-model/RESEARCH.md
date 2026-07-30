# Research: AR-teleop shared-model collision safety + registration

Research pass feeding a design for an AR teleoperation system driving a HuggingFace
SO-100/SO-101 (LeRobot) 6-DOF arm (Feetech STS3215 servos). The user's arm is the
"leader" — an iPhone shows a virtual target pose and the human matches it.

Legend: **[FACT]** = established/cited. **[INFER]** = my inference for this design.
Cited URLs inline; full list at bottom.

---

## Key findings for the design (load-bearing)

1. **The simplest self-collision model that works for a 6-DOF arm is one capsule per
   link + all-pairs capsule-capsule minimum-distance, excluding adjacent-link pairs
   via an "allowed collision matrix."** This is exactly what MoveIt/FCL do, minus the
   mesh precision. **[FACT]** on the method; **[INFER]** that capsules alone suffice here.

2. **Capsule-capsule distance is a closed form:** compute the closest distance between
   the two capsule *axis segments* (Ericson's `ClosestPtSegmentSegment`), then subtract
   the two radii. Collision ⇔ `segDist < r1 + r2`. No iteration, no library needed.
   ~50–100 FLOPs per pair. **[FACT]**

3. **Per-tick geometric checking is absurdly cheap here.** ~15 non-adjacent pairs on a
   6-link arm × ~100 FLOPs ≈ 1,500 FLOPs/tick; at 100 Hz that's ~150 kFLOP/s — a
   rounding error on any CPU, including a Raspberry Pi. Latency is not a concern; you
   can check *before* every servo write. **[INFER from FLOP arithmetic]**

4. **The SO-101 URDF exists and is usable** (`TheRobotStudio/SO-ARM100/Simulation/SO101/
   so101_new_calib.urdf`) with real joint limits and mesh geometry, BUT the repo's own
   README warns the base collision meshes were *removed* and an open issue flags the
   URDF as under-defined (missing limits, inverted axes, no decomposed collision
   meshes). So: harvest joint limits + link origins from it, but hand-author your
   capsule params. **[FACT]**

5. **ARKit body tracking is NOISY at the level that matters.** Published validation
   shows joint-*angle* RMSE of ~7–14° per joint vs Vicon, and systematic
   under-estimation of range of motion. Joint *position* error is not crisply
   published but is realistically several cm, i.e. **larger than the ~12 mm link
   radius.** This means ARKit body pose cannot be trusted as absolute ground truth for
   a 12 mm safety margin — **[FACT]** on the angles; **[INFER]** on the position scale
   and its implication.

6. **The "handshake" absolute-registration idea has real precedent in spirit**
   (kinesthetic teaching, hand-eye calibration) but I found **no system that replaces
   per-joint servo zeroing with a human aligning to a shown virtual pose.** It's a
   genuinely novel move. The nearest analogue (trzy/robot-arm) sidesteps registration
   entirely by doing *relative* end-effector control after a button-press "clutch."
   **[FACT]** on trzy; **[INFER]** on novelty.

7. **Registration error and collision margin trade directly:** every cm of
   registration/tracking error must be added to the capsule radius as inflation. With
   multi-cm ARKit noise you'd inflate 12 mm capsules to ~30–50 mm, which on a ~340 mm-
   reach arm sterilizes a lot of the workspace. This is the central tension the design
   must resolve. **[INFER]**

8. **You already have a second, independent safety channel: STS3215 `PRESENT_LOAD`
   /current.** The servos report load/current, so reactive contact-stop is available
   regardless of geometric model quality. Belt-and-suspenders: geometry to *avoid*,
   current to *catch what geometry misses*. **[FACT]** servos expose it; **[INFER]** on
   the architecture.

---

## 1. Geometric self-collision detection for articulated arms

### How it's done in practice
The industry-standard stack is **URDF collision geometry → FCL (Flexible Collision
Library) → broadphase (AABB) cull → narrowphase distance → Allowed Collision Matrix
(ACM) to skip pairs that are always/necessarily touching.** MoveIt is the reference
implementation:
- MoveIt's FCL detector adds all robot links to an FCL manager, runs a broadphase
  collision check on global AABBs, then narrowphase only on overlapping pairs. Link
  geometries and local AABBs are computed once and cached.
  ([moveit_core CollisionEnvFCL](http://docs.ros.org/en/noetic/api/moveit_core/html/classcollision__detection_1_1CollisionEnvFCL.html))
- **The ACM is the crucial concept:** "Adjacent links that are connected are by design
  in collision" — so the matrix encodes pairs that never need checking due to kinematic
  infeasibility. MoveIt auto-generates it by sampling: pairs that are *always* colliding,
  *never* colliding, adjacent, or default-touching get disabled.
  ([MoveIt developer concepts](https://moveit.ai/documentation/concepts/developer_concepts/),
  [Optimizing self-collision checks #1317](https://github.com/moveit/moveit/issues/1317))

### The simplest approach that works for a 6-DOF arm
**One capsule per link, all-pairs distance, minus adjacent pairs.** A capsule = a line
segment (the link's axis, from its proximal joint to its distal joint) plus a radius R.
For a 6-link chain that's 6 capsules and 15 unordered pairs, of which the 5
adjacent-link pairs are excluded → **~10 real checks per tick** (a couple more if you
model the gripper jaws as extra capsules). This is the representation used for
self-collision on continuum/industrial arms precisely because it's analytic and fast:
"a sequence of capsules along the computed backbone… checked analytically by
determining the closest points between the two line segments, then checking if their
distance is less than the robot diameter."
([Interactive-Rate Supervisory Control, arXiv:2211.16452](https://arxiv.org/pdf/2211.16452);
[On-line collision avoidance, arXiv:1909.05159](https://arxiv.org/pdf/1909.05159))

### The closed form (capsule-capsule minimum distance)
This is textbook Christer Ericson, *Real-Time Collision Detection* (2005), §5.1.9
`ClosestPtSegmentSegment`. Two-step:

1. **Closest distance between the two axis segments** `P1P2` and `Q1Q2`. Let
   `d1 = P2−P1`, `d2 = Q2−Q1`, `r = P1−Q1`. Solve for parameters `s,t ∈ [0,1]`
   minimizing `|(P1 + s·d1) − (Q1 + t·d2)|`. Analytically: compute
   `a=d1·d1, e=d2·d2, f=d2·r, c=d1·r, b=d1·d2, denom=a·e−b²`; if `denom>0`,
   `s = clamp((b·f − c·e)/denom, 0, 1)`, then `t = (b·s + f)/e`, clamp `t` to `[0,1]`
   and recompute `s = (b·t − c)/a` clamped — the standard clamp-and-reproject that
   handles parallel/degenerate segments. The closest points are `C1 = P1 + s·d1`,
   `C2 = Q1 + t·d2`.
2. **Capsule distance** `= |C1 − C2| − (r1 + r2)`. **Collision iff `< 0`** (or
   `< safety_margin` for a keep-out buffer).

Reference implementations of exactly this port: 
[Ericson functions ported (gist)](https://gist.github.com/jakubtomsu/2acd84731d3c2613c91e40c2e064ffe6),
book errata/companion at [realtimecollisiondetection.net](https://realtimecollisiondetection.net/).
This is a few dozen lines of Python/NumPy — **no FCL dependency required** for the
capsule-only case. FCL is worth it only if you later want exact mesh-mesh distance.

### Accuracy / tolerance vs real hardware (cables, brackets)
- Capsules **over-approximate** thin links well (a straight servo-to-servo link is
  almost a capsule) and **under-approximate** wide/bracketed parts (motor housings,
  the gripper). The honest move is to size each capsule to the *bounding* radius of its
  link including brackets, which makes it conservative (fewer false-negatives, more
  false-positives / lost workspace).
- **Cables are the classic unmodeled hazard** — they droop and snag outside any
  link capsule. No geometric model catches this; it's an argument for the current-based
  reactive channel (§2) as backstop. **[INFER]**
- Realistic tolerance: with hand-tuned capsule radii you can hold real-vs-model
  agreement to roughly the bracket over-hang, ~1–2 cm. Trying to be tighter than ~1 cm
  on a 3D-printed arm with hand-assembly variance is false precision. **[INFER]**

---

## 2. Configuration-space vs reactive collision safety

Three architectures, as framed in the task:

| | (a) Reactive load/current | (b) Precomputed forbidden-config map | (c) Online geometric (per-tick capsule) |
|---|---|---|---|
| **Signal** | STS3215 `PRESENT_LOAD`/current spikes on contact | Offline C-space occupancy / sampled self-collision table | Capsule distances each control tick |
| **When it acts** | *After* contact begins | Before motion (planning-time) | Before each commanded move |
| **Latency** | Detection lag = sense + threshold + torque-off (tens of ms) | Zero at runtime (table lookup) | ~µs (see FLOP budget) |
| **Cost** | ~free (already reading registers) | Expensive to precompute for 6-DOF; big table; must recompute if geometry changes | Trivial per tick |
| **Catches** | *Any* contact incl. cables, world objects, the user | Only modeled self-collisions | Only modeled self-collisions |
| **Misses** | Nothing physical, but only *after* force builds | Unmodeled geometry; anything dynamic | Cables, world, unmodeled brackets |

**What real teleop systems use:** a *layered* combination. Planners (MoveIt/OMPL) use
(c)-style checking during planning; collaborative/contact-rich systems add (a)
current/force reaction as a hard safety reflex. Pure (b) precomputed C-space maps are
common in research (e.g. neural C-space barriers,
[arXiv:2503.04929](https://arxiv.org/pdf/2503.04929)) but heavyweight for a hobby 6-DOF
arm and brittle to any hardware change.

**Is per-tick capsule checking cheap enough at 30–100 Hz?** Overwhelmingly yes.
`ClosestPtSegmentSegment` is a handful of dot products + a few branches ≈ 50–100 FLOPs.
~10–15 pairs → **~1,000–1,500 FLOPs/tick**. At 100 Hz ≈ **150 kFLOP/s**, i.e. sub-
microsecond of CPU per tick even in pure Python with NumPy vectorization over pairs.
Forward kinematics to place the capsule endpoints (6 4×4 matrix multiplies) dwarfs the
distance math and is still trivial. **The geometric check is essentially free; there is
no latency argument against running it every tick.** **[INFER from arithmetic; method FACT]**

**Design recommendation [INFER]:** run **(c)** as the primary *predictive* guard
(reject/clamp any commanded pose whose capsules violate margin — this prevents the arm
from *entering* contested state, which is the structurally-correct fix rather than
reacting to it) and keep **(a)** current-sensing as an independent reflex for everything
geometry can't see (cables, the operator's hand, world objects). Skip **(b)** — its only
advantage (runtime speed) is moot when (c) is already free, and it's the most brittle to
hardware drift.

---

## 3. SO-100 / SO-101 / LeRobot specifics

### URDF and dimensions [FACT]
Source: `TheRobotStudio/SO-ARM100/Simulation/SO101/so101_new_calib.urdf`.

Kinematic chain (6-DOF + gripper):
`base_link → shoulder_link → upper_arm_link → lower_arm_link → wrist_link →
gripper_link → moving_jaw`.

| Joint | Parent→Child | Lower (rad) | Upper (rad) |
|---|---|---|---|
| shoulder_pan | base→shoulder | −1.91986 | 1.91986 |
| shoulder_lift | shoulder→upper_arm | −1.74533 | 1.74533 |
| elbow_flex | upper_arm→lower_arm | −1.69 | 1.69 |
| wrist_flex | lower_arm→wrist | −1.65806 | 1.65806 |
| wrist_roll | wrist→gripper | −2.74385 | 2.84121 |
| gripper | gripper→moving_jaw | −0.174533 | 1.74533 |

Link offsets (from joint origins, → approximate capsule segment lengths):
- base→shoulder: ~62 mm (Z)
- shoulder→upper_arm: ~54 mm (Z)
- upper_arm→lower_arm: ~28 mm (Y)
- lower_arm→wrist: ~135 mm (X)  ← the long forearm link
- wrist→gripper: ~61 mm (Y)

Total reach on the order of ~340 mm. The **forearm (~135 mm) is the dominant collision
body**; the shorter proximal links are stubby and mostly excluded as adjacent pairs.
Collision geometry in the URDF is **STL meshes, not primitives** — so you must derive
capsule radii yourself (measure the printed parts or bound the meshes).

### URDF caveats [FACT]
- The repo README (`Simulation/SO101/README.md`) states base collision meshes were
  **removed** "due to problematic collision behavior during simulation and planning,"
  and that URDFs were edited to use relative mesh paths.
  ([README](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/README.md))
- Open issue [#54 "Improved URDF"](https://github.com/TheRobotStudio/SO-ARM100/issues/54)
  flags the URDF as under-defined: missing joint limits, some inverted joint axes, no
  decomposed collision meshes. **Treat joint limits + link origins as reliable; treat
  collision meshes/axes as needing verification against your physical arm.**
- Community URDF forks exist: [brukg/SO-100-arm](https://github.com/brukg/SO-100-arm),
  [MuammerBay/SO-ARM_ROS2_URDF](https://github.com/MuammerBay/SO-ARM_ROS2_URDF).

### Known self-collision issues / community collision-avoidance
I found **no dedicated self-collision-avoidance implementation for the SO-100/SO-101 in
LeRobot or the community** — collision handling shows up only via generic MoveIt/sim
planning on the URDF, and the sim README's note about removing base meshes is itself a
symptom of collision-model friction. **[FACT: absence within searched scope; the
instrument here is web search + the repo, not exhaustive.]** This gap is consistent with
the project's own memory notes that firmware angle-limit clamps (not a geometric model)
were doing the "restriction" so far.

### STS3215 for contact detection [FACT]
The STS3215 reports **position, speed, voltage, current, temperature, and load**, and
Feetech/tutorials explicitly cite it as suitable for "force sensing, collision
detection, and adaptive motion planning."
([RobotShop](https://www.robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215),
[commanderfun/STS3215 tutorial](https://github.com/commanderfun/STS3215)).
12-bit magnetic encoder = 4096 steps/rev = 0.088°/step. Your `feetech_controller.py`
already uses tick math on this (4096 ticks/rev). `PRESENT_LOAD` is the register to poll
for a reactive contact-stop; the project memory already reached the same conclusion
(reactive load-based avoidance is the stated next step). **[FACT + project context]**

---

## 4. AR registration / hand-eye for teleoperation

### The open question
Can a human aligning their real arm to a shown virtual pose ("handshake") supply
**absolute** registration, replacing per-joint zero calibration? Verdict from the
research: **partially, but ARKit body-tracking noise is the binding constraint, and it's
likely larger than your safety margin.**

### ARKit body-tracking accuracy [FACT on angles, INFER on positions]
- ARKit (`ARBodyAnchor` / `ARSkeleton3D`) tracks one person's 3D skeleton from the rear
  camera in real time. ([Apple ARBodyAnchor docs](https://developer.apple.com/documentation/arkit/arbodyanchor))
- Validation vs Vicon: **raw ARKit joint-angle RMSE ~6.8°–13.9%** across hip/knee
  flexion in one study; another reports a **weighted mean absolute error of 18.8° ±
  12.1°** across joints/exercises, with **systematic under-estimation of range of
  motion.**
  ([Markerless MoCap iPad+LiDAR, ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0966636222002533),
  [Mobile Motion Tracking using ARKit](https://www.researchgate.net/publication/351446785_Mobile_Motion_Tracking_for_Disease_Prevention_and_Rehabilitation_Using_Apple_ARKit))
- **Joint-*position* error in meters is not crisply published by Apple**, and Apple
  forum threads report accuracy problems (e.g. bad height estimates, non-updating wrist
  joints). ([ARSkeleton accuracy thread](https://developer.apple.com/forums/thread/725730),
  [wrist joints not updating](https://developer.apple.com/forums/thread/722675))
- **[INFER]:** Given ~7–19° angle error over ~25–30 cm human limb segments, propagated
  positional error at the wrist/hand is realistically **several centimeters** (a 10°
  error over a 30 cm forearm ≈ 5 cm tip displacement). ARKit body pose is a *coarse*
  absolute reference, good to a few cm, not to millimeters. Note this is distinct from
  ARKit's *world/device* 6-DOF tracking (VIO), which is far better (sub-cm drift over
  short spans) — the phone knows where *it* is much better than where your *elbow* is.

### Is body-tracking noise larger or smaller than a few cm / the 12 mm radius?
**Larger.** ARKit body-joint noise (several cm) exceeds both a "few cm" and dramatically
exceeds the ~12 mm link radius. **Implication for the design:** you cannot use raw ARKit
body pose as a millimeter-accurate absolute registration source. The handshake can
remove *gross* ambiguity (which way is the arm pointing, roughly where zero is) but the
residual few-cm error must be absorbed by capsule inflation — and inflating 12 mm
capsules by ~3–5 cm materially shrinks a 340 mm-reach workspace. **[INFER — this is the
central design tension.]**

### Hand-eye calibration & kinesthetic registration (the precedent) [FACT]
- **Hand-eye calibration** is the classic robotics problem of solving the fixed
  transform between a camera and a robot/marker frame from paired observations
  (AX = XB). Vision-based variants are standard in surgical robotics.
  ([Vision-based hand-eye calibration](https://discovery.researcher.life/article/vision-based-hand-eye-calibration-for-robot-assisted-minimally-invasive-surgery/c995c92e770f3fefbfdfcb1222dbea56))
- **Kinesthetic teaching** (physically guiding a gravity-compensated arm) is the
  most efficient teaching modality for *precise alignment* tasks — but requires physical
  contact with the robot. Your "handshake" is a **contactless, mirror-image analogue**:
  the human poses themselves to a target rather than posing the robot.
  ([How Should We Teach Robots?, arXiv:2605.28033](https://arxiv.org/html/2605.28033v1))
- **Calibration-free retargeting** is an active trend (AnyDexRT self-supervised
  fingertip correspondence + few-shot human guidance;
  [arXiv:2607.08341](https://arxiv.org/pdf/2607.08341)) — evidence the field is actively
  trying to *remove* explicit calibration, which is the same instinct as the handshake.

### How much registration error can a collision margin tolerate?
Directly: **margin_needed ≈ physical_capsule_radius + registration_error +
tracking_jitter + control_lag_travel.** Every source of registration uncertainty adds
linearly to the keep-out buffer. With 12 mm radii and multi-cm ARKit error, the buffer
is *dominated by registration error*, not by the true geometry. **[INFER]** Practical
consequence: the design should (a) minimize reliance on ARKit body pose for the parts of
the state that feed the collision model, (b) prefer *relative/clutched* control (like
trzy) that never needs absolute body registration, or (c) treat the handshake as a
one-time coarse alignment refined by a tighter secondary signal (servo encoders give
you exact *joint* state regardless of ARKit — the arm always knows its own configuration
to 0.088°, so the collision model can run on *encoder* state, not on ARKit at all). This
last point is important: **self-collision checking never needs ARKit** — it needs the
follower's own joint angles, which the STS3215 encoders report exactly. ARKit noise only
threatens *command* accuracy, not the *safety* model. **[INFER — key architectural
separation.]**

---

## 5. Prior art in phone-AR robot teleoperation

- **trzy/robot-arm** ([GitHub](https://github.com/trzy/robot-arm)) — the closest
  neighbor. iPhone + ARKit tracks the phone's 6-DOF pose; press "Move" and the robot
  mirrors the phone's motion; server does **IK from end-effector pose** to servo angles
  on Alex Koch's low-cost 4-DOF arm. Key differences from this project: it's **relative
  end-effector control gated by a clutch button** (no absolute body registration, no AR
  overlay of the robot, no joint-matching, no self-collision model). It deliberately
  *avoids* the registration problem rather than solving it. **[FACT]**
- **SpesRobotics/teleop** ([GitHub](https://github.com/SpesRobotics/teleop)) — turns a
  phone or VR headset into a teleop device via **WebXR**; pose → IK → follower. Again
  end-effector-pose relative control. **[FACT]**
- **U-ARM** ([arXiv:2509.02437](https://arxiv.org/html/2509.02437v5)),
  **PAPRLE** ([arXiv:2507.05555](https://arxiv.org/pdf/2507.05555)),
  **OpenArm leader-follower** ([docs](https://docs.openarm.dev/teleop/)) — general
  leader-follower teleop ecosystems. When the leader is a non-jointed device (phone,
  VR, gamepad) the universal pattern is **leader emits end-effector pose → IK → follower
  joints.** **[FACT]**
- **Apple Vision Pro** hand/wrist pose is used as an end-effector source in several of
  the above. **[FACT]**

**What I did NOT find [FACT: absence within search scope]:** any system that (a) renders
the *robot's target pose* in AR on the phone and asks the human to *physically match it
with their own body* as the control input, or (b) uses that alignment as *absolute
registration to replace servo zeroing*. The dominant paradigm is relative/clutched
end-effector control with IK, which structurally dodges the exact problem this project is
taking on. **This project's core mechanic appears genuinely novel; the flip side is that
nobody has de-risked it for you. [INFER]**

---

## Failure modes others hit

1. **Trusting the URDF as ground truth.** SO-ARM100's own URDF has missing limits,
   inverted axes, and had base collision meshes removed for causing planning failures.
   Anyone who dropped it into MoveIt unedited hit self-collision false positives/negatives.
   → Harvest limits + origins; hand-verify axes and hand-author capsule radii against the
   physical arm. ([issue #54](https://github.com/TheRobotStudio/SO-ARM100/issues/54))
2. **ARKit range-of-motion under-estimation.** Validation studies repeatedly find ARKit
   *compresses* joint range and carries several-degree systematic bias — so a handshake
   at an extreme pose will be *more* wrong than at a neutral pose. → Register near a
   neutral, well-tracked pose; don't ask the human to match near their range limits.
3. **Adjacent-link pairs left in the collision check.** The #1 self-collision bug in
   MoveIt is forgetting the ACM — adjacent links are *always* touching and will fire
   constant false collisions. → Explicitly exclude the 5 adjacent pairs (and any
   always-touching non-adjacent pairs) before you ship.
4. **Modeling geometry but not cables/brackets.** Capsules miss drooping cables and wide
   motor housings; a geometry-only guard gives false confidence. → Keep the STS3215
   current/`PRESENT_LOAD` reflex as an independent backstop.
5. **Reactive-only safety = damage before stop.** Current sensing reacts *after* contact;
   at speed the arm can deform a printed part or pinch a cable before torque cuts. → Use
   geometric prediction to *prevent entry*, current only to catch the unmodeled.
6. **Confusing ARKit world-tracking accuracy (good) with body-joint accuracy (poor).**
   The phone knows its own 6-DOF pose to sub-cm (VIO) but your elbow to only a few cm.
   Designs that assume body-joint precision equals device-tracking precision over-trust
   the registration. → Run the *safety* model on servo encoder state, not ARKit.
7. **Precomputed C-space maps that rot on hardware change.** Any bracket swap / re-print
   invalidates a baked forbidden-config table. → Prefer the online capsule check; it
   re-derives from current link params for free.

---

## Sources

Collision / capsule geometry:
- MoveIt CollisionEnvFCL — http://docs.ros.org/en/noetic/api/moveit_core/html/classcollision__detection_1_1CollisionEnvFCL.html
- MoveIt developer concepts (ACM) — https://moveit.ai/documentation/concepts/developer_concepts/
- Optimizing self-collision checks #1317 — https://github.com/moveit/moveit/issues/1317
- Interactive-Rate Supervisory Control (capsule self-collision) — https://arxiv.org/pdf/2211.16452
- On-line collision avoidance for cobots — https://arxiv.org/pdf/1909.05159
- Neural C-space barriers — https://arxiv.org/pdf/2503.04929
- Ericson, Real-Time Collision Detection (companion) — https://realtimecollisiondetection.net/
- Ericson functions port (ClosestPtSegmentSegment) — https://gist.github.com/jakubtomsu/2acd84731d3c2613c91e40c2e064ffe6

SO-100/SO-101/LeRobot:
- SO101 URDF — https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
- SO101 Simulation README — https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/README.md
- Improved URDF issue #54 — https://github.com/TheRobotStudio/SO-ARM100/issues/54
- brukg/SO-100-arm URDF — https://github.com/brukg/SO-100-arm
- MuammerBay/SO-ARM_ROS2_URDF — https://github.com/MuammerBay/SO-ARM_ROS2_URDF

STS3215:
- RobotShop STS3215 — https://www.robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215
- commanderfun/STS3215 tutorial — https://github.com/commanderfun/STS3215

ARKit body tracking / accuracy:
- ARBodyAnchor docs — https://developer.apple.com/documentation/arkit/arbodyanchor
- ARKit iPad+LiDAR markerless MoCap (ScienceDirect) — https://www.sciencedirect.com/science/article/abs/pii/S0966636222002533
- Mobile Motion Tracking using ARKit — https://www.researchgate.net/publication/351446785_Mobile_Motion_Tracking_for_Disease_Prevention_and_Rehabilitation_Using_Apple_ARKit
- ARSkeleton accuracy forum thread — https://developer.apple.com/forums/thread/725730
- Wrist joints not updating — https://developer.apple.com/forums/thread/722675

Registration / teleop:
- Vision-based hand-eye calibration — https://discovery.researcher.life/article/vision-based-hand-eye-calibration-for-robot-assisted-minimally-invasive-surgery/c995c92e770f3fefbfdfcb1222dbea56
- How Should We Teach Robots? — https://arxiv.org/html/2605.28033v1
- AnyDexRT calibration-free retargeting — https://arxiv.org/pdf/2607.08341

Prior art (phone/AR teleop):
- trzy/robot-arm — https://github.com/trzy/robot-arm
- SpesRobotics/teleop (WebXR) — https://github.com/SpesRobotics/teleop
- U-ARM — https://arxiv.org/html/2509.02437v5
- PAPRLE — https://arxiv.org/pdf/2507.05555
- OpenArm leader-follower — https://docs.openarm.dev/teleop/
