// Client-side port of server/arm_kinematics.py — FK capsules + self-collision.
//
// WHY a port at all: the interactive arc control sweeps collides() across a
// joint's whole range every frame to paint the safe/unsafe envelope, which wants
// to be instant + local (no server round-trip per sample). The risk of a second
// copy of safety-relevant math is SILENT DRIFT from the Python. That risk is
// neutralised by selfTest(): it replays collision_fixtures.json (generated from
// the tested Python) and asserts this port reproduces every verdict. If the port
// drifts, the viewer console screams on load. Keep this file in lockstep with
// arm_kinematics.py; regenerate fixtures when either changes.

import * as THREE from 'three';

// ─── Link dims / radii — MIRROR arm_kinematics.py exactly ────────────────────
export const BASE_HEIGHT = 0.048;
const UPPER_ARM_LEN = 0.1126;
const FOREARM_LEN = 0.1349;
const WRIST_LEN = 0.0601;
const FINGER_H = 0.040;
const GRIP_MAX_HALF = 0.0175;
const FINGER_TOP_Y = -0.012;
const CLOSED_OFFSET = 0.004;

const R = 0.012;
const RADII = {
  base: 0.035,
  upper_arm: R,
  forearm: R * 0.85,
  wrist: R * 0.7,
  gripper_finger: 0.010,
};

export const DEFAULT_MARGIN = 0.010;
export const GROUND_PLANE_Y = 0.0;

// Adjacent/by-design pairs excluded from collision checks (see Python _ACM).
const ACM = new Set([
  'base|upper_arm',
  'upper_arm|forearm',
  'forearm|wrist',
  'gripper_left|wrist',
  'gripper_right|wrist',
  'gripper_left|gripper_right',
].map(sortKey));

function sortKey(s) {
  if (s.includes('|')) { const [a, b] = s.split('|'); return [a, b].sort().join('|'); }
  return s;
}
function pairKey(a, b) { return [a, b].sort().join('|'); }
function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

const T = (x, y, z) => new THREE.Matrix4().makeTranslation(x, y, z);
const Rx = (t) => new THREE.Matrix4().makeRotationX(t);
const Ry = (t) => new THREE.Matrix4().makeRotationY(t);

// Forward kinematics — mirrors arm_kinematics.frames() (Matrix4 chain).
function fkFrames(a) {
  const mYaw = T(0, BASE_HEIGHT, 0).multiply(Ry(a.shoulder_yaw ?? 0));
  const mPitch = mYaw.clone().multiply(Rx(a.shoulder_pitch ?? 0));
  const mElbow = mPitch.clone().multiply(T(0, -UPPER_ARM_LEN, 0)).multiply(Rx(a.elbow_pitch ?? 0));
  const mWpitch = mElbow.clone().multiply(T(0, -FOREARM_LEN, 0)).multiply(Rx(a.wrist_pitch ?? 0));
  const mWroll = mWpitch.clone().multiply(T(0, -WRIST_LEN, 0)).multiply(Ry(a.wrist_roll ?? 0));
  return { shoulder: mPitch, elbow: mElbow, wrist_pitch: mWpitch, wrist_roll: mWroll };
}

const originOf = (m) => new THREE.Vector3().setFromMatrixPosition(m);
const applyTo = (m, x, y, z) => new THREE.Vector3(x, y, z).applyMatrix4(m);

// Joint angles -> capsule set (same 7 capsules as Python capsules()).
export function capsules(a) {
  const f = fkFrames(a);
  const pShoulder = originOf(f.shoulder);
  const pElbow = originOf(f.elbow);
  const pWpitch = originOf(f.wrist_pitch);
  const pWroll = originOf(f.wrist_roll);

  const caps = [
    { name: 'base', p0: new THREE.Vector3(0, 0, 0), p1: new THREE.Vector3(0, BASE_HEIGHT, 0), r: RADII.base },
    { name: 'upper_arm', p0: pShoulder, p1: pElbow, r: RADII.upper_arm },
    { name: 'forearm', p0: pElbow, p1: pWpitch, r: RADII.forearm },
    { name: 'wrist', p0: pWpitch, p1: pWroll, r: RADII.wrist },
  ];

  const halfOpen = (a.gripper ?? 1.0) * GRIP_MAX_HALF;
  const xOff = CLOSED_OFFSET + halfOpen;
  const botY = FINGER_TOP_Y - FINGER_H;
  for (const [side, sx] of [['left', -1], ['right', 1]]) {
    caps.push({
      name: `gripper_${side}`,
      p0: applyTo(f.wrist_roll, sx * xOff, FINGER_TOP_Y, 0),
      p1: applyTo(f.wrist_roll, sx * xOff, botY, 0),
      r: RADII.gripper_finger,
    });
  }
  return caps;
}

// Ericson RTCD §5.1.9 closest distance between two segments.
function closestSegSeg(p1, q1, p2, q2) {
  const d1 = q1.clone().sub(p1), d2 = q2.clone().sub(p2), r = p1.clone().sub(p2);
  const a = d1.dot(d1), e = d2.dot(d2), f = d2.dot(r), eps = 1e-12;
  let s, t;
  if (a <= eps && e <= eps) { s = 0; t = 0; }
  else if (a <= eps) { s = 0; t = clamp(f / e, 0, 1); }
  else {
    const c = d1.dot(r);
    if (e <= eps) { t = 0; s = clamp(-c / a, 0, 1); }
    else {
      const b = d1.dot(d2), denom = a * e - b * b;
      s = denom > eps ? clamp((b * f - c * e) / denom, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); }
      else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
    }
  }
  const c1 = p1.clone().add(d1.clone().multiplyScalar(s));
  const c2 = p2.clone().add(d2.clone().multiplyScalar(t));
  return c1.distanceTo(c2);
}

function capsuleDistance(a, b) {
  return closestSegSeg(a.p0, a.p1, b.p0, b.p1) - a.r - b.r;
}

// All colliding pairs, worst-first — mirrors collisions().
export function collisions(a, margin = DEFAULT_MARGIN, includeGround = true) {
  const caps = capsules(a);
  const hits = [];
  for (let i = 0; i < caps.length; i++) {
    for (let j = i + 1; j < caps.length; j++) {
      if (ACM.has(pairKey(caps[i].name, caps[j].name))) continue;
      const clr = capsuleDistance(caps[i], caps[j]);
      if (clr < margin) hits.push([caps[i].name, caps[j].name, clr]);
    }
  }
  if (includeGround) {
    for (const c of caps) {
      if (c.name === 'base') continue;
      const clr = Math.min(c.p0.y, c.p1.y) - c.r - GROUND_PLANE_Y;
      if (clr < margin) hits.push([c.name, 'ground', clr]);
    }
  }
  hits.sort((x, y) => x[2] - y[2]);
  return hits;
}

// Pose -> {hit, pair} — mirrors collides().
export function collides(a, margin = DEFAULT_MARGIN, includeGround = true) {
  const hits = collisions(a, margin, includeGround);
  if (!hits.length) return { hit: false, pair: null };
  return { hit: true, pair: [hits[0][0], hits[0][1]] };
}

// Replay Python-generated fixtures; throw on any mismatch (drift guard).
export async function selfTest() {
  let fixtures;
  try {
    const resp = await fetch('collision_fixtures.json');
    fixtures = await resp.json();
  } catch (e) {
    console.warn('[arm_collision] selfTest skipped — could not load fixtures:', e.message);
    return { ok: null, checked: 0 };
  }
  let bad = 0;
  for (const fx of fixtures) {
    const got = collides(fx.angles);
    const gotPair = got.pair ? [...got.pair].sort() : null;
    const wantPair = fx.pair ? [...fx.pair].sort() : null;
    const pairMatch = JSON.stringify(gotPair) === JSON.stringify(wantPair);
    if (got.hit !== fx.collides || !pairMatch) {
      bad++;
      console.error('[arm_collision] DRIFT vs Python:', fx.angles,
        'want', fx.collides, wantPair, 'got', got.hit, gotPair);
    }
  }
  if (bad === 0) console.log(`[arm_collision] selfTest OK — ${fixtures.length} fixtures match Python`);
  else console.error(`[arm_collision] selfTest FAILED — ${bad}/${fixtures.length} mismatches`);
  return { ok: bad === 0, checked: fixtures.length, mismatches: bad };
}
