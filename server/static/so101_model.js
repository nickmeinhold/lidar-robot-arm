// Build the real SO-101 arm in three.js from the baked URDF (so101_urdf.json)
// + the STL meshes. This is the SINGLE SOURCE the viewer uses for both the
// visible arm and (via computeLinkFrames) the collision model, so the two can't
// disagree. Transforms verified against a Blender headless assembly render.
//
// URDF is Z-up (robotics); the viewer scene is Y-up, so the caller rotates the
// returned root by -90° about X.

import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

// Our control-angle name -> URDF revolute joint name.
export const JOINT_MAP = {
  shoulder_yaw: 'shoulder_pan',
  shoulder_pitch: 'shoulder_lift',
  elbow_pitch: 'elbow_flex',
  wrist_pitch: 'wrist_flex',
  wrist_roll: 'wrist_roll',
  gripper: 'gripper',
};

// URDF rpy (fixed-axis roll,pitch,yaw) -> quaternion. R = Rz(y)·Ry(p)·Rx(r),
// matching the Blender assembly that rendered correctly.
function rpyQuat([r, p, y]) {
  const m = new THREE.Matrix4().makeRotationZ(y)
    .multiply(new THREE.Matrix4().makeRotationY(p))
    .multiply(new THREE.Matrix4().makeRotationX(r));
  return new THREE.Quaternion().setFromRotationMatrix(m);
}

// Load + assemble. Returns { root, setJoints, linkGroups, urdf } once all STLs
// load. opts.onProgress(loaded, total) fires per mesh for diagnostics.
export async function loadSO101(baseUrl, opts = {}) {
  const { material, onProgress } = opts;
  const urdf = await (await fetch(baseUrl + 'so101_urdf.json')).json();
  const stl = new STLLoader();
  const mat = material || new THREE.MeshStandardMaterial(
    { color: 0x9aa4bf, roughness: 0.6, metalness: 0.25 });

  const linkGroups = {};
  for (const l of urdf.links) linkGroups[l.name] = new THREE.Group();

  // Joint hierarchy: parentLink -> jointOrigin(fixed) -> childLink(articulated).
  const jointRot = {};
  for (const j of urdf.joints) {
    const jg = new THREE.Group();
    jg.position.set(...j.origin.xyz);
    jg.quaternion.copy(rpyQuat(j.origin.rpy));
    linkGroups[j.parent].add(jg);
    jg.add(linkGroups[j.child]);
    if (j.type === 'revolute' || j.type === 'continuous') {
      jointRot[j.name] = { child: linkGroups[j.child], axis: new THREE.Vector3(...j.axis).normalize() };
    }
  }

  // Visual meshes (each link has several — servo + structural brackets).
  const loads = [];
  const total = urdf.links.reduce((n, l) => n + l.visuals.length, 0);
  let done = 0;
  const tick = (mesh, err) => {
    done++;
    if (err) console.warn('STL failed:', mesh, err);
    if (onProgress) onProgress(done, total);
  };
  for (const l of urdf.links) {
    for (const v of l.visuals) {
      loads.push(new Promise((res) => {
        stl.load(baseUrl + 'assets/' + v.mesh, (geo) => {
          const mesh = new THREE.Mesh(geo, mat);
          mesh.position.set(...v.origin.xyz);
          mesh.quaternion.copy(rpyQuat(v.origin.rpy));
          mesh.castShadow = true; mesh.receiveShadow = true;
          linkGroups[l.name].add(mesh);
          tick(v.mesh); res();
        }, undefined, (e) => { tick(v.mesh, e); res(); });
      }));
    }
  }
  await Promise.all(loads);

  function setJoints(angles) {
    for (const [ourName, urdfName] of Object.entries(JOINT_MAP)) {
      const jr = jointRot[urdfName];
      if (jr) jr.child.quaternion.setFromAxisAngle(jr.axis, angles[ourName] ?? 0);
    }
  }

  return { root: linkGroups['base_link'], setJoints, linkGroups, jointRot, urdf };
}
