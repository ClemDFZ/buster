import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const URDF_URL = '/urdf_export/urdf/urdf_export.urdf';
const PACKAGE_PREFIX = 'package://urdf_export/';
const PACKAGE_ROOT = '/urdf_export/';
const RAD2DEG = 180 / Math.PI;
const DEG2RAD = Math.PI / 180;

const statusEl = document.getElementById('status');
const jointsEl = document.getElementById('joints');
const progressEl = document.getElementById('progress');
const progressFill = document.getElementById('progress-fill');
const progressLabel = document.getElementById('progress-label');

function setStatus(msg) {
  statusEl.textContent = msg;
}

function parseVec(str, fallback = [0, 0, 0]) {
  if (!str) return [...fallback];
  return str.trim().split(/\s+/).map(Number);
}

function rpyToQuaternion(rpy) {
  const [r, p, y] = rpy;
  const e = new THREE.Euler(r, p, y, 'ZYX');
  return new THREE.Quaternion().setFromEuler(e);
}

function resolveMeshUrl(filename) {
  if (filename.startsWith(PACKAGE_PREFIX)) {
    return PACKAGE_ROOT + filename.slice(PACKAGE_PREFIX.length);
  }
  if (filename.startsWith('package://')) {
    return '/' + filename.slice('package://'.length);
  }
  return filename;
}

function parseUrdf(xmlText) {
  const doc = new DOMParser().parseFromString(xmlText, 'application/xml');
  const robot = doc.querySelector('robot');
  if (!robot) throw new Error('no <robot> in URDF');

  const links = {};
  for (const linkEl of robot.querySelectorAll(':scope > link')) {
    const name = linkEl.getAttribute('name');
    const visual = linkEl.querySelector(':scope > visual');
    const collision = linkEl.querySelector(':scope > collision');

    const readGeom = (el) => {
      if (!el) return null;
      const originEl = el.querySelector(':scope > origin');
      const meshEl = el.querySelector(':scope > geometry > mesh');
      const boxEl = el.querySelector(':scope > geometry > box');
      const cylEl = el.querySelector(':scope > geometry > cylinder');
      const sphereEl = el.querySelector(':scope > geometry > sphere');
      const colorEl = el.querySelector(':scope > material > color');
      const geom = { type: 'empty', size: null, radius: null, length: null, filename: null };
      if (meshEl) {
        geom.type = 'mesh';
        geom.filename = meshEl.getAttribute('filename');
      } else if (boxEl) {
        geom.type = 'box';
        geom.size = parseVec(boxEl.getAttribute('size'), [0.1, 0.1, 0.1]);
      } else if (cylEl) {
        geom.type = 'cylinder';
        geom.radius = Number(cylEl.getAttribute('radius') || 0.05);
        geom.length = Number(cylEl.getAttribute('length') || 0.1);
      } else if (sphereEl) {
        geom.type = 'sphere';
        geom.radius = Number(sphereEl.getAttribute('radius') || 0.05);
      }
      return {
        xyz: parseVec(originEl?.getAttribute('xyz')),
        rpy: parseVec(originEl?.getAttribute('rpy')),
        ...geom,
        rgba: parseVec(colorEl?.getAttribute('rgba'), [0.7, 0.7, 0.75, 1]),
      };
    };

    links[name] = {
      name,
      visual: readGeom(visual),
      collision: readGeom(collision),
    };
  }

  const joints = {};
  for (const jointEl of robot.querySelectorAll(':scope > joint')) {
    const name = jointEl.getAttribute('name');
    const type = jointEl.getAttribute('type');
    const originEl = jointEl.querySelector(':scope > origin');
    const axisEl = jointEl.querySelector(':scope > axis');
    const limitEl = jointEl.querySelector(':scope > limit');
    const parent = jointEl.querySelector(':scope > parent')?.getAttribute('link');
    const child = jointEl.querySelector(':scope > child')?.getAttribute('link');

    const lowerAttr = limitEl?.getAttribute('lower');
    const upperAttr = limitEl?.getAttribute('upper');

    let lower = lowerAttr != null ? Number(lowerAttr) : null;
    let upper = upperAttr != null ? Number(upperAttr) : null;
    let effectiveType = type;

    // SolidWorks often exports revolute with lower==upper (locked). Treat as free.
    if (type === 'revolute' && lower != null && upper != null && lower === upper) {
      effectiveType = 'continuous';
      lower = null;
      upper = null;
    }

    joints[name] = {
      name,
      type: effectiveType,
      urdfType: type,
      parent,
      child,
      xyz: parseVec(originEl?.getAttribute('xyz')),
      rpy: parseVec(originEl?.getAttribute('rpy')),
      axis: parseVec(axisEl?.getAttribute('xyz'), [0, 0, 1]),
      lower,
      upper,
      effort: limitEl ? Number(limitEl.getAttribute('effort') || 0) : 0,
      velocity: limitEl ? Number(limitEl.getAttribute('velocity') || 0) : 0,
    };
  }

  return { name: robot.getAttribute('name'), links, joints };
}

function findRootLink(links, joints) {
  const children = new Set(Object.values(joints).map((j) => j.child));
  const roots = Object.keys(links).filter((n) => !children.has(n));
  if (roots.length === 0) throw new Error('no root link');
  return roots[0];
}

// --- scene ---
const viewport = document.getElementById('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1115);

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 50);
camera.up.set(0, 0, 1);
camera.position.set(0.55, -0.55, 0.35);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewport.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.08);
controls.enableDamping = true;
controls.update();

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dir = new THREE.DirectionalLight(0xffffff, 0.85);
dir.position.set(1.5, -1.2, 2.5);
scene.add(dir);
const fill = new THREE.DirectionalLight(0x8899bb, 0.35);
fill.position.set(-1.5, 1.0, 0.5);
scene.add(fill);

const grid = new THREE.GridHelper(1.0, 20, 0x3a4558, 0x232936);
grid.rotation.x = Math.PI / 2; // XY plane, Z-up
scene.add(grid);

function resize() {
  const w = viewport.clientWidth;
  const h = viewport.clientHeight;
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
  renderer.setSize(w, h, false);
}
window.addEventListener('resize', resize);
resize();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

// --- robot build ---
const movable = {}; // name -> { joint, movableGroup, value }
const visualMeshes = [];
const collisionMeshes = [];
const frameMarkers = []; // AxesHelper + labels (all link frames)
let robotRoot = null;

function makeFrameLabel(text) {
  const pad = 6;
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  ctx.font = '600 28px ui-sans-serif, system-ui, sans-serif';
  const tw = Math.ceil(ctx.measureText(text).width);
  canvas.width = tw + pad * 2;
  canvas.height = 36 + pad;
  ctx.font = '600 28px ui-sans-serif, system-ui, sans-serif';
  ctx.fillStyle = 'rgba(10, 12, 18, 0.72)';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#e8ecf5';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, pad, canvas.height / 2);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: tex,
      transparent: true,
      depthTest: false,
      depthWrite: false,
    }),
  );
  const h = 0.028;
  sprite.scale.set((canvas.width / canvas.height) * h, h, 1);
  sprite.position.set(0.012, 0.012, 0.055);
  sprite.visible = false;
  return sprite;
}

function makeMaterial(rgba, wireframe = false) {
  const [r, g, b, a] = rgba;
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(r, g, b),
    metalness: 0.15,
    roughness: 0.65,
    transparent: a < 1,
    opacity: a,
    side: THREE.DoubleSide,
    wireframe,
  });
}

async function loadStl(url, onProgress) {
  const loader = new STLLoader();
  return new Promise((resolve, reject) => {
    loader.load(
      url,
      (geom) => {
        geom.computeVertexNormals();
        resolve(geom);
      },
      (ev) => {
        if (onProgress && ev.total) onProgress(ev.loaded / ev.total, ev.loaded, ev.total);
      },
      reject,
    );
  });
}

function isContinuousLike(joint) {
  return joint.type === 'continuous' || joint.lower == null || joint.upper == null;
}

function setJointValue(name, rad) {
  const m = movable[name];
  if (!m) return;
  let v = rad;
  if (isContinuousLike(m.joint)) {
    v = ((v + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
  } else {
    v = Math.min(m.joint.upper, Math.max(m.joint.lower, v));
  }
  m.value = v;
  const axis = new THREE.Vector3(...m.joint.axis).normalize();
  if (axis.lengthSq() < 1e-12) axis.set(0, 0, 1);
  m.movableGroup.quaternion.setFromAxisAngle(axis, v);
  if (m.ui) m.ui.sync(v);
}

function buildJointUI(joint) {
  const wrap = document.createElement('div');
  wrap.className = 'joint';

  const header = document.createElement('div');
  header.className = 'joint-header';
  const typeLabel = joint.urdfType && joint.urdfType !== joint.type
    ? `${joint.type}←${joint.urdfType}`
    : joint.type;
  header.innerHTML = `<span class="joint-name">${joint.name}</span><span class="joint-type">${typeLabel}</span>`;
  wrap.appendChild(header);

  const vals = document.createElement('div');
  vals.className = 'joint-vals';
  const degInput = document.createElement('input');
  degInput.type = 'number';
  degInput.step = '0.1';
  const radSpan = document.createElement('span');
  vals.appendChild(degInput);
  vals.appendChild(Object.assign(document.createElement('span'), { className: 'unit', textContent: 'deg' }));
  vals.appendChild(radSpan);
  vals.appendChild(Object.assign(document.createElement('span'), { className: 'unit', textContent: 'rad' }));
  wrap.appendChild(vals);

  const continuous = isContinuousLike(joint);
  const lower = continuous ? -Math.PI : joint.lower;
  const upper = continuous ? Math.PI : joint.upper;

  const slider = document.createElement('input');
  slider.type = 'range';
  slider.min = String(lower * RAD2DEG);
  slider.max = String(upper * RAD2DEG);
  slider.step = '0.1';
  slider.value = '0';
  wrap.appendChild(slider);

  degInput.min = slider.min;
  degInput.max = slider.max;

  const sync = (rad) => {
    const deg = rad * RAD2DEG;
    slider.value = String(deg);
    degInput.value = deg.toFixed(1);
    radSpan.textContent = rad.toFixed(4);
  };
  sync(0);

  const applyDeg = (deg) => {
    let d = Number(deg);
    if (Number.isNaN(d)) return;
    if (continuous) {
      d = ((d + 180) % 360 + 360) % 360 - 180;
    } else {
      d = Math.min(Number(slider.max), Math.max(Number(slider.min), d));
    }
    setJointValue(joint.name, d * DEG2RAD);
  };

  slider.addEventListener('input', () => applyDeg(slider.value));
  degInput.addEventListener('change', () => applyDeg(degInput.value));

  jointsEl.appendChild(wrap);
  return { sync, lower, upper };
}

async function buildRobot(model) {
  const { links, joints } = model;
  const rootName = findRootLink(links, joints);

  const linkGroups = {};
  for (const name of Object.keys(links)) {
    const g = new THREE.Group();
    g.name = name;
    linkGroups[name] = g;

    // one TF frame per link (RGB = XYZ)
    const axes = new THREE.AxesHelper(0.05);
    axes.visible = false;
    g.add(axes);
    const label = makeFrameLabel(name);
    g.add(label);
    frameMarkers.push(axes, label);
  }

  // joint frames: parentLink -> jointOriginGroup -> movableGroup -> childLink
  for (const joint of Object.values(joints)) {
    const parentG = linkGroups[joint.parent];
    const childG = linkGroups[joint.child];
    if (!parentG || !childG) {
      console.warn('missing parent/child for', joint.name);
      continue;
    }

    const originG = new THREE.Group();
    originG.name = `joint_origin_${joint.name}`;
    originG.position.set(...joint.xyz);
    originG.quaternion.copy(rpyToQuaternion(joint.rpy));
    parentG.add(originG);

    const movableG = new THREE.Group();
    movableG.name = `joint_move_${joint.name}`;
    originG.add(movableG);
    movableG.add(childG);

    if (joint.type !== 'fixed') {
      movable[joint.name] = {
        joint,
        movableGroup: movableG,
        value: 0,
        ui: null,
      };
    }
  }

  robotRoot = linkGroups[rootName];
  scene.add(robotRoot);

  // UI for movable joints (pan/tilt first, then wheels FL/FR/RL/RR)
  const order = [
    'pan_joint',
    'tilt_joint',
    'FL_joint',
    'FR_wheel_joint',
    'RL_joint',
    'RR_joint',
  ];
  const names = [
    ...order.filter((n) => movable[n]),
    ...Object.keys(movable).filter((n) => !order.includes(n)).sort(),
  ];
  for (const name of names) {
    movable[name].ui = buildJointUI(movable[name].joint);
  }

  // Collect geometry jobs: primitives first, STLs next, base_link last
  const jobs = [];
  for (const link of Object.values(links)) {
    for (const kind of ['visual', 'collision']) {
      const g = link[kind];
      if (!g || g.type === 'empty') continue;
      const sizeHint =
        link.name === 'base_link' && g.type === 'mesh'
          ? 1e9
          : g.type === 'mesh'
            ? 1
            : 0;
      jobs.push({ linkName: link.name, kind, geom: g, sizeHint });
    }
  }
  jobs.sort((a, b) => a.sizeHint - b.sizeHint);

  function makeThreeGeometry(g) {
    if (g.type === 'box') {
      const [sx, sy, sz] = g.size;
      return new THREE.BoxGeometry(sx, sy, sz);
    }
    if (g.type === 'cylinder') {
      // URDF cylinder axis = Z; Three CylinderGeometry axis = Y → rotate
      const cyl = new THREE.CylinderGeometry(g.radius, g.radius, g.length, 24);
      cyl.rotateX(Math.PI / 2);
      return cyl;
    }
    if (g.type === 'sphere') {
      return new THREE.SphereGeometry(g.radius, 16, 12);
    }
    return null;
  }

  progressEl.classList.add('visible');
  const totalJobs = jobs.length;
  let doneJobs = 0;

  for (const job of jobs) {
    progressLabel.textContent = `Loading ${job.kind}: ${job.linkName}`;
    setStatus(`loading ${job.linkName} (${job.kind})…`);

    let threeGeom;
    if (job.geom.type === 'mesh') {
      const url = resolveMeshUrl(job.geom.filename);
      threeGeom = await loadStl(url, (frac) => {
        const overall = (doneJobs + frac) / totalJobs;
        progressFill.style.width = `${(overall * 100).toFixed(1)}%`;
      });
    } else {
      threeGeom = makeThreeGeometry(job.geom);
    }

    const rgba =
      job.kind === 'collision'
        ? [1, 0.35, 0.2, 0.35]
        : job.geom.rgba;

    const mat = makeMaterial(rgba, false);
    const mesh = new THREE.Mesh(threeGeom, mat);
    mesh.position.set(...job.geom.xyz);
    mesh.quaternion.copy(rpyToQuaternion(job.geom.rpy));
    mesh.name = `${job.linkName}_${job.kind}`;
    mesh.visible = job.kind === 'visual';
    linkGroups[job.linkName].add(mesh);

    if (job.kind === 'visual') visualMeshes.push(mesh);
    else collisionMeshes.push(mesh);

    doneJobs += 1;
    progressFill.style.width = `${((doneJobs / totalJobs) * 100).toFixed(1)}%`;
  }

  progressEl.classList.remove('visible');
  setStatus(`${Object.keys(links).length} links · ${names.length} movable joints`);
}

// --- controls ---
document.getElementById('btn-reset').addEventListener('click', () => {
  for (const name of Object.keys(movable)) setJointValue(name, 0);
});

document.getElementById('btn-home').addEventListener('click', () => {
  for (const name of Object.keys(movable)) setJointValue(name, 0);
  if (movable.tilt_joint) setJointValue('tilt_joint', -Math.PI / 4);
});

document.getElementById('tog-frames').addEventListener('change', (e) => {
  for (const m of frameMarkers) m.visible = e.target.checked;
});

document.getElementById('tog-wire').addEventListener('change', (e) => {
  for (const m of visualMeshes) m.material.wireframe = e.target.checked;
});

document.getElementById('tog-grid').addEventListener('change', (e) => {
  grid.visible = e.target.checked;
});

document.getElementById('tog-collision').addEventListener('change', (e) => {
  const showCol = e.target.checked;
  for (const m of collisionMeshes) m.visible = showCol;
  for (const m of visualMeshes) m.visible = !showCol || true; // keep visuals
  if (showCol) {
    for (const m of visualMeshes) m.material.opacity = 0.35;
    for (const m of visualMeshes) m.material.transparent = true;
  } else {
    for (const m of visualMeshes) {
      m.material.opacity = 1;
      m.material.transparent = false;
      m.visible = true;
    }
  }
});

// --- boot ---
(async () => {
  try {
    setStatus('fetching URDF…');
    const res = await fetch(URDF_URL);
    if (!res.ok) throw new Error(`URDF HTTP ${res.status}`);
    const xml = await res.text();
    const model = parseUrdf(xml);
    setStatus(`building ${model.name}…`);
    await buildRobot(model);
  } catch (err) {
    console.error(err);
    setStatus(`error: ${err.message}`);
    progressEl.classList.remove('visible');
  }
})();
