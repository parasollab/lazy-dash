// three.js view shared by the scene and result viewers: Z-up world, orbit
// camera, theme-aware colours, picking, obstacle / block / goal / region
// builders and the UR5e + Hand-e model from the repository URDF.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";
import { cssVar, repoUrl } from "./util.js";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export const URDF = { mesh: "resources/ur5e_hande/ur5e_hande.urdf", spheres: "resources/ur5e_hande/ur5e_hande_spherized.urdf" };
export const GRIPPER_LINK = "robotiq_hande_base_link"; // carries the objects (Ur5eModel.h)
export const JOINTS = ["ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint",
  "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"];
// Block colours, as in viewer/render_video.py (red, green, blue, white by index).
export const BLOCK_COLORS = [0xd91f1a, 0x1ab333, 0x1f40cc, 0xebebe6];

export function createView(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  container.append(renderer.domElement);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.02, 200);
  camera.position.set(3, -4, 3);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.15;
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8478, 2.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(2, -3, 6);
  scene.add(sun);
  const head = new THREE.DirectionalLight(0xffffff, 0.6);
  camera.add(head);
  scene.add(camera);

  let dirty = true;
  const before = [];
  const view = {
    scene, camera, renderer, controls,
    requestRender() { dirty = true; },
    onFrame(fn) { before.push(fn); },
    themed: [], // [material, css token] pairs refreshed on theme change
    theme(material, token, opacity) { this.themed.push([material, token]); material.color.set(cssVar(token) || "#888"); if (opacity != null) { material.transparent = opacity < 1; material.opacity = opacity; } return material; },
    cache: new Map(),
    /** A themed material made once per key. */
    material(key, make, token, opacity) {
      if (!this.cache.has(key)) this.cache.set(key, this.theme(make(), token, opacity));
      return this.cache.get(key);
    },
  };
  const applyTheme = () => {
    scene.background = new THREE.Color(cssVar("--scene-bg") || "#f3f4f2");
    for (const [m, token] of view.themed) m.color.set(cssVar(token) || "#888");
    dirty = true;
  };
  addEventListener("themechange", applyTheme);
  applyTheme();
  controls.addEventListener("change", () => { dirty = true; });

  const resize = () => {
    const w = container.clientWidth || 1, hgt = container.clientHeight || 1;
    renderer.setSize(w, hgt, false);
    camera.aspect = w / hgt;
    camera.updateProjectionMatrix();
    dirty = true;
  };
  new ResizeObserver(resize).observe(container);
  resize();

  (function loop(now) {
    for (const fn of before) if (fn(now)) dirty = true;
    if (controls.update()) dirty = true;
    if (dirty) { renderer.render(scene, camera); dirty = false; }
    requestAnimationFrame(loop);
  })(performance.now());

  /** Fit the camera to points ([x, y, z] arrays), keeping the view direction. */
  view.frame = (points, dir = [1, -1.4, 1.1]) => {
    if (!points.length) return;
    const box = new THREE.Box3().setFromPoints(points.map(p => new THREE.Vector3(...p)));
    const center = box.getCenter(new THREE.Vector3()), r = Math.max(0.4, box.getSize(new THREE.Vector3()).length() / 2);
    const d = new THREE.Vector3(...dir).normalize();
    const dist = r / Math.sin(THREE.MathUtils.degToRad(camera.fov) / 2) * 0.9;
    controls.target.copy(center);
    camera.position.copy(center).addScaledVector(d, dist);
    camera.near = Math.max(0.01, dist / 200);
    camera.far = dist * 20;
    camera.updateProjectionMatrix();
    dirty = true;
  };
  view.focus = (p) => {
    const off = camera.position.clone().sub(controls.target);
    controls.target.set(...p);
    camera.position.copy(controls.target).add(off);
    dirty = true;
  };

  /** Pointer picking: objects carry userData.pick = {label, ref}. */
  const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
  view.pickAt = (ev) => {
    const b = renderer.domElement.getBoundingClientRect();
    ndc.set(((ev.clientX - b.left) / b.width) * 2 - 1, -((ev.clientY - b.top) / b.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
    for (const hit of ray.intersectObjects(scene.children, true)) {
      let o = hit.object;
      if (!o.visible || o.userData.nopick) continue;
      while (o && !o.userData.pick) o = o.parent;
      if (o && isVisible(o)) return o.userData.pick;
    }
    return null;
  };
  return view;
}

function isVisible(o) {
  for (; o; o = o.parent) if (!o.visible) return false;
  return true;
}

export const vec = a => new THREE.Vector3(a[0], a[1], a[2]);

/** URDF-style rpy: R = Rz(yaw) Ry(pitch) Rx(roll). */
export function rpyQuaternion(rpy = [0, 0, 0]) {
  return new THREE.Quaternion().setFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], "ZYX"));
}

/** Obstacles in either format: scene ({center, axis, radius, half_height}) or result ({pose, geometry}). */
export function normalizeObstacle(o) {
  if (o.pose) return { type: o.type, id: o.id, center: o.pose.position, axis: o.pose.axis || [0, 0, 1],
    radius: o.geometry?.radius, half_height: o.geometry?.half_height };
  return { type: o.type, id: o.id, center: o.center, axis: o.axis || [0, 0, 1], radius: o.radius, half_height: o.half_height };
}

const Y = new THREE.Vector3(0, 1, 0);
/** Obstacle primitives (spheres, cylinders); upright thin ones see-through like walls. */
const slabMat = view => view.material("slab", () => new THREE.MeshStandardMaterial({ roughness: 0.9 }), "--obstacle");
const wallMat = view => view.material("wall", () => new THREE.MeshStandardMaterial({ roughness: 0.9, depthWrite: false }), "--wall", 0.55);

export function obstacleGroup(view, obstacles) {
  const g = new THREE.Group();
  const slab = slabMat(view), wall = wallMat(view);
  obstacles.forEach((raw, i) => {
    const o = normalizeObstacle(raw);
    let mesh;
    if (o.type === "sphere") mesh = new THREE.Mesh(new THREE.SphereGeometry(o.radius, 24, 16), slab);
    else if (o.type === "cylinder") {
      const upright = Math.abs(o.axis[2]) > 0.99, flat = 2 * o.half_height < o.radius;
      mesh = new THREE.Mesh(new THREE.CylinderGeometry(o.radius, o.radius, 2 * o.half_height, o.radius > 1 ? 96 : 28),
        upright && flat ? slab : wall);
      mesh.quaternion.setFromUnitVectors(Y, vec(o.axis).normalize());
    } else return;
    mesh.position.copy(vec(o.center));
    mesh.userData.pick = { label: `obstacle ${o.id ?? i} (${o.type}, r ${o.radius})`, ref: { kind: "obstacle", index: i } };
    g.add(mesh);
  });
  return g;
}

/** The scenery boxes a scene was approximated from (lazydash.approximations), plus the
 *  scene obstacles no box covers (e.g. a pillar or a wall given directly as cylinders). */
export function boxGroup(view, approximations, obstacles = []) {
  const g = new THREE.Group();
  const slab = slabMat(view), wall = wallMat(view);
  const boxes = (approximations || []).filter(a => a.box);
  for (const a of boxes) {
    const lo = vec(a.box.min), hi = vec(a.box.max), size = hi.clone().sub(lo);
    const thin = Math.min(size.x, size.y, size.z) < 0.1 && size.z > 0.2;
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(size.x, size.y, size.z), thin ? wall : slab);
    mesh.position.copy(lo.add(hi).multiplyScalar(0.5));
    mesh.userData.pick = { label: `${a.label} (box, ${a.primitives} primitives)` };
    g.add(mesh);
  }
  const covered = id => boxes.some(a => id === a.label || String(id).startsWith(`${a.label}_`));
  g.add(obstacleGroup(view, obstacles.filter(o => !covered(o.id))));
  return g;
}

export function boundsLines(view, env) {
  const box = new THREE.Box3(vec(env.min), vec(env.max));
  const helper = new THREE.Box3Helper(box);
  helper.material = view.material("bounds", () => new THREE.LineBasicMaterial(), "--faint");
  helper.userData.nopick = true;
  return helper;
}

const blockMats = new Map();
export function blockMaterial(i) {
  const c = BLOCK_COLORS[i % BLOCK_COLORS.length];
  if (!blockMats.has(c)) blockMats.set(c, new THREE.MeshStandardMaterial({ color: c, roughness: 0.6 }));
  return blockMats.get(c);
}
export function blockMesh(i, size) {
  return new THREE.Mesh(new THREE.BoxGeometry(...size), blockMaterial(i));
}

/** A goal alternative: dashed edges in the block's colour over a faint fill. */
const goalMats = new Map();
export function goalMesh(i, size) {
  const c = BLOCK_COLORS[i % BLOCK_COLORS.length];
  if (!goalMats.has(c)) goalMats.set(c, [
    new THREE.MeshBasicMaterial({ color: c, transparent: true, opacity: 0.16, depthWrite: false }),
    new THREE.LineDashedMaterial({ color: c === 0xebebe6 ? 0x9a9a94 : c, dashSize: 0.006, gapSize: 0.004 })]);
  const [fillMat, edgeMat] = goalMats.get(c);
  const geo = new THREE.BoxGeometry(...size);
  const fill = new THREE.Mesh(geo, fillMat);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), edgeMat);
  edges.computeLineDistances();
  edges.userData.nopick = true;
  fill.add(edges);
  return fill;
}

export function regionMesh(view, r) {
  const lo = vec(r.min), hi = vec(r.max), size = hi.clone().sub(lo), c = lo.clone().add(hi).multiplyScalar(0.5);
  const mat = view.material("region", () => new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, depthWrite: false }), "--s3", 0.22);
  let mesh;
  if (size.z < 1e-6) {
    mesh = new THREE.Mesh(new THREE.PlaneGeometry(Math.max(size.x, 1e-3), Math.max(size.y, 1e-3)), mat);
    c.z += 0.0015;
  } else mesh = new THREE.Mesh(new THREE.BoxGeometry(size.x, size.y, size.z), mat);
  mesh.position.copy(c);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry), view.material("regionEdge", () => new THREE.LineBasicMaterial(), "--s3"));
  edges.userData.nopick = true;
  mesh.add(edges);
  return mesh;
}

/* ---------- arms ---------- */
const templates = {};
/** The arm model, loaded once per kind ("mesh" or "spheres"); resolves after every mesh loaded. */
export function armTemplate(kind) {
  templates[kind] ??= new Promise((resolve, reject) => {
    const manager = new THREE.LoadingManager();
    const failed = [];
    let robot;
    manager.onError = url => failed.push(url);
    manager.onLoad = () => (robot ? resolve(Object.assign(robot, { failedMeshes: failed })) : reject(new Error("URDF did not load")));
    const loader = new URDFLoader(manager);
    loader.parseVisual = kind === "mesh";
    loader.parseCollision = kind === "spheres";
    loader.load(repoUrl(URDF[kind]), r => { robot = r; }, null, e => reject(e));
  });
  return templates[kind];
}

/** A copy of the arm template at a base pose; spheres get a themed see-through material. */
export function makeArm(view, template, kind, base, label) {
  const arm = template.clone();
  if (kind === "spheres") {
    const mat = view.material("armSpheres", () => new THREE.MeshStandardMaterial({ roughness: 0.7 }), "--layer-2", 0.85);
    arm.traverse(c => {
      if (c.isURDFCollider) c.visible = true;
      if (c.isMesh) c.material = mat;
    });
  }
  arm.position.copy(vec(base.position));
  const q = base.quaternion_xyzw || [0, 0, 0, 1];
  arm.quaternion.set(q[0], q[1], q[2], q[3]);
  arm.userData.pick = { label, ref: { kind: "robot" } };
  return arm;
}

/** Remove `group`'s children, freeing their geometry (arm copies share the template's and are kept). */
export function clearGroup(group) {
  for (const c of group.children) if (!c.isURDFRobot) c.traverse(o => o.geometry?.dispose());
  group.clear();
}

export function setJoints(arm, names, q) {
  for (let i = 0; i < names.length && i < q.length; i++) arm.joints[names[i]]?.setJointValue(q[i]);
}

/** Highlight marker: a wireframe sphere (and optionally a reach sphere). */
export function marker(view) {
  const g = new THREE.Group();
  const ring = new THREE.Mesh(new THREE.SphereGeometry(1, 24, 16),
    view.theme(new THREE.MeshBasicMaterial({ wireframe: true, depthTest: false }), "--accent", 0.9));
  ring.renderOrder = 10;
  const reach = new THREE.Mesh(new THREE.SphereGeometry(1, 48, 24),
    view.theme(new THREE.MeshBasicMaterial({ wireframe: true, depthWrite: false }), "--accent", 0.18));
  const link = new THREE.Line(new THREE.BufferGeometry(), view.theme(new THREE.LineBasicMaterial({ depthTest: false }), "--accent"));
  link.renderOrder = 10;
  g.add(ring, reach, link);
  g.traverse(o => { o.userData.nopick = true; });
  g.visible = false;
  return {
    group: g,
    show(p, r, reachRadius, linkTo) {
      g.visible = true;
      ring.position.set(...p);
      ring.scale.setScalar(r);
      reach.visible = !!reachRadius;
      if (reachRadius) { reach.position.set(...p); reach.scale.setScalar(reachRadius); }
      link.visible = !!linkTo?.length;
      if (linkTo?.length) link.geometry.setFromPoints([vec(p), ...linkTo.flatMap(q => [vec(q), vec(p)])]);
      view.requestRender();
    },
    hide() { g.visible = false; view.requestRender(); },
  };
}
