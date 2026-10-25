// Result viewer: play back a lazydash_solve result (arms and blocks over
// time) with the arm timeline synced to the playhead.
import * as THREE from "three";
import { $, h, fmt, flash, initTheme, fileInput, dropFiles, fileSelect, fetchText, listFiles, query, setQuery,
  splitter, showTip, hideTip, clamp } from "./common/util.js";
import { createView, obstacleGroup, boxGroup, blockMesh, goalMesh, armTemplate, makeArm, setJoints, rpyQuaternion,
  vec, clearGroup, GRIPPER_LINK } from "./common/view3d.js";
import { createGantt } from "./common/gantt.js";

initTheme($("theme"));
const view = createView($("view"));
const content = new THREE.Group(), scenery = new THREE.Group();
view.scene.add(content, scenery);
const trailFull = new THREE.Line(new THREE.BufferGeometry(),
  view.theme(new THREE.LineBasicMaterial({ depthTest: false }), "--accent", 0.3));
const trailPast = new THREE.Line(new THREE.BufferGeometry(), view.theme(new THREE.LineBasicMaterial({ depthTest: false }), "--accent"));
for (const l of [trailFull, trailPast]) { l.renderOrder = 5; l.userData.nopick = true; l.visible = false; view.scene.add(l); }

const LAYERS = [
  ["task_query_time", "1 task query", "var(--layer-1)"],
  ["task_conflict_time", "2 task conflicts", "var(--layer-2)"],
  ["motion_query_time", "3 motion query", "var(--layer-3)"],
  ["conflict_resolution_time", "4 conflict resolution", "var(--layer-4)"],
];
const opts = { model: "mesh", scenery: "boxes" };
let R = null;            // the loaded result, prepared
let gen = 0, time = 0, playing = false, speed = 1, gantt = null, selected = -1;

/* ---------- loading ---------- */
async function openText(text, src) {
  let doc;
  try { doc = JSON.parse(text); } catch (e) { return flash(`Not JSON: ${e.message}`); }
  if (!Array.isArray(doc.robots) || !doc.lazydash) return flash("Not a Lazy-DaSH result (needs robots and lazydash).");
  $("file").textContent = src.path || src.name;
  $("file").title = src.path || src.name;
  setQuery("file", src.path || null);
  $("hint").hidden = true;
  await prepare(doc);
}
async function openPath(path) {
  try { await openText(await fetchText(path), { path }); }
  catch (e) { flash(`Could not open ${path}: ${e.message}`); }
}
fileInput($("open"), ".json,application/json", async f => openText(await f.text(), { name: f.name }));
dropFiles(async f => openText(await f.text(), { name: f.name }));

/** The scene file the result was solved from, when the repository has it (goals, start orientations, scenery boxes). */
async function findScene(name) {
  if (!name) return null;
  const hit = (await listFiles("resources/scenes")).find(f => f.path.endsWith(`/${name}.json`));
  if (!hit) return null;
  try { return JSON.parse(await fetchText(hit.path)); } catch (e) { return null; }
}

/** Waypoint times: every `stride` steps, the last one at the makespan (viewer/render_video.py times_of). */
function timesOf(n, stride, makespan) {
  const t = Array.from({ length: n }, (_, i) => i * stride);
  if (n > 1 && t[n - 1] !== makespan && makespan > t[n - 2]) t[n - 1] = makespan;
  return t;
}
/** Linear interpolation of a waypoint list at timestep t. */
function sample(path, times, t, out) {
  const n = path.length;
  if (n === 1 || t <= times[0]) return copy(out, path[0]);
  if (t >= times[n - 1]) return copy(out, path[n - 1]);
  let k = Math.min(n - 2, Math.floor(t / (times[1] - times[0])));
  while (k > 0 && times[k] > t) k--;
  while (k < n - 2 && times[k + 1] < t) k++;
  const a = path[k], b = path[k + 1], f = (t - times[k]) / (times[k + 1] - times[k] || 1);
  for (let i = 0; i < a.length; i++) out[i] = a[i] + (b[i] - a[i]) * f;
  return out;
}
function copy(out, a) { for (let i = 0; i < a.length; i++) out[i] = a[i]; return out; }

async function prepare(doc) {
  const my = ++gen;
  pause();
  const ld = doc.lazydash, stats = ld.stats || {};
  const stride = Math.max(1, ld.stride || 1), res = doc.resolution || 128;
  const armsRaw = doc.robots.filter(r => r.robot_type !== "sphere");
  const blocksRaw = doc.robots.filter(r => r.robot_type === "sphere");
  const lastIdx = Math.max(...doc.robots.map(r => (r.path?.length || 1) - 1));
  const makespan = stats.makespan > 0 ? stats.makespan : lastIdx * stride;
  const scene = await findScene(doc.scene);
  if (my !== gen) return;
  R = {
    doc, scene, stats, stride, res, makespan,
    arms: armsRaw.map(r => ({ raw: r, names: r.joint_names, path: r.path, times: timesOf(r.path.length, stride, makespan), q: new Array(r.joint_names.length).fill(0) })),
    blocks: blocksRaw.map((r, i) => ({ raw: r, path: r.path, times: timesOf(r.path.length, stride, makespan), p: [0, 0, 0],
      rpy: scene?.lazydash?.objects?.[i]?.start?.rpy, size: scene?.lazydash?.objects?.[i]?.shape?.size })),
    timeline: (ld.timeline || []).map(e => ({ ...e, start: e.start / res, end: e.end / res })),
    events: [...new Set((ld.timeline || []).flatMap(e => [e.start, e.end]))].sort((a, b) => a - b),
  };
  selected = -1;
  time = 0;
  renderStats();
  buildGantt();
  const slider = $("slider");
  slider.max = makespan;
  slider.step = 1;
  for (const id of ["play", "prev", "next", "slider"]) $(id).disabled = false;
  await buildScene(true);
}

async function buildScene(frame) {
  if (!R) return;
  const my = gen;
  clearGroup(content);
  buildScenery();
  const objs = R.scene?.lazydash?.objects || [];
  R.blocks.forEach((b, i) => {
    const size = b.size || Array(3).fill(2 * (b.raw.robot_radius || 0.033) / Math.sqrt(3));
    b.mesh = blockMesh(i, size);
    b.mesh.userData.pick = { label: b.raw.name };
    content.add(b.mesh);
    for (const [k, g] of (objs[i]?.goals || []).entries()) {
      const m = goalMesh(i, size);
      m.position.copy(vec(g.position));
      m.quaternion.copy(rpyQuaternion(g.rpy));
      m.userData.pick = { label: `${objs[i].name} goal ${k}` };
      content.add(m);
    }
  });
  if (frame) {
    const pts = [];
    for (const a of R.arms) {
      const p = a.raw.base_pose.position;
      for (const dx of [-0.6, 0.6]) for (const dy of [-0.6, 0.6]) pts.push([p[0] + dx, p[1] + dy, p[2]], [p[0] + dx, p[1] + dy, p[2] + 0.9]);
    }
    for (const b of R.blocks) pts.push(b.path[0], b.path[b.path.length - 1]);
    view.frame(pts);
  }
  let template;
  try { template = await armTemplate(opts.model); }
  catch (e) { return flash(`Arm model did not load: ${e.message}`); }
  if (my !== gen) return;
  if (template.failedMeshes?.length) flash(`${template.failedMeshes.length} arm meshes did not load.`);
  R.arms.forEach((a, i) => {
    a.model = makeArm(view, template, opts.model, a.raw.base_pose, `${a.raw.name} (r${i})`);
    a.model.userData.pick.ref.index = i;
    a.grip = a.model.links[GRIPPER_LINK];
    a.flange = a.model.links.flange;
    content.add(a.model);
  });
  holdSegments();
  setTrail(selected);
  seek(time);
}

function buildScenery() {
  clearGroup(scenery);
  const hasBoxes = !!R.scene?.lazydash?.approximations?.some(a => a?.box);
  $("scenery").querySelector('[data-v="boxes"]').disabled = !hasBoxes;
  const mode = opts.scenery === "boxes" && !hasBoxes ? "primitives" : opts.scenery;
  if (mode === "primitives") scenery.add(obstacleGroup(view, R.doc.obstacles || []));
  else if (mode === "boxes") scenery.add(boxGroup(view, R.scene.lazydash.approximations, R.scene.obstacles || []));
  view.requestRender();
}

/* ---------- carried block orientation ----------
   A held block keeps its pose relative to the Hand-e base link from the grasp. */
function poseArms(t) {
  for (const a of R.arms) {
    if (!a.model) continue;
    setJoints(a.model, a.names, sample(a.path, a.times, t, a.q));
    a.model.updateMatrixWorld(true);
  }
}
function holdSegments() {
  const q = new THREE.Quaternion();
  R.blocks.forEach((b, i) => {
    const held = R.timeline.filter(e => e.object === i && e.holding).sort((x, y) => x.start - y.start);
    const segs = [];
    for (const e of held) {
      const last = segs[segs.length - 1];
      if (last && last.robot === e.robot) last.t1 = Math.max(last.t1, e.end * R.res); // the same hand keeps it
      else segs.push({ robot: e.robot, t0: e.start * R.res, t1: e.end * R.res });
    }
    let w = rpyQuaternion(b.rpy);
    b.rest0 = w.clone();
    for (const s of segs) {
      const arm = R.arms[s.robot];
      if (!arm?.grip) continue;
      poseArms(s.t0);
      s.rel = arm.grip.getWorldQuaternion(q).clone().invert().multiply(w);
      poseArms(s.t1);
      w = arm.grip.getWorldQuaternion(q).clone().multiply(s.rel);
      s.after = w.clone();
    }
    b.segs = segs.filter(s => s.rel);
  });
}
const tmpQ = new THREE.Quaternion();
function blockOrientation(b, t) {
  let rest = b.rest0;
  for (const s of b.segs || []) {
    if (t < s.t0) break;
    if (t <= s.t1) return R.arms[s.robot].grip.getWorldQuaternion(tmpQ).multiply(s.rel);
    rest = s.after;
  }
  return rest;
}

/* ---------- time ---------- */
function seek(t) {
  if (!R) return;
  time = clamp(t, 0, R.makespan);
  poseArms(time);
  for (const b of R.blocks) {
    if (!b.mesh) continue;
    b.mesh.position.fromArray(sample(b.path, b.times, time, b.p));
    b.mesh.quaternion.copy(blockOrientation(b, time));
  }
  if (trailPast.visible) trailPast.geometry.setDrawRange(0, Math.max(0, Math.round(time / R.makespan * (trailPast.userData.n - 1)) + 1));
  $("slider").value = time;
  $("clock").textContent = `${fmt(time / R.res)} / ${fmt(R.makespan / R.res)} s`;
  gantt?.setTime(time / R.res);
  view.requestRender();
}
let last = null;
view.onFrame(now => {
  if (!playing || !R) { last = null; return false; }
  if (last != null) {
    const t = time + (now - last) / 1000 * R.res * speed;
    if (t >= R.makespan) { seek(R.makespan); pause(); } else seek(t);
  }
  last = now;
  return true;
});
function play() {
  if (!R) return;
  if (time >= R.makespan) seek(0);
  playing = true;
  $("play").textContent = "Pause";
  $("play").setAttribute("aria-label", "Pause");
}
function pause() {
  playing = false;
  $("play").textContent = "Play";
  $("play").setAttribute("aria-label", "Play");
}
const toggle = () => (playing ? pause() : play());
function stepEvent(dir) {
  if (!R) return;
  const ev = R.events;
  const t = dir > 0 ? ev.find(x => x > time + 0.5) : [...ev].reverse().find(x => x < time - 0.5);
  seek(t ?? (dir > 0 ? R.makespan : 0));
}
$("play").addEventListener("click", toggle);
$("prev").addEventListener("click", () => stepEvent(-1));
$("next").addEventListener("click", () => stepEvent(1));
$("slider").addEventListener("input", e => seek(+e.target.value));
$("speed").addEventListener("change", e => { speed = +e.target.value; });
addEventListener("keydown", e => {
  if (!R || e.target.closest?.("input, select, textarea, button, [tabindex]")) return;
  if (e.key === " ") toggle();
  else if (e.key === "ArrowRight") seek(time + R.res * (e.shiftKey ? 10 : 1));
  else if (e.key === "ArrowLeft") seek(time - R.res * (e.shiftKey ? 10 : 1));
  else if (e.key === ".") stepEvent(1);
  else if (e.key === ",") stepEvent(-1);
  else return;
  e.preventDefault();
});

/* ---------- Gantt and stats ---------- */
function buildGantt() {
  gantt?.destroy();
  gantt = createGantt($("gantt"), R.timeline, {
    arms: R.arms.length, makespan: R.makespan / R.res,
    onSeek: t => seek(t * R.res),
    onSelectArm: a => select(a),
  });
}
function renderStats() {
  const s = R.stats, plan = R.doc.lazydash.task_plan || [];
  const steps = plan.filter(t => /^(pick|place|handover)\b/.test(t));
  const hand = steps.filter(t => t.startsWith("handover")).length;
  const total = LAYERS.reduce((a, [k]) => a + (s[k] || 0), 0) || 1;
  const box = $("stats");
  box.innerHTML = "";
  box.append(
    h("div", {}, h("p", { class: "scene-name" }, R.doc.scene || "result"),
      h("p", { class: "scene-meta" }, `${R.arms.length} arms, ${R.blocks.length} blocks, ${R.doc.success ? "solved" : `not solved${s.failure ? ": " + s.failure : ""}`}`)),
    h("div", { class: "tiles" },
      tile("Makespan", fmt(R.makespan / R.res), "s"),
      tile("Planning", fmt(s.planning_time, 2), "s"),
      tile("Transitions", steps.length, `${hand} handovers`),
      tile("Constraints", `${s.task_constraints ?? "-"}/${s.motion_constraints ?? "-"}`, "task/motion")),
    h("div", {}, h("span", { class: "k" }, "Planning time by layer"),
      h("div", { class: "lbar" }, ...LAYERS.map(([k, , c]) => h("span", { style: `flex:${(s[k] || 0) / total};background:${c}`, title: k }))),
      h("div", { class: "lkey" }, ...LAYERS.map(([k, label, c]) => h("span", {}, h("span", {}, h("i", { style: `background:${c}` }), label), h("b", {}, `${fmt(s[k], 2)} s`))))),
    h("div", {}, h("span", { class: "k" }, "Representations"),
      h("div", { class: "lkey" },
        row("H_T vertices, hyperarcs", `${s.ht_vertices ?? "-"}, ${s.ht_hyperarcs ?? "-"}`),
        row("H_M vertices, hyperarcs", `${s.hm_vertices ?? "-"}, ${s.hm_hyperarcs ?? "-"}`),
        row("Iterations outer, inner", `${s.outer_iterations ?? "-"}, ${s.inner_iterations ?? "-"}`),
        row("Conflicts h-h, h-v", `${s.hh_conflicts ?? "-"}, ${s.hv_conflicts ?? "-"}`),
        row("Move-outs", s.move_outs ?? "-"))),
  );
}
const tile = (k, v, unit) => h("div", { class: "tile" }, h("span", { class: "k" }, k), h("span", { class: "v" }, String(v), " ", h("small", {}, unit)));
const row = (k, v) => h("span", {}, h("span", {}, k), h("b", {}, String(v)));

/* ---------- arm selection and gripper trace ---------- */
function select(a) {
  selected = a;
  gantt?.setSelected(a);
  setTrail(a);
  $("selnote").textContent = a >= 0 ? `Tracing the gripper of ${R.arms[a].raw.name} (r${a}). Click empty space to clear.`
    : "Click an arm or a timeline row to trace its gripper.";
}
function setTrail(a) {
  const arm = R?.arms[a];
  trailFull.visible = trailPast.visible = !!arm?.flange;
  if (!arm?.flange) return view.requestRender();
  const n = Math.min(1500, Math.ceil(R.makespan / R.stride) + 1), pts = [];
  const tip = new THREE.Vector3();
  for (let k = 0; k < n; k++) {
    const t = R.makespan * k / (n - 1);
    setJoints(arm.model, arm.names, sample(arm.path, arm.times, t, arm.q));
    arm.model.updateMatrixWorld(true);
    pts.push(arm.flange.localToWorld(tip.set(0.15, 0, 0)).clone()); // the grasped object's centre (the paper's grasps)
  }
  for (const l of [trailFull, trailPast]) { l.geometry.dispose(); l.geometry = new THREE.BufferGeometry().setFromPoints(pts); }
  trailPast.userData.n = n;
  seek(time);
}

/* ---------- 3D hover and click ---------- */
const canvas = view.renderer.domElement;
let pending = null, downAt = null;
canvas.addEventListener("pointermove", ev => {
  if (ev.buttons) return hideTip();
  if (pending) { pending = ev; return; }
  pending = ev;
  requestAnimationFrame(() => {
    const e = pending; pending = null;
    const hit = view.pickAt(e);
    if (hit) showTip(hit.label, e.clientX, e.clientY); else hideTip();
  });
});
canvas.addEventListener("pointerleave", hideTip);
canvas.addEventListener("pointerdown", ev => { downAt = [ev.clientX, ev.clientY]; });
canvas.addEventListener("click", ev => {
  if (!R || !downAt || Math.hypot(ev.clientX - downAt[0], ev.clientY - downAt[1]) > 4) return;
  const hit = view.pickAt(ev);
  select(hit?.ref?.kind === "robot" ? hit.ref.index : -1);
});

/* ---------- toolbar and layout ---------- */
function seg(id, key, rebuild) {
  const box = $(id);
  box.addEventListener("click", e => {
    const b = e.target.closest("button");
    if (!b || b.disabled) return;
    opts[key] = b.dataset.v;
    for (const x of box.querySelectorAll("button")) x.setAttribute("aria-pressed", x === b);
    if (R) rebuild();
  });
}
seg("model", "model", () => buildScene(false));
seg("scenery", "scenery", buildScenery);
const app = $("app"), top = $("top");
splitter($("gv"), top, "x", (x, b) => top.style.setProperty("--side", clamp(b.width - x, 220, b.width - 240) + "px"));
splitter($("gh"), app, "y", (y, b) => app.style.setProperty("--gantt", clamp(b.height - y, 80, b.height - 200) + "px"));

const initial = query("file");
fileSelect($("pick"), "results", p => p.endsWith(".json") && !p.endsWith(".stats.json") && !p.endsWith("/results.json"), openPath, initial);
if (initial) openPath(initial);
