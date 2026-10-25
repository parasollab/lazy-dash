// Scene viewer and checker: render a lazydash.scene/1 file, edit its JSON with
// live re-render, watch the file on disk, run the checks in common/checks.js.
import * as THREE from "three";
import { $, h, debounce, flash, initTheme, launcher, fileInput, dropFiles, fileSelect, fetchText,
  saveScene, query, setQuery, splitter, showTip, hideTip, clamp } from "./common/util.js";
import { createView, obstacleGroup, boxGroup, boundsLines, blockMesh, goalMesh, regionMesh, armTemplate, makeArm,
  setJoints, rpyQuaternion, vec, marker, clearGroup, JOINTS } from "./common/view3d.js";
import { checkScene, DEFAULT_PARAMS } from "./common/checks.js";

initTheme($("theme"));
const view = createView($("view"));
const mark = marker(view);
view.scene.add(mark.group);
const content = new THREE.Group();
view.scene.add(content);

let doc = null, source = null, diskText = null, gen = 0, framed = false, selected = null, watchTimer = null;
const opts = { model: "mesh", scenery: "primitives" };
const editor = $("editor"), status = $("status");

/* ---------- loading ---------- */
const lf = text => text.replace(/\r\n?/g, "\n"); // a textarea holds LF only
async function openText(text, src) {
  text = lf(text);
  source = src;
  diskText = text;
  framed = false;
  selected = null;
  mark.hide();
  editor.value = text;
  $("file").textContent = src.path || src.name;
  $("file").title = src.path || src.name;
  $("hint").hidden = true;
  $("save").disabled = false;
  $("watch").disabled = !src.path;
  if (!src.path) setWatch(false);
  setQuery("file", src.path || null);
  update();
}
async function openPath(path) {
  try { await openText(await fetchText(path), { path }); }
  catch (e) { flash(`Could not open ${path}: ${e.message}`); }
}
fileInput($("open"), ".json,application/json", async f => openText(await f.text(), { name: f.name }));
dropFiles(async f => openText(await f.text(), { name: f.name }));

/* ---------- editing ---------- */
function parseError(e, text) {
  const m = e.message;
  let pos = null, line, col;
  const lc = m.match(/line (\d+) column (\d+)/);
  const p = m.match(/position (\d+)/);
  if (lc) { line = +lc[1]; col = +lc[2]; }
  if (p) pos = +p[1];
  if (pos != null && !lc) {
    const before = text.slice(0, pos).split("\n");
    line = before.length; col = before[before.length - 1].length + 1;
  }
  if (pos == null && lc) pos = text.split("\n").slice(0, line - 1).reduce((s, l) => s + l.length + 1, 0) + col - 1;
  const msg = m.replace(/^JSON\.parse: /, "").replace(/ in JSON at position \d+.*$/, "").replace(/ at line \d+ column \d+.*$/, "");
  return { pos, text: line ? `Line ${line}, column ${col}: ${msg}` : msg };
}
function update() {
  const text = editor.value;
  let parsed;
  try { parsed = JSON.parse(text); }
  catch (e) {
    const pe = parseError(e, text);
    status.textContent = pe.text;
    status.className = "bad";
    status.title = "Show in the editor";
    status.onclick = () => { if (pe.pos != null) select(pe.pos, pe.pos + 1); };
    return;
  }
  status.textContent = text === diskText ? (source?.path ? "Valid, as on disk" : "Valid") : "Valid, edited";
  status.className = "";
  status.title = "";
  status.onclick = null;
  doc = parsed;
  render();
}
editor.addEventListener("input", debounce(update, 300));
editor.addEventListener("keydown", e => {
  if (e.key === "Tab" && !e.shiftKey) { e.preventDefault(); editor.setRangeText("  ", editor.selectionStart, editor.selectionEnd, "end"); }
});
function select(from, to) {
  editor.focus();
  editor.setSelectionRange(from, to);
  const lineH = parseFloat(getComputedStyle(editor).lineHeight) || 18;
  const line = editor.value.slice(0, from).split("\n").length - 1;
  editor.scrollTop = Math.max(0, line * lineH - editor.clientHeight / 3);
}

/* ---------- rendering ---------- */
const num = v => typeof v === "number" && Number.isFinite(v);
const v3 = a => Array.isArray(a) && a.length === 3 && a.every(num);
async function render() {
  const my = ++gen;
  clearGroup(content);
  const d = doc;
  if (!d || typeof d !== "object") return runChecks();
  const ld = d.lazydash || {};
  const frame = [];
  if (v3(d.environment?.min) && v3(d.environment?.max)) content.add(boundsLines(view, d.environment));
  const obstacles = (Array.isArray(d.obstacles) ? d.obstacles : []).filter(o =>
    v3(o?.center) && num(o?.radius) && (o.type === "sphere" || (v3(o.axis) && num(o.half_height))));
  const hasBoxes = Array.isArray(ld.approximations) && ld.approximations.some(a => a?.box);
  $("scenery").querySelector('[data-v="boxes"]').disabled = !hasBoxes;
  const scenery = opts.scenery === "boxes" && !hasBoxes ? "primitives" : opts.scenery;
  if (scenery === "primitives") content.add(obstacleGroup(view, obstacles));
  else if (scenery === "boxes") content.add(boxGroup(view, ld.approximations, obstacles));
  (Array.isArray(ld.placement_regions) ? ld.placement_regions : []).forEach((r, i) => {
    if (!v3(r?.min) || !v3(r?.max)) return;
    const m = regionMesh(view, r);
    m.userData.pick = { label: `placement region ${i}${r.object != null ? ` (object ${r.object})` : ""}`, ref: { kind: "region", index: i } };
    content.add(m);
    frame.push(r.min, r.max);
  });
  (Array.isArray(ld.objects) ? ld.objects : []).forEach((o, i) => {
    const size = v3(o?.shape?.size) ? o.shape.size : [0.038, 0.038, 0.038];
    if (v3(o?.start?.position)) {
      const m = blockMesh(i, size);
      m.position.copy(vec(o.start.position));
      m.quaternion.copy(rpyQuaternion(v3(o.start.rpy) ? o.start.rpy : undefined));
      m.userData.pick = { label: `${o.name} start`, ref: { kind: "object", index: i } };
      content.add(m);
      frame.push(o.start.position);
    }
    (Array.isArray(o?.goals) ? o.goals : []).forEach((g, k) => {
      if (!v3(g?.position)) return;
      const m = goalMesh(i, size);
      m.position.copy(vec(g.position));
      m.quaternion.copy(rpyQuaternion(v3(g.rpy) ? g.rpy : undefined));
      m.userData.pick = { label: `${o.name} goal ${k}`, ref: { kind: "goal", index: i, goal: k } };
      content.add(m);
      frame.push(g.position);
    });
  });
  const robots = (Array.isArray(d.robots) ? d.robots : []).filter(r => v3(r?.base?.position));
  for (const r of robots) {
    const b = r.base.position;
    for (const dx of [-0.6, 0.6]) for (const dy of [-0.6, 0.6]) frame.push([b[0] + dx, b[1] + dy, b[2]], [b[0] + dx, b[1] + dy, b[2] + 0.9]);
  }
  if (!framed && frame.length) { view.frame(frame); framed = true; }
  runChecks();
  view.requestRender();
  if (!robots.length) return;
  let template;
  try { template = await armTemplate(opts.model); }
  catch (e) { return flash(`Arm model did not load: ${e.message}`); }
  if (my !== gen) return;
  if (template.failedMeshes?.length) flash(`${template.failedMeshes.length} arm meshes did not load.`);
  (Array.isArray(d.robots) ? d.robots : []).forEach((r, i) => {
    if (!v3(r?.base?.position)) return;
    const q = Array.isArray(r.base.quaternion_xyzw) && r.base.quaternion_xyzw.length === 4 ? r.base.quaternion_xyzw : undefined;
    const arm = makeArm(view, template, opts.model, { position: r.base.position, quaternion_xyzw: q },
      `${r.name ?? "robot " + i} (base ${r.base.position.map(v => +v.toFixed(3)).join(", ")})`);
    arm.userData.pick.ref.index = i;
    if (Array.isArray(r.start)) setJoints(arm, JOINTS, r.start.map(Number));
    content.add(arm);
  });
  view.requestRender();
  if (selected) highlight(selected.ref, selected.pos, false);
}

/* ---------- checks and highlighting ---------- */
function runChecks() {
  const list = $("checks");
  const found = doc ? checkScene(doc) : [];
  const errors = found.filter(f => f.severity === "error").length, warnings = found.length - errors;
  $("summary").textContent = doc ? (found.length ? `${errors} errors, ${warnings} warnings` : "") : "";
  list.replaceChildren(...(found.length ? found.map(f => h("li", {}, h("button", {
    type: "button", title: f.ref ? "Show in the view and the editor" : null,
    "aria-current": selected && same(selected.ref, f.ref) ? "true" : null,
    onclick: ev => { highlight(f.ref, f.pos, true); markCurrent(ev.currentTarget); },
  }, h("span", { class: `dot ${f.severity}`, "aria-label": f.severity }), h("span", {}, f.text))))
    : [h("li", { class: "ok" }, doc ? "No problems found." : "")]));
}
const same = (a, b) => a && b && a.kind === b.kind && a.index === b.index && a.goal === b.goal;
function markCurrent(btn) {
  for (const b of $("checks").querySelectorAll("button")) b.setAttribute("aria-current", b === btn ? "true" : "false");
}

/** Where an item is, and the editor text that names it. */
function locate(ref) {
  const d = doc, ld = d?.lazydash || {};
  if (!ref) return null;
  if (ref.kind === "object" || ref.kind === "goal") {
    const o = ld.objects?.[ref.index];
    if (!o) return null;
    const goals = (o.goals || []).filter(g => v3(g?.position)).map(g => g.position);
    const pos = ref.kind === "goal" ? o.goals?.[ref.goal]?.position : o.start?.position;
    const links = v3(o.start?.position) ? (ref.kind === "goal" ? [o.start.position] : goals) : [];
    return { pos, r: 0.06, links, name: o.name };
  }
  if (ref.kind === "robot") {
    const r = d.robots?.[ref.index];
    const range = num(ld.params?.robot_range) ? ld.params.robot_range : DEFAULT_PARAMS.robot_range;
    return r && { pos: r.base?.position, r: 0.12, reach: range, name: r.name };
  }
  if (ref.kind === "obstacle") {
    const o = d.obstacles?.[ref.index];
    return o && { pos: o.center, r: Math.min(0.3, (o.radius || 0.05) + 0.02), id: o.id };
  }
  if (ref.kind === "region") {
    const r = ld.placement_regions?.[ref.index];
    return r && { pos: r.min.map((v, k) => (v + r.max[k]) / 2), r: 0.06 + Math.hypot(...r.min.map((v, k) => r.max[k] - v)) / 2 };
  }
  return null;
}
function highlight(ref, pos, fromUser) {
  selected = { ref, pos };
  const at = locate(ref);
  const p = (at && v3(at.pos) ? at.pos : null) || (v3(pos) ? pos : null);
  if (!p) { mark.hide(); return; }
  mark.show(p, at?.r ?? 0.06, at?.reach, at?.links);
  if (!fromUser) return;
  view.focus(p);
  const key = at?.name != null ? `"name"\\s*:\\s*"${at.name.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")}"`
    : at?.id != null ? `"id"\\s*:\\s*"${String(at.id).replace(/[.*+?^${}()|[\]\\]/g, "\\$&")}"` : null;
  if (!key) return;
  const m = new RegExp(key).exec(editor.value);
  if (m) {
    select(m.index, m.index + m[0].length);
  }
}

/* ---------- 3D hover and click ---------- */
const canvas = view.renderer.domElement;
let pending = null;
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
let downAt = null;
canvas.addEventListener("pointerdown", ev => { downAt = [ev.clientX, ev.clientY]; });
canvas.addEventListener("click", ev => {
  if (!downAt || Math.hypot(ev.clientX - downAt[0], ev.clientY - downAt[1]) > 4) return;
  const hit = view.pickAt(ev);
  if (hit?.ref?.index != null) { highlight(hit.ref, null, true); markCurrent(null); }
  else { selected = null; mark.hide(); }
});

/* ---------- toolbar ---------- */
function seg(id, key) {
  const box = $(id);
  box.addEventListener("click", e => {
    const b = e.target.closest("button");
    if (!b || b.disabled) return;
    opts[key] = b.dataset.v;
    for (const x of box.querySelectorAll("button")) x.setAttribute("aria-pressed", x === b);
    render();
  });
}
seg("model", "model");
seg("scenery", "scenery");

function setWatch(on) {
  $("watch").setAttribute("aria-pressed", on);
  clearInterval(watchTimer);
  watchTimer = null;
  if (!on || !source?.path) return;
  watchTimer = setInterval(async () => {
    let text;
    try { text = lf(await fetchText(source.path)); } catch (e) { return; }
    if (text === diskText) return;
    const edited = editor.value !== diskText;
    diskText = text;
    editor.value = text;
    update();
    flash(edited ? "File changed on disk. Reloaded; in-page edits were replaced." : "File changed on disk. Reloaded.");
  }, 1000);
}
$("watch").addEventListener("click", () => setWatch($("watch").getAttribute("aria-pressed") !== "true"));

$("save").addEventListener("click", async () => {
  const text = editor.value;
  try { JSON.parse(text); } catch (e) { return flash("Fix the JSON before saving."); }
  const served = await launcher();
  if (!served) {
    const a = h("a", { href: URL.createObjectURL(new Blob([text.endsWith("\n") ? text : text + "\n"], { type: "application/json" })),
      download: (source?.path || source?.name || "scene.json").split("/").pop() });
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    return flash("Downloaded. Saving in place needs viewer/serve.py.");
  }
  const def = source?.path?.startsWith("resources/scenes/") ? source.path : `resources/scenes/${source?.name || "scene.json"}`;
  const path = prompt("Save the scene to (a path under resources/scenes/). An existing file is overwritten.", def);
  if (!path) return;
  try {
    const saved = await saveScene(path.trim(), text);
    source = { path: saved };
    diskText = text.endsWith("\n") ? text : text + "\n";
    if (editor.value === text) editor.value = diskText;
    $("file").textContent = saved;
    $("watch").disabled = false;
    setQuery("file", saved);
    update();
    flash(`Saved ${saved}`);
  } catch (e) { flash(`Not saved: ${e.message}`); }
});

/* ---------- layout ---------- */
const app = $("app"), side = $("side");
splitter($("gv"), app, "x", (x, b) => app.style.setProperty("--side", clamp(b.width - x, 260, b.width - 260) + "px"));
splitter($("gh"), side, "y", (y, b) => side.style.setProperty("--editor", clamp(y, 80, b.height - 80) + "px"));

/* ---------- start ---------- */
const initial = query("file");
fileSelect($("pick"), "resources/scenes", p => p.endsWith(".json"), openPath, initial);
if (initial) openPath(initial);
else launcher().then(ok => { if (!ok) $("hint").querySelector("p").append(h("br"), "Run viewer/serve.py to list the repository scenes."); });
runChecks();
