// Shared helpers: DOM, formatting, theme, tooltip, files, launcher API, splitters.

export const $ = id => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";

/** An SVG element with attributes, appended to `parent` when given. */
export function el(tag, attrs = {}, parent) {
  const n = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) if (v != null) n.setAttribute(k, v);
  if (parent) parent.appendChild(n);
  return n;
}

/** An HTML element; `children` are nodes or strings. */
export function h(tag, attrs = {}, ...children) {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v == null || v === false) continue;
    if (k.startsWith("on")) n.addEventListener(k.slice(2), v);
    else if (k === "class") n.className = v;
    else n.setAttribute(k, v === true ? "" : v);
  }
  for (const c of children.flat()) if (c != null) n.append(c);
  return n;
}

export const fmt = (x, d = 1) => (x == null || Number.isNaN(x) ? "-" : Number(x).toFixed(d));
export const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

export function niceStep(span, target) {
  const raw = span / target, p = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 2.5, 5, 10]) if (m * p >= raw) return m * p;
  return 10 * p;
}

export function debounce(fn, ms) {
  let t;
  return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); };
}

export const cssVar = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

/* ---------- tooltip and messages ---------- */
let tipEl;
export function showTip(text, x, y) {
  if (!tipEl) { tipEl = h("div", { class: "tip" }); document.body.append(tipEl); }
  tipEl.textContent = text;
  tipEl.hidden = false;
  const w = tipEl.offsetWidth, hh = tipEl.offsetHeight;
  tipEl.style.left = clamp(x + 12, 8, window.innerWidth - w - 8) + "px";
  tipEl.style.top = (y - hh - 10 < 8 ? y + 16 : y - hh - 10) + "px";
}
export function hideTip() { if (tipEl) tipEl.hidden = true; }
window.addEventListener("scroll", hideTip, true);

let msgEl, msgTimer;
export function flash(text, ms = 3500) {
  if (!msgEl) { msgEl = h("div", { class: "msg", role: "status" }); document.body.append(msgEl); }
  msgEl.textContent = text;
  msgEl.hidden = false;
  clearTimeout(msgTimer);
  msgTimer = setTimeout(() => { msgEl.hidden = true; }, ms);
}

/* ---------- theme: auto, light, dark ---------- */
const THEME_KEY = "lazydash-viewer-theme";
export function initTheme(button) {
  const order = ["auto", "light", "dark"];
  let mode = "auto";
  try { mode = localStorage.getItem(THEME_KEY) || "auto"; } catch (e) { /* storage blocked */ }
  const apply = () => {
    if (mode === "auto") delete document.documentElement.dataset.theme;
    else document.documentElement.dataset.theme = mode;
    if (button) {
      button.textContent = { auto: "Theme: auto", light: "Theme: light", dark: "Theme: dark" }[mode];
    }
    window.dispatchEvent(new Event("themechange"));
  };
  if (button) button.addEventListener("click", () => {
    mode = order[(order.indexOf(mode) + 1) % order.length];
    try { localStorage.setItem(THEME_KEY, mode); } catch (e) { /* storage blocked */ }
    apply();
  });
  matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => mode === "auto" && apply());
  apply();
}

/* ---------- repository URLs and the launcher API ---------- */
const ROOT = new URL("../../", import.meta.url); // viewer/common/ -> repository root
export const repoUrl = path => new URL(path, ROOT).href;

let info;
/** True when the page is served by viewer/serve.py. */
export function launcher() {
  info ??= fetch(new URL("api/info", ROOT), { cache: "no-store" })
    .then(r => (r.ok ? r.json() : {})).then(j => !!j.launcher).catch(() => false);
  return info;
}
export async function listFiles(dir) {
  if (!(await launcher())) return [];
  const r = await fetch(new URL("api/list?dir=" + encodeURIComponent(dir), ROOT), { cache: "no-store" });
  return r.ok ? (await r.json()).files : [];
}
export async function saveScene(path, text) {
  const r = await fetch(new URL("api/save?path=" + encodeURIComponent(path), ROOT), {
    method: "POST", headers: { "X-Lazydash": "save", "Content-Type": "application/json" }, body: text });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || r.statusText);
  return j.saved;
}
export async function fetchText(path) {
  const r = await fetch(repoUrl(path), { cache: "no-store" });
  if (!r.ok) throw new Error(`${path}: ${r.status} ${r.statusText}`);
  return r.text();
}

/* ---------- files: picker button and drag and drop ---------- */
export function fileInput(button, accept, onFile) {
  const input = h("input", { type: "file", accept, hidden: true });
  input.addEventListener("change", () => { if (input.files[0]) onFile(input.files[0]); input.value = ""; });
  document.body.append(input);
  button.addEventListener("click", () => input.click());
}
export function dropFiles(onFile) {
  let depth = 0;
  const on = v => document.body.classList.toggle("dragging", v);
  addEventListener("dragenter", e => { if (e.dataTransfer?.types.includes("Files")) { depth++; on(true); e.preventDefault(); } });
  addEventListener("dragover", e => { if (e.dataTransfer?.types.includes("Files")) e.preventDefault(); });
  addEventListener("dragleave", () => { if (--depth <= 0) { depth = 0; on(false); } });
  addEventListener("drop", e => {
    depth = 0; on(false);
    const f = e.dataTransfer?.files?.[0];
    if (f) { e.preventDefault(); onFile(f); }
  });
}
export const query = name => new URLSearchParams(location.search).get(name);
export function setQuery(name, value) {
  const u = new URL(location.href);
  if (value) u.searchParams.set(name, value); else u.searchParams.delete(name);
  try { history.replaceState(null, "", u); } catch (e) { /* sandboxed */ }
}

/** A <select> of launcher-listed files; `onPick(path)` on change. */
export async function fileSelect(select, dir, filter, onPick, current) {
  const files = (await listFiles(dir)).filter(f => filter(f.path));
  select.hidden = !files.length;
  select.replaceChildren(h("option", { value: "" }, select.dataset.placeholder || "Open from repository"),
    ...files.map(f => h("option", { value: f.path, selected: f.path === current }, f.path.slice(dir.length + 1))));
  select.onchange = () => select.value && onPick(select.value);
  return files;
}

/* ---------- resizable panels ---------- */
/** Drag `gutter` to resize: `set(px)` gets the pointer offset along the axis
 *  from the start of `container`. */
export function splitter(gutter, container, axis, set) {
  gutter.addEventListener("pointerdown", e => {
    e.preventDefault();
    gutter.setPointerCapture(e.pointerId);
    gutter.classList.add("drag");
    const move = ev => {
      const b = container.getBoundingClientRect();
      set(axis === "x" ? ev.clientX - b.left : ev.clientY - b.top, b);
      window.dispatchEvent(new Event("resize"));
    };
    const up = () => {
      gutter.classList.remove("drag");
      gutter.removeEventListener("pointermove", move);
      gutter.removeEventListener("pointerup", up);
    };
    gutter.addEventListener("pointermove", move);
    gutter.addEventListener("pointerup", up);
  });
}
