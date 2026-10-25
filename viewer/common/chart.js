// Small line charts: median lines with interquartile bands, log or linear y,
// nearest-point tooltips, optional direct labels. Pixel-sized SVG.
import { el, fmt, niceStep, showTip, hideTip } from "./util.js";

export const SERIES_COLORS = ["var(--s1)", "var(--s2)", "var(--s3)", "var(--s4)", "var(--s5)"];

const tickLabel = v => {
  if (v >= 1000) return `${+(v / 1000).toPrecision(3)}k`;
  if (v >= 1) return `${+v.toPrecision(3)}`;
  return `${+v.toPrecision(2)}`;
};

/** [lo, hi] covering `values` (all > 0 for log), padded to whole decades for log. */
export function domain(values, log, fixed) {
  if (fixed) return fixed;
  const vs = values.filter(v => v != null && Number.isFinite(v) && (!log || v > 0));
  if (!vs.length) return log ? [0.1, 10] : [0, 1];
  let lo = Math.min(...vs), hi = Math.max(...vs);
  if (log) return [Math.pow(10, Math.floor(Math.log10(lo))), Math.pow(10, Math.ceil(Math.log10(hi * 1.0001)))];
  if (hi === lo) hi = lo + 1;
  return [Math.min(0, lo), hi * 1.08];
}

/**
 * series: [{label, color, dash, hollow, points: [{x, y, lo, hi, n}]}]
 * o: {log, y: [lo, hi], x: [lo, hi], height, unit, digits, xLabel, direct, aria, context}
 */
export function linePlot(box, series, o) {
  const W = Math.max(200, box.clientWidth), H = o.height || 180;
  const L = 40, R = o.direct ? 64 : o.xLabel ? 44 : 10, T = 8, B = 22;
  const svg = el("svg", { width: W, height: H, viewBox: `0 0 ${W} ${H}`, role: "img", "aria-label": o.aria || "" }, box);
  svg.style.display = "block";
  const [x0, x1] = o.x, [y0, y1] = o.y;
  const tx = v => L + (W - L - R) * ((v - x0) / (x1 - x0 || 1));
  const ty = o.log
    ? v => T + (H - T - B) * (1 - (Math.log10(v) - Math.log10(y0)) / (Math.log10(y1) - Math.log10(y0)))
    : v => T + (H - T - B) * (1 - (v - y0) / (y1 - y0));
  const ax = el("g", { class: "axis" }, svg), grid = el("g", { class: "grid" }, svg);
  const yt = [];
  if (o.log) {
    const decades = Math.log10(y1 / y0);
    for (let v = y0; v <= y1 * 1.0001; v *= 10) {
      yt.push(v);
      if (decades <= 1.01) for (const m of [2, 5]) if (v * m < y1) yt.push(v * m);
    }
  } else {
    const s = niceStep(y1 - y0, H > 150 ? 4 : 3);
    for (let v = Math.ceil(y0 / s) * s; v <= y1 + 1e-9; v += s) yt.push(v);
  }
  for (const v of yt) {
    el("line", { x1: L, x2: W - R, y1: ty(v), y2: ty(v) }, grid);
    el("text", { x: L - 5, y: ty(v) + 4, "text-anchor": "end" }, ax).textContent = tickLabel(v);
  }
  const xs = niceStep(x1 - x0, Math.max(2, Math.floor((W - L - R) / 60)));
  for (let v = Math.ceil(x0 / xs) * xs; v <= x1 + 1e-9; v += xs)
    el("text", { x: tx(v), y: H - 6, "text-anchor": "middle" }, ax).textContent = v;
  el("line", { x1: L, x2: W - R, y1: H - B, y2: H - B }, ax);
  if (o.xLabel) el("text", { x: W - R + 6, y: H - 6, "text-anchor": "start" }, ax).textContent = o.xLabel;

  const ok = p => p.y != null && (!o.log || p.y > 0);
  for (const s of series) {   // bands below lines
    const pts = s.points.filter(p => ok(p) && p.lo != null && p.hi != null && (!o.log || p.lo > 0));
    if (pts.length < 2) continue;
    const d = pts.map((p, i) => `${i ? "L" : "M"}${tx(p.x).toFixed(1)},${ty(p.hi).toFixed(1)}`).join("") +
      [...pts].reverse().map(p => `L${tx(p.x).toFixed(1)},${ty(p.lo).toFixed(1)}`).join("") + "Z";
    el("path", { d, fill: s.color, opacity: 0.12, stroke: "none" }, svg);
  }
  const labels = [], all = [];
  for (const s of series) {
    const pts = s.points.filter(ok);
    if (!pts.length) continue;
    const d = pts.map((p, i) => `${i ? "L" : "M"}${tx(p.x).toFixed(1)},${ty(p.y).toFixed(1)}`).join("");
    el("path", { d, fill: "none", stroke: s.color, "stroke-width": 1.75, "stroke-dasharray": s.dash || null,
      "stroke-linejoin": "round" }, svg);
    for (const p of pts) {
      el("circle", { cx: tx(p.x), cy: ty(p.y), r: 3, fill: s.hollow ? "var(--surface)" : s.color,
        stroke: s.hollow ? s.color : "var(--surface)", "stroke-width": s.hollow ? 1.5 : 1 }, svg);
      all.push({ s, p, cx: tx(p.x), cy: ty(p.y) });
    }
    const last = pts[pts.length - 1];
    if (o.direct && s.short) labels.push({ x: tx(last.x), y: ty(last.y), text: s.short });
  }
  labels.sort((a, b) => a.y - b.y);
  for (let i = 1; i < labels.length; i++) if (labels[i].y - labels[i - 1].y < 12) labels[i].y = labels[i - 1].y + 12;
  for (const l of labels)
    el("text", { x: l.x + 7, y: l.y + 4, fill: "var(--muted)", "font-size": 11 }, svg).textContent = l.text;

  const ring = el("circle", { r: 6, fill: "none", stroke: "var(--ink)", "stroke-width": 1.5, visibility: "hidden" }, svg);
  svg.addEventListener("pointermove", ev => {
    const b = svg.getBoundingClientRect(), px = ev.clientX - b.left, py = ev.clientY - b.top;
    let best = null, bd = 1e9;
    for (const a of all) { const d = (a.cx - px) ** 2 + (a.cy - py) ** 2; if (d < bd) { bd = d; best = a; } }
    if (!best || bd > 600) { ring.setAttribute("visibility", "hidden"); return hideTip(); }
    ring.setAttribute("cx", best.cx); ring.setAttribute("cy", best.cy); ring.setAttribute("visibility", "visible");
    const { p, s } = best, d = o.digits ?? 1, u = o.unit || "";
    const iqr = p.lo != null && p.n > 1 ? `\nIQR ${fmt(p.lo, d)} to ${fmt(p.hi, d)}${u}` : "";
    const runs = p.n != null ? `\n${p.n} runs` + (p.runs != null && p.runs !== p.n ? ` (of ${p.runs})` : "") : "";
    showTip(`${s.label}${s.part ? `, ${s.part}` : ""}\n${o.context || ""}${p.x} blocks: ${fmt(p.y, d)}${u}${iqr}${runs}`,
      ev.clientX, ev.clientY);
  });
  svg.addEventListener("pointerleave", () => { ring.setAttribute("visibility", "hidden"); hideTip(); });
  return svg;
}
