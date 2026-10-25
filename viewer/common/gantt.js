// Arm timeline (Gantt): one row per arm, moves as bars (empty hand or
// carrying), grasp and release stages as ticks, handovers as links between
// the two rows. Drawn at the container's pixel size; times in seconds.
import { el, fmt, niceStep, showTip, hideTip, clamp } from "./util.js";

/**
 * entries: [{robot, start, end, kind: "move"|"stage", action, object, holding, step}]
 * opts: {arms, makespan, onSeek(t), onSelectArm(a), labels}
 */
export function createGantt(box, entries, opts) {
  const svg = el("svg", { class: "gantt", tabindex: 0, role: "img", "aria-label": "Arm timeline" }, box);
  svg.style.display = "block";
  svg.style.cursor = "crosshair";
  svg.style.touchAction = "none";
  const { arms, makespan } = opts;
  const byRow = Array.from({ length: arms }, () => []);
  for (const e of entries) if (e.robot < arms) byRow[e.robot].push(e);
  byRow.forEach(r => r.sort((p, q) => p.start - q.start));
  let time = 0, selected = -1, geom = null, playhead;

  function draw() {
    const W = Math.max(320, box.clientWidth), Hbox = Math.max(60, box.clientHeight);
    const left = 40, right = 12, top = 6, axisH = 20, gap = arms > 9 ? 2 : 4;
    const rowH = clamp(Math.floor((Hbox - top - axisH) / arms) - gap, 6, 26);
    const H = top + arms * (rowH + gap) + axisH, plotW = W - left - right;
    svg.replaceChildren();
    svg.setAttribute("width", W);
    svg.setAttribute("height", H);
    svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
    const x = t => left + (t / makespan) * plotW, yRow = a => top + a * (rowH + gap);
    geom = { left, top, plotW, rowH, gap, W, H, x };
    const ax = el("g", { class: "axis" }, svg), grid = el("g", { class: "grid" }, svg);
    const step = niceStep(makespan, Math.max(3, Math.floor(plotW / 90)));
    for (let t = 0; t <= makespan + 1e-9; t += step) {
      el("line", { x1: x(t), x2: x(t), y1: top, y2: H - axisH + 2 }, grid);
      el("text", { x: x(t), y: H - 5, "text-anchor": "middle" }, ax).textContent = `${+t.toFixed(1)} s`;
    }
    const fs = Math.min(11, rowH + 2);
    for (let a = 0; a < arms; a++) {
      if (a === selected) el("rect", { x: 0, y: yRow(a) - gap / 2, width: W, height: rowH + gap, fill: "var(--surface-2)" }, svg);
      const t = el("text", { x: left - 6, y: yRow(a) + rowH / 2 + fs / 2.8, "text-anchor": "end", "font-size": fs,
        fill: a === selected ? "var(--ink)" : "var(--muted)", "font-weight": a === selected ? 600 : 400 }, svg);
      t.textContent = (opts.labels?.[a]) ?? "r" + a;
      t.style.cursor = "pointer";
      t.addEventListener("click", ev => { ev.stopPropagation(); opts.onSelectArm?.(a === selected ? -1 : a); });
    }
    const bars = el("g", {}, svg), ticks = el("g", {}, svg), links = el("g", { opacity: 0.45 }, svg);
    const handover = new Map();
    for (const e of entries) {
      if (e.robot >= arms) continue;
      if (e.kind === "move") {
        el("rect", { x: x(e.start), y: yRow(e.robot), width: Math.max(0.6, x(e.end) - x(e.start) - 0.6), height: rowH,
          rx: 1.5, fill: e.holding ? "var(--carry)" : "var(--empty)" }, bars);
      } else {
        el("rect", { x: x(e.start), y: yRow(e.robot) - 1, width: Math.max(1.6, x(e.end) - x(e.start)), height: rowH + 2,
          fill: "var(--stage)" }, ticks);
        if (e.action === "handover") {
          const k = `${e.step}/${e.object}`;
          if (!handover.has(k)) handover.set(k, []);
          handover.get(k).push(e);
        }
      }
    }
    for (const pts of handover.values()) {
      const rows = [...new Set(pts.map(p => p.robot))];
      if (rows.length < 2) continue;
      const t = Math.min(...pts.map(p => p.start)), ys = rows.map(a => yRow(a) + rowH / 2);
      el("line", { x1: x(t), x2: x(t), y1: Math.min(...ys), y2: Math.max(...ys), stroke: "var(--stage)", "stroke-width": 1 }, links);
    }
    playhead = el("line", { y1: top - 4, y2: H - axisH + 2, stroke: "var(--accent)", "stroke-width": 2 }, svg);
    setTime(time);
  }

  function setTime(t) {
    time = t;
    if (!playhead || !geom) return;
    const px = geom.x(clamp(t, 0, makespan));
    playhead.setAttribute("x1", px);
    playhead.setAttribute("x2", px);
  }

  const local = ev => {
    const b = svg.getBoundingClientRect();
    return [ev.clientX - b.left, ev.clientY - b.top];
  };
  const timeAt = px => clamp(((px - geom.left) / geom.plotW) * makespan, 0, makespan);
  let scrubbing = false;
  svg.addEventListener("pointerdown", ev => {
    const [px] = local(ev);
    if (px < geom.left - 4) return;
    scrubbing = true;
    svg.setPointerCapture(ev.pointerId);
    opts.onSeek?.(timeAt(px));
  });
  svg.addEventListener("pointerup", () => { scrubbing = false; });
  svg.addEventListener("pointermove", ev => {
    const [px, py] = local(ev);
    if (scrubbing) { opts.onSeek?.(timeAt(px)); return hideTip(); }
    const a = Math.floor((py - geom.top) / (geom.rowH + geom.gap)), t = ((px - geom.left) / geom.plotW) * makespan;
    if (a < 0 || a >= arms || t < 0 || t > makespan) return hideTip();
    const tol = makespan / geom.plotW * 2;
    const hit = byRow[a].find(e => e.kind === "stage" && e.start - tol <= t && t <= e.end + tol) ||
                byRow[a].find(e => e.start <= t && t <= e.end);
    const name = opts.labels?.[a] ?? "r" + a;
    if (!hit) return showTip(`${name} waiting\nt = ${fmt(t)} s`, ev.clientX, ev.clientY);
    const what = hit.kind === "stage" ? `${hit.action} o${hit.object} (${hit.holding ? "lift or place" : "approach or retreat"})`
      : hit.holding ? `carrying o${hit.object} to ${hit.action}` : `moving to ${hit.action} o${hit.object}`;
    showTip(`${name}, step ${hit.step}: ${what}\n${fmt(hit.start)} to ${fmt(hit.end)} s`, ev.clientX, ev.clientY);
  });
  svg.addEventListener("pointerleave", hideTip);
  svg.addEventListener("keydown", ev => {
    const d = ev.shiftKey ? makespan / 10 : makespan / 100;
    if (ev.key === "ArrowRight") opts.onSeek?.(Math.min(makespan, time + d));
    else if (ev.key === "ArrowLeft") opts.onSeek?.(Math.max(0, time - d));
    else return;
    ev.preventDefault();
  });
  const ro = new ResizeObserver(() => draw());
  ro.observe(box);
  draw();
  return {
    setTime,
    setSelected(a) { selected = a; draw(); },
    destroy() { ro.disconnect(); svg.remove(); },
  };
}
