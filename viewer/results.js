// Results browser: curves against the paper's runs, from the JSON written by
// runner/aggregate_results.py. Medians over solved runs with interquartile bands.
import { $, h, fmt, debounce, flash, initTheme, fileInput, dropFiles, fetchText, query, setQuery } from "./common/util.js";
import { linePlot, domain, SERIES_COLORS } from "./common/chart.js";

initTheme($("theme"));
const DEFAULT = "results/paper/results.json";
const VIEWS = [
  ["times", "Times"], ["sizes", "Sizes"], ["success", "Success"], ["makespan", "Makespan"], ["table", "Table"],
];
const TIME_COLS = [
  ["(1) task and motion planning (s)", [["task", "task", true], ["motion", "motion", false]]],
  ["(2) conflict resolution (s)", [["conflict", null, false]]],
  ["(3) total (s)", [["total", null, false]]],
];
const SIZE_COLS = [
  ["H_T vertices", [["htv", null, false]]], ["H_T hyperarcs", [["hte", null, false]]],
  ["H_M vertices", [["hmv", null, false]]], ["H_M hyperarcs", [["hme", null, false]]],
];
let DATA = null, view = "times";
const hidden = { series: new Set(), families: new Set() };
try { const v = (location.hash || "").slice(1); if (VIEWS.some(([k]) => k === v)) view = v; } catch (e) { /* no hash */ }

/* ---------- loading ---------- */
function load(text, name) {
  let d;
  try { d = JSON.parse(text); } catch (e) { return flash(`Not JSON: ${e.message}`); }
  if (d.format !== "lazydash.results/1" || !Array.isArray(d.families)) return flash("Not an aggregate_results.py file (format lazydash.results/1).");
  DATA = d;
  hidden.series.clear();
  hidden.families.clear();
  $("file").textContent = name;
  $("file").title = name;
  controls();
  render();
}
fileInput($("open"), ".json,application/json", async f => load(await f.text(), f.name));
dropFiles(async f => load(await f.text(), f.name));

/* ---------- controls ---------- */
const seriesList = () => DATA.series.map((s, i) => ({ ...s,
  color: s.reference ? "var(--paper)" : SERIES_COLORS[(i - 1) % SERIES_COLORS.length],
  dash: s.reference ? "5 4" : null, hollow: !!s.reference, short: s.reference ? "paper" : s.label }));
function controls() {
  const tabs = $("tabs");
  tabs.replaceChildren(...VIEWS.map(([k, label]) => h("button", { class: "tab", type: "button", role: "tab",
    "aria-selected": String(k === view), onclick: () => { view = k; try { history.replaceState(null, "", "#" + k); } catch (e) { /* sandboxed */ } controls(); render(); } }, label)));
  $("series").replaceChildren(h("span", { class: "k" }, "Series"), ...seriesList().map(s => h("button", {
    class: "chip", type: "button", "aria-pressed": String(!hidden.series.has(s.id)),
    title: `${s.source}, ${s.runs} runs`,
    onclick: () => { toggle(hidden.series, s.id); controls(); render(); },
  }, h("span", { class: `sw${s.dash ? " dash" : ""}`, style: `border-color:${s.color}` }), s.label)));
  $("families").replaceChildren(h("span", { class: "k" }, "Scenes"), ...DATA.families.map(f => h("button", {
    class: "chip", type: "button", "aria-pressed": String(!hidden.families.has(f.id)),
    onclick: () => { toggle(hidden.families, f.id); controls(); render(); },
  }, f.label)));
}
const toggle = (set, k) => (set.has(k) ? set.delete(k) : set.add(k));
const visibleSeries = () => seriesList().filter(s => !hidden.series.has(s.id));
const visibleFamilies = () => DATA.families.filter(f => !hidden.families.has(f.id));

/* ---------- views ---------- */
const stat = (p, m) => m === "success" ? { y: 100 * p.success, n: p.runs }
  : p[m] ? { y: p[m][0], lo: p[m][1], hi: p[m][2], n: p[m][3], runs: p.runs } : null;
function lines(fam, metric, part, dashed) {
  return visibleSeries().map(s => ({
    label: s.label, short: s.short, color: s.color, hollow: s.hollow, part,
    dash: part ? (dashed ? "4 3" : null) : s.dash,
    points: (fam.points[s.id] || []).map(p => ({ x: p.objects, ...stat(p, metric) })).filter(p => p.y != null),
  }));
}
const xDomain = fam => {
  const xs = Object.values(fam.points).flat().map(p => p.objects);
  return [0, Math.max(4, ...xs) * 1.04];
};

function gridView(main, columns, note) {
  main.append(h("p", { class: "note" }, note));
  const tmpl = `repeat(${columns.length}, minmax(0, 1fr))`;
  main.append(h("div", { class: "cols colhead", style: `grid-template-columns:${tmpl}` }, ...columns.map(([t]) => h("h3", {}, t))));
  for (const fam of visibleFamilies()) {
    const cells = columns.map(([, parts]) => parts.flatMap(([m, part, dashed]) => lines(fam, m, part, dashed)));
    const vals = cells.flat().flatMap(s => s.points.flatMap(p => [p.y, p.lo, p.hi]));
    const y = domain(vals, true);
    const row = h("div", { class: "cols", style: `grid-template-columns:${tmpl}` });
    main.append(h("section", { class: "fam" }, h("h3", {}, fam.label), row));
    columns.forEach(([title], j) => {
      const cell = h("div", { class: "cell", "data-col": title });
      row.append(cell);
      linePlot(cell, cells[j], { log: true, y, x: xDomain(fam), height: 165, xLabel: "blocks",
        digits: 2, aria: `${fam.label}, ${title}`, context: `${fam.label}\n` });
    });
  }
}

function wrapView(main, metric, o, note) {
  main.append(h("p", { class: "note" }, note));
  const wrap = h("div", { class: "wrap" });
  main.append(wrap);
  for (const fam of visibleFamilies()) {
    const series = lines(fam, metric, null, false);
    const cell = h("div", { class: "cell" }, h("h3", {}, fam.label));
    wrap.append(cell);
    const vals = series.flatMap(s => s.points.flatMap(p => [p.y, p.hi]));
    linePlot(cell, series, { ...o, y: domain(vals, false, o.y), x: xDomain(fam), height: 220, xLabel: "blocks",
      direct: series.length <= 4, aria: `${fam.label}, ${metric}`, context: `${fam.label}\n` });
  }
}

function tableView(main) {
  const ss = visibleSeries();
  const cols = [["success", "success", p => p.success == null ? null : `${fmt(100 * p.success, 0)}%`],
    ["time s", "total", p => p.total && fmt(p.total[0], 1)], ["makespan", "makespan", p => p.makespan && fmt(p.makespan[0], 0)],
    ["H_T V", "htv", p => p.htv && fmt(p.htv[0], 0)], ["runs", "runs", p => p.runs]];
  const head1 = h("tr", {}, h("th", { class: "l", rowspan: 2 }, "scene"), h("th", { rowspan: 2 }, "blocks"),
    ...ss.map(s => h("th", { class: "grp", colspan: cols.length }, s.label)));
  const head2 = h("tr", {}, ...ss.flatMap(() => cols.map(([label], i) => h("th", { class: i ? null : "first" }, label))));
  const body = [];
  for (const fam of visibleFamilies()) {
    const ns = [...new Set(Object.values(fam.points).flat().map(p => p.objects))].sort((a, b) => a - b);
    for (const n of ns) body.push(h("tr", {}, h("td", { class: "fam-l" }, fam.label), h("td", {}, String(n)),
      ...ss.flatMap(s => {
        const p = (fam.points[s.id] || []).find(q => q.objects === n);
        return cols.map(([, , f], i) => h("td", { class: i ? null : "first" }, p ? (f(p) ?? "-") : "-"));
      })));
  }
  main.append(h("p", { class: "note" }, `Medians over solved runs. Makespan in the paper's timesteps (ours divided by ${DATA.paper_step}).`),
    h("div", { class: "tablebox" }, h("table", {}, h("thead", {}, head1, head2), h("tbody", {}, ...body))));
}

function render() {
  const main = $("main");
  main.replaceChildren();
  if (!DATA) return;
  if (!visibleSeries().length || !visibleFamilies().length) return main.append(h("p", { class: "empty" }, "Select at least one series and one scene."));
  if (view === "times") gridView(main, TIME_COLS, "Planning time by stage, as in the paper's Fig. 8. Median over solved runs, band = interquartile range, log y shared across a row. In (1) dashed is task planning and solid is motion planning, for every series.");
  else if (view === "sizes") gridView(main, SIZE_COLS, "Representation sizes, as in the paper's Fig. 7. Median over solved runs, band = interquartile range, log y shared across a row.");
  else if (view === "success") wrapView(main, "success", { y: [0, 105], unit: "%", digits: 0 }, "Share of runs solved within the time limit.");
  else if (view === "makespan") wrapView(main, "makespan", { unit: "", digits: 0 }, `Makespan in the paper's timesteps (ours divided by ${DATA.paper_step}). Median over solved runs, band = interquartile range.`);
  else tableView(main);
}
addEventListener("resize", debounce(render, 150));

/* ---------- start ---------- */
const initial = query("file") || DEFAULT;
fetchText(initial).then(t => { setQuery("file", initial === DEFAULT ? null : initial); load(t, initial); }).catch(() => {
  $("main").replaceChildren(h("div", { class: "empty" },
    h("p", {}, `No aggregated results at ${initial}. Drop one here or open it, or write it with:`),
    h("p", {}, h("code", {}, "python3 runner/aggregate_results.py lazydash=results/paper/trials.jsonl"))));
});
