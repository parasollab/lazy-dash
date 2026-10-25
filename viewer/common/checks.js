// Quick sanity checks of a scene (format lazydash.scene/1). Schema as read by
// src/lazydash/scene/Scene.cpp; geometric checks are point and sphere tests,
// not IK. Each finding: {severity: "error"|"warning", text, ref, pos}.

export const FORMAT = "lazydash.scene/1";
// TaskParams defaults (Scene.h).
export const DEFAULT_PARAMS = { robot_range: 1.5, handover_max_base_distance: 1.6, planning_padding: 0.01 };
const BASE_CLEARANCE = 0.15; // UR5e base footprint diameter (m)

const isNum = v => typeof v === "number" && Number.isFinite(v);
const isVec = (v, n = 3) => Array.isArray(v) && v.length === n && v.every(isNum);
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const p3 = p => p.map(v => +v.toFixed(3)).join(", ");

/** Signed distance from point p to a sphere or finite cylinder obstacle. */
export function obstacleDistance(o, p) {
  const d = [p[0] - o.center[0], p[1] - o.center[1], p[2] - o.center[2]];
  if (o.type === "sphere") return Math.hypot(...d) - o.radius;
  const n = Math.hypot(...o.axis) || 1, a = o.axis.map(v => v / n);
  const along = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
  const radial = Math.hypot(d[0] - along * a[0], d[1] - along * a[1], d[2] - along * a[2]);
  const dr = radial - o.radius, dh = Math.abs(along) - o.half_height;
  if (dr <= 0 && dh <= 0) return Math.max(dr, dh);
  return Math.hypot(Math.max(dr, 0), Math.max(dh, 0));
}

export function checkScene(doc) {
  const out = [];
  const add = (severity, text, ref, pos) => out.push({ severity, text, ref, pos });
  const err = (text, ref, pos) => add("error", text, ref, pos);
  const warn = (text, ref, pos) => add("warning", text, ref, pos);

  // ---- schema (what Scene::parse requires)
  if (!doc || typeof doc !== "object" || Array.isArray(doc)) return [{ severity: "error", text: "Not a JSON object." }];
  if (doc.format !== FORMAT) err(`format must be "${FORMAT}" (got ${JSON.stringify(doc.format)}).`);
  if (!isVec(doc.environment?.min) || !isVec(doc.environment?.max)) err("environment.min and environment.max must be 3-vectors.");
  const obstacles = Array.isArray(doc.obstacles) ? doc.obstacles : (err("obstacles must be an array."), []);
  const goodObs = [];
  obstacles.forEach((o, i) => {
    const ref = { kind: "obstacle", index: i }, name = `Obstacle ${o?.id ?? i}`;
    if (o?.type === "sphere") {
      if (!isVec(o.center) || !isNum(o.radius)) return err(`${name}: a sphere needs center and radius.`, ref);
    } else if (o?.type === "cylinder") {
      if (!isVec(o.center) || !isVec(o.axis) || !isNum(o.radius) || !isNum(o.half_height))
        return err(`${name}: a cylinder needs center, axis, radius and half_height.`, ref);
    } else return err(`${name}: type must be sphere or cylinder (got ${JSON.stringify(o?.type)}).`, ref);
    goodObs.push([i, o]);
  });
  const robots = Array.isArray(doc.robots) ? doc.robots : (err("robots must be an array."), []);
  const arms = [];
  const names = new Map();
  const unique = (name, what, ref) => {
    if (names.has(name)) err(`${what} name "${name}" is used twice.`, ref);
    names.set(name, ref);
  };
  robots.forEach((r, i) => {
    const ref = { kind: "robot", index: i }, name = r?.name ?? `robot ${i}`;
    if (typeof r?.name !== "string") err(`Robot ${i}: name missing.`, ref); else unique(r.name, "Robot", ref);
    if (r?.type !== "ur5e_hande") err(`${name}: type must be "ur5e_hande" (got ${JSON.stringify(r?.type)}).`, ref);
    const pos = r?.base?.position, q = r?.base?.quaternion_xyzw;
    if (!isVec(pos)) return err(`${name}: base.position must be a 3-vector.`, ref);
    if (q != null) {
      if (!isVec(q, 4)) err(`${name}: base.quaternion_xyzw must have 4 numbers.`, ref, pos);
      else {
        const [x, y, z, w] = q, nn = Math.hypot(x, y, z, w) || 1;
        const zz = 1 - 2 * ((x * x + y * y) / (nn * nn)); // world z of the base z axis
        if (zz < 1 - 1e-9) err(`${name}: the base is not upright (the planner requires a yaw-only base).`, ref, pos);
      }
    }
    const start = r?.start, lo = r?.joint_limits?.lower, hi = r?.joint_limits?.upper;
    if (!isVec(start, 6)) err(`${name}: start must have 6 joint values.`, ref, pos);
    if (!isVec(lo, 6) || !isVec(hi, 6)) err(`${name}: joint_limits.lower and .upper must have 6 values.`, ref, pos);
    else if (isVec(start, 6)) {
      for (let j = 0; j < 6; j++) {
        if (lo[j] > hi[j]) err(`${name}: joint ${j + 1} lower limit is above the upper limit.`, ref, pos);
        else if (start[j] < lo[j] - 1e-9 || start[j] > hi[j] + 1e-9)
          err(`${name}: start joint ${j + 1} = ${start[j].toFixed(3)} is outside [${lo[j].toFixed(3)}, ${hi[j].toFixed(3)}].`, ref, pos);
      }
    }
    arms.push({ i, name, pos });
  });
  const ld = doc.lazydash;
  if (!ld || typeof ld !== "object") { err("lazydash block missing."); return out; }
  const objects = Array.isArray(ld.objects) ? ld.objects : (err("lazydash.objects must be an array."), []);
  if (!Array.isArray(ld.grasps) || !ld.grasps.length) err("lazydash.grasps must be a non-empty array.");
  else ld.grasps.forEach((g, i) => {
    if (typeof g?.name !== "string" || typeof g?.link !== "string" || !isVec(g?.translation) || !isVec(g?.rpy))
      err(`Grasp ${i}: needs name, link, translation and rpy.`);
  });
  const params = { ...DEFAULT_PARAMS, ...(ld.params || {}) };
  if (!isNum(params.robot_range)) { err("params.robot_range must be a number."); params.robot_range = DEFAULT_PARAMS.robot_range; }
  (ld.placement_regions || []).forEach((r, i) => {
    if (!isVec(r?.min) || !isVec(r?.max) || !isVec(r?.yaw, 2)) err(`Placement region ${i}: needs min, max and yaw [lo, hi].`, { kind: "region", index: i });
    else if (r.object != null && !(r.object >= 0 && r.object < objects.length)) err(`Placement region ${i}: object ${r.object} does not exist.`, { kind: "region", index: i });
  });
  const blocks = [];
  objects.forEach((o, i) => {
    const ref = { kind: "object", index: i }, name = o?.name ?? `object ${i}`;
    if (typeof o?.name !== "string") err(`Object ${i}: name missing.`, ref); else unique(o.name, "Object", ref);
    if (!isNum(o?.collision_radius) || o.collision_radius <= 0) err(`${name}: collision_radius must be a positive number.`, ref);
    if (o?.shape && !isVec(o.shape.size)) err(`${name}: shape.size must be a 3-vector.`, ref);
    const sp = o?.start?.position;
    if (!isVec(sp)) return err(`${name}: start.position must be a 3-vector.`, ref);
    if (!Array.isArray(o.goals) || !o.goals.length) return err(`${name}: needs at least one goal.`, ref, sp);
    const goals = [];
    o.goals.forEach((g, k) => {
      if (!isVec(g?.position)) err(`${name}: goal ${k} position must be a 3-vector.`, { kind: "goal", index: i, goal: k });
      else goals.push([k, g.position]);
    });
    blocks.push({ i, name, r: isNum(o.collision_radius) ? o.collision_radius : 0, start: sp, goals });
  });

  // ---- geometry
  const env = doc.environment;
  const inEnv = p => !isVec(env?.min) || !isVec(env?.max) || p.every((v, k) => v >= env.min[k] && v <= env.max[k]);
  const poses = blocks.flatMap(b => [
    { b, p: b.start, what: "start", ref: { kind: "object", index: b.i } },
    ...b.goals.map(([k, p]) => ({ b, p, what: b.goals.length > 1 ? `goal ${k}` : "goal", ref: { kind: "goal", index: b.i, goal: k } })),
  ]);
  for (const { b, p, what, ref } of poses) {
    if (!inEnv(p)) err(`${b.name} ${what} (${p3(p)}) is outside the environment bounds.`, ref, p);
    for (const [j, o] of goodObs) {
      const d = obstacleDistance(o, p);
      if (d < 0) { err(`${b.name} ${what} is inside obstacle ${o.id ?? j}.`, ref, p); break; }
      if (d < b.r - 1e-6) { warn(`${b.name} ${what}: its collision sphere reaches ${((b.r - d) * 1000).toFixed(0)} mm into obstacle ${o.id ?? j}.`, ref, p); break; }
    }
  }
  // overlapping blocks: starts (an error), goals of different objects (they cannot all be used)
  for (let a = 0; a < blocks.length; a++)
    for (let c = a + 1; c < blocks.length; c++) {
      const A = blocks[a], C = blocks[c], lim = A.r + C.r - 1e-6;
      if (dist(A.start, C.start) < lim)
        err(`${A.name} and ${C.name} overlap at the start (${(dist(A.start, C.start) * 1000).toFixed(0)} mm apart).`, { kind: "object", index: A.i }, A.start);
      const hit = A.goals.find(([, p]) => C.goals.some(([, q]) => dist(p, q) < lim));
      if (hit) {
        const allA = A.goals.every(([, p]) => C.goals.some(([, q]) => dist(p, q) < lim));
        (allA ? err : warn)(`Goals of ${A.name} and ${C.name} overlap${allA ? "; both cannot be placed" : ""}.`, { kind: "goal", index: A.i, goal: hit[0] }, hit[1]);
      }
    }
  // reach: base-to-object distance at most robot_range (Capabilities::reaches)
  const R = params.robot_range;
  const reachers = p => arms.filter(a => dist(a.pos, p) <= R);
  const same = (a, b) => isVec(a) && isVec(b) && a.every((v, k) => Math.abs(v - b[k]) < 1e-6);
  if (arms.length) for (const b of blocks) {
    const o = objects[b.i];
    if (o.goals.some(g => same(g.position, o.start.position) && same(g.rpy || [0, 0, 0], o.start.rpy || [0, 0, 0]))) continue; // already at a goal
    if (!reachers(b.start).length) err(`${b.name} start is out of every arm's reach (${R} m).`, { kind: "object", index: b.i }, b.start);
    const out = b.goals.filter(([, p]) => !reachers(p).length);
    if (out.length === b.goals.length && b.goals.length)
      err(`${b.name}: every goal is out of every arm's reach (${R} m).`, { kind: "goal", index: b.i, goal: out[0][0] }, out[0][1]);
    else for (const [k, p] of out) warn(`${b.name} goal ${k} is out of every arm's reach (${R} m).`, { kind: "goal", index: b.i, goal: k }, p);
  }
  // arm bases
  for (let a = 0; a < arms.length; a++)
    for (let c = a + 1; c < arms.length; c++) {
      const d = dist(arms[a].pos, arms[c].pos);
      if (d < BASE_CLEARANCE) err(`Bases of ${arms[a].name} and ${arms[c].name} overlap (${(d * 1000).toFixed(0)} mm apart).`, { kind: "robot", index: arms[a].i }, arms[a].pos);
    }
  for (const a of arms) if (!inEnv(a.pos)) warn(`${a.name} base is outside the environment bounds.`, { kind: "robot", index: a.i }, a.pos);
  return out;
}
