#include "lazydash/planning/LazyDash.h"

#include "lazydash/motion/MotionLayer.h"
#include "lazydash/robot/BodyModels.h"
#include "lazydash/schedule/MeshVerifier.h"
#include "lazydash/task/Capabilities.h"
#include "lazydash/task/DurationModel.h"
#include "lazydash/task/Heuristic.h"
#include "lazydash/task/TaskConflictDetection.h"
#include "lazydash/task/TaskHypergraph.h"
#include "lazydash/task/TransitionSampler.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

namespace lazydash {

namespace {

using Clock = std::chrono::steady_clock;

double since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

} // namespace

LazyDash::LazyDash(const Scene &scene, LazyDashParams params) : scene_(scene), params_(params) {}

LazyDashResult LazyDash::solve() {
    const auto t_start = Clock::now();
    LazyDashResult result;
    result.resolution = params_.resolution;
    auto &stats = result.stats;

    // Layer 1 representation: H_T (capability-only closure).
    auto t = Clock::now();
    const BodyModels models(scene_);
    const Capabilities caps(scene_, scene_.makeArms());
    TaskHypergraph hg = TaskHypergraph::build(caps);
    Heuristic h(hg);
    DurationParams duration_params = params_.durations;
    duration_params.vmax = params_.vmax;
    const DurationModel durations(caps, hg, duration_params);
    stats.task_query += since(t);
    stats.ht_vertices = hg.graph().numVertices();
    stats.ht_arcs = hg.graph().numArcs();

    const MeshVerifier mesh(scene_);
    TransitionSampler sampler(hg, caps, models, mesh, params_.seed);
    TaskConflictDetection layer2(hg, sampler, models);
    MotionLayer layer3(hg, models, mesh, params_.roadmap, params_.seed);

    TaskConstraints ct;
    MotionConstraints cm;
    TaskSearchBudget task_budget = params_.task;
    double scale = 1.0;
    // Tentative C_T is lifted only when no new S_T exists: motion failures (tier 1)
    // first with larger motion budgets, then anchor failures (tier 2) with larger sampling budgets.
    double anchor_scale = 1.0;
    const auto lift = [&] {
        if (ct.numTentative(1) > 0) {
            stats.tentative_lifted += static_cast<int>(ct.liftTentative(1));
            scale += params_.lift_growth - 1.0; // grows linearly
            layer3.setBudgetScale(scale);
            result.log.push_back("lift tentative C_T (motion); motion budgets x" + std::to_string(scale));
        } else {
            // Re-test each set-aside transition at the larger budget (one sample
            // per handover key); lift only those that now have anchors.
            anchor_scale += params_.lift_growth - 1.0;
            sampler.setBudgetScale(anchor_scale);
            std::map<int, bool> retested; // arc -> anchors found
            std::map<std::tuple<int, int, int, int>, bool> by_key;
            const auto t_retest = Clock::now();
            const auto feasible = [&](int arc) {
                if (auto it = retested.find(arc); it != retested.end())
                    return it->second;
                if (since(t_start) > params_.time_limit)
                    return false; // out of time: kept set aside
                const auto &d = hg.graph().arc(arc).data;
                bool ok;
                if (d.type == TransitionType::Handover) {
                    const auto key = std::make_tuple(d.robot, d.grasp, d.robot2, d.grasp2);
                    auto it = by_key.find(key);
                    ok = it != by_key.end() ? it->second : by_key.emplace(key, sampler.sample(arc).has_value()).first->second;
                } else {
                    ok = sampler.sample(arc).has_value();
                }
                retested[arc] = ok;
                return ok;
            };
            const std::size_t lifted = ct.removeIf([&](const TaskConstraint &c) {
                return c.tentative && c.tier == 2 && (c.arc < 0 || feasible(c.arc));
            });
            stats.tentative_lifted += static_cast<int>(lifted);
            stats.task_conflicts += since(t_retest); // the re-test is layer-2 sampling
            result.log.push_back("lift tentative C_T (anchors): " + std::to_string(lifted) +
                                 " lifted after re-test; sampling budgets x" + std::to_string(anchor_scale));
        }
    };
    const auto addTask = [&](const std::vector<TaskConstraint> &cs) {
        bool changed = false;
        for (const auto &c : cs) {
            if (ct.add(c)) {
                changed = true;
                ++stats.task_constraints;
                result.log.push_back(c.origin);
            }
        }
        return changed;
    };
    // Task-space expansion: an object that blocks the same transition wherever it
    // can rest (start, goal, every move-out) gets a new move-out pose.
    const auto expandTaskSpace = [&] {
        // (blocked head, its source, object) -> the object's elements that block it
        std::map<std::tuple<int, int, int>, std::set<int>> resting_blockers;
        for (const auto &c : ct.all()) {
            if (c.kind != TaskConstraint::Kind::Frontier)
                continue;
            const auto &pre = hg.graph().vertex(c.pre);
            if (pre.kind == ElementKind::ObjectAt)
                resting_blockers[{c.post, c.via, pre.object}].insert(c.pre);
        }
        std::set<int> flagged;
        for (const auto &[key, elements] : resting_blockers) {
            const int b = std::get<2>(key);
            const auto &outs = hg.moveOutElements(b);
            bool everywhere = static_cast<int>(outs.size()) < params_.max_move_outs &&
                              elements.count(hg.startElement(b)) && elements.count(hg.goalElement(b));
            for (int m : outs)
                everywhere = everywhere && elements.count(m);
            if (everywhere)
                flagged.insert(b);
        }
        bool grew = false;
        for (int b : flagged) {
            std::vector<Eigen::Vector3d> occupied; // every other object's places
            for (const auto &loc : hg.locations())
                if (loc.object != b)
                    for (const auto &p : loc.poses)
                        occupied.push_back(p.translation());
            const auto pose = sampler.sampleMoveOut(b, occupied);
            if (!pose) {
                result.log.push_back("task space: no move-out pose for o" + std::to_string(b));
                continue;
            }
            const int arcs_before = hg.graph().numArcs();
            const int e = hg.expandMoveOut(caps, b, *pose);
            ++stats.move_outs;
            grew = true;
            std::ostringstream s;
            s << "task space: " << hg.describe(e) << " at (" << pose->translation().transpose() << "), "
              << hg.graph().numArcs() - arcs_before << " new transitions";
            result.log.push_back(s.str());
            if (params_.verbose)
                for (int a = arcs_before; a < hg.graph().numArcs(); ++a)
                    result.log.push_back("  + " + hg.describeArc(a));
        }
        if (grew) {
            h = Heuristic(hg);
            stats.ht_vertices = hg.graph().numVertices();
            stats.ht_arcs = hg.graph().numArcs();
        }
        return grew;
    };
    const auto out_of_time = [&] { return since(t_start) > params_.time_limit; };
    const auto progress = [&](const std::string &what) {
        if (params_.verbose)
            std::cerr << "[" << since(t_start) << " s] " << what << std::endl;
    };

    for (int outer = 0; outer < params_.max_outer && !out_of_time(); ++outer) {
        ++stats.outer_iterations;

        // Layer 1: the task query, on H_T expanded first if C_T contradicts.
        t = Clock::now();
        if (expandTaskSpace())
            progress("outer " + std::to_string(outer) + ": task space expanded (" +
                     std::to_string(stats.move_outs) + " move-outs)");
        TaskSearch search(hg, h, durations);
        TaskSearchBudget query_budget = task_budget;
        query_budget.time_limit = std::min(query_budget.time_limit, params_.time_limit - since(t_start));
        const auto st = search.query(ct, query_budget);
        stats.task_query += since(t);
        stats.hte_expansions += search.stats().expansions;
        progress("outer " + std::to_string(outer) + ": task query " + (st ? "found S_T" : "failed") + " (" +
                 std::to_string(search.stats().expansions) + " expansions, C_T " + std::to_string(ct.size()) + ")");
        if (!st) {
            if (search.budgetExhausted()) {
                // Not a proof that H_T is exhausted: retry with a larger budget and perturbed order.
                task_budget.max_expansions *= 2;
                task_budget.perturbation = params_.seed * 7919ull + static_cast<std::uint64_t>(outer) + 1;
                result.log.push_back("task query budget spent; retry with " +
                                     std::to_string(task_budget.max_expansions) + " expansions");
                if (ct.numTentative() > 0)
                    lift();
                continue;
            }
            if (ct.numTentative() > 0) { // tentative bans may hide the solution
                lift();
                continue;
            }
            result.failure = "H_T exhausted under C_T";
            break;
        }

        // Layer 2: task conflict detection.
        t = Clock::now();
        const auto l2 = layer2.run(*st);
        result.log.insert(result.log.end(), l2.log.begin(), l2.log.end());
        stats.task_conflicts += since(t);
        progress("  layer 2: " + std::to_string(l2.constraints.size()) + " constraints, " +
                 std::to_string(l2.resamples) + " resamples, " + std::to_string(since(t)) + " s");
        if (!l2.valid()) {
            if (!addTask(l2.constraints)) { // nothing new: only a lift can change the next S_T
                if (ct.numTentative() == 0) {
                    result.failure = "layer 2 repeats a known conflict";
                    break;
                }
                lift();
            }
            continue;
        }

        // Layers 3-4: the motion loop for this S_T.
        std::optional<TimedSchedule> previous;
        for (int inner = 0; inner < params_.max_inner && !out_of_time(); ++inner) {
            ++stats.inner_iterations;
            t = Clock::now();
            auto l3 = layer3.plan(*st, l2, cm);
            stats.motion_query += since(t);
            progress("  inner " + std::to_string(inner) + ": layer 3 " + (l3.schedule ? "S_M" : "failed") + " (" +
                     std::to_string(l3.stats.queries) + " queries, " + std::to_string(l3.stats.seconds) + " s)");
            stats.roadmap_vertices = layer3.roadmapVertices();
            if (!l3.schedule) {
                for (const auto &line : l3.log)
                    result.log.push_back(line);
                // The resampling rule of MotionHg: anchors this move had to join are resampled.
                for (int arc : l3.failed_arcs)
                    layer2.invalidate(arc);
                if (!addTask(l3.constraints)) {
                    ++stats.known_failure_repeats;
                    if (ct.numTentative() == 0) {
                        result.failure = "layer 3 repeats a known failure";
                        stats.motion_constraints = static_cast<int>(cm.size());
                        stats.total = since(t_start);
                        return result;
                    }
                    lift();
                }
                break; // C_T changed: back to layer 1
            }
            t = Clock::now();
            TimedSchedule schedule(*l3.schedule, *st, hg, models, params_.resolution, params_.vmax);
            if (previous)
                schedule.inheritDelays(*previous);
            Layer4Params l4params = params_.layer4;
            l4params.time_limit = std::max(1.0, params_.time_limit - since(t_start));
            MotionConflicts layer4(hg, *st, models, mesh, l4params, params_.seed + inner);
            auto l4 = layer4.resolve(schedule);
            stats.conflict_resolution += since(t);
            progress("  layer 4: valid " + std::to_string(l4.valid) + ", h-h " + std::to_string(l4.hh) + ", h-v " +
                     std::to_string(l4.hv) + ", waits " + std::to_string(l4.timing_fixes) + ", ARC " +
                     std::to_string(l4.arc_repairs) + ", C_M " + std::to_string(l4.cm.size()) + ", C_T " +
                     std::to_string(l4.ct.size()) + ", makespan " + std::to_string(schedule.makespan()) + ", " +
                     std::to_string(l4.seconds) + " s");
            stats.hh += l4.hh;
            stats.hv += l4.hv;
            stats.timing_fixes += l4.timing_fixes;
            stats.arc_repairs += l4.arc_repairs;
            stats.mesh_conflicts += l4.mesh_conflicts;
            for (const auto &line : l4.log)
                result.log.push_back(line);
            if (l4.valid) {
                result.success = true;
                stats.hm_vertices = l3.hm.numVertices();
                stats.hm_arcs = l3.hm.numArcs();
                stats.makespan = schedule.makespan();
                for (const auto &step : st->steps)
                    result.task_plan.push_back(hg.describeArc(step.arc));
                for (int r = 0; r < schedule.numRobots(); ++r)
                    result.robot_paths.push_back(schedule.robotSlice(r, 0, stats.makespan));
                for (int o = 0; o < schedule.numObjects(); ++o)
                    result.object_paths.push_back(schedule.objectSlice(o, 0, stats.makespan));
                for (const auto &s : schedule.segments()) {
                    if (s.seg.robot < 0 || s.seg.step < 0)
                        continue;
                    const auto &t = hg.graph().arc(st->steps[static_cast<std::size_t>(s.seg.step)].arc).data;
                    const char *action = t.type == TransitionType::Pick    ? "pick"
                                         : t.type == TransitionType::Place ? "place"
                                                                           : "handover";
                    result.timeline.push_back({s.seg.robot, s.start, s.end(),
                                               s.seg.kind == MotionSegment::Kind::Stage, action, t.object,
                                               s.seg.holding_grasp >= 0, s.seg.step});
                }
                stats.motion_constraints = static_cast<int>(cm.size());
                stats.total = since(t_start);
                return result;
            }
            previous.emplace(schedule);
            bool grew = false;
            for (auto &m : l4.cm)
                grew = cm.add(std::move(m)) || grew;
            if (!l4.ct.empty() && addTask(l4.ct))
                break; // back to layer 1
            if (!grew) {
                if (out_of_time()) // layer 4 stopped at the run's deadline
                    break;
                // Nothing new for layer 3: set this S_T aside (tentatively).
                const int consumer = l4.cm.empty() ? -1 : l4.cm.front().consumer_arc;
                bool added =
                    consumer >= 0 &&
                    addTask({{TaskConstraint::Kind::Frontier, l4.cm.front().element, hg.objectHead(consumer),
                              "layer4: unresolved conflict persists under C_M", true}});
                // An object resting at a move-out pose blocks the move: order the
                // transition after the object leaves (h-v mapping C_f(B@q, head)).
                if (consumer >= 0) {
                    const int context = l4.cm.front().context;
                    const auto &e = hg.graph().vertex(context);
                    const auto &arc = hg.graph().arc(consumer);
                    if (e.kind == ElementKind::ObjectAt &&
                        hg.locations()[static_cast<std::size_t>(e.location)].kind == Location::Kind::MoveOut &&
                        std::find(arc.tail.begin(), arc.tail.end(), context) == arc.tail.end())
                        added = addTask({{TaskConstraint::Kind::Frontier, context, hg.objectHead(consumer),
                                          "layer4: move persists against " + hg.describe(context), true}}) ||
                                added;
                }
                result.log.push_back("layer4: no progress within its budget");
                if (!added) { // the same S_T would come back unchanged
                    ++stats.known_failure_repeats;
                    if (ct.numTentative() == 0) {
                        result.failure = "layer 4 repeats a known failure";
                        stats.motion_constraints = static_cast<int>(cm.size());
                        stats.total = since(t_start);
                        return result;
                    }
                    lift();
                }
                break;
            }
        }
    }
    if (result.failure.empty())
        result.failure = out_of_time() ? "time limit" : "iteration budget";
    stats.motion_constraints = static_cast<int>(cm.size());
    stats.total = since(t_start);
    return result;
}

} // namespace lazydash
