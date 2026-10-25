#pragma once

#include "lazydash/motion/LazyRoadmap.h"
#include "lazydash/schedule/MotionConflicts.h"
#include "lazydash/schedule/TimedSchedule.h"
#include "lazydash/scene/Scene.h"
#include "lazydash/task/TaskSearch.h"

#include "comotion/planning/Path.h"

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lazydash {

struct LazyDashParams {
    std::uint64_t seed = 1;
    double time_limit = 300.0;       // s, whole run
    int max_outer = 200;             // task iterations (layer 1)
    int max_inner = 30;              // motion iterations per S_T (layers 3-4)
    double lift_growth = 2.0;        // budget growth per lift of tentative C_T
    int max_move_outs = 2;           // move-out poses per object (task-space expansion)
    std::size_t resolution = 128;    // timesteps per second of the schedule
    double vmax = 1.0;               // rad/s per joint
    DurationParams durations;        // task-query duration estimates
    RoadmapParams roadmap;
    Layer4Params layer4;
    TaskSearchBudget task;
    bool verbose = false;            // stream progress to stderr
};

/// Statistics of a run; per-layer clocks in seconds.
struct LazyDashStats {
    int outer_iterations = 0, inner_iterations = 0;
    int task_constraints = 0, tentative_lifted = 0, motion_constraints = 0;
    int known_failure_repeats = 0; // layer-3/4 failures that added nothing to C_T
    int move_outs = 0;             // move-out poses added to H_T
    int ht_vertices = 0, ht_arcs = 0;      // H_T
    std::size_t hte_expansions = 0;        // H_TE vertices expanded, summed over queries
    int hm_vertices = 0, hm_arcs = 0;      // H_M of the final S_T
    std::size_t roadmap_vertices = 0;
    int hh = 0, hv = 0, timing_fixes = 0, arc_repairs = 0, mesh_conflicts = 0;
    double task_query = 0, task_conflicts = 0, motion_query = 0, conflict_resolution = 0;
    double total = 0;
    std::size_t makespan = 0;              // timesteps
};

/// One timed segment of an arm in the final schedule (for Gantt charts).
struct TimelineEntry {
    int robot = -1;
    std::size_t start = 0, end = 0; // timesteps
    bool stage = false;             // a transition stage (else a move)
    std::string action;             // the step's transition: pick, place or handover
    int object = -1;                // the step's object
    bool holding = false;           // the arm carries an object during the segment
    int step = -1;                  // S_T step
};

struct LazyDashResult {
    bool success = false;
    std::string failure;                    // why, when not successful
    LazyDashStats stats;
    std::vector<std::string> log;
    std::vector<std::string> task_plan;     // S_T, one transition per line
    std::vector<comotion::Path> robot_paths;  // per arm, dense over [0, makespan]
    std::vector<comotion::Path> object_paths; // per object, dense positions
    std::vector<TimelineEntry> timeline;      // every arm segment, timed
    std::size_t resolution = 128;             // timesteps per second of the paths
};

/// Lazy-DaSH (Alg. 1): task query, task conflict detection, motion query and
/// conflict resolution, with their loops. Each loop has a budget; every failure
/// adds a scoped constraint or grows a representation.
class LazyDash {
public:
    explicit LazyDash(const Scene &scene, LazyDashParams params = {});
    LazyDashResult solve();

private:
    const Scene &scene_;
    LazyDashParams params_;
};

} // namespace lazydash
