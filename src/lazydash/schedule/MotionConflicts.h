#pragma once

#include "lazydash/motion/MotionConstraints.h"
#include "lazydash/robot/BodyModels.h"
#include "lazydash/schedule/MeshVerifier.h"
#include "lazydash/schedule/TimedSchedule.h"
#include "lazydash/task/TaskConstraints.h"
#include "lazydash/task/TaskHypergraph.h"
#include "lazydash/task/TaskSearch.h"

#include "comotion/robot/RobotModel.h"

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lazydash {

struct Layer4Params {
    int max_iterations = 300;
    std::size_t max_delay = 4000;   // timesteps of waiting one segment may accumulate
    int max_timing_tries = 3;       // timing fixes per conflicting pair before replanning
    double arc_time = 2.0;          // s per ScheduledARC subproblem
    int arc_max_repairs = 10;
    double region_margin = 0.01;    // m added to C_M regions
    double time_limit = 1e9;        // s for one resolve() (the run's remaining time)
};

struct Layer4Result {
    bool valid = false;                    // conflict-free on both validity levels
    std::vector<MotionConstraint> cm;      // new C_M -> layer 3
    std::vector<TaskConstraint> ct;        // new C_T (anchored stages) -> layer 1
    int hh = 0, hv = 0, timing_fixes = 0, arc_repairs = 0, mesh_conflicts = 0;
    std::vector<std::string> log;
    double seconds = 0.0;
};

/// Layer 4, motion conflict resolution (Alg. 3): ScheduledARC over the timed
/// schedule, per epoch, earliest conflict first. h-h: timing fix, else ARC repair,
/// else C_M to layer 3; h-v: C_M for the mover (C_f directly inside anchored stages).
class MotionConflicts {
public:
    MotionConflicts(const TaskHypergraph &hg, const TaskSchedule &st, const BodyModels &models,
                    const MeshVerifier &mesh, Layer4Params params = {}, std::uint64_t seed = 1);

    Layer4Result resolve(TimedSchedule &schedule);

    struct Bodies {
        std::vector<const comotion::RobotModel *> robots;
        std::vector<std::shared_ptr<comotion::RobotModel>> owned;
        std::vector<int> arm;    // robot index -> arm (or -1)
        std::vector<comotion::Path> paths;
    };
    /// The bodies of the epoch starting at t0 (timesteps t0..t1), on the
    /// planning (`mesh` false) or verification level.
    Bodies epochBodies(const TimedSchedule &schedule, std::size_t t0, std::size_t t1, bool mesh) const;

    struct Found {
        std::size_t t = 0;
        int arm_a = -1, arm_b = -1; // arms involved
        int object = -1;            // resting object involved (h-v)
        bool mesh = false;
    };
    /// The earliest conflict at or after `from` (earlier epochs are skipped).
    std::optional<Found> firstConflict(const TimedSchedule &schedule, bool mesh,
                                       std::size_t from = 0) const;

private:
    // On success, `changed` is the earliest time whose paths changed.
    bool tryTiming(TimedSchedule &schedule, const Found &c, Layer4Result &out, std::size_t &changed);
    bool tryArc(TimedSchedule &schedule, const Found &c, Layer4Result &out, std::size_t &changed);
    std::vector<comotion::ObstacleSphere> armRegion(const TimedSchedule &schedule, int arm, std::size_t t) const;

    const TaskHypergraph &hg_;
    const TaskSchedule &st_;
    const BodyModels &models_;
    const MeshVerifier &mesh_;
    Layer4Params params_;
    std::uint64_t seed_;
    comotion::CollisionChecker spheres_;
    std::map<std::pair<int, int>, int> timing_tries_;
};

} // namespace lazydash
