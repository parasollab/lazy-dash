#pragma once

#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/ARC.h"

#include <optional>
#include <vector>

namespace lazydash {

/// CoMotion's ARC applied to an externally built schedule (Alg. 3 local repair):
/// skips ARC's initial planning and runs conflict-find and windowed subproblem
/// repair on caller-owned timed paths, once per epoch between formation changes.
class ScheduledARC : public comotion::ARC {
public:
    struct Outcome {
        bool conflict_free = false;
        int repairs = 0;
    };

    /// Repair inter-robot conflicts in `paths` in place, until conflict-free,
    /// a repair fails, `max_repairs` is reached, or `time_limit` seconds pass.
    Outcome repairPaths(std::vector<comotion::Path> &paths, double time_limit,
                        int max_repairs);

    std::string name() const override { return "ScheduledARC"; }
};

} // namespace lazydash
