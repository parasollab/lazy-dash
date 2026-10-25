#include "lazydash/schedule/ScheduledARC.h"

#include <chrono>
#include <stdexcept>

namespace lazydash {

ScheduledARC::Outcome ScheduledARC::repairPaths(std::vector<comotion::Path> &paths,
                                                double time_limit,
                                                int max_repairs) {
    if (!problem_)
        throw std::logic_error("ScheduledARC: setProblem() first");
    if (paths.size() != static_cast<std::size_t>(problem_->numRobots()))
        throw std::invalid_argument("ScheduledARC: one path per problem robot");

    resetArcSolveState();
    initializeConflictScanStarts(paths.size());
    const auto start = Clock::now();
    const auto robots = problem_->robotModelPtrs();

    Outcome outcome;
    while (true) {
        // Full rescan after every splice: repairs shift suffixes in time, so a
        // conflict-free prefix can change.
        comotion::ConflictChecker checker(problem_->collisionChecker());
        comotion::CompositePathValidationOptions options;
        options.check_environment = false;
        const auto conflict = checker.findConflict(paths, robots, options);
        if (!conflict) {
            outcome.conflict_free = true;
            return outcome;
        }
        const auto subproblem = expandConflictForSubproblem(*conflict);
        const double elapsed =
            std::chrono::duration<double>(Clock::now() - start).count();
        if (outcome.repairs >= max_repairs || elapsed >= time_limit)
            return outcome;
        const auto repair = resolveConflictOnPaths(subproblem, start, time_limit, paths);
        if (!repair.resolved)
            return outcome;
        ++outcome.repairs;
    }
}

} // namespace lazydash
