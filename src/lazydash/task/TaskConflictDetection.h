#pragma once

#include "lazydash/robot/BodyModels.h"
#include "lazydash/task/TaskConstraints.h"
#include "lazydash/task/TaskHypergraph.h"
#include "lazydash/task/TaskSearch.h"
#include "lazydash/task/TransitionSampler.h"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace lazydash {

/// Output of layer 2 for one S_T.
struct TaskConflictResult {
    /// Anchors per S_T step (empty for the source and sink steps).
    std::vector<std::optional<TransitionAnchors>> anchors;
    /// Constraints emitted, from all conflicts found (not only the first).
    std::vector<TaskConstraint> constraints;
    int conflicts = 0;  // vertex-vertex conflicts found
    int resamples = 0;  // anchors resampled to avoid a conflict
    std::vector<std::string> log;

    bool valid() const { return constraints.empty(); }
};

/// Layer 2, task conflict detection (Alg. 2). Walks S_T with the frontier of
/// resting objects and checks each transition's anchors for vertex-vertex
/// conflicts; unresolved ones emit C_f(B@q, head of the blocked transition).
class TaskConflictDetection {
public:
    TaskConflictDetection(const TaskHypergraph &hg, TransitionSampler &sampler,
                          const BodyModels &models, int resample_budget = 5);

    TaskConflictResult run(const TaskSchedule &st);

    /// Forget the cached anchors of `arc` so its next use samples new ones.
    void invalidate(int arc) {
        cache_.erase(arc);
        branches_.erase(arc);
    }

private:
    struct Resting {
        int element;
        Eigen::Vector3d position;
    };
    /// Resting objects the anchors of `a` collide with (other than its own object).
    std::vector<int> blockers(const TransitionAnchors &a, const std::map<int, Resting> &resting) const;

    const TaskHypergraph &hg_;
    TransitionSampler &sampler_;
    const BodyModels &models_;
    int resample_budget_;
    std::map<int, TransitionAnchors> cache_;
    std::map<int, std::vector<TransitionAnchors>> branches_; // pick/place: every IK branch
};

} // namespace lazydash
