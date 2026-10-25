#pragma once

#include "lazydash/hypergraph/Hypergraph.h"
#include "lazydash/motion/LazyRoadmap.h"
#include "lazydash/motion/MotionConstraints.h"
#include "lazydash/motion/OptimisticSchedule.h"
#include "lazydash/robot/BodyModels.h"
#include "lazydash/schedule/MeshVerifier.h"
#include "lazydash/task/TaskConflictDetection.h"
#include "lazydash/task/TaskHypergraph.h"
#include "lazydash/task/TaskSearch.h"

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lazydash {

/// A vertex of H_M: v_M = <v_T, q>, per H_TE vertex of S_T (`te`) with the arm
/// configuration at that point.
struct MotionVertex {
    int te = -1;
    int element = -1;
    int robot = -1;
    std::vector<double> q;
};

/// A hyperarc of H_M: E^comp (bookkeeping), E^tran (one transition of S_T,
/// its stage segments) or E^move (one arm's move segment).
struct MotionArc {
    enum class Kind { Composition, Transition, Move };
    Kind kind = Kind::Composition;
    int step = -1;
    std::vector<int> segments; // ids into OptimisticSchedule::segments
};

using MotionHypergraph = Hypergraph<MotionVertex, MotionArc>;

struct MotionLayerStats {
    int queries = 0;
    int rrt = 0;
    int mesh_rejections = 0;
    int edges_checked = 0;
    double seconds = 0.0;
};

struct MotionLayerResult {
    std::optional<OptimisticSchedule> schedule; // set when every E^move was found
    std::vector<TaskConstraint> constraints;    // C_T emitted on failure
    std::vector<int> failed_arcs;               // H_T arcs whose anchors a failed move joined
    MotionHypergraph hm;                         // H_M of this S_T
    MotionLayerStats stats;
    std::vector<std::string> log;
};

/// Layer 3, motion planning (MotionHg and QueryMotionPlan): builds H_M for the current
/// S_T and answers each E^move with its composition space's lazy roadmap, adding
/// matching C_M regions as obstacles. A failed E^move emits C_T and returns.
class MotionLayer {
public:
    MotionLayer(const TaskHypergraph &hg, const BodyModels &models, const MeshVerifier &mesh,
                RoadmapParams params = {}, std::uint64_t seed = 1);

    MotionLayerResult plan(const TaskSchedule &st, const TaskConflictResult &layer2,
                           const MotionConstraints &cm);

    /// Total roadmap vertices (representation size, for lifting tentative C_T).
    std::size_t roadmapVertices() const;
    LazyRoadmap &roadmap(int robot, int grasp);
    /// Grow every roadmap query budget (on lifts of tentative C_T).
    void setBudgetScale(double scale);

private:
    /// Step a precedes-or-equals step b in the hyperpath (H_TE partial order).
    using Precedence = std::vector<std::vector<char>>;
    static Precedence precedence(const TaskSchedule &st);
    std::vector<comotion::ObstacleSphere> regions(const TaskSchedule &st, const std::vector<int> &consumers,
                                                  const Precedence &before, int te, int step,
                                                  const MotionConstraints &cm, int *context) const;

    const TaskHypergraph &hg_;
    const BodyModels &models_;
    const MeshVerifier &mesh_;
    RoadmapParams params_;
    std::uint64_t seed_;
    comotion::CollisionChecker scene_spheres_;
    std::map<std::pair<int, int>, std::unique_ptr<LazyRoadmap>> roadmaps_;
    double scale_ = 1.0;
    // (robot, grasp, from, to, region) -> path; reusable since static validity never changes.
    std::map<std::vector<long long>, comotion::Path> query_cache_;
};

} // namespace lazydash
