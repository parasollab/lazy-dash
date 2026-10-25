#pragma once

#include "lazydash/schedule/MeshVerifier.h"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ObstacleShapes.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/RobotModel.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <vector>

namespace lazydash {

/// Budgets of a roadmap query.
struct RoadmapParams {
    int initial_samples = 10;     // valid samples added when the roadmap is created
    int k = 10;                   // lazy edges per new vertex (nearest neighbours)
    int local_samples = 30;       // rung 2: samples near invalidated edges, per round
    double local_sigma = 0.3;     // rad, spread of local samples
    int global_samples = 30;      // rung 3: uniform samples, per round
    int rounds = 3;               // local + global rounds before the RRT rung
    double rrt_time = 2.0;        // s, rung 4: CoMotion CompositeRRT
    double time_limit = 5.0;      // s, per query
    double edge_step = MeshVerifier::kCheckStep; // rad per collision check along an edge
    int shortcut_checks = 64;     // straight motions tried per query to shortcut the found path
};

/// Lazy roadmap (Lazy-PRM) for one composition space (an arm, or an arm carrying
/// an object); answers E^move queries. Edges are checked only when a query uses
/// them; repair ladder: lazy query, local samples, global samples, CompositeRRT.
class LazyRoadmap {
public:
    using Config = std::vector<double>;

    struct QueryStats {
        int edges_checked = 0;
        int mesh_rejections = 0;
        int local_samples = 0;
        int global_samples = 0;
        bool rrt_used = false;
        double seconds = 0.0;
    };

    LazyRoadmap(std::shared_ptr<const comotion::RobotModel> model,
                std::shared_ptr<const comotion::RobotModel> mesh_model,
                const comotion::CollisionChecker &scene_checker, const MeshVerifier &mesh,
                std::vector<double> lower, std::vector<double> upper, std::uint64_t seed,
                RoadmapParams params = {});

    std::optional<comotion::Path> query(const Config &from, const Config &to,
                                        const std::vector<comotion::ObstacleSphere> &extra = {},
                                        QueryStats *stats = nullptr);

    /// Validity of a configuration / straight motion on sphere and mesh levels,
    /// plus `extra` (C_M) regions on the sphere level.
    bool configValid(const Config &q, const std::vector<comotion::ObstacleSphere> &extra = {}) const;
    bool motionValid(const Config &a, const Config &b,
                     const std::vector<comotion::ObstacleSphere> &extra = {}) const;

    /// Multiplies the query time and sampling rounds (grown on lifts).
    void setBudgetScale(double scale) { scale_ = scale; }

    int numVertices() const { return static_cast<int>(vertices_.size()); }
    int numEdges() const { return static_cast<int>(edges_.size()); }
    const comotion::RobotModel &model() const { return *model_; }

private:
    enum class Status { Unknown, Valid, Invalid };
    struct Edge {
        int a, b;
        double length;
        Status status = Status::Unknown;
    };

    int addVertex(const Config &q);
    int findVertex(const Config &q) const;
    int edgeBetween(int a, int b) const;
    void addEdge(int a, int b, Status status);
    std::optional<std::vector<int>> shortestPath(int from, int to,
                                                 const std::vector<char> &blocked_for_query) const;
    bool sampleValid(Config &q, const Config *around, double sigma);
    comotion::Path shortcut(const std::vector<int> &vertex_path, const comotion::CollisionChecker *with_extra);
    bool sphereMotionValid(const Config &a, const Config &b,
                           const comotion::CollisionChecker &checker) const;
    std::optional<comotion::Path> rrt(const Config &from, const Config &to,
                                      const std::vector<comotion::ObstacleSphere> &extra);
    static double distance(const Config &a, const Config &b);

    std::shared_ptr<const comotion::RobotModel> model_, mesh_model_;
    const comotion::CollisionChecker &scene_checker_;
    const MeshVerifier &mesh_;
    std::vector<double> lower_, upper_;
    std::mt19937_64 rng_;
    RoadmapParams params_;
    double scale_ = 1.0;

    std::vector<Config> vertices_;
    std::vector<std::vector<int>> adjacency_; // vertex -> edge ids
    std::vector<Edge> edges_;
};

} // namespace lazydash
