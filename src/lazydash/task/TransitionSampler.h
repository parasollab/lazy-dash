#pragma once

#include "lazydash/robot/BodyModels.h"
#include "lazydash/schedule/MeshVerifier.h"
#include "lazydash/task/Capabilities.h"
#include "lazydash/task/TaskHypergraph.h"

#include "comotion/collision/CollisionChecker.h"

#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <optional>
#include <tuple>
#include <random>
#include <vector>

namespace lazydash {

/// The anchors of a transition's E^tran hyperarc: approach and grasp
/// configurations for each arm, sampled by layer 2 and reused by layer 3.
struct TransitionAnchors {
    int arc = -1;
    TransitionType type = TransitionType::Pick;
    int object = -1;
    Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
    int robot = -1, grasp = -1; // pick/place robot, or handover giver
    std::vector<double> q_approach, q_grasp;
    int robot2 = -1, grasp2 = -1; // handover receiver
    std::vector<double> q2_approach, q2_grasp;

    /// Anchors not tied to a start or goal pose can be resampled (handovers).
    bool resampleable() const { return type == TransitionType::Handover; }
};

/// Samples anchors for the transitions of S_T whose configurations and
/// straight stage motions are valid against the static scene and the arms.
/// Collisions with resting objects are left to layer 2.
class TransitionSampler {
public:
    TransitionSampler(const TaskHypergraph &hg, const Capabilities &caps, const BodyModels &models,
                      const MeshVerifier &mesh, std::uint64_t seed);

    std::optional<TransitionAnchors> sample(int arc);
    /// Every anchor of a pick or place: one per valid IK branch, in the order
    /// `sample` would try them.
    std::vector<TransitionAnchors> pickPlaceBranches(int arc);

    /// Multiplies the handover pose budget; grown when tentative constraints
    /// are lifted.
    void setBudgetScale(double scale) { budget_scale_ = scale; }

    /// Whether `robot` at `q` (carrying with `holding_grasp` if >= 0) is clear
    /// of `object` resting at `position`.
    bool clearOfObject(int robot, int holding_grasp, const std::vector<double> &q, int object,
                       const Eigen::Vector3d &position) const;

    /// A move-out pose for `object`: uniform in a placement region, clear of
    /// the static scene and of `occupied`, with pick/place anchors for some
    /// robot. None if `attempts` samples fail.
    std::optional<Eigen::Isometry3d> sampleMoveOut(int object, const std::vector<Eigen::Vector3d> &occupied,
                                                   int attempts = 1000);

private:
    /// The valid pick/place anchors in branch order, at most `limit` of them.
    std::vector<TransitionAnchors> pickPlace(int arc, std::size_t limit);
    std::optional<TransitionAnchors> sampleHandover(int arc);
    /// (approach, grasp) IK pairs on one branch with valid straight stage motions.
    std::vector<std::pair<std::vector<double>, std::vector<double>>>
    stagePairs(int robot, int grasp, const Eigen::Isometry3d &object_pose);
    bool lineValid(const comotion::RobotModel &model, const comotion::RobotModel &mesh_model,
                   const std::vector<double> &a, const std::vector<double> &b) const;
    bool pairLineValid(const comotion::RobotModel &moving, const std::vector<double> &a,
                       const std::vector<double> &b, const comotion::RobotModel &fixed,
                       const std::vector<double> &q_fixed,
                       const comotion::CollisionChecker &checker) const;
    Eigen::Isometry3d sampleHandoverPose(int giver, int giver_grasp, int receiver);

    const TaskHypergraph &hg_;
    const Capabilities &caps_;
    const BodyModels &models_;
    const MeshVerifier &mesh_;
    comotion::CollisionChecker spheres_;
    std::mt19937_64 rng_;
    std::mt19937_64 move_out_rng_; // own stream: move-outs leave anchor sampling unchanged
    double budget_scale_ = 1.0;
    // Handover failures per (giver, grasp, receiver, grasp) and the budget
    // scale they failed at; shared by all objects (one collision sphere).
    std::map<std::tuple<int, int, int, int>, double> handover_failed_;
};

} // namespace lazydash
