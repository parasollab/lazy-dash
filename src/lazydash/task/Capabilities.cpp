#include "lazydash/task/Capabilities.h"

#include <cmath>
#include <utility>

namespace lazydash {

namespace {

/// Pose key at 0.1 mm / 1e-4 resolution for caching.
std::vector<long> poseKey(const Eigen::Isometry3d &T) {
    std::vector<long> key;
    for (int i = 0; i < 3; ++i)
        key.push_back(std::lround(T.translation()[i] * 1e4));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            key.push_back(std::lround(T.linear()(r, c) * 1e4));
    return key;
}

} // namespace

Capabilities::Capabilities(const Scene &scene,
                           std::vector<std::shared_ptr<comotion::RobotModel>> arms)
    : scene_(scene), arms_(std::move(arms)),
      checker_(comotion::CollisionChecker::Backend::Spheres) {
    scene_.applyObstacles(checker_);
    for (std::size_t r = 0; r < arms_.size(); ++r) {
        ik_.emplace_back(arms_[r]);
        ik_.back().setJointLimits(scene_.arms[r].lower, scene_.arms[r].upper);
        grasp_sets_.push_back(scene_.graspSet(*arms_[r]));
    }
    // The approach direction (tool z) seen from the object, per grasp.
    if (!grasp_sets_.empty()) {
        const auto &gs = grasp_sets_.front();
        for (std::size_t g = 0; g < gs.size(); ++g) {
            const Eigen::Isometry3d object_T_ee = gs.grasp(g).ee_T_object.inverse();
            approach_in_object_.push_back(object_T_ee.linear() * Eigen::Vector3d::UnitZ());
        }
    }
}

double Capabilities::baseDistance(int a, int b) const {
    return (scene_.arms.at(static_cast<std::size_t>(a)).base.position -
            scene_.arms.at(static_cast<std::size_t>(b)).base.position)
        .norm();
}

bool Capabilities::reaches(int robot, const Eigen::Vector3d &point) const {
    return (point - scene_.arms.at(static_cast<std::size_t>(robot)).base.position).norm() <=
           scene_.params.robot_range;
}

std::vector<std::vector<double>>
Capabilities::holdConfigs(int robot, int grasp, const Eigen::Isometry3d &object_pose,
                          GraspSet::Stage stage) const {
    const auto r = static_cast<std::size_t>(robot);
    const auto &gs = grasp_sets_.at(r);
    std::vector<std::vector<double>> valid;
    for (auto &q : ik_.at(r).solve(gs.eePose(static_cast<std::size_t>(grasp), object_pose, stage),
                                   gs.eeLink()))
        if (checker_.isValidSingleFull(*arms_.at(r), q))
            valid.push_back(std::move(q));
    return valid;
}

bool Capabilities::canHold(int robot, int grasp, const Eigen::Isometry3d &object_pose) const {
    if (!reaches(robot, object_pose.translation()))
        return false;
    const auto key = std::make_tuple(robot, grasp, poseKey(object_pose));
    if (auto it = hold_cache_.find(key); it != hold_cache_.end())
        return it->second;
    const bool ok =
        !holdConfigs(robot, grasp, object_pose, GraspSet::Stage::Grasp).empty() &&
        !holdConfigs(robot, grasp, object_pose, GraspSet::Stage::Approach).empty();
    hold_cache_.emplace(key, ok);
    return ok;
}

bool Capabilities::canHandOver(int giver, int receiver) const {
    return giver != receiver &&
           baseDistance(giver, receiver) <= scene_.params.handover_max_base_distance;
}

bool Capabilities::compatibleGrasps(int giver_grasp, int receiver_grasp) const {
    return approach_in_object_.at(static_cast<std::size_t>(giver_grasp))
               .dot(approach_in_object_.at(static_cast<std::size_t>(receiver_grasp))) < -0.9;
}

} // namespace lazydash
