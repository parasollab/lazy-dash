#pragma once

#include "lazydash/robot/GraspSet.h"
#include "lazydash/robot/UrIk.h"
#include "lazydash/scene/Scene.h"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

namespace lazydash {

/// Context-free capability checks that admit transitions into H_T: reach
/// (base distance), hold (a static-scene-valid IK solution at grasp and
/// approach), and handover (bases close enough, opposite-face grasps).
class Capabilities {
public:
    Capabilities(const Scene &scene, std::vector<std::shared_ptr<comotion::RobotModel>> arms);

    int numRobots() const { return static_cast<int>(arms_.size()); }
    int numGrasps() const { return static_cast<int>(scene_.grasps.size()); }

    bool reaches(int robot, const Eigen::Vector3d &point) const;
    bool canHold(int robot, int grasp, const Eigen::Isometry3d &object_pose) const;
    bool canHandOver(int giver, int receiver) const;
    /// Grasps whose approach axes are opposite in the object frame.
    bool compatibleGrasps(int giver_grasp, int receiver_grasp) const;

    /// All IK branches of `robot` holding the object with `grasp` at `stage`
    /// that are valid against the static scene.
    std::vector<std::vector<double>> holdConfigs(int robot, int grasp,
                                                 const Eigen::Isometry3d &object_pose,
                                                 GraspSet::Stage stage) const;

    const Scene &scene() const { return scene_; }
    const comotion::RobotModel &arm(int robot) const { return *arms_.at(static_cast<std::size_t>(robot)); }
    const std::shared_ptr<comotion::RobotModel> &armPtr(int robot) const {
        return arms_.at(static_cast<std::size_t>(robot));
    }
    const GraspSet &graspSet(int robot) const { return grasp_sets_.at(static_cast<std::size_t>(robot)); }
    const comotion::CollisionChecker &checker() const { return checker_; }
    double baseDistance(int a, int b) const;

private:
    const Scene &scene_;
    std::vector<std::shared_ptr<comotion::RobotModel>> arms_;
    std::vector<UrIk> ik_;
    std::vector<GraspSet> grasp_sets_;
    comotion::CollisionChecker checker_;
    std::vector<Eigen::Vector3d> approach_in_object_; // per grasp
    // (robot, grasp, quantized pose) -> result
    mutable std::map<std::tuple<int, int, std::vector<long>>, bool> hold_cache_;
};

} // namespace lazydash
