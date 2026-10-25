#pragma once

#include "lazydash/robot/Formation.h"

#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace lazydash {

/// One grasp as a scene defines it: the object frame in `link` when holding it
/// and at the approach (pre-grasp / lift) offset.
struct GraspDef {
    std::string name;
    std::string link;
    Eigen::Isometry3d link_T_object = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d link_T_object_approach = Eigen::Isometry3d::Identity();
};

/// The grasp set G of a manipulator, a scene parameter. Definitions on any link rigid
/// with the end effector are re-expressed on the link that carries the object.
class GraspSet {
public:
    enum class Stage { Grasp, Approach };

    GraspSet(const comotion::RobotModel &arm, std::vector<GraspDef> defs,
             std::string ee_link, std::vector<std::string> contact_links,
             std::vector<std::string> hand_links = {});

    std::size_t size() const { return defs_.size(); }

    /// The formation grasp g for grasp `i` (end-effector to object).
    Grasp grasp(std::size_t i) const;

    /// World pose of the end-effector link holding an object at `world_T_object`.
    Eigen::Isometry3d eePose(std::size_t i, const Eigen::Isometry3d &world_T_object,
                             Stage stage = Stage::Grasp) const;

    const std::string &eeLink() const { return ee_link_; }

private:
    std::vector<GraspDef> defs_;
    std::string ee_link_;
    std::vector<std::string> contact_links_, hand_links_;
    std::vector<Eigen::Isometry3d> ee_T_object_, ee_T_object_approach_;
};

} // namespace lazydash
