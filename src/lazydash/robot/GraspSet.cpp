#include "lazydash/robot/GraspSet.h"

#include <stdexcept>
#include <utility>

namespace lazydash {

namespace {

/// Fixed transform from link `a` to link `b` on one rigid body of `arm`.
Eigen::Isometry3d relative(const comotion::RobotModel &arm, const std::string &a,
                           const std::string &b) {
    const int ia = arm.linkIndex(a), ib = arm.linkIndex(b);
    if (ia < 0 || ib < 0)
        throw std::invalid_argument("GraspSet: unknown link " + (ia < 0 ? a : b));
    const auto T = arm.getLinkTransforms(
        std::vector<double>(static_cast<std::size_t>(arm.numJoints()), 0.0));
    return Eigen::Isometry3d((T[static_cast<std::size_t>(ia)].inverse() *
                              T[static_cast<std::size_t>(ib)]).matrix());
}

} // namespace

GraspSet::GraspSet(const comotion::RobotModel &arm, std::vector<GraspDef> defs,
                   std::string ee_link, std::vector<std::string> contact_links,
                   std::vector<std::string> hand_links)
    : defs_(std::move(defs)), ee_link_(std::move(ee_link)),
      contact_links_(std::move(contact_links)), hand_links_(std::move(hand_links)) {
    for (const auto &d : defs_) {
        const Eigen::Isometry3d ee_T_link = relative(arm, ee_link_, d.link);
        ee_T_object_.push_back(ee_T_link * d.link_T_object);
        ee_T_object_approach_.push_back(ee_T_link * d.link_T_object_approach);
    }
}

Grasp GraspSet::grasp(std::size_t i) const {
    return {ee_link_, ee_T_object_.at(i), contact_links_, hand_links_};
}

Eigen::Isometry3d GraspSet::eePose(std::size_t i, const Eigen::Isometry3d &world_T_object,
                                   Stage stage) const {
    const auto &ee_T_object =
        stage == Stage::Grasp ? ee_T_object_.at(i) : ee_T_object_approach_.at(i);
    return world_T_object * ee_T_object.inverse();
}

} // namespace lazydash
