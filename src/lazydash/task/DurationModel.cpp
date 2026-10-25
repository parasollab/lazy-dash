#include "lazydash/task/DurationModel.h"

#include "lazydash/robot/UrIk.h"

#include <algorithm>
#include <cmath>

namespace lazydash {

DurationModel::DurationModel(const Capabilities &caps, const TaskHypergraph &hg, DurationParams params)
    : scene_(caps.scene()), hg_(hg), params_(params) {
    for (int r = 0; r < caps.numRobots(); ++r)
        home_.push_back(UrIk(caps.armPtr(r)).linkPose(scene_.arms[static_cast<std::size_t>(r)].start).translation());
}

DurationModel::DurationModel(const Scene &scene, const TaskHypergraph &hg, std::vector<Eigen::Vector3d> home,
                             DurationParams params)
    : scene_(scene), hg_(hg), params_(params), home_(std::move(home)) {}

Eigen::Vector3d DurationModel::point(int arc) const {
    const auto &t = hg_.graph().arc(arc).data;
    if (t.type == TransitionType::Handover) {
        const auto &p = scene_.params;
        const Eigen::Vector3d mid = 0.5 * (scene_.arms[static_cast<std::size_t>(t.robot)].base.position +
                                           scene_.arms[static_cast<std::size_t>(t.robot2)].base.position);
        return mid + Eigen::Vector3d(0.0, 0.0, 0.5 * (p.handover_z_min + p.handover_z_max));
    }
    if (t.location < 0)
        return Eigen::Vector3d::Zero();
    return hg_.locations()[static_cast<std::size_t>(t.location)].poses[static_cast<std::size_t>(t.pose)].translation();
}

double DurationModel::travel(int robot, const Eigen::Vector3d &from, const Eigen::Vector3d &to) const {
    const Eigen::Vector3d &b = scene_.arms[static_cast<std::size_t>(robot)].base.position;
    const Eigen::Vector3d u = from - b, v = to - b;
    const double yaw = std::abs(std::remainder(std::atan2(v.y(), v.x()) - std::atan2(u.y(), u.x()), 2.0 * M_PI));
    const double reach = std::hypot(v.head<2>().norm() - u.head<2>().norm(), v.z() - u.z());
    return params_.detour * std::max(yaw, reach / params_.link_length) / params_.vmax;
}

} // namespace lazydash
