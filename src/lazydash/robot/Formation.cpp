#include "lazydash/robot/Formation.h"

#include "comotion/robot/FlyingSphere.h"

#include <stdexcept>
#include <utility>

namespace lazydash {

std::shared_ptr<comotion::RobotModel> makeObject(double radius,
                                                 const Eigen::Vector3d &workspace_min,
                                                 const Eigen::Vector3d &workspace_max) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{workspace_min.x(), workspace_min.y(), workspace_min.z()},
        std::vector<double>{workspace_max.x(), workspace_max.y(), workspace_max.z()});
}

namespace {

/// Follower spheres in the follower's own frame (its configuration at the origin).
std::vector<comotion::CollisionSphere> localSpheres(const comotion::RobotModel &follower) {
    return follower.getCollisionSpheres(std::vector<double>(
        static_cast<std::size_t>(follower.numJoints()), 0.0));
}

Eigen::Affine3d linkPose(const comotion::RobotModel &robot, int link,
                         const std::vector<double> &q) {
    return robot.getLinkTransforms(q)[static_cast<std::size_t>(link)];
}

} // namespace

FormationModel::FormationModel(const comotion::RobotModel &leader,
                               const comotion::RobotModel &follower, const Grasp &grasp)
    : comotion::RobotModel(leader) {
    if (linkIndex(grasp.ee_link) < 0)
        throw std::invalid_argument("FormationModel: unknown link " + grasp.ee_link);
    if (linkIndex(kHeldObjectLink) >= 0)
        throw std::invalid_argument("FormationModel: leader already holds an object");
    // The follower is its own link so contact rules can name it apart from the hand.
    comotion::LinkInfo held;
    held.name = kHeldObjectLink;
    held.index = static_cast<int>(links_.size());
    for (const auto &s : localSpheres(follower)) {
        held.collision_spheres.push_back({s.center, s.radius, held.index});
        ++num_follower_spheres_;
    }
    comotion::JointInfo joint;
    joint.name = std::string(kHeldObjectLink) + "_joint";
    joint.type = "fixed";
    joint.parent_link = grasp.ee_link;
    joint.child_link = kHeldObjectLink;
    joint.origin = Eigen::Affine3d(grasp.ee_T_object.matrix());
    link_name_to_index_[held.name] = held.index;
    links_.push_back(std::move(held));
    joints_.push_back(std::move(joint));
    // The held object touches the grasp link's body and the contact links.
    disabled_collisions_.insert(std::minmax(std::string(kHeldObjectLink), grasp.ee_link));
    for (const auto &other : grasp.contact_links) {
        if (linkIndex(other) < 0)
            throw std::invalid_argument("FormationModel: unknown link " + other);
        disabled_collisions_.insert(std::minmax(std::string(kHeldObjectLink), other));
    }
    buildKinematicChain(); // also rebuilds the self-collision mask
}

void allowGraspContact(comotion::RobotModel &arm, const Grasp &grasp,
                       const comotion::RobotModel &other, const std::string &other_link) {
    if (other.contactId() < 0)
        throw std::invalid_argument("allowGraspContact: the other robot has no contact id");
    if (!other_link.empty() && other.linkIndex(other_link) < 0)
        throw std::invalid_argument("allowGraspContact: unknown link " + other_link);
    for (const auto &link : grasp.contact_links)
        arm.allowContact(link, other.contactId(), other_link);
    arm.allowContact(grasp.ee_link, other.contactId(), other_link);
}

void allowExchangeContact(comotion::RobotModel &arm, const Grasp &grasp,
                          const comotion::RobotModel &other, const Grasp &other_grasp) {
    const auto hand = [](const Grasp &g) {
        if (!g.hand_links.empty())
            return g.hand_links;
        auto links = g.contact_links;
        links.push_back(g.ee_link);
        return links;
    };
    const auto mine = hand(grasp), theirs = hand(other_grasp);
    for (const auto &a : mine) {
        arm.allowContact(a, other.contactId(), kHeldObjectLink);
        for (const auto &b : theirs)
            arm.allowContact(a, other.contactId(), b);
    }
}

Formation::Formation(std::shared_ptr<const comotion::RobotModel> leader,
                     std::shared_ptr<const comotion::RobotModel> follower, Grasp grasp)
    : leader_(std::move(leader)), follower_(std::move(follower)), grasp_(std::move(grasp)) {
    ee_index_ = leader_->linkIndex(grasp_.ee_link);
    if (ee_index_ < 0)
        throw std::invalid_argument("Formation: unknown link " + grasp_.ee_link);
    if (follower_->numJoints() != 3)
        throw std::invalid_argument("Formation: follower must be a positional object");
    model_ = std::make_shared<FormationModel>(*leader_, *follower_, grasp_);
}

Eigen::Isometry3d Formation::followerPose(const std::vector<double> &leader_q) const {
    return Eigen::Isometry3d(linkPose(*leader_, ee_index_, leader_q).matrix()) *
           grasp_.ee_T_object;
}

std::vector<double> Formation::followerConfig(const std::vector<double> &leader_q) const {
    const Eigen::Vector3d p = followerPose(leader_q).translation();
    return {p.x(), p.y(), p.z()};
}

comotion::Path Formation::followerPath(const comotion::Path &leader_path) const {
    // Dense: follower waypoints follow FK of the interpolated leader.
    comotion::Path path;
    if (leader_path.empty())
        return path;
    for (std::size_t t = 0; t <= leader_path.arrival_timestep(); ++t)
        path.push_back(followerConfig(leader_path.config_at_timestep(t)));
    path.markDenseTimestepsImplicit();
    return path;
}

Eigen::Isometry3d Formation::graspTransform(const comotion::RobotModel &leader,
                                            const std::string &ee_link,
                                            const std::vector<double> &leader_q,
                                            const Eigen::Isometry3d &object_pose) {
    const int index = leader.linkIndex(ee_link);
    if (index < 0)
        throw std::invalid_argument("Formation: unknown link " + ee_link);
    return Eigen::Isometry3d(linkPose(leader, index, leader_q).matrix()).inverse() * object_pose;
}

} // namespace lazydash
