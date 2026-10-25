#pragma once

#include "comotion/planning/Path.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

namespace lazydash {

/// A movable object as a passive CoMotion robot (a `comotion::FlyingSphere`,
/// configuration {x, y, z}); it moves only while in a formation.
std::shared_ptr<comotion::RobotModel>
makeObject(double radius, const Eigen::Vector3d &workspace_min,
           const Eigen::Vector3d &workspace_max);

/// Link of a `FormationModel` that carries the follower's geometry.
inline constexpr const char *kHeldObjectLink = "held_object";

/// How a manipulator holds an object: object frame in `ee_link`, the links that
/// touch it, and the hand links (rigid with the gripper).
struct Grasp {
    std::string ee_link;
    Eigen::Isometry3d ee_T_object = Eigen::Isometry3d::Identity();
    std::vector<std::string> contact_links;
    std::vector<std::string> hand_links;
};

/// Collision model of a formation: the leader's links plus `kHeldObjectLink`
/// carrying the follower's spheres at the grasp transform. Its configuration is
/// the leader's.
class FormationModel : public comotion::RobotModel {
public:
    FormationModel(const comotion::RobotModel &leader, const comotion::RobotModel &follower,
                   const Grasp &grasp);

    std::size_t numFollowerSpheres() const { return num_follower_spheres_; }

private:
    std::size_t num_follower_spheres_ = 0;
};

/// Let the gripping links of `grasp` on `arm` touch robot `other` (only
/// `other_link`, or all of it if empty) at transitions; `other` needs a contact id.
void allowGraspContact(comotion::RobotModel &arm, const Grasp &grasp,
                       const comotion::RobotModel &other,
                       const std::string &other_link = "");

/// Handover exchange: the hand of `grasp` on `arm` may touch the object `other`
/// holds and the hand of `other_grasp` on `other`.
void allowExchangeContact(comotion::RobotModel &arm, const Grasp &grasp,
                          const comotion::RobotModel &other, const Grasp &other_grasp);

/// Formation constraint f_c(r, o, g): a manipulator (leader) and an object (follower)
/// coupled by a fixed end-effector-to-object transform. The composition space is
/// the leader's configuration space; the follower's path comes from `followerPath`.
class Formation {
public:
    Formation(std::shared_ptr<const comotion::RobotModel> leader,
              std::shared_ptr<const comotion::RobotModel> follower, Grasp grasp);

    const comotion::RobotModel &leader() const { return *leader_; }
    const comotion::RobotModel &follower() const { return *follower_; }
    const Grasp &grasp() const { return grasp_; }
    std::shared_ptr<const FormationModel> model() const { return model_; }

    /// World pose of the follower when the leader is at `leader_q`.
    Eigen::Isometry3d followerPose(const std::vector<double> &leader_q) const;
    /// Follower configuration (position) when the leader is at `leader_q`.
    std::vector<double> followerConfig(const std::vector<double> &leader_q) const;
    /// Follower path with the same timing as `leader_path`.
    comotion::Path followerPath(const comotion::Path &leader_path) const;

    /// Grasp that holds the follower at `object_pose` when the leader is at `leader_q`.
    static Eigen::Isometry3d graspTransform(const comotion::RobotModel &leader,
                                            const std::string &ee_link,
                                            const std::vector<double> &leader_q,
                                            const Eigen::Isometry3d &object_pose);

private:
    std::shared_ptr<const comotion::RobotModel> leader_;
    std::shared_ptr<const comotion::RobotModel> follower_;
    Grasp grasp_;
    int ee_index_ = -1;
    std::shared_ptr<const FormationModel> model_;
};

} // namespace lazydash
