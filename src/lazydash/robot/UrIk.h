#pragma once

#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

namespace lazydash {

/// Standard DH parameters of a UR arm (a2, a3 negative as in UR's tables).
struct UrDh {
    double d1, a2, a3, d4, d5, d6;
    static UrDh ur5e() { return {0.1625, -0.425, -0.3922, 0.1333, 0.0997, 0.0996}; }
};

/// Analytic 6-D inverse kinematics of a UR arm: up to 8 solutions (shoulder x wrist x
/// elbow). Targets may name any link rigidly attached to `tool0`.
class UrIk {
public:
    explicit UrIk(std::shared_ptr<const comotion::RobotModel> arm, UrDh dh = UrDh::ur5e());

    /// Joint limits that solutions must satisfy (default: the model's).
    void setJointLimits(std::vector<double> lower, std::vector<double> upper);

    /// One configuration per IK branch placing `link` at `world_T_link` within limits;
    /// among 2 pi equivalents, the one closest to `reference` (default zero).
    std::vector<std::vector<double>> solve(const Eigen::Isometry3d &world_T_link,
                                           const std::string &link = "tool0",
                                           const std::vector<double> *reference = nullptr) const;

    /// World pose of `link` at `q` (the robot model's FK).
    Eigen::Isometry3d linkPose(const std::vector<double> &q, const std::string &link = "tool0") const;

    /// DH forward kinematics, base frame to tool0 (for tests).
    static Eigen::Isometry3d dhForward(const UrDh &dh, const std::vector<double> &q);

    const comotion::RobotModel &arm() const { return *arm_; }

private:
    /// Fixed transform tool0 -> link.
    Eigen::Isometry3d toolToLink(const std::string &link) const;

    std::shared_ptr<const comotion::RobotModel> arm_;
    UrDh dh_;
    Eigen::Isometry3d world_T_base_;
    std::vector<double> lower_, upper_;
};

} // namespace lazydash
