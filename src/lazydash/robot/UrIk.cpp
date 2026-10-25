#include "lazydash/robot/UrIk.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace lazydash {

namespace {

constexpr double kEps = 1e-10;

/// One standard DH link: Rz(theta) Tz(d) Tx(a) Rx(alpha).
Eigen::Isometry3d dhLink(double theta, double d, double a, double alpha) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
    T.translate(Eigen::Vector3d(a, 0.0, d)); // Tz(d) Tx(a) commute
    T.rotate(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()));
    return T;
}

std::vector<Eigen::Isometry3d> dhLinks(const UrDh &dh, const std::vector<double> &q) {
    return {dhLink(q[0], dh.d1, 0.0, M_PI / 2), dhLink(q[1], 0.0, dh.a2, 0.0),
            dhLink(q[2], 0.0, dh.a3, 0.0),      dhLink(q[3], dh.d4, 0.0, M_PI / 2),
            dhLink(q[4], dh.d5, 0.0, -M_PI / 2), dhLink(q[5], dh.d6, 0.0, 0.0)};
}

Eigen::Isometry3d asIsometry(const Eigen::Affine3d &T) { return Eigen::Isometry3d(T.matrix()); }

/// Clamp-free acos that reports out-of-range arguments (beyond rounding).
bool safeAcos(double c, double &out) {
    if (c > 1.0 + 1e-9 || c < -1.0 - 1e-9)
        return false;
    out = std::acos(std::clamp(c, -1.0, 1.0));
    return true;
}

} // namespace

UrIk::UrIk(std::shared_ptr<const comotion::RobotModel> arm, UrDh dh)
    : arm_(std::move(arm)), dh_(dh) {
    if (arm_->numJoints() != 6)
        throw std::invalid_argument("UrIk: expected a 6-joint UR arm");
    const int base = arm_->linkIndex("base");
    if (base < 0)
        throw std::invalid_argument("UrIk: the model has no `base` link");
    world_T_base_ = asIsometry(
        arm_->getLinkTransforms(std::vector<double>(6, 0.0))[static_cast<std::size_t>(base)]);
    for (int i = 0; i < 6; ++i) {
        lower_.push_back(arm_->jointLower(i));
        upper_.push_back(arm_->jointUpper(i));
    }
}

void UrIk::setJointLimits(std::vector<double> lower, std::vector<double> upper) {
    if (lower.size() != 6 || upper.size() != 6)
        throw std::invalid_argument("UrIk: limits need 6 entries");
    lower_ = std::move(lower);
    upper_ = std::move(upper);
}

Eigen::Isometry3d UrIk::dhForward(const UrDh &dh, const std::vector<double> &q) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    for (const auto &A : dhLinks(dh, q))
        T = T * A;
    return T;
}

Eigen::Isometry3d UrIk::linkPose(const std::vector<double> &q, const std::string &link) const {
    const int index = arm_->linkIndex(link);
    if (index < 0)
        throw std::invalid_argument("UrIk: unknown link " + link);
    return asIsometry(arm_->getLinkTransforms(q)[static_cast<std::size_t>(index)]);
}

Eigen::Isometry3d UrIk::toolToLink(const std::string &link) const {
    if (link == "tool0")
        return Eigen::Isometry3d::Identity();
    const std::vector<double> zero(6, 0.0);
    return linkPose(zero, "tool0").inverse() * linkPose(zero, link);
}

std::vector<std::vector<double>> UrIk::solve(const Eigen::Isometry3d &world_T_link,
                                             const std::string &link,
                                             const std::vector<double> *reference) const {
    // Target of the DH chain: tool0 in the DH base frame.
    const Eigen::Isometry3d T =
        world_T_base_.inverse() * world_T_link * toolToLink(link).inverse();
    const Eigen::Matrix3d R = T.linear();
    const Eigen::Vector3d p06 = T.translation();
    const Eigen::Vector3d p05 = p06 - dh_.d6 * R.col(2);

    std::vector<std::vector<double>> solutions;
    // theta1: frame 5's origin lies d4 off the arm plane along z1 = (s1, -c1, 0).
    const double rho = std::hypot(p05.x(), p05.y());
    if (rho < std::abs(dh_.d4) - 1e-12)
        return solutions;
    const double phi = std::atan2(p05.y(), p05.x());
    const double offset = std::asin(std::clamp(dh_.d4 / rho, -1.0, 1.0));
    for (double theta1 : {phi + offset, phi + M_PI - offset}) {
        const double s1 = std::sin(theta1), c1 = std::cos(theta1);
        // theta5: z1 . p06 = d4 + d6 cos(theta5).
        double acos5;
        if (!safeAcos((p06.x() * s1 - p06.y() * c1 - dh_.d4) / dh_.d6, acos5))
            continue;
        for (double theta5 : {acos5, -acos5}) {
            const double s5 = std::sin(theta5);
            // theta6: z1 seen from frame 6 is (s5 c6, -s5 s6, c5).
            double theta6 = 0.0; // wrist singularity (s5 = 0): theta6 is free
            if (std::abs(s5) > kEps)
                theta6 = std::atan2((-R(0, 1) * s1 + R(1, 1) * c1) / s5,
                                    (R(0, 0) * s1 - R(1, 0) * c1) / s5);
            // Planar 3R problem for joints 2-4 in frame 1.
            const auto A = dhLinks(dh_, {theta1, 0, 0, 0, theta5, theta6});
            const Eigen::Isometry3d T14 = A[0].inverse() * T * A[5].inverse() * A[4].inverse();
            const double x = T14.translation().x(), y = T14.translation().y();
            double acos3;
            if (!safeAcos((x * x + y * y - dh_.a2 * dh_.a2 - dh_.a3 * dh_.a3) /
                              (2.0 * dh_.a2 * dh_.a3),
                          acos3))
                continue;
            for (double theta3 : {acos3, -acos3}) {
                const double theta2 = std::atan2(y, x) - std::atan2(dh_.a3 * std::sin(theta3),
                                                                     dh_.a2 + dh_.a3 * std::cos(theta3));
                // R14 = Rz(theta2 + theta3 + theta4) Rx(pi/2).
                const double theta234 = std::atan2(T14.linear()(1, 0), T14.linear()(0, 0));
                std::vector<double> q = {theta1, theta2, theta3, theta234 - theta2 - theta3,
                                         theta5, theta6};
                bool within = true;
                for (std::size_t i = 0; i < 6 && within; ++i) {
                    // Among the 2 pi equivalents within limits, the closest to the reference.
                    const double base = std::remainder(q[i], 2.0 * M_PI); // (-pi, pi]
                    const double ref = reference ? (*reference)[i] : 0.0;
                    within = false;
                    for (double v : {base - 2.0 * M_PI, base, base + 2.0 * M_PI}) {
                        if (v < lower_[i] - 1e-9 || v > upper_[i] + 1e-9)
                            continue;
                        if (!within || std::abs(v - ref) < std::abs(q[i] - ref))
                            q[i] = v;
                        within = true;
                    }
                }
                if (within)
                    solutions.push_back(std::move(q));
            }
        }
    }
    return solutions;
}

} // namespace lazydash
