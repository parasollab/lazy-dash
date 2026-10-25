#include "lazydash/robot/Ur5eModel.h"
#include "lazydash/robot/UrIk.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>
#include <sstream>

using lazydash::UrIk;

namespace {

std::vector<double> randomConfig(std::mt19937 &rng, double limit = M_PI) {
    std::uniform_real_distribution<double> u(-limit, limit);
    std::vector<double> q(6);
    for (auto &v : q)
        v = u(rng);
    return q;
}

bool samePose(const Eigen::Isometry3d &a, const Eigen::Isometry3d &b, double tol = 1e-8) {
    return (a.translation() - b.translation()).norm() < tol &&
           (a.linear() - b.linear()).norm() < tol;
}

bool sameAngles(const std::vector<double> &a, const std::vector<double> &b, double tol = 1e-6) {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::abs(std::remainder(a[i] - b[i], 2.0 * M_PI)) > tol)
            return false;
    return true;
}

} // namespace

TEST_CASE("The UR5e DH chain matches the URDF from base to tool0") {
    const auto arm = lazydash::makeUr5e({Eigen::Vector3d(-1.4, -0.7, 0.705), M_PI});
    const UrIk ik(arm);
    std::mt19937 rng(1);
    for (int i = 0; i < 200; ++i) {
        const auto q = randomConfig(rng);
        const Eigen::Isometry3d urdf = ik.linkPose(q, "base").inverse() * ik.linkPose(q, "tool0");
        REQUIRE(samePose(urdf, UrIk::dhForward(lazydash::UrDh::ur5e(), q), 1e-6));
    }
}

TEST_CASE("Analytic IK returns valid solutions that include the sampled configuration") {
    const auto arm = lazydash::makeUr5e({Eigen::Vector3d(0.3, 0.7, 0.705), M_PI});
    UrIk ik(arm);
    ik.setJointLimits(std::vector<double>(6, -M_PI), std::vector<double>(6, M_PI));
    std::mt19937 rng(2);
    int checked = 0, max_solutions = 0;
    for (int i = 0; i < 500; ++i) {
        const auto q = randomConfig(rng);
        if (std::abs(std::sin(q[4])) < 1e-3) // wrist singularity: theta6 is not unique
            continue;
        // Shoulder singularity: the wrist centre on the radius-d4 cylinder around
        // the base axis merges the two shoulder branches (asin near 1).
        const auto dh = lazydash::UrDh::ur5e();
        const Eigen::Isometry3d T = UrIk::dhForward(dh, q);
        const Eigen::Vector3d wrist = T.translation() - dh.d6 * T.linear().col(2);
        if (std::abs(wrist.head<2>().norm() - dh.d4) < 1e-3)
            continue;
        for (const char *link : {"tool0", lazydash::kUr5eGripperLink, "flange"}) {
            const auto target = ik.linkPose(q, link);
            const auto solutions = ik.solve(target, link);
            bool found = false;
            INFO("link " << link << ", q " << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
                 << " " << q[4] << " " << q[5] << ", " << solutions.size() << " solutions");
            std::ostringstream listing;
            for (const auto &s : solutions)
                listing << "\n  " << s[0] << " " << s[1] << " " << s[2] << " " << s[3] << " "
                        << s[4] << " " << s[5];
            INFO("solutions:" << listing.str());
            for (const auto &s : solutions) {
                REQUIRE(samePose(ik.linkPose(s, link), target));
                for (double v : s)
                    REQUIRE(std::abs(v) <= M_PI + 1e-9);
                found = found || sameAngles(s, q);
            }
            REQUIRE(found);
            max_solutions = std::max(max_solutions, static_cast<int>(solutions.size()));
        }
        ++checked;
    }
    CHECK(checked > 450);
    CHECK(max_solutions == 8);
}

TEST_CASE("IK reports unreachable targets as having no solution") {
    const auto arm = lazydash::makeUr5e({});
    const UrIk ik(arm);
    Eigen::Isometry3d far = Eigen::Isometry3d::Identity();
    far.translation() = Eigen::Vector3d(2.0, 0.0, 0.5);
    CHECK(ik.solve(far).empty());
}

TEST_CASE("With +-2 pi joint limits IK returns the equivalent closest to a reference") {
    const auto arm = lazydash::makeUr5e({});
    UrIk ik(arm);
    const std::vector<double> lower = {-2 * M_PI, -2 * M_PI, -M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI};
    const std::vector<double> upper = {2 * M_PI, 2 * M_PI, M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI};
    ik.setJointLimits(lower, upper);
    const std::vector<double> q = {0.4, -1.1, 1.2, -0.6, 1.0, -0.3};
    const auto target = ik.linkPose(q);
    std::vector<double> reference = q;
    reference[0] -= 2 * M_PI; // the same pose, one turn back on the base
    reference[5] += 2 * M_PI; // and one turn further on the wrist
    bool exact = false, near_zero = false;
    for (const auto &s : ik.solve(target, "tool0", &reference)) {
        bool same = true;
        for (std::size_t i = 0; i < 6; ++i)
            same = same && std::abs(s[i] - reference[i]) < 1e-6;
        exact = exact || same;
    }
    for (const auto &s : ik.solve(target))
        near_zero = near_zero || (sameAngles(s, q) && std::abs(s[0] - q[0]) < 1e-6);
    CHECK(exact);
    CHECK(near_zero);
}
