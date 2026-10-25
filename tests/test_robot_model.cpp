#include "lazydash/robot/Formation.h"
#include "lazydash/robot/Ur5eModel.h"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ConflictChecker.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <random>

using comotion::CollisionChecker;
using Catch::Matchers::WithinAbs;

namespace {

const std::vector<double> kHome = {0.0, -M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2, 0.0};
constexpr auto kFcl = CollisionChecker::Backend::Fcl;
constexpr auto kSpheres = CollisionChecker::Backend::Spheres;
constexpr double kBlockRadius = 0.033; // sphere around the 3.8 cm cube

Eigen::Vector3d linkPosition(const comotion::RobotModel &robot,
                             const std::vector<double> &q, const std::string &link) {
    return robot.getLinkTransforms(q)[static_cast<std::size_t>(robot.linkIndex(link))]
        .translation();
}

std::vector<double> randomConfig(const comotion::RobotModel &robot, std::mt19937 &rng) {
    std::vector<double> q(static_cast<std::size_t>(robot.numJoints()));
    for (int i = 0; i < robot.numJoints(); ++i)
        q[static_cast<std::size_t>(i)] = std::uniform_real_distribution<double>(
            robot.jointLower(i), robot.jointUpper(i))(rng);
    return q;
}

std::shared_ptr<comotion::RobotModel> makeBlock() {
    return lazydash::makeObject(kBlockRadius, Eigen::Vector3d::Constant(-3.0),
                                Eigen::Vector3d::Constant(3.0));
}

/// The cube held between the Hand-e fingers, 14.5 cm along the gripper axis.
lazydash::Grasp fingerGrasp() {
    lazydash::Grasp grasp;
    grasp.ee_link = lazydash::kUr5eGripperLink;
    grasp.ee_T_object.translation() = Eigen::Vector3d(0.0, 0.0, 0.145);
    grasp.contact_links = {"left_finger", "right_finger"};
    return grasp;
}

/// Table slab under the base plus a cloud of small obstacles around the arm.
void addClutter(CollisionChecker &cc, std::mt19937 &rng) {
    std::vector<comotion::ObstacleSphere> obstacles;
    for (int i = 0; i < 12; ++i) {
        std::uniform_real_distribution<double> xy(-0.9, 0.9), z(0.05, 0.9);
        obstacles.push_back({Eigen::Vector3d(xy(rng), xy(rng), z(rng)), 0.05});
    }
    cc.setObstacles(obstacles);
    cc.setCylinderObstacles(
        {{Eigen::Vector3d(0.0, 0.0, -0.03), Eigen::Vector3d::UnitZ(), 2.0, 0.02}});
}

} // namespace

TEST_CASE("UR5e model has 6 prefixed arm joints and is not mistaken for CoMotion's UR5") {
    for (auto geometry : {lazydash::Ur5eGeometry::Meshes, lazydash::Ur5eGeometry::Spheres}) {
        const auto arm = lazydash::makeUr5e({}, geometry);
        REQUIRE(arm->numJoints() == 6);
        CHECK(arm->activeJointNames().front() == "ur5e_shoulder_pan_joint");
        CHECK(arm->robotFamily() == comotion::RobotModel::RobotFamily::Unknown);
    }
}

TEST_CASE("UR5e forward kinematics matches the DH zero pose and the base transform") {
    const auto at_origin = lazydash::makeUr5e({});
    const auto zero = std::vector<double>(6, 0.0);
    const Eigen::Vector3d p0 = linkPosition(*at_origin, zero, lazydash::kUr5eToolLink);
    CHECK_THAT(p0.x(), WithinAbs(0.8172, 1e-4));
    CHECK_THAT(p0.y(), WithinAbs(0.2329, 1e-4));
    CHECK_THAT(p0.z(), WithinAbs(0.0628, 1e-4));

    // A relay base: (-1.4, -0.7, 0.705), yaw pi.
    const auto placed = lazydash::makeUr5e({Eigen::Vector3d(-1.4, -0.7, 0.705), M_PI});
    const Eigen::Vector3d p1 = linkPosition(*placed, zero, lazydash::kUr5eToolLink);
    CHECK_THAT(p1.x(), WithinAbs(-1.4 - 0.8172, 1e-4));
    CHECK_THAT(p1.y(), WithinAbs(-0.7 - 0.2329, 1e-4));
    CHECK_THAT(p1.z(), WithinAbs(0.705 + 0.0628, 1e-4));
}

TEST_CASE("Home configuration is collision free on a table, spheres and meshes") {
    const auto spheres = lazydash::makeUr5e({});
    const auto meshes = lazydash::makeUr5e({}, lazydash::Ur5eGeometry::Meshes);
    CollisionChecker sphere_cc(kSpheres), fcl_cc(kFcl);
    for (auto *cc : {&sphere_cc, &fcl_cc})
        cc->setCylinderObstacles(
            {{Eigen::Vector3d(0.0, 0.0, -0.03), Eigen::Vector3d::UnitZ(), 2.0, 0.02}});
    CHECK(sphere_cc.isValidSingleFull(*spheres, kHome));
    CHECK(fcl_cc.isValidSingleFull(*meshes, kHome));
}

TEST_CASE("Sphere model agrees with the mesh model") {
    const auto spheres = lazydash::makeUr5e({});
    const auto meshes = lazydash::makeUr5e({}, lazydash::Ur5eGeometry::Meshes);
    CollisionChecker sphere_cc(kSpheres), fcl_cc(kFcl);
    std::mt19937 clutter(3), clutter_copy(3), rng(5);
    addClutter(sphere_cc, clutter);
    addClutter(fcl_cc, clutter_copy);
    int mesh_valid = 0, rejected = 0, mesh_invalid = 0, missed = 0;
    for (int i = 0; i < 3000; ++i) {
        const auto q = randomConfig(*meshes, rng);
        const bool s = sphere_cc.isValidSingleFull(*spheres, q);
        if (fcl_cc.isValidSingleFull(*meshes, q)) {
            ++mesh_valid;
            rejected += !s;
        } else {
            ++mesh_invalid;
            missed += s;
        }
    }
    // Inscribed spheres: almost no false collisions; the few contacts they
    // miss are caught by the final mesh re-check of a schedule.
    CHECK(rejected <= 0.02 * mesh_valid);
    CHECK(missed <= 0.05 * mesh_invalid);
    WARN("sphere model: rejects " << rejected << "/" << mesh_valid << " mesh-valid, misses "
         << missed << "/" << mesh_invalid << " mesh-invalid configurations");
}

TEST_CASE("Collision checking throughput", "[benchmark]") {
    for (auto geometry : {lazydash::Ur5eGeometry::Spheres, lazydash::Ur5eGeometry::Meshes}) {
        const auto arm = lazydash::makeUr5e({}, geometry);
        const bool use_spheres = geometry == lazydash::Ur5eGeometry::Spheres;
        CollisionChecker cc(use_spheres ? kSpheres : kFcl);
        std::mt19937 rng(3);
        addClutter(cc, rng);
        const int n = 5000;
        int valid = 0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < n; ++i)
            valid += cc.isValidSingleFull(*arm, randomConfig(*arm, rng));
        const double s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        WARN((use_spheres ? "Spheres" : "Fcl meshes") << ": " << n / s << " full checks/s, "
             << valid << "/" << n << " valid");
        CHECK(valid > 0);
    }
}

/// Score sphere models against the mesh model. Candidates come from
/// LAZYDASH_SPHERE_URDFS (':'-separated URDF paths), default the shipped one.
TEST_CASE("Sphere models versus meshes", "[.experimental]") {
    std::vector<std::string> candidates;
    if (const char *env = std::getenv("LAZYDASH_SPHERE_URDFS")) {
        std::string list(env);
        for (std::size_t begin = 0, end; begin <= list.size(); begin = end + 1) {
            end = std::min(list.find(':', begin), list.size());
            if (end > begin)
                candidates.push_back(list.substr(begin, end - begin));
        }
    } else {
        candidates.push_back(lazydash::Ur5eResources::defaults().spherized_urdf);
    }
    const auto meshes = lazydash::makeUr5e({}, lazydash::Ur5eGeometry::Meshes);
    CollisionChecker fcl_cc(kFcl);
    std::mt19937 clutter_rng(7);
    addClutter(fcl_cc, clutter_rng);

    // Tool-down poses around home: pan and wrist_3 vary.
    std::vector<std::vector<double>> tool_down;
    for (double pan = -M_PI; pan < M_PI; pan += M_PI / 4)
        for (double w3 = -M_PI; w3 < M_PI; w3 += M_PI / 2) {
            auto q = kHome;
            q[0] = pan;
            q[5] = w3;
            tool_down.push_back(q);
        }

    const int n = 10000;
    std::mt19937 config_rng(11);
    std::vector<std::vector<double>> configs;
    std::vector<char> mesh_valid;
    for (int i = 0; i < n; ++i) {
        configs.push_back(randomConfig(*meshes, config_rng));
        mesh_valid.push_back(fcl_cc.isValidSingleFull(*meshes, configs.back()));
    }

    for (const auto &urdf : candidates) {
        auto resources = lazydash::Ur5eResources::defaults();
        resources.spherized_urdf = urdf;
        const auto spheres = lazydash::makeUr5e({}, lazydash::Ur5eGeometry::Spheres, resources);
        std::size_t count = 0;
        for (const auto &s : spheres->getCollisionSpheres(kHome))
            count += s.radius > 0.0;
        CollisionChecker sphere_cc(CollisionChecker::Backend::Spheres);
        std::mt19937 rng(7);
        addClutter(sphere_cc, rng);

        // Deepest sphere overlap per colliding link pair at home.
        std::map<std::string, double> home_pairs;
        const auto hs = spheres->getCollisionSpheres(kHome);
        for (std::size_t i = 0; i < hs.size(); ++i)
            for (std::size_t j = i + 1; j < hs.size(); ++j) {
                const auto &a = spheres->links()[static_cast<std::size_t>(hs[i].link_index)];
                const auto &b = spheres->links()[static_cast<std::size_t>(hs[j].link_index)];
                const double depth =
                    hs[i].radius + hs[j].radius - (hs[i].center - hs[j].center).norm();
                if (hs[i].link_index != hs[j].link_index &&
                    !spheres->isSelfCollisionDisabled(a.name, b.name) && depth > 0.0) {
                    auto &d = home_pairs[a.name + "|" + b.name];
                    d = std::max(d, depth);
                }
            }
        std::string pairs;
        for (const auto &[p, d] : home_pairs)
            pairs += " " + p + "(" + std::to_string(d * 1000).substr(0, 4) + "mm)";
        WARN("home self-collision pairs:" << pairs);

        int tool_down_ok = 0;
        for (const auto &q : tool_down)
            tool_down_ok += sphere_cc.isSelfCollisionFree(*spheres, q);
        int both_valid = 0, conservative = 0, missed = 0, self_only = 0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < n; ++i) {
            const bool s = sphere_cc.isValidSingleFull(*spheres, configs[i]);
            both_valid += s && mesh_valid[i];
            conservative += !s && mesh_valid[i];
            missed += s && !mesh_valid[i];
        }
        const double secs =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        for (int i = 0; i < n; ++i)
            self_only += mesh_valid[i] && !sphere_cc.isSelfCollisionFree(*spheres, configs[i]);
        const int valid = both_valid + conservative;
        WARN(urdf << "\n  spheres " << count << ", home self-free "
             << sphere_cc.isSelfCollisionFree(*spheres, kHome) << ", tool-down self-free "
             << tool_down_ok << "/" << tool_down.size() << "\n  mesh-valid " << valid << "/" << n
             << ": sphere-valid " << both_valid << " (" << 100.0 * both_valid / std::max(valid, 1)
             << "%), rejected " << conservative << " (self " << self_only << ")"
             << "\n  missed (mesh-invalid, sphere-valid) " << missed << "/" << n - valid
             << "\n  " << n / secs << " full checks/s");
    }
}


TEST_CASE("A formation's object collides with the world but not with its own gripper") {
    const auto arm = lazydash::makeUr5e({});
    const lazydash::Formation formation(arm, makeBlock(), fingerGrasp());
    const auto &holding = *formation.model();
    CHECK(holding.numFollowerSpheres() == 1);
    CHECK(holding.numJoints() == arm->numJoints());

    CollisionChecker cc(kSpheres);
    REQUIRE(cc.isSelfCollisionFree(holding, kHome));

    // An obstacle exactly where the held block is: only the formation hits it.
    const auto at = formation.followerConfig(kHome);
    cc.setObstacles({{Eigen::Vector3d(at[0], at[1], at[2]), 0.01}});
    CHECK(cc.isValidSingleFull(*arm, kHome));
    CHECK_FALSE(cc.isValidSingleFull(holding, kHome));
}

TEST_CASE("The formation constraint fixes the follower to the leader's end effector") {
    const auto arm = lazydash::makeUr5e({Eigen::Vector3d(-1.4, -0.7, 0.705), M_PI});
    const auto block = makeBlock();
    // A pick: the block rests at the gripper's grasp point in the home pose.
    Eigen::Isometry3d resting = Eigen::Isometry3d::Identity();
    resting.translation() = lazydash::Formation(arm, block, fingerGrasp())
                                .followerPose(kHome)
                                .translation();
    auto grasp = fingerGrasp();
    grasp.ee_T_object =
        lazydash::Formation::graspTransform(*arm, grasp.ee_link, kHome, resting);
    const lazydash::Formation formation(arm, block, grasp);
    CHECK(formation.followerPose(kHome).isApprox(resting, 1e-9));

    // Carrying: the follower path is FK of the leader at every timestep.
    comotion::Path carry{kHome, {M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2, 0.0}};
    carry.computeTimestepsFromDistance(64, 1.0);
    const auto follower = formation.followerPath(carry);
    REQUIRE(follower.size() == carry.arrival_timestep() + 1);
    for (std::size_t t = 0; t < follower.size(); t += 7) {
        const auto expected = formation.followerConfig(carry.config_at_timestep(t));
        for (int k = 0; k < 3; ++k)
            CHECK_THAT(follower[t][k], WithinAbs(expected[k], 1e-12));
    }
}

TEST_CASE("Objects are robots: a carried object hitting a resting one is an inter-robot conflict") {
    const auto arm = lazydash::makeUr5e({});
    const auto carried = makeBlock(), resting = makeBlock();
    const lazydash::Formation formation(arm, carried, fingerGrasp());

    // Fingertip reach along the gripper axis, from the sphere model.
    const auto &gripper = arm->links()[static_cast<std::size_t>(
        arm->linkIndex(lazydash::kUr5eGripperLink))];
    const Eigen::Affine3d T_gripper =
        arm->getLinkTransforms(kHome)[static_cast<std::size_t>(gripper.index)];
    double tip = 0.0;
    for (const auto &s : arm->getCollisionSpheres(kHome)) {
        const auto &name = arm->links()[static_cast<std::size_t>(s.link_index)].name;
        if (name == "left_finger" || name == "right_finger")
            tip = std::max(tip, (T_gripper.inverse() * s.center).z() + s.radius);
    }
    // The resting block sits just beyond the fingertips on the swept arc at
    // pan pi/2: the bare arm clears it, the carried block does not.
    const double offset = tip + kBlockRadius + 0.004;
    REQUIRE(offset - 0.145 < 2 * kBlockRadius);
    auto sweep_mid = kHome;
    sweep_mid[0] = M_PI / 2;
    const Eigen::Vector3d p =
        arm->getLinkTransforms(sweep_mid)[static_cast<std::size_t>(gripper.index)] *
        Eigen::Vector3d(0.0, 0.0, offset);

    comotion::Path arm_path{kHome, {M_PI, -M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2, 0.0}};
    arm_path.computeTimestepsFromDistance(64, 1.0);
    const comotion::Path still{{p.x(), p.y(), p.z()}};

    CollisionChecker cc(kSpheres);
    const comotion::ConflictChecker checker(cc);
    // Bare arm (robot-only element) and the resting object: no conflict.
    CHECK_FALSE(checker.findConflict({arm_path, still}, {arm.get(), resting.get()}).has_value());
    // Formation (robot+object element) and the resting object: conflict, an
    // h-v conflict between the formation and the object robot.
    const auto conflict =
        checker.findConflict({arm_path, still}, {formation.model().get(), resting.get()});
    REQUIRE(conflict.has_value());
    CHECK(conflict->robot_i == 0);
    CHECK(conflict->robot_j == 1);
}
