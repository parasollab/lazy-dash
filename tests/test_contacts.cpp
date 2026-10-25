// Allowed inter-robot contacts at transitions: a gripper closing on a resting
// object, and a receiver taking an object from a giver's formation. Plus
// RobotModel copies and collision padding.
#include "lazydash/robot/Formation.h"
#include "lazydash/robot/UrIk.h"
#include "lazydash/scene/Scene.h"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ConflictChecker.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <random>
#include <stdexcept>

using comotion::CollisionChecker;
using lazydash::GraspSet;

namespace {

const auto kScene = std::filesystem::path(LAZYDASH_RESOURCE_DIR) / "scenes" / "paper" /
                    "sorting-4robots-4objects.json";

std::vector<double> position(const Eigen::Isometry3d &T) {
    return {T.translation().x(), T.translation().y(), T.translation().z()};
}

/// A collision-free IK solution of `arm` for grasp `g` of an object at `object`.
std::optional<std::vector<double>> graspConfig(const lazydash::Scene &scene,
                                               const std::shared_ptr<comotion::RobotModel> &arm,
                                               std::size_t arm_index, const CollisionChecker &cc,
                                               std::size_t g, const Eigen::Isometry3d &object,
                                               GraspSet::Stage stage = GraspSet::Stage::Grasp) {
    lazydash::UrIk ik(arm);
    ik.setJointLimits(scene.arms[arm_index].lower, scene.arms[arm_index].upper);
    const auto grasps = scene.graspSet(*arm);
    for (const auto &q : ik.solve(grasps.eePose(g, object, stage), grasps.eeLink()))
        if (cc.isValidSingleFull(*arm, q))
            return q;
    return std::nullopt;
}

struct PickSetup {
    lazydash::Scene scene;
    std::shared_ptr<comotion::RobotModel> arm, block;
    std::vector<double> grasp_q, approach_q;
    lazydash::Grasp grasp;
};

/// Arm 0 picking block 0 of the paper's Sorting: 4 robots scene.
PickSetup pickSetup(CollisionChecker::Backend backend, lazydash::Ur5eGeometry geometry) {
    PickSetup s{lazydash::Scene::load(kScene.string()), nullptr, nullptr, {}, {}, {}};
    CollisionChecker cc(backend);
    s.scene.applyObstacles(cc);
    s.arm = s.scene.makeArms(geometry)[0];
    s.block = s.scene.makeObject(0);
    s.block->setContactId(100);
    const auto &start = s.scene.objects[0].start;
    s.grasp_q = graspConfig(s.scene, s.arm, 0, cc, 0, start).value();
    s.approach_q = graspConfig(s.scene, s.arm, 0, cc, 0, start, GraspSet::Stage::Approach).value();
    s.grasp = s.scene.graspSet(*s.arm).grasp(0);
    return s;
}

} // namespace

TEST_CASE("Closing on a resting object is a conflict unless the contact is allowed") {
    for (auto [backend, geometry] :
         {std::pair{CollisionChecker::Backend::Spheres, lazydash::Ur5eGeometry::Spheres},
          std::pair{CollisionChecker::Backend::Fcl, lazydash::Ur5eGeometry::Meshes}}) {
        INFO((backend == CollisionChecker::Backend::Spheres ? "spheres" : "fcl meshes"));
        auto s = pickSetup(backend, geometry);
        CollisionChecker cc(backend);
        const auto at_rest = position(s.scene.objects[0].start);

        // The fingers straddle the block: without a rule this is a collision.
        REQUIRE_FALSE(cc.isValidPair(*s.arm, s.grasp_q, *s.block, at_rest));

        comotion::RobotModel closing(*s.arm);
        lazydash::allowGraspContact(closing, s.grasp, *s.block);
        CHECK(cc.isValidPair(closing, s.grasp_q, *s.block, at_rest));

        // The rule names this block only: another block in the same spot still collides.
        auto other = s.scene.makeObject(0);
        other->setContactId(101);
        CHECK_FALSE(cc.isValidPair(closing, s.grasp_q, *other, at_rest));

        // The approach motion (0.22 m -> 0.15 m) in a schedule scan.
        comotion::Path approach{s.approach_q, s.grasp_q};
        approach.computeTimestepsFromDistance(128, 1.0);
        const comotion::Path still{at_rest};
        const comotion::ConflictChecker scan(cc);
        CHECK(scan.findConflict({approach, still}, {s.arm.get(), s.block.get()}).has_value());
        CHECK_FALSE(scan.findConflict({approach, still}, {&closing, s.block.get()}).has_value());
    }
}

TEST_CASE("At a handover only the receiver's fingers and the held object may touch") {
    auto s = pickSetup(CollisionChecker::Backend::Spheres, lazydash::Ur5eGeometry::Spheres);
    CollisionChecker cc(CollisionChecker::Backend::Spheres);
    s.scene.applyObstacles(cc);
    const auto arms = s.scene.makeArms();
    // Arms 0 and 3 stand 1.4 m apart along x: sample handover poses between them.
    const auto &giver_base = s.scene.arms[0].base.position;
    const auto &receiver_base = s.scene.arms[3].base.position;
    const Eigen::Vector3d middle = 0.5 * (giver_base + receiver_base);
    std::mt19937 rng(4);
    std::uniform_real_distribution<double> u(-1.0, 1.0);

    std::optional<std::pair<std::vector<double>, std::vector<double>>> found;
    Eigen::Isometry3d object = Eigen::Isometry3d::Identity();
    for (int attempt = 0; attempt < 5000 && !found; ++attempt) {
        object = lazydash::poseFromRpy(
            middle + Eigen::Vector3d(0.05 * u(rng), 0.05 * u(rng),
                                     s.scene.params.handover_z_min +
                                         0.5 * (1.0 + u(rng)) * (s.scene.params.handover_z_max -
                                                                 s.scene.params.handover_z_min)),
            Eigen::Vector3d(M_PI * u(rng), M_PI / 2 * u(rng), M_PI * u(rng)));
        const auto giver_q = graspConfig(s.scene, arms[0], 0, cc, 0, object);
        const auto receiver_q = giver_q ? graspConfig(s.scene, arms[3], 3, cc, 1, object)
                                        : std::nullopt;
        if (giver_q && receiver_q)
            found = {{*giver_q, *receiver_q}};
    }
    REQUIRE(found.has_value());
    const auto &[giver_q, receiver_q] = *found;

    const lazydash::Formation giving(arms[0], s.block, s.scene.graspSet(*arms[0]).grasp(0));
    comotion::RobotModel giver(*giving.model());
    giver.setContactId(200);
    const auto receiver_grasp = s.scene.graspSet(*arms[3]).grasp(1);

    comotion::RobotModel receiver(*arms[3]);
    if (cc.isValidPair(receiver, receiver_q, giver, giver_q))
        WARN("this handover pose has no finger-object overlap; the rule is not exercised");
    lazydash::allowGraspContact(receiver, receiver_grasp, giver, lazydash::kHeldObjectLink);
    CHECK(cc.isValidPair(receiver, receiver_q, giver, giver_q));

    // The exemption is link-specific: the receiver's fingers still collide
    // with the giver's hand.
    const int finger = receiver.linkIndex("left_finger");
    CHECK(comotion::RobotModel::contactAllowed(receiver, finger, giver,
                                               giver.linkIndex(lazydash::kHeldObjectLink)));
    CHECK_FALSE(comotion::RobotModel::contactAllowed(
        receiver, finger, giver, giver.linkIndex(lazydash::kUr5eGripperLink)));
    CHECK_FALSE(comotion::RobotModel::contactAllowed(
        receiver, receiver.linkIndex("wrist_3_link"), giver,
        giver.linkIndex(lazydash::kHeldObjectLink)));
}

TEST_CASE("The Vamp backend refuses allowed contacts instead of ignoring them") {
    auto s = pickSetup(CollisionChecker::Backend::Spheres, lazydash::Ur5eGeometry::Spheres);
    comotion::RobotModel closing(*s.arm);
    lazydash::allowGraspContact(closing, s.grasp, *s.block);
    CollisionChecker vamp(CollisionChecker::Backend::Vamp);
    CHECK_THROWS_AS(vamp.isValidPair(closing, s.grasp_q, *s.block,
                                     position(s.scene.objects[0].start)),
                    std::invalid_argument);
}

TEST_CASE("RobotModel copies are independent of the original") {
    auto original = lazydash::makeUr5e({Eigen::Vector3d(0.3, 0.0, 0.705), M_PI});
    const auto q = std::vector<double>{0.1, -1.2, 1.3, -0.4, 0.9, 0.2};
    const auto before = original->getLinkTransforms(q);
    auto copy = std::make_unique<comotion::RobotModel>(*original);
    CHECK(copy->instanceId() != original->instanceId());
    original.reset(); // the copy must not point into the original's joints
    REQUIRE(copy->numJoints() == 6);
    CHECK(copy->jointLower(0) < 0.0);
    const auto after = copy->getLinkTransforms(q);
    for (std::size_t i = 0; i < before.size(); ++i)
        CHECK(after[i].isApprox(before[i], 1e-12));
}

TEST_CASE("Collision padding widens environment and inter-robot checks, not self checks") {
    const std::vector<double> home = {0.0, -M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2, 0.0};
    const auto arm = lazydash::makeUr5e({});
    comotion::RobotModel padded(*arm);
    padded.setCollisionPadding(0.02);
    CollisionChecker cc(CollisionChecker::Backend::Spheres);
    // Self: the base / upper-arm gap is 1.7 cm in every pose; padding must not close it.
    CHECK(cc.isSelfCollisionFree(padded, home));

    // Environment: an obstacle 1 cm beyond the gripper's outermost sphere.
    const auto spheres = arm->getCollisionSpheres(home);
    const auto lowest = *std::min_element(spheres.begin(), spheres.end(), [](auto &a, auto &b) {
        return a.center.z() - a.radius < b.center.z() - b.radius;
    });
    const Eigen::Vector3d below = lowest.center - Eigen::Vector3d(0, 0, lowest.radius + 0.01 + 0.005);
    cc.setObstacles({{below, 0.005}});
    CHECK(cc.isValidSingleFull(*arm, home));
    CHECK_FALSE(cc.isValidSingleFull(padded, home));

    // Other robots: a block 1 cm below the same sphere.
    cc.setObstacles({});
    const auto block = lazydash::makeObject(0.005, Eigen::Vector3d::Constant(-3), Eigen::Vector3d::Constant(3));
    const std::vector<double> at{below.x(), below.y(), below.z()};
    CHECK(cc.isValidPair(*arm, home, *block, at));
    CHECK_FALSE(cc.isValidPair(padded, home, *block, at));

    // Exact backends refuse padding instead of ignoring it.
    CollisionChecker fcl(CollisionChecker::Backend::Fcl);
    CHECK_THROWS_AS(fcl.isValidSingle(padded, home), std::invalid_argument);
}
