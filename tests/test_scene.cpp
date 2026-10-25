#include "lazydash/robot/UrIk.h"
#include "lazydash/scene/Scene.h"

#include "comotion/collision/CollisionChecker.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <filesystem>

using Catch::Matchers::WithinAbs;
using comotion::CollisionChecker;

namespace {

const std::filesystem::path kScenes = std::filesystem::path(LAZYDASH_RESOURCE_DIR) / "scenes";

std::vector<std::filesystem::path> sceneFiles() {
    std::vector<std::filesystem::path> files;
    for (const auto &dir : {kScenes / "generated", kScenes / "paper", kScenes / "extra" / "ring",
                            kScenes / "extra" / "workcell"})
        for (const auto &e : std::filesystem::directory_iterator(dir))
            if (e.path().extension() == ".json")
                files.push_back(e.path());
    return files;
}

std::vector<double> position(const Eigen::Isometry3d &T) {
    return {T.translation().x(), T.translation().y(), T.translation().z()};
}

bool inside(const lazydash::Scene &s, const Eigen::Vector3d &p) {
    return (p.array() >= s.workspace_min.array()).all() &&
           (p.array() <= s.workspace_max.array()).all();
}

/// Arms that reach an object pose: within robot_range, with an IK solution at
/// both grasp stages that is collision free against the static scene.
int reachingArms(const lazydash::Scene &scene,
                 const std::vector<std::shared_ptr<comotion::RobotModel>> &arms,
                 const CollisionChecker &cc, const Eigen::Isometry3d &object, std::size_t grasp) {
    int count = 0;
    for (std::size_t a = 0; a < arms.size(); ++a) {
        const auto &spec = scene.arms[a];
        if ((object.translation() - spec.base.position).head<2>().norm() > scene.params.robot_range)
            continue;
        lazydash::UrIk ik(arms[a]);
        ik.setJointLimits(spec.lower, spec.upper);
        const auto grasps = scene.graspSet(*arms[a]);
        bool ok = true;
        for (auto stage : {lazydash::GraspSet::Stage::Grasp, lazydash::GraspSet::Stage::Approach}) {
            bool any = false;
            for (const auto &q : ik.solve(grasps.eePose(grasp, object, stage), grasps.eeLink()))
                any = any || cc.isValidSingleFull(*arms[a], q);
            ok = ok && any;
        }
        count += ok;
    }
    return count;
}

} // namespace

TEST_CASE("Every shipped scene loads and its start state is collision free") {
    const auto files = sceneFiles();
    REQUIRE(files.size() >= 18);
    for (const auto &file : files) {
        INFO(file.filename().string());
        const auto scene = lazydash::Scene::load(file.string());
        REQUIRE_FALSE(scene.arms.empty());
        REQUIRE_FALSE(scene.objects.empty());
        // Two grasps; the Shelf-wall scenes add their quarter turns.
        REQUIRE(scene.grasps.size() == (file.filename().string().find("shelf-wall") != std::string::npos ? 4u : 2u));

        CollisionChecker cc(CollisionChecker::Backend::Spheres);
        scene.applyObstacles(cc);
        const auto arms = scene.makeArms();
        for (std::size_t a = 0; a < arms.size(); ++a)
            CHECK(cc.isValidSingleFull(*arms[a], scene.arms[a].start));
        for (std::size_t o = 0; o < scene.objects.size(); ++o) {
            const auto object = scene.makeObject(o);
            CHECK(cc.isValidSingleFull(*object, position(scene.objects[o].start)));
            for (const auto &goal : scene.objects[o].goals)
                CHECK(inside(scene, goal.translation()));
        }
    }
}

TEST_CASE("The converted Sorting: 4 robots scene keeps the paper's poses") {
    const auto scene =
        lazydash::Scene::load((kScenes / "paper" / "sorting-4robots-4objects.json").string());
    REQUIRE(scene.arms.size() == 4);
    CHECK_THAT(scene.arms[0].base.position.x(), WithinAbs(-1.4, 1e-9));
    CHECK_THAT(scene.arms[0].base.position.y(), WithinAbs(-0.7, 1e-9));
    CHECK_THAT(std::abs(scene.arms[0].base.yaw), WithinAbs(M_PI, 1e-6));
    CHECK_THAT(scene.arms[0].start[1], WithinAbs(-M_PI / 2, 1e-6));
    REQUIRE(scene.objects.size() == 4);
    CHECK_THAT(scene.objects[0].start.translation().x(), WithinAbs(-1.405, 1e-9));
    CHECK_THAT(scene.objects[0].start.translation().y(), WithinAbs(-0.325, 1e-9));
    REQUIRE(scene.objects[0].goals.size() == 2);
    // The second goal is the block rolled over: its z axis points down.
    CHECK((scene.objects[0].goals[1].linear() * Eigen::Vector3d::UnitZ()).z() < -0.999);
    CHECK_THAT(scene.params.robot_range, WithinAbs(1.5, 1e-12));
}

TEST_CASE("Blocks are graspable: upright from above, flipped blocks need the other grasp") {
    for (const char *name : {"sorting-4robots-4objects.json",
                             "sorting-4robots-40objects.json",
                             "wall-2walls-16objects.json"}) {
        INFO(name);
        const auto scene = lazydash::Scene::load((kScenes / "paper" / name).string());
        CollisionChecker cc(CollisionChecker::Backend::Spheres);
        scene.applyObstacles(cc);
        const auto arms = scene.makeArms();
        int start_ok = 0, goal_ok = 0, wrong_grasp = 0;
        for (const auto &obj : scene.objects) {
            // Upright start: the "upright" grasp (0) from some arm.
            start_ok += reachingArms(scene, arms, cc, obj.start, 0) > 0;
            // The flipped grasp would approach an upright block from below.
            wrong_grasp += reachingArms(scene, arms, cc, obj.start, 1) > 0;
            // Goals: upright with grasp 0, or rolled over with grasp 1.
            goal_ok += reachingArms(scene, arms, cc, obj.goals[0], 0) > 0 ||
                       reachingArms(scene, arms, cc, obj.goals[1], 1) > 0;
        }
        const int n = static_cast<int>(scene.objects.size());
        WARN(name << ": starts reachable " << start_ok << "/" << n << ", goals reachable "
             << goal_ok << "/" << n << ", upright blocks reachable with the flipped grasp "
             << wrong_grasp);
        CHECK(start_ok == n);
        CHECK(goal_ok == n);
        CHECK(wrong_grasp == 0);
    }
}
