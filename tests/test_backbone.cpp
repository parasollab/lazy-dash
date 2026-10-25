// The CoMotion pieces Lazy-DaSH builds on, with the UR5e model: single-arm
// planning, conflict scanning, and ARC repair of a given schedule.
#include "lazydash/robot/Ur5eModel.h"
#include "lazydash/schedule/ScheduledARC.h"

#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>

using comotion::CollisionChecker;
using comotion::MultiRobotProblem;
using comotion::Path;

namespace {

const std::vector<double> kHome = {0.0, -M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2, 0.0};

std::vector<double> panned(std::vector<double> q, double pan) {
    q[0] = pan;
    return q;
}

Path straightPath(const std::vector<double> &from, const std::vector<double> &to,
                  const MultiRobotProblem &problem) {
    Path path{from, to};
    path.computeTimestepsFromDistance(problem.resolution(), problem.vmax());
    return path;
}

bool sameConfig(const std::vector<double> &a, const std::vector<double> &b) {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > 1e-6)
            return false;
    return true;
}

} // namespace

TEST_CASE("A spawned single-arm problem is solved by CoMotion CompositeRRT") {
    comotion::seedOmplGlobalFromUserPlanningSeed(7);
    auto problem = std::make_shared<MultiRobotProblem>(CollisionChecker::Backend::Spheres);
    const auto arm = lazydash::makeUr5e({});
    const auto start = kHome;
    const auto goal = panned(kHome, M_PI);
    problem->addRobot(arm, start, goal);
    // A post between start and goal on the direct pan sweep, plus a table.
    problem->setObstacles({{Eigen::Vector3d(0.0, 0.35, 0.35), 0.12}});
    problem->setCylinderObstacles(
        {{Eigen::Vector3d(0.0, 0.0, -0.03), Eigen::Vector3d::UnitZ(), 2.0, 0.02}});
    REQUIRE(problem->collisionChecker().isValidSingleFull(*arm, start));
    REQUIRE(problem->collisionChecker().isValidSingleFull(*arm, goal));
    REQUIRE_FALSE(problem->collisionChecker().isMotionValid(*arm, start, goal, 200));

    comotion::CompositeRRT planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(7);
    REQUIRE(planner.solve(10.0) == ompl::base::PlannerStatus::EXACT_SOLUTION);
    const auto paths = planner.getSolutionPaths();
    REQUIRE(paths.size() == 1);
    CHECK(sameConfig(paths[0].front(), start));
    CHECK(sameConfig(paths[0].back(), goal));
    CHECK(problem->collisionChecker().isRobotPathValid(*arm, paths[0]));

    // Verification on the mesh model (the final re-check of a schedule).
    CollisionChecker fcl(CollisionChecker::Backend::Fcl);
    fcl.setObstacles(problem->collisionChecker().obstacles());
    fcl.setCylinderObstacles(problem->collisionChecker().cylinders());
    const auto meshes = lazydash::makeUr5e({}, lazydash::Ur5eGeometry::Meshes);
    WARN("sphere-planned path valid on meshes: " << fcl.isRobotPathValid(*meshes, paths[0]));
}

TEST_CASE("ScheduledARC repairs two arms that sweep through each other at the same time") {
    comotion::seedOmplGlobalFromUserPlanningSeed(1);
    auto problem = std::make_shared<MultiRobotProblem>(CollisionChecker::Backend::Spheres);
    problem->setResolution(64);
    problem->setVmax(1.0);
    // Facing arms, extended horizontally (q = 0), 1.2 m apart: their pan sweeps
    // cross in the middle at the same time.
    const auto a = lazydash::makeUr5e({Eigen::Vector3d(-0.6, 0.0, 0.0), 0.0});
    const auto b = lazydash::makeUr5e({Eigen::Vector3d(0.6, 0.0, 0.0), M_PI});
    const std::vector<double> zero(6, 0.0);
    const auto a_start = panned(zero, M_PI / 2), a_goal = panned(zero, -M_PI / 2);
    const auto b_start = panned(zero, -M_PI / 2), b_goal = panned(zero, M_PI / 2);
    problem->addRobot(a, a_start, a_goal);
    problem->addRobot(b, b_start, b_goal);

    std::vector<Path> schedule = {straightPath(a_start, a_goal, *problem),
                                  straightPath(b_start, b_goal, *problem)};
    const auto arrival_before = schedule[0].arrival_timestep();
    const auto robots = problem->robotModelPtrs();
    comotion::ConflictChecker checker(problem->collisionChecker());
    const auto initial = checker.findConflict(schedule, robots);
    REQUIRE(initial.has_value());

    lazydash::ScheduledARC arc;
    arc.setProblem(problem);
    arc.setPlanningSeed(1);
    const auto outcome = arc.repairPaths(schedule, 30.0, 20);
    REQUIRE(outcome.conflict_free);
    CHECK(outcome.repairs >= 1);
    CHECK_FALSE(checker.findConflict(schedule, robots).has_value());

    // Repairs keep each robot's endpoints (the task anchors) but may change
    // when it arrives: ARC shifts the suffix by the local duration change.
    CHECK(sameConfig(schedule[0].front(), a_start));
    CHECK(sameConfig(schedule[0].back(), a_goal));
    CHECK(sameConfig(schedule[1].front(), b_start));
    CHECK(sameConfig(schedule[1].back(), b_goal));
    WARN("repairs " << outcome.repairs << ", arm A arrival " << arrival_before << " -> "
         << schedule[0].arrival_timestep() << ", arm B arrival "
         << schedule[1].arrival_timestep() << " timesteps");
}
