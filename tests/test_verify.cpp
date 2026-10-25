// The failure path of two-level validity: a schedule the sphere model accepts
// but the meshes reject is caught by MeshVerifier and repaired by ScheduledARC
// on the verification model.
#include "lazydash/robot/Ur5eModel.h"
#include "lazydash/schedule/MeshVerifier.h"
#include "lazydash/schedule/ScheduledARC.h"

#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <optional>
#include <random>

using comotion::CollisionChecker;
using comotion::Path;

namespace {

using Config = std::vector<double>;

Config random(std::mt19937 &rng, double half_width, const Config &around = Config(6, 0.0)) {
    std::uniform_real_distribution<double> u(-half_width, half_width);
    Config q = around;
    for (auto &v : q)
        v = std::clamp(v + u(rng), -M_PI, M_PI);
    return q;
}

Path through(const Config &start, const Config &middle, const Config &goal) {
    Path path{start, middle, goal};
    path.set_waypoint_timesteps({0, 100, 200});
    return path;
}

struct MissCase {
    std::vector<Path> paths;
};

} // namespace

TEST_CASE("A conflict the spheres miss is found on meshes and repaired on meshes") {
    comotion::seedOmplGlobalFromUserPlanningSeed(3);
    const lazydash::BasePose left{Eigen::Vector3d(-0.7, 0.0, 0.0), 0.0};
    const lazydash::BasePose right{Eigen::Vector3d(0.7, 0.0, 0.0), M_PI};
    // Unpadded spheres, so that misses exist to be found.
    const auto sph_a = lazydash::makeUr5e(left), sph_b = lazydash::makeUr5e(right);
    const auto mesh_a = lazydash::makeUr5e(left, lazydash::Ur5eGeometry::Meshes);
    const auto mesh_b = lazydash::makeUr5e(right, lazydash::Ur5eGeometry::Meshes);
    const std::vector<const comotion::RobotModel *> spheres = {sph_a.get(), sph_b.get()};
    const std::vector<const comotion::RobotModel *> meshes = {mesh_a.get(), mesh_b.get()};

    const lazydash::Scene empty;
    const lazydash::MeshVerifier verifier(empty);
    CollisionChecker sphere_cc(CollisionChecker::Backend::Spheres);
    const comotion::ConflictChecker sphere_scan(sphere_cc);
    const auto &fcl = verifier.checker();

    // Search a two-arm schedule that passes on spheres and fails on meshes
    // only where the arms meet.
    std::mt19937 rng(9);
    std::optional<MissCase> found;
    for (int attempt = 0; attempt < 200000 && !found; ++attempt) {
        const Config qa = random(rng, M_PI), qb = random(rng, M_PI);
        if (fcl.isValidPair(*mesh_a, qa, *mesh_b, qb) ||
            !sphere_cc.isValidPair(*sph_a, qa, *sph_b, qb))
            continue;
        const Config a0 = random(rng, 0.6, qa), a1 = random(rng, 0.6, qa);
        const Config b0 = random(rng, 0.6, qb), b1 = random(rng, 0.6, qb);
        if (!fcl.isValidPair(*mesh_a, a0, *mesh_b, b0) || !fcl.isValidPair(*mesh_a, a1, *mesh_b, b1))
            continue;
        std::vector<Path> paths = {through(a0, qa, a1), through(b0, qb, b1)};
        if (!verifier.pathValid(*mesh_a, paths[0]) || !verifier.pathValid(*mesh_b, paths[1]))
            continue;
        if (sphere_scan.findConflict(paths, spheres).has_value())
            continue;
        found = MissCase{paths};
    }
    REQUIRE(found.has_value());
    auto paths = found->paths;

    // The verification model catches what the planning model accepted.
    const auto conflict = verifier.firstConflict(paths, meshes);
    REQUIRE(conflict.has_value());
    CHECK(conflict->scope == comotion::ConflictScope::InterRobot);

    // Repair on the verification model: ScheduledARC on an Fcl problem.
    auto problem = std::make_shared<comotion::MultiRobotProblem>(CollisionChecker::Backend::Fcl);
    problem->setResolution(64);
    problem->setVmax(1.0);
    problem->addRobot(mesh_a, paths[0].front(), paths[0].back());
    problem->addRobot(mesh_b, paths[1].front(), paths[1].back());
    lazydash::ScheduledARC arc;
    arc.setProblem(problem);
    arc.setPlanningSeed(3);
    const auto outcome = arc.repairPaths(paths, 60.0, 20);
    REQUIRE(outcome.conflict_free);
    CHECK_FALSE(verifier.firstConflict(paths, meshes).has_value());
    WARN("mesh-only conflict at timestep " << conflict->timestep << ", repaired in "
         << outcome.repairs << " ARC repair(s) on the mesh model");
}
