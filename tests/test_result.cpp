// The CoMotion viewer result written for a run.
#include "lazydash/planning/ResultJson.h"
#include "lazydash/scene/Scene.h"

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <filesystem>

using namespace lazydash;

TEST_CASE("The result JSON has the viewer's robots, paths and obstacles, plus lazydash stats") {
    const auto scene = Scene::load(
        (std::filesystem::path(LAZYDASH_RESOURCE_DIR) / "scenes" / "generated" / "wall-2walls-4objects-s0.json").string());
    LazyDashResult result;
    result.success = true;
    result.stats.makespan = 9;
    for (const auto &arm : scene.arms) {
        comotion::Path p;
        for (int t = 0; t <= 9; ++t)
            p.push_back(arm.start);
        result.robot_paths.push_back(p);
    }
    for (const auto &obj : scene.objects) {
        comotion::Path p;
        const auto x = obj.start.translation();
        for (int t = 0; t <= 9; ++t)
            p.push_back({x.x(), x.y(), x.z()});
        result.object_paths.push_back(p);
    }
    result.task_plan = {"pick ...", "place ..."};
    const auto doc = nlohmann::json::parse(resultJson(scene, result, "resources/ur5e_hande/ur5e_hande.urdf", 4));

    REQUIRE(doc.at("robots").size() == scene.arms.size() + scene.objects.size());
    const auto &arm = doc.at("robots").at(0);
    CHECK(arm.at("robot_type") == "ur5e_hande");
    CHECK(arm.at("joint_names").size() == 6);
    CHECK(arm.at("base_pose").at("quaternion_xyzw").size() == 4);
    CHECK(arm.at("path").size() == 4); // timesteps 0, 4, 8 and the last one (9)
    const auto &block = doc.at("robots").at(scene.arms.size());
    CHECK(block.at("robot_type") == "sphere");
    CHECK(block.at("robot_radius").get<double>() > 0.0);
    CHECK(block.at("path").at(0).size() == 3);
    CHECK(doc.at("obstacles").size() == scene.spheres.size() + scene.cylinders.size());
    CHECK(doc.at("obstacles").at(0).at("geometry").contains("radius"));
    CHECK(doc.at("lazydash").at("stats").at("success") == true);
    CHECK(doc.at("lazydash").at("stats").at("makespan") == 9);
    CHECK(doc.at("lazydash").at("task_plan").size() == 2);
}
