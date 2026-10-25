// Solve a scene; print JSON statistics, and with --out write a CoMotion viewer result.
//   lazydash_solve <scene.json> [--out result.json] [--seed N] [--time-limit S]
//                  [--stride K] [--log] [--verbose]
//                  [--stage-time S] [--detour F] [--rrt-time S] [--roadmap-samples N]
#include "lazydash/planning/LazyDash.h"
#include "lazydash/planning/ResultJson.h"
#include "lazydash/scene/Scene.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "usage: lazydash_solve <scene.json> [--out result.json] [--seed N] [--time-limit S]"
                     " [--stride K] [--log] [--verbose] [--stage-time S] [--detour F] [--rrt-time S]"
                     " [--roadmap-samples N]\n";
        return 2;
    }
    std::string scene_path = argv[1], out;
    lazydash::LazyDashParams params;
    int stride = 1;
    bool log = false;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << a << " needs a value\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--out")
            out = next();
        else if (a == "--seed")
            params.seed = std::stoull(next());
        else if (a == "--time-limit")
            params.time_limit = std::stod(next());
        else if (a == "--stride")
            stride = std::stoi(next());
        else if (a == "--log")
            log = true;
        else if (a == "--verbose")
            params.verbose = true;
        else if (a == "--stage-time")
            params.durations.stage_time = std::stod(next());
        else if (a == "--detour")
            params.durations.detour = std::stod(next());
        else if (a == "--rrt-time")
            params.roadmap.rrt_time = std::stod(next());
        else if (a == "--roadmap-samples")
            params.roadmap.initial_samples = std::stoi(next());
        else {
            std::cerr << "unknown option " << a << "\n";
            return 2;
        }
    }
    const auto scene = lazydash::Scene::load(scene_path);
    lazydash::LazyDash planner(scene, params);
    const auto result = planner.solve();
    if (log)
        for (const auto &line : result.log)
            std::cerr << line << "\n";
    std::cout << lazydash::statsJson(result) << std::endl;
    if (!out.empty()) {
        // The viewer serves the repository root; the URDF is found relative to it.
        const std::string urdf = "resources/ur5e_hande/ur5e_hande.urdf";
        std::ofstream(out) << lazydash::resultJson(scene, result, urdf, stride);
    }
    return result.success ? 0 : 1;
}
