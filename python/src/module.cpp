// Python API of lazydash (nanobind): load a scene, solve it, read the result.
//   import lazydash
//   scene = lazydash.Scene.load("resources/scenes/generated/sorting-4robots-4objects-s0.json")
//   params = lazydash.Params(); params.seed = 3; params.time_limit = 120
//   result = lazydash.solve(scene, params)
//   result.success, lazydash.stats(result), result.robot_paths
#include "lazydash/planning/LazyDash.h"
#include "lazydash/planning/ResultJson.h"
#include "lazydash/scene/Scene.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace lazydash;

namespace {

std::vector<std::vector<std::vector<double>>> plain(const std::vector<comotion::Path> &paths) {
    std::vector<std::vector<std::vector<double>>> out;
    for (const auto &p : paths)
        out.emplace_back(p.begin(), p.end());
    return out;
}

} // namespace

NB_MODULE(_core, m) {
    m.doc() = "Lazy-DaSH multi-robot task and motion planning on CoMotion";

    nb::class_<Scene>(m, "Scene")
        .def_static("load", &Scene::load, nb::arg("path"))
        .def_static("parse", &Scene::parse, nb::arg("json_text"))
        .def_ro("name", &Scene::name)
        .def_prop_ro("num_arms", [](const Scene &s) { return s.arms.size(); })
        .def_prop_ro("num_objects", [](const Scene &s) { return s.objects.size(); });

    nb::class_<LazyDashParams>(m, "Params")
        .def(nb::init<>())
        .def_rw("seed", &LazyDashParams::seed)
        .def_rw("time_limit", &LazyDashParams::time_limit)
        .def_rw("max_outer", &LazyDashParams::max_outer)
        .def_rw("max_inner", &LazyDashParams::max_inner)
        .def_rw("lift_growth", &LazyDashParams::lift_growth)
        .def_rw("resolution", &LazyDashParams::resolution)
        .def_rw("vmax", &LazyDashParams::vmax);

    nb::class_<LazyDashResult>(m, "Result")
        .def_ro("success", &LazyDashResult::success)
        .def_ro("failure", &LazyDashResult::failure)
        .def_ro("log", &LazyDashResult::log)
        .def_ro("task_plan", &LazyDashResult::task_plan)
        .def_prop_ro("stats_json", [](const LazyDashResult &r) { return statsJson(r); })
        .def_prop_ro("robot_paths", [](const LazyDashResult &r) { return plain(r.robot_paths); })
        .def_prop_ro("object_paths", [](const LazyDashResult &r) { return plain(r.object_paths); });

    m.def(
        "solve",
        [](const Scene &scene, const LazyDashParams &params) {
            nb::gil_scoped_release release;
            return LazyDash(scene, params).solve();
        },
        nb::arg("scene"), nb::arg("params") = LazyDashParams{});
    m.def("result_json", &resultJson, nb::arg("scene"), nb::arg("result"),
          nb::arg("urdf_path") = "resources/ur5e_hande/ur5e_hande.urdf", nb::arg("stride") = 1,
          "CoMotion viewer result JSON of a run.");
}
