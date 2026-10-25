#include "lazydash/planning/ResultJson.h"

#include "lazydash/robot/Ur5eModel.h"

#include <nlohmann/json.hpp>

#include <cmath>

namespace lazydash {

using nlohmann::json;

namespace {

json statsObject(const LazyDashResult &r) {
    const auto &s = r.stats;
    return {
        {"success", r.success},
        {"failure", r.failure},
        {"makespan", s.makespan},
        {"planning_time", s.total},
        {"task_query_time", s.task_query},
        {"task_conflict_time", s.task_conflicts},
        {"motion_query_time", s.motion_query},
        {"conflict_resolution_time", s.conflict_resolution},
        {"ht_vertices", s.ht_vertices},
        {"ht_hyperarcs", s.ht_arcs},
        {"hte_expansions", s.hte_expansions},
        {"hm_vertices", s.hm_vertices},
        {"hm_hyperarcs", s.hm_arcs},
        {"roadmap_vertices", s.roadmap_vertices},
        {"outer_iterations", s.outer_iterations},
        {"inner_iterations", s.inner_iterations},
        {"task_constraints", s.task_constraints},
        {"motion_constraints", s.motion_constraints},
        {"tentative_lifted", s.tentative_lifted},
        {"known_failure_repeats", s.known_failure_repeats},
        {"move_outs", s.move_outs},
        {"hh_conflicts", s.hh},
        {"hv_conflicts", s.hv},
        {"timing_fixes", s.timing_fixes},
        {"arc_repairs", s.arc_repairs},
        {"mesh_conflicts", s.mesh_conflicts},
    };
}

json quaternion(double yaw) { return {0.0, 0.0, std::sin(yaw / 2), std::cos(yaw / 2)}; }

json pathJson(const comotion::Path &p, int stride) {
    json out = json::array();
    for (std::size_t i = 0; i < p.size(); i += static_cast<std::size_t>(stride))
        out.push_back(p[i]);
    if (!p.empty() && (p.size() - 1) % static_cast<std::size_t>(stride) != 0)
        out.push_back(p.back());
    return out;
}

} // namespace

std::string statsJson(const LazyDashResult &result) { return statsObject(result).dump(); }

std::string resultJson(const Scene &scene, const LazyDashResult &result, const std::string &urdf_path,
                       int stride) {
    stride = std::max(stride, 1);
    json doc;
    doc["planner"] = "lazydash";
    doc["scene"] = scene.name;
    doc["success"] = result.success;
    doc["resolution"] = result.resolution;

    json obstacles = json::array();
    for (const auto &s : scene.spheres)
        obstacles.push_back({{"type", "sphere"},
                             {"pose", {{"position", {s.center.x(), s.center.y(), s.center.z()}},
                                       {"quaternion_xyzw", {0, 0, 0, 1}}}},
                             {"geometry", {{"radius", s.radius}}}});
    for (const auto &c : scene.cylinders)
        obstacles.push_back({{"type", "cylinder"},
                             {"pose", {{"position", {c.center.x(), c.center.y(), c.center.z()}},
                                       {"axis", {c.axis.x(), c.axis.y(), c.axis.z()}}}},
                             {"geometry", {{"radius", c.radius}, {"half_height", c.half_height}}}});
    doc["obstacles"] = obstacles;

    json robots = json::array();
    const auto names = makeUr5e({})->activeJointNames();
    for (std::size_t r = 0; r < scene.arms.size(); ++r) {
        const auto &arm = scene.arms[r];
        json j = {{"name", arm.name},
                  {"robot_type", "ur5e_hande"},
                  {"urdf_path", urdf_path},
                  {"visual_urdf_path", urdf_path},
                  {"joint_names", names},
                  {"base_pose", {{"position", {arm.base.position.x(), arm.base.position.y(), arm.base.position.z()}},
                                 {"quaternion_xyzw", quaternion(arm.base.yaw)}}}};
        j["path"] = r < result.robot_paths.size() ? pathJson(result.robot_paths[r], stride) : json::array({arm.start});
        robots.push_back(j);
    }
    for (std::size_t o = 0; o < scene.objects.size(); ++o) {
        const auto &obj = scene.objects[o];
        json j = {{"name", obj.name}, {"robot_type", "sphere"}, {"robot_radius", obj.collision_radius}};
        if (o < result.object_paths.size())
            j["path"] = pathJson(result.object_paths[o], stride);
        else {
            const auto p = obj.start.translation();
            j["path"] = json::array({{p.x(), p.y(), p.z()}});
        }
        robots.push_back(j);
    }
    doc["robots"] = robots;
    json timeline = json::array();
    for (const auto &e : result.timeline)
        timeline.push_back({{"robot", e.robot}, {"start", e.start}, {"end", e.end},
                            {"kind", e.stage ? "stage" : "move"}, {"action", e.action},
                            {"object", e.object}, {"holding", e.holding}, {"step", e.step}});
    doc["lazydash"] = {{"stats", statsObject(result)}, {"task_plan", result.task_plan}, {"log", result.log},
                       {"stride", stride}, {"timeline", timeline}};
    return doc.dump();
}

} // namespace lazydash
