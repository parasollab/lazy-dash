#include "lazydash/scene/Scene.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace lazydash {

using nlohmann::json;

namespace {

constexpr const char *kFormat = "lazydash.scene/1";
constexpr const char *kUr5eType = "ur5e_hande";

Eigen::Vector3d vec3(const json &j) {
    return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}

Eigen::Isometry3d pose(const json &j) {
    return poseFromRpy(vec3(j.at("position")),
                       j.contains("rpy") ? vec3(j.at("rpy")) : Eigen::Vector3d::Zero());
}

/// Base orientation as a yaw; lazydash bases stand upright.
double baseYaw(const json &base, const std::string &robot) {
    if (!base.contains("quaternion_xyzw"))
        return 0.0;
    const auto &q = base.at("quaternion_xyzw");
    const Eigen::Quaterniond quat(q.at(3).get<double>(), q.at(0).get<double>(),
                                  q.at(1).get<double>(), q.at(2).get<double>());
    const Eigen::Vector3d z = quat.normalized() * Eigen::Vector3d::UnitZ();
    if (z.z() < 1.0 - 1e-9)
        throw std::runtime_error("scene: base of " + robot + " is not upright");
    const Eigen::Vector3d x = quat.normalized() * Eigen::Vector3d::UnitX();
    return std::atan2(x.y(), x.x());
}

} // namespace

Eigen::Isometry3d poseFromRpy(const Eigen::Vector3d &position, const Eigen::Vector3d &rpy) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = position;
    T.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
                     .toRotationMatrix();
    return T;
}

Scene Scene::load(const std::string &path) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("scene: cannot open " + path);
    std::stringstream text;
    text << in.rdbuf();
    return parse(text.str());
}

Scene Scene::parse(const std::string &json_text) {
    const json doc = json::parse(json_text);
    if (doc.value("format", "") != kFormat)
        throw std::runtime_error(std::string("scene: expected format ") + kFormat);
    Scene scene;
    scene.name = doc.value("name", "");
    scene.workspace_min = vec3(doc.at("environment").at("min"));
    scene.workspace_max = vec3(doc.at("environment").at("max"));

    for (const auto &obs : doc.at("obstacles")) {
        const auto type = obs.at("type").get<std::string>();
        if (type == "sphere") {
            scene.spheres.push_back({vec3(obs.at("center")), obs.at("radius").get<double>()});
        } else if (type == "cylinder") {
            scene.cylinders.push_back({vec3(obs.at("center")), vec3(obs.at("axis")).normalized(),
                                       obs.at("radius").get<double>(),
                                       obs.at("half_height").get<double>()});
        } else {
            throw std::runtime_error("scene: obstacles must be spheres or cylinders, got " + type);
        }
    }

    for (const auto &r : doc.at("robots")) {
        ArmSpec arm;
        arm.name = r.at("name").get<std::string>();
        arm.type = r.at("type").get<std::string>();
        if (arm.type != kUr5eType)
            throw std::runtime_error("scene: unsupported robot type " + arm.type);
        arm.base = {vec3(r.at("base").at("position")), baseYaw(r.at("base"), arm.name)};
        arm.start = r.at("start").get<std::vector<double>>();
        arm.lower = r.at("joint_limits").at("lower").get<std::vector<double>>();
        arm.upper = r.at("joint_limits").at("upper").get<std::vector<double>>();
        if (arm.start.size() != 6 || arm.lower.size() != 6 || arm.upper.size() != 6)
            throw std::runtime_error("scene: " + arm.name + " needs 6 joint values");
        scene.arms.push_back(std::move(arm));
    }

    const auto &ld = doc.at("lazydash");
    for (const auto &o : ld.at("objects")) {
        ObjectSpec obj;
        obj.name = o.at("name").get<std::string>();
        if (o.contains("shape"))
            obj.size = vec3(o.at("shape").at("size"));
        obj.collision_radius = o.at("collision_radius").get<double>();
        obj.start = pose(o.at("start"));
        for (const auto &g : o.at("goals"))
            obj.goals.push_back(pose(g));
        if (obj.goals.empty())
            throw std::runtime_error("scene: object " + obj.name + " has no goal");
        scene.objects.push_back(std::move(obj));
    }
    for (const auto &g : ld.at("grasps")) {
        GraspDef def;
        def.name = g.at("name").get<std::string>();
        def.link = g.at("link").get<std::string>();
        const Eigen::Vector3d rpy = vec3(g.at("rpy"));
        def.link_T_object = poseFromRpy(vec3(g.at("translation")), rpy);
        def.link_T_object_approach = poseFromRpy(
            vec3(g.value("approach_translation", g.at("translation"))), rpy);
        scene.grasps.push_back(std::move(def));
    }
    if (ld.contains("params")) {
        const auto &p = ld.at("params");
        auto &t = scene.params;
        t.robot_range = p.value("robot_range", t.robot_range);
        t.handover_max_base_distance =
            p.value("handover_max_base_distance", t.handover_max_base_distance);
        t.planning_padding = p.value("planning_padding", t.planning_padding);
        if (p.contains("handover_region")) {
            const auto &h = p.at("handover_region");
            t.handover_xy_half_width = h.value("xy_half_width", t.handover_xy_half_width);
            if (h.contains("z_above_base")) {
                t.handover_z_min = h.at("z_above_base").at(0).get<double>();
                t.handover_z_max = h.at("z_above_base").at(1).get<double>();
            }
            t.handover_attempts = h.value("attempts", t.handover_attempts);
        }
    }
    if (ld.contains("placement_regions"))
        for (const auto &r : ld.at("placement_regions")) {
            PlacementRegion region;
            region.min = vec3(r.at("min"));
            region.max = vec3(r.at("max"));
            region.yaw_min = r.at("yaw").at(0).get<double>();
            region.yaw_max = r.at("yaw").at(1).get<double>();
            region.object = r.value("object", -1);
            scene.placement_regions.push_back(region);
        }
    return scene;
}

std::vector<std::shared_ptr<comotion::RobotModel>>
Scene::makeArms(Ur5eGeometry geometry) const {
    std::vector<std::shared_ptr<comotion::RobotModel>> out;
    for (const auto &arm : arms) {
        out.push_back(makeUr5e(arm.base, geometry));
        if (geometry == Ur5eGeometry::Spheres)
            out.back()->setCollisionPadding(params.planning_padding);
    }
    return out;
}

std::shared_ptr<comotion::RobotModel> Scene::makeObject(std::size_t i) const {
    return lazydash::makeObject(objects.at(i).collision_radius, workspace_min, workspace_max);
}

GraspSet Scene::graspSet(const comotion::RobotModel &arm) const {
    return GraspSet(arm, grasps, kUr5eGripperLink, {"left_finger", "right_finger"},
                    {"wrist_3_link", kUr5eGripperLink, "left_finger", "right_finger"});
}

void Scene::applyObstacles(comotion::CollisionChecker &checker) const {
    checker.setObstacles(spheres);
    checker.setCylinderObstacles(cylinders);
}

} // namespace lazydash
