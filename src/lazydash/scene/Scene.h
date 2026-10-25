#pragma once

#include "lazydash/robot/GraspSet.h"
#include "lazydash/robot/Ur5eModel.h"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ObstacleShapes.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

namespace lazydash {

/// A manipulator of the scene.
struct ArmSpec {
    std::string name;
    std::string type; // "ur5e_hande"
    BasePose base;
    std::vector<double> start;
    std::vector<double> lower, upper; // joint limits used for planning and IK
};

/// A movable object: a rigid block with a start pose and alternative goals.
struct ObjectSpec {
    std::string name;
    Eigen::Vector3d size = Eigen::Vector3d::Zero(); // box, for display
    double collision_radius = 0.0;
    Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
    /// Alternatives (XOR): the object is done when it rests at any of them.
    std::vector<Eigen::Isometry3d> goals;
};

/// Task-level parameters.
struct TaskParams {
    double robot_range = 1.5;                // base-to-object reach for pick/place
    double handover_max_base_distance = 1.6; // bases further apart cannot hand over
    double handover_xy_half_width = 0.05;    // around the midpoint between the bases
    double handover_z_min = 0.65, handover_z_max = 0.9; // above the bases
    int handover_attempts = 1000;
    /// Collision padding of the sphere planning models.
    double planning_padding = 0.01;
};

/// Where move-out poses are sampled: positions in [min, max] (equal z: a surface),
/// upright, yaw in [yaw_min, yaw_max], for one `object` or any (-1).
struct PlacementRegion {
    Eigen::Vector3d min = Eigen::Vector3d::Zero(), max = Eigen::Vector3d::Zero();
    double yaw_min = 0.0, yaw_max = 0.0;
    int object = -1;
};

/// A Lazy-DaSH scene (JSON "lazydash.scene/1"): the CoMotion part (workspace,
/// obstacles, robots) and the `lazydash` block (objects, grasp set, parameters).
struct Scene {
    std::string name;
    Eigen::Vector3d workspace_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d workspace_max = Eigen::Vector3d::Zero();
    std::vector<comotion::ObstacleSphere> spheres;
    std::vector<comotion::ObstacleCylinder> cylinders;
    std::vector<ArmSpec> arms;
    std::vector<ObjectSpec> objects;
    std::vector<GraspDef> grasps;
    TaskParams params;
    std::vector<PlacementRegion> placement_regions;

    static Scene load(const std::string &path);
    static Scene parse(const std::string &json_text);

    /// Robot models of the arms, in scene order. Sphere models (planning)
    /// carry `params.planning_padding`; mesh models (verification) none.
    std::vector<std::shared_ptr<comotion::RobotModel>>
    makeArms(Ur5eGeometry geometry = Ur5eGeometry::Spheres) const;
    /// Object `i` as a CoMotion robot (passive, positional).
    std::shared_ptr<comotion::RobotModel> makeObject(std::size_t i) const;
    /// The grasp set for `arm` (the Hand-e carries objects on its base link).
    GraspSet graspSet(const comotion::RobotModel &arm) const;
    /// Install the static obstacles in a collision checker.
    void applyObstacles(comotion::CollisionChecker &checker) const;
};

/// URDF-style rpy (R = Rz(yaw) Ry(pitch) Rx(roll)) plus position.
Eigen::Isometry3d poseFromRpy(const Eigen::Vector3d &position, const Eigen::Vector3d &rpy);

} // namespace lazydash
