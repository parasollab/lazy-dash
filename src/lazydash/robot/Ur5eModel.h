#pragma once

#include "comotion/robot/RobotModel.h"

#include <Eigen/Core>
#include <memory>
#include <string>

namespace lazydash {

/// Planar base placement: position plus rotation about world z.
struct BasePose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;
};

/// Which collision geometry to load for the UR5e + Hand-e model.
enum class Ur5eGeometry {
    /// ur5e_hande_spherized.urdf: inscribed spheres for the Spheres backend (planning).
    Spheres,
    /// ur5e_hande.urdf: convex meshes for the Fcl backend (verification).
    Meshes,
};

/// The robot model files in resources/ur5e_hande/.
struct Ur5eResources {
    std::string spherized_urdf;
    std::string mesh_urdf;
    std::string srdf;

    static Ur5eResources fromDirectory(const std::string &dir);
    /// resources/ur5e_hande of this source tree.
    static Ur5eResources defaults();
};

/// Link frames used by grasping and attachments.
inline constexpr const char *kUr5eToolLink = "tool0";
inline constexpr const char *kUr5eGripperLink = "robotiq_hande_base_link";

/// Load a UR5e + Hand-e arm at the given base. Planning with it needs the Spheres
/// (or Fcl) backend; there is no UR5e VAMP kernel.
std::shared_ptr<comotion::RobotModel>
makeUr5e(const BasePose &base, Ur5eGeometry geometry = Ur5eGeometry::Spheres,
         const Ur5eResources &resources = Ur5eResources::defaults());

} // namespace lazydash
