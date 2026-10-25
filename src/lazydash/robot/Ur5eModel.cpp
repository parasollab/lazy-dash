#include "lazydash/robot/Ur5eModel.h"

#include <Eigen/Geometry>
#include <filesystem>

namespace lazydash {

Ur5eResources Ur5eResources::fromDirectory(const std::string &dir) {
    const std::filesystem::path root(dir);
    return {(root / "ur5e_hande_spherized.urdf").string(),
            (root / "ur5e_hande.urdf").string(),
            (root / "ur5e_hande.srdf").string()};
}

Ur5eResources Ur5eResources::defaults() {
    return fromDirectory(std::string(LAZYDASH_RESOURCE_DIR) + "/ur5e_hande");
}

std::shared_ptr<comotion::RobotModel> makeUr5e(const BasePose &base,
                                               Ur5eGeometry geometry,
                                               const Ur5eResources &resources) {
    auto model = std::make_shared<comotion::RobotModel>();
    model->loadURDF(geometry == Ur5eGeometry::Spheres ? resources.spherized_urdf
                                                      : resources.mesh_urdf);
    model->loadSRDF(resources.srdf);
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translate(base.position);
    T.rotate(Eigen::AngleAxisd(base.yaw, Eigen::Vector3d::UnitZ()));
    model->setBaseTransform(T);
    return model;
}

} // namespace lazydash
