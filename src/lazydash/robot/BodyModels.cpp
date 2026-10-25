#include "lazydash/robot/BodyModels.h"

#include <cmath>
#include <stdexcept>

namespace lazydash {

BodyModels::BodyModels(const Scene &scene) : scene_(scene) {
    arms_ = scene.makeArms(Ur5eGeometry::Spheres);
    arm_meshes_ = scene.makeArms(Ur5eGeometry::Meshes);
    for (std::size_t r = 0; r < arms_.size(); ++r) {
        arms_[r]->setContactId(static_cast<int>(r));
        arm_meshes_[r]->setContactId(static_cast<int>(r));
        grasp_sets_.push_back(scene.graspSet(*arms_[r]));
    }
    for (std::size_t o = 0; o < scene.objects.size(); ++o) {
        if (std::abs(scene.objects[o].collision_radius - scene.objects[0].collision_radius) > 1e-9)
            throw std::invalid_argument("BodyModels: blocks must share one collision radius");
        objects_.push_back(scene.makeObject(o));
        objects_.back()->setContactId(objectContactId(static_cast<int>(o)));
    }
}

const BodyModels::Holding &BodyModels::holdingEntry(int r, int g) const {
    auto it = holding_.find({r, g});
    if (it == holding_.end()) {
        if (objects_.empty())
            throw std::logic_error("BodyModels: no objects to hold");
        const auto grasp = grasp_sets_.at(idx(r)).grasp(static_cast<std::size_t>(g));
        Holding h;
        h.planning = std::make_unique<Formation>(arms_.at(idx(r)), objects_.front(), grasp);
        h.verification = std::make_unique<Formation>(arm_meshes_.at(idx(r)), objects_.front(), grasp);
        it = holding_.emplace(std::make_pair(r, g), std::move(h)).first;
    }
    return it->second;
}

std::shared_ptr<const comotion::RobotModel> BodyModels::holding(int r, int g) const {
    return holdingEntry(r, g).planning->model();
}

std::shared_ptr<const comotion::RobotModel> BodyModels::holdingMesh(int r, int g) const {
    return holdingEntry(r, g).verification->model();
}

const Formation &BodyModels::formation(int r, int g) const {
    return *holdingEntry(r, g).planning;
}

} // namespace lazydash
