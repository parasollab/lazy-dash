#pragma once

#include "lazydash/robot/Formation.h"
#include "lazydash/robot/GraspSet.h"
#include "lazydash/scene/Scene.h"

#include "comotion/robot/RobotModel.h"

#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace lazydash {

/// Every body of a scene as CoMotion robots at both validity levels: padded sphere
/// models for planning, unpadded meshes for verification. Composition spaces are
/// `arm(r)` and `holding(r, g)` (a formation of arm and block).
class BodyModels {
public:
    explicit BodyModels(const Scene &scene);

    int numRobots() const { return static_cast<int>(arms_.size()); }

    const std::shared_ptr<comotion::RobotModel> &arm(int r) const { return arms_.at(idx(r)); }
    const std::shared_ptr<comotion::RobotModel> &armMesh(int r) const { return arm_meshes_.at(idx(r)); }
    /// Arm r carrying a block with grasp g (planning / verification model).
    std::shared_ptr<const comotion::RobotModel> holding(int r, int g) const;
    std::shared_ptr<const comotion::RobotModel> holdingMesh(int r, int g) const;
    const Formation &formation(int r, int g) const;
    const std::shared_ptr<comotion::RobotModel> &object(int o) const { return objects_.at(idx(o)); }

    const GraspSet &graspSet(int r) const { return grasp_sets_.at(idx(r)); }
    static int objectContactId(int o) { return 1000 + o; }
    const Scene &scene() const { return scene_; }

private:
    static std::size_t idx(int i) { return static_cast<std::size_t>(i); }
    struct Holding {
        std::unique_ptr<Formation> planning, verification;
    };
    const Holding &holdingEntry(int r, int g) const;

    const Scene &scene_;
    std::vector<std::shared_ptr<comotion::RobotModel>> arms_, arm_meshes_, objects_;
    std::vector<GraspSet> grasp_sets_;
    mutable std::map<std::pair<int, int>, Holding> holding_;
};

} // namespace lazydash
