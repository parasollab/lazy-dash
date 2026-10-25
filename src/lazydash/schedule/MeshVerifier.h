#pragma once

#include "lazydash/scene/Scene.h"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ValidationTypes.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/RobotModel.h"

#include <optional>
#include <vector>

namespace lazydash {

/// Second validity level: checks motions on exact UR5e meshes (Fcl backend);
/// planners search on padded spheres and their motions are accepted only if
/// they also pass here. Robots must be mesh twins of the planning models.
class MeshVerifier {
public:
    explicit MeshVerifier(const Scene &scene);

    /// One robot's path against the static scene and itself, checked densely
    /// along every segment (at most `step` rad per joint between checks).
    bool pathValid(const comotion::RobotModel &robot, const comotion::Path &path,
                   double step = kCheckStep) const;
    /// Index of the first invalid segment of `path` (-1: valid). Segment i
    /// runs from waypoint i to i + 1; a lone invalid waypoint reports 0.
    int firstInvalidSegment(const comotion::RobotModel &robot, const comotion::Path &path,
                            double step = kCheckStep) const;

    /// Joint-space resolution of motion checks (rad per check).
    static constexpr double kCheckStep = 0.01;

    /// First conflict of a schedule (environment, self or inter-robot).
    std::optional<comotion::CompositeConflict>
    firstConflict(const std::vector<comotion::Path> &paths,
                  const std::vector<const comotion::RobotModel *> &robots) const;

    const comotion::CollisionChecker &checker() const { return checker_; }

private:
    comotion::CollisionChecker checker_;
};

} // namespace lazydash
