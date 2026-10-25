#include "lazydash/schedule/MeshVerifier.h"

#include <algorithm>
#include <cmath>

namespace lazydash {

MeshVerifier::MeshVerifier(const Scene &scene)
    : checker_(comotion::CollisionChecker::Backend::Fcl) {
    scene.applyObstacles(checker_);
}

bool MeshVerifier::pathValid(const comotion::RobotModel &robot, const comotion::Path &path,
                             double step) const {
    return firstInvalidSegment(robot, path, step) < 0;
}

int MeshVerifier::firstInvalidSegment(const comotion::RobotModel &robot,
                                      const comotion::Path &path, double step) const {
    if (path.size() == 1)
        return checker_.isValidSingleFull(robot, path[0]) ? -1 : 0;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        double span = 0.0;
        for (std::size_t j = 0; j < path[i].size(); ++j)
            span = std::max(span, std::abs(path[i + 1][j] - path[i][j]));
        const int checks = std::max(1, static_cast<int>(std::ceil(span / step)));
        if (!checker_.isMotionValid(robot, path[i], path[i + 1], checks))
            return static_cast<int>(i);
    }
    return -1;
}

std::optional<comotion::CompositeConflict>
MeshVerifier::firstConflict(const std::vector<comotion::Path> &paths,
                            const std::vector<const comotion::RobotModel *> &robots) const {
    return checker_.findFirstCompositePathConflict(paths, robots, {});
}

} // namespace lazydash
