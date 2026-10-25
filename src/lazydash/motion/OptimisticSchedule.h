#pragma once

#include "lazydash/task/TransitionSampler.h"

#include "comotion/planning/Path.h"

#include <optional>
#include <vector>

namespace lazydash {

/// One arm's piece of the optimistic schedule S_M (untimed): a Move (an E^move to
/// the anchor of transition `step`) or a Stage (one straight stage of its E^tran,
/// possibly allowing finger contact with `contact_object` or `contact_robot`'s object).
struct MotionSegment {
    enum class Kind { Move, Stage };
    Kind kind = Kind::Move;
    int robot = -1;
    int step = -1;
    int stage = -1;
    int element = -1;       // H_T element the arm is in during the segment
    int te = -1;            // its H_TE vertex
    int holding_grasp = -1; // -1: hand free
    int object = -1;        // carried object (holding) or transition object (stage)
    int contact_object = -1;
    int contact_robot = -1;
    int contact_grasp = -1;
    comotion::Path path;
};

/// The optimistic schedule S_M: every arm's moves and transition stages in S_T
/// order, valid against the static scene and own body; inter-body collisions are layer 4's.
struct OptimisticSchedule {
    std::vector<MotionSegment> segments;
    std::vector<std::vector<int>> by_robot;            // arm -> segment ids in order
    std::vector<std::optional<TransitionAnchors>> anchors; // per S_T step
    std::vector<std::vector<double>> start;            // arm -> start configuration
};

} // namespace lazydash
