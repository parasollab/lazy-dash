#pragma once

#include "lazydash/task/Capabilities.h"
#include "lazydash/task/TaskHypergraph.h"

#include <Eigen/Core>
#include <vector>

namespace lazydash {

/// Parameters of the duration estimates. A transition costs an arm a
/// near-constant time; travel distance is only a tie-break.
struct DurationParams {
    double stage_time = 6.5;  // s per arm and transition (move and stages)
    double detour = 0.1;      // travel weight: s per radian of the joint-space proxy
    double link_length = 0.4; // m of reach or height per radian of shoulder and elbow
    double vmax = 1.0;        // rad/s per joint
};

/// Duration estimates for the time-aware task query in layer 1. Durations
/// come from transition points: a joint-space travel proxy from the arm's
/// last transition point, plus a fixed `stage_time`.
class DurationModel {
public:
    DurationModel(const Capabilities &caps, const TaskHypergraph &hg, DurationParams params = {});
    /// With the arms' start tool points given (`home`, one per arm).
    DurationModel(const Scene &scene, const TaskHypergraph &hg, std::vector<Eigen::Vector3d> home,
                  DurationParams params = {});

    /// Where a transition happens: the pick/place object pose, or the middle
    /// of the handover region.
    Eigen::Vector3d point(int arc) const;
    /// The arm's tool point in its start configuration.
    const Eigen::Vector3d &home(int robot) const { return home_.at(static_cast<std::size_t>(robot)); }
    double travel(int robot, const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    double stageTime() const { return params_.stage_time; }
    /// Seconds per unit of transition cost estimate (m), for the time-to-go.
    double costScale() const { return params_.detour / (params_.link_length * params_.vmax); }

private:
    const Scene &scene_;
    const TaskHypergraph &hg_;
    DurationParams params_;
    std::vector<Eigen::Vector3d> home_;
};

} // namespace lazydash
