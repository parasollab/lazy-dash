#pragma once

#include "lazydash/motion/OptimisticSchedule.h"
#include "lazydash/robot/BodyModels.h"
#include "lazydash/task/TaskHypergraph.h"
#include "lazydash/task/TaskSearch.h"

#include "comotion/planning/Path.h"

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace lazydash {

/// A segment of the optimistic schedule with timing (timesteps).
struct TimedSegment {
    MotionSegment seg;
    comotion::Path path;      // waypoint timesteps from 0 (at the schedule's resolution and vmax)
    std::size_t delay = 0;    // extra wait before it starts (timing repairs, layer 4)
    std::size_t start = 0;    // set by retime()

    std::size_t duration() const { return path.arrival_timestep(); }
    std::size_t end() const { return start + duration(); }
};

/// The schedule of layer 4: S_M mapped onto time (MapTaskMotion) under the
/// dependency graph D (per-arm S_T order, handover barriers, C_f orderings).
/// Objects rest until grasped, then follow the carrying arm's FK.
class TimedSchedule {
public:
    TimedSchedule(const OptimisticSchedule &sm, const TaskSchedule &st, const TaskHypergraph &hg,
                  const BodyModels &models, std::size_t resolution = 128, double vmax = 1.0);

    void retime();
    /// Take the delays of `previous` for unchanged segments, then retime
    /// (warm start after layer 3 replanned some moves).
    void inheritDelays(const TimedSchedule &previous);

    std::vector<TimedSegment> &segments() { return segments_; }
    const std::vector<TimedSegment> &segments() const { return segments_; }
    const std::vector<std::vector<int>> &byRobot() const { return by_robot_; }
    std::size_t makespan() const;
    int numRobots() const { return static_cast<int>(by_robot_.size()); }
    int numObjects() const { return static_cast<int>(object_events_.size()); }
    std::size_t resolution() const { return resolution_; }
    double vmax() const { return vmax_; }

    /// Arm r's configuration at timestep t (holding still between segments).
    std::vector<double> robotConfigAt(int r, std::size_t t) const;
    /// The segment arm r executes at t (-1 when it holds still).
    int segmentAt(int r, std::size_t t) const;
    /// The grasp arm r holds an object with at t (-1: hand free), and which object.
    int heldGraspAt(int r, std::size_t t, int *object = nullptr) const;
    /// H_T element arm r is in at t.
    int robotElementAt(int r, std::size_t t) const;

    struct ObjectState {
        bool carried = false;
        int robot = -1, grasp = -1;  // when carried
        int element = -1;            // when resting: its ObjectAt element
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
    };
    ObjectState objectAt(int o, std::size_t t) const;

    /// Times where the set of bodies or their models change: every stage
    /// start and end (grasp, handover and release events, contact windows),
    /// plus 0 and the makespan. Sorted, unique.
    std::vector<std::size_t> epochBoundaries() const;

    /// Dense per-timestep paths over [t0, t1] (inclusive).
    comotion::Path robotSlice(int r, std::size_t t0, std::size_t t1) const;
    comotion::Path objectSlice(int o, std::size_t t0, std::size_t t1) const;

    /// Replace the part of segment `id` between local timesteps [a, b] by
    /// `repaired` (dense, starting at the config at a and ending at the
    /// config at b); its duration changes by the repaired length difference.
    void spliceSegment(int id, std::size_t a, std::size_t b, const comotion::Path &repaired);

private:
    struct ObjectEvent {
        std::size_t time;     // when the change takes effect
        bool carried;
        int robot, grasp;     // new carrier
        int element;          // new resting element (when released)
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        int segment;          // segment whose end defines `time`
    };
    void timeSegment(TimedSegment &s) const;
    void rebuildEvents();

    const TaskHypergraph &hg_;
    const BodyModels &models_;
    std::size_t resolution_;
    double vmax_;
    std::vector<TimedSegment> segments_;
    std::vector<std::vector<int>> by_robot_;
    std::vector<std::vector<double>> start_;
    // Barriers: segment -> segments that must end before it starts.
    std::vector<std::vector<int>> after_;
    // Per object: initial state + events (ordered by time after retime).
    std::vector<ObjectState> object_initial_;
    std::vector<std::vector<ObjectEvent>> object_events_;
};

} // namespace lazydash
