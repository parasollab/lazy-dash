#include "lazydash/schedule/TimedSchedule.h"

#include <algorithm>
#include <set>

namespace lazydash {

TimedSchedule::TimedSchedule(const OptimisticSchedule &sm, const TaskSchedule &st,
                             const TaskHypergraph &hg, const BodyModels &models,
                             std::size_t resolution, double vmax)
    : hg_(hg), models_(models), resolution_(resolution), vmax_(vmax), by_robot_(sm.by_robot),
      start_(sm.start) {
    for (const auto &seg : sm.segments) {
        TimedSegment ts;
        ts.seg = seg;
        ts.path = seg.path;
        timeSegment(ts);
        segments_.push_back(std::move(ts));
    }
    after_.resize(segments_.size());

    const auto previousOf = [&](int robot, int id) {
        const auto &list = by_robot_[static_cast<std::size_t>(robot)];
        const auto it = std::find(list.begin(), list.end(), id);
        return it == list.begin() ? -1 : *(it - 1);
    };
    const auto nextOf = [&](int robot, int id) {
        const auto &list = by_robot_[static_cast<std::size_t>(robot)];
        const auto it = std::find(list.begin(), list.end(), id);
        return (it == list.end() || it + 1 == list.end()) ? -1 : *(it + 1);
    };
    const auto stageOf = [&](int step, int stage) {
        for (std::size_t i = 0; i < segments_.size(); ++i)
            if (segments_[i].seg.kind == MotionSegment::Kind::Stage && segments_[i].seg.step == step &&
                segments_[i].seg.stage == stage)
                return static_cast<int>(i);
        return -1;
    };

    // Objects: initial rest at their start; events at grasp, handover, release.
    const int O = hg.numObjects();
    object_events_.resize(static_cast<std::size_t>(O));
    for (int o = 0; o < O; ++o) {
        ObjectState s;
        s.element = hg.startElement(o);
        s.position = hg.locations()[static_cast<std::size_t>(hg.graph().vertex(s.element).location)]
                         .poses[0]
                         .translation();
        object_initial_.push_back(s);
    }
    for (std::size_t step = 0; step < st.steps.size(); ++step) {
        const auto &t = hg.graph().arc(st.steps[step].arc).data;
        if (t.object < 0) // source, sink
            continue;
        const int s0 = stageOf(static_cast<int>(step), 0), s1 = stageOf(static_cast<int>(step), 1);
        auto &events = object_events_[static_cast<std::size_t>(t.object)];
        if (t.type == TransitionType::Pick) {
            events.push_back({0, true, t.robot, t.grasp, -1, Eigen::Vector3d::Zero(), s0}); // closed after the approach
        } else if (t.type == TransitionType::Place) {
            const auto &a = sm.anchors[step];
            events.push_back({0, false, -1, -1, hg.objectHead(st.steps[step].arc),
                              a ? Eigen::Vector3d(a->object_pose.translation()) : Eigen::Vector3d::Zero().eval(),
                              s0}); // opened at the goal or move-out
        } else if (t.type == TransitionType::Handover) {
            events.push_back({0, true, t.robot2, t.grasp2, -1, Eigen::Vector3d::Zero(), s0}); // receiver closed
            // D: stage barriers of the exchange.
            const int giver_prev = previousOf(t.robot, s1);
            if (giver_prev >= 0)
                after_[static_cast<std::size_t>(s0)].push_back(giver_prev);
            after_[static_cast<std::size_t>(s1)].push_back(s0);
            const int receiver_next = nextOf(t.robot2, s0);
            if (receiver_next >= 0)
                after_[static_cast<std::size_t>(receiver_next)].push_back(s1);
        }
    }
    // D: C_f orderings; every segment of step b starts after every segment of
    // each step in st.steps[b].after.
    std::vector<std::vector<int>> of_step(st.steps.size());
    for (std::size_t i = 0; i < segments_.size(); ++i)
        if (segments_[i].seg.step >= 0)
            of_step[static_cast<std::size_t>(segments_[i].seg.step)].push_back(static_cast<int>(i));
    for (std::size_t b = 0; b < st.steps.size(); ++b)
        for (int p : st.steps[b].after)
            for (int i : of_step[b])
                for (int j : of_step[static_cast<std::size_t>(p)])
                    after_[static_cast<std::size_t>(i)].push_back(j);
    retime();
}

void TimedSchedule::timeSegment(TimedSegment &s) const {
    if (s.path.size() == 1)
        s.path.push_back(s.path.front());
    s.path.clearWaypointTimesteps();
    s.path.computeTimestepsFromDistance(resolution_, vmax_);
}

void TimedSchedule::retime() {
    std::vector<int> previous(segments_.size(), -1);
    for (const auto &list : by_robot_)
        for (std::size_t k = 1; k < list.size(); ++k)
            previous[static_cast<std::size_t>(list[k])] = list[k - 1];
    // Segments were created in S_T order, which is a topological order of D.
    for (std::size_t i = 0; i < segments_.size(); ++i) {
        std::size_t ready = 0;
        if (previous[i] >= 0)
            ready = segments_[static_cast<std::size_t>(previous[i])].end();
        for (int b : after_[i])
            ready = std::max(ready, segments_[static_cast<std::size_t>(b)].end());
        segments_[i].start = ready + segments_[i].delay;
    }
    rebuildEvents();
}

void TimedSchedule::inheritDelays(const TimedSchedule &previous) {
    for (auto &s : segments_) {
        for (const auto &p : previous.segments_) {
            if (p.seg.robot != s.seg.robot || p.seg.step != s.seg.step || p.seg.kind != s.seg.kind ||
                p.seg.stage != s.seg.stage || p.seg.path.size() != s.seg.path.size())
                continue;
            bool same = true;
            for (std::size_t i = 0; i < s.seg.path.size() && same; ++i)
                same = s.seg.path[i] == p.seg.path[i];
            if (same && p.path.size() == p.seg.path.size()) // not spliced by a repair
                s.delay = p.delay;
            break;
        }
    }
    retime();
}

void TimedSchedule::rebuildEvents() {
    for (auto &events : object_events_) {
        for (auto &e : events)
            e.time = segments_[static_cast<std::size_t>(e.segment)].end();
        std::stable_sort(events.begin(), events.end(),
                         [](const ObjectEvent &a, const ObjectEvent &b) { return a.time < b.time; });
    }
}

std::size_t TimedSchedule::makespan() const {
    std::size_t m = 0;
    for (const auto &s : segments_)
        m = std::max(m, s.end());
    return m;
}

int TimedSchedule::segmentAt(int r, std::size_t t) const {
    for (int id : by_robot_[static_cast<std::size_t>(r)]) {
        const auto &s = segments_[static_cast<std::size_t>(id)];
        if (s.start <= t && t < s.end())
            return id;
    }
    return -1;
}

std::vector<double> TimedSchedule::robotConfigAt(int r, std::size_t t) const {
    std::vector<double> q = start_[static_cast<std::size_t>(r)];
    for (int id : by_robot_[static_cast<std::size_t>(r)]) {
        const auto &s = segments_[static_cast<std::size_t>(id)];
        if (t < s.start)
            break;
        if (t <= s.end())
            return s.path.config_at_timestep(t - s.start);
        q = s.path.back();
    }
    return q;
}

TimedSchedule::ObjectState TimedSchedule::objectAt(int o, std::size_t t) const {
    ObjectState s = object_initial_[static_cast<std::size_t>(o)];
    for (const auto &e : object_events_[static_cast<std::size_t>(o)]) {
        if (e.time > t)
            break;
        s.carried = e.carried;
        s.robot = e.robot;
        s.grasp = e.grasp;
        if (!e.carried) {
            s.element = e.element;
            s.position = e.position;
        }
    }
    if (s.carried) {
        const auto p = models_.formation(s.robot, s.grasp).followerConfig(robotConfigAt(s.robot, t));
        s.position = Eigen::Vector3d(p[0], p[1], p[2]);
        s.element = -1;
    }
    return s;
}

int TimedSchedule::heldGraspAt(int r, std::size_t t, int *object) const {
    for (int o = 0; o < numObjects(); ++o) {
        const auto s = objectAt(o, t);
        if (s.carried && s.robot == r) {
            if (object)
                *object = o;
            return s.grasp;
        }
    }
    return -1;
}

int TimedSchedule::robotElementAt(int r, std::size_t t) const {
    int object = -1;
    const int grasp = heldGraspAt(r, t, &object);
    if (grasp < 0)
        return hg_.robotElement(r);
    return hg_.findElement({ElementKind::RobotObject, r, object, grasp});
}

std::vector<std::size_t> TimedSchedule::epochBoundaries() const {
    std::set<std::size_t> times{0, makespan()};
    for (const auto &s : segments_)
        if (s.seg.kind == MotionSegment::Kind::Stage) {
            times.insert(s.start);
            times.insert(s.end());
        }
    return {times.begin(), times.end()};
}

comotion::Path TimedSchedule::robotSlice(int r, std::size_t t0, std::size_t t1) const {
    // One pass over the arm's segments (they are in time order).
    comotion::Path p;
    std::vector<double> hold = start_[static_cast<std::size_t>(r)];
    const auto &list = by_robot_[static_cast<std::size_t>(r)];
    std::size_t k = 0, t = t0;
    while (t <= t1) {
        while (k < list.size() && segments_[static_cast<std::size_t>(list[k])].end() < t) {
            hold = segments_[static_cast<std::size_t>(list[k])].path.back();
            ++k;
        }
        if (k < list.size()) {
            const auto &s = segments_[static_cast<std::size_t>(list[k])];
            if (t >= s.start) {
                for (; t <= t1 && t <= s.end(); ++t)
                    p.push_back(s.path.config_at_timestep(t - s.start));
                continue;
            }
            for (; t <= t1 && t < s.start; ++t)
                p.push_back(hold);
            continue;
        }
        for (; t <= t1; ++t)
            p.push_back(hold);
    }
    p.markDenseTimestepsImplicit();
    return p;
}

comotion::Path TimedSchedule::objectSlice(int o, std::size_t t0, std::size_t t1) const {
    comotion::Path p;
    for (std::size_t t = t0; t <= t1; ++t) {
        const auto s = objectAt(o, t);
        p.push_back({s.position.x(), s.position.y(), s.position.z()});
    }
    p.markDenseTimestepsImplicit();
    return p;
}

void TimedSchedule::spliceSegment(int id, std::size_t a, std::size_t b, const comotion::Path &repaired) {
    auto &s = segments_[static_cast<std::size_t>(id)];
    comotion::Path dense;
    for (std::size_t t = 0; t < a; ++t)
        dense.push_back(s.path.config_at_timestep(t));
    for (const auto &q : repaired)
        dense.push_back(q);
    for (std::size_t t = b + 1; t <= s.duration(); ++t)
        dense.push_back(s.path.config_at_timestep(t));
    dense.markDenseTimestepsImplicit();
    s.path = dense;
}

} // namespace lazydash
