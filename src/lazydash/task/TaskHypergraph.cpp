#include "lazydash/task/TaskHypergraph.h"

#include <deque>
#include <sstream>
#include <stdexcept>

namespace lazydash {

namespace {

constexpr double kTerminalArcCost = 0.01;

bool carriesObject(const TaskElement &e, int object) {
    return e.object == object &&
           (e.kind == ElementKind::ObjectAt || e.kind == ElementKind::RobotObject);
}

} // namespace

TaskHypergraph::TaskHypergraph() {
    source_ = element({ElementKind::Source});
    sink_ = element({ElementKind::Sink});
}

int TaskHypergraph::addLocation(Location location) {
    const int id = static_cast<int>(locations_.size());
    const int object = location.object;
    if (object >= static_cast<int>(start_location_.size())) {
        start_location_.resize(static_cast<std::size_t>(object) + 1, -1);
        goal_location_.resize(static_cast<std::size_t>(object) + 1, -1);
        move_out_elements_.resize(static_cast<std::size_t>(object) + 1);
    }
    if (location.kind != Location::Kind::MoveOut)
        (location.kind == Location::Kind::Start ? start_location_ : goal_location_)
            [static_cast<std::size_t>(object)] = id;
    locations_.push_back(std::move(location));
    return id;
}

int TaskHypergraph::findElement(const TaskElement &e) const {
    const auto it = index_.find(e.key());
    return it == index_.end() ? -1 : it->second;
}

int TaskHypergraph::element(const TaskElement &e) {
    if (const int found = findElement(e); found >= 0)
        return found;
    const int id = graph_.addVertex(e);
    index_.emplace(e.key(), id);
    if (e.kind == ElementKind::RobotOnly) {
        if (e.robot >= static_cast<int>(robot_element_.size()))
            robot_element_.resize(static_cast<std::size_t>(e.robot) + 1, -1);
        robot_element_[static_cast<std::size_t>(e.robot)] = id;
    }
    return id;
}

int TaskHypergraph::addTransition(std::vector<int> tail, std::vector<int> head, Transition t) {
    return graph_.addArc(std::move(tail), std::move(head), t);
}

int TaskHypergraph::robotElement(int robot) const {
    return robot_element_.at(static_cast<std::size_t>(robot));
}

int TaskHypergraph::startElement(int object) const {
    return findElement({ElementKind::ObjectAt, -1, object, -1,
                        start_location_.at(static_cast<std::size_t>(object))});
}

int TaskHypergraph::goalElement(int object) const {
    return findElement({ElementKind::ObjectAt, -1, object, -1,
                        goal_location_.at(static_cast<std::size_t>(object))});
}

void TaskHypergraph::finalize(int num_robots, int num_objects) {
    std::vector<int> initial, goals;
    for (int r = 0; r < num_robots; ++r)
        initial.push_back(element({ElementKind::RobotOnly, r}));
    for (int o = 0; o < num_objects; ++o) {
        initial.push_back(startElement(o));
        goals.push_back(goalElement(o));
    }
    Transition src;
    src.type = TransitionType::Source;
    src.cost = kTerminalArcCost;
    source_arc_ = addTransition({source_}, initial, src);
    Transition snk;
    snk.type = TransitionType::Sink;
    snk.cost = kTerminalArcCost;
    sink_arc_ = addTransition(goals, {sink_}, snk);
}

int TaskHypergraph::objectTail(int arc) const {
    const auto &a = graph_.arc(arc);
    for (int v : a.tail)
        if (carriesObject(graph_.vertex(v), a.data.object))
            return v;
    return -1;
}

int TaskHypergraph::objectHead(int arc) const {
    const auto &a = graph_.arc(arc);
    for (int v : a.head)
        if (carriesObject(graph_.vertex(v), a.data.object))
            return v;
    return -1;
}

namespace {

double baseToPoint(const Scene &scene, int r, const Eigen::Vector3d &p) {
    return (p - scene.arms[static_cast<std::size_t>(r)].base.position).norm();
}

} // namespace

void TaskHypergraph::addPlace(const Capabilities &caps, int held, int location) {
    // place(r, o, g, l): {RobotObject(r, o, g)} -> {RobotOnly r, ObjectAt(o, l)}
    const TaskElement e = graph_.vertex(held);
    const int at = findElement({ElementKind::ObjectAt, -1, e.object, -1, location});
    const auto &poses = locations_[static_cast<std::size_t>(location)].poses;
    for (std::size_t p = 0; p < poses.size(); ++p) {
        if (!caps.canHold(e.robot, e.grasp, poses[p]))
            continue;
        Transition t;
        t.type = TransitionType::Place;
        t.object = e.object;
        t.location = location;
        t.pose = static_cast<int>(p);
        t.robot = e.robot;
        t.grasp = e.grasp;
        t.cost = baseToPoint(caps.scene(), e.robot, poses[p].translation());
        addTransition({held}, {robotElement(e.robot), at}, t);
    }
}

void TaskHypergraph::close(const Capabilities &caps, std::deque<int> open) {
    const int R = caps.numRobots(), G = caps.numGrasps();
    // Closure: every element reachable from `open` by admitted transitions.
    // Each element is expanded once.
    while (!open.empty()) {
        const int v = open.front();
        open.pop_front();
        if (static_cast<int>(expanded_.size()) <= v)
            expanded_.resize(static_cast<std::size_t>(v) + 1, 0);
        if (expanded_[static_cast<std::size_t>(v)])
            continue;
        expanded_[static_cast<std::size_t>(v)] = 1;
        const TaskElement e = graph_.vertex(v);

        if (e.kind == ElementKind::ObjectAt &&
            locations_[static_cast<std::size_t>(e.location)].kind != Location::Kind::Goal) {
            // pick(r, o, g, l): {RobotOnly r, ObjectAt(o, l)} -> {RobotObject(r, o, g)},
            // l = start or move-out
            const auto &pose = locations_[static_cast<std::size_t>(e.location)].poses[0];
            for (int r = 0; r < R; ++r)
                for (int g = 0; g < G; ++g) {
                    if (!caps.canHold(r, g, pose))
                        continue;
                    const int held = element({ElementKind::RobotObject, r, e.object, g});
                    Transition t;
                    t.type = TransitionType::Pick;
                    t.object = e.object;
                    t.location = e.location;
                    t.pose = 0;
                    t.robot = r;
                    t.grasp = g;
                    t.cost = baseToPoint(caps.scene(), r, pose.translation());
                    addTransition({robotElement(r), v}, {held}, t);
                    open.push_back(held);
                }
        } else if (e.kind == ElementKind::RobotObject) {
            addPlace(caps, v, graph_.vertex(goalElement(e.object)).location);
            for (int m : move_out_elements_[static_cast<std::size_t>(e.object)])
                addPlace(caps, v, graph_.vertex(m).location);
            // handover(r1, r2, o, g1, g2):
            //   {RobotObject(r1, o, g1), RobotOnly r2} -> {RobotObject(r2, o, g2), RobotOnly r1}
            for (int r2 = 0; r2 < R; ++r2) {
                if (!caps.canHandOver(e.robot, r2))
                    continue;
                for (int g2 = 0; g2 < G; ++g2) {
                    if (!caps.compatibleGrasps(e.grasp, g2))
                        continue;
                    const int received = element({ElementKind::RobotObject, r2, e.object, g2});
                    Transition t;
                    t.type = TransitionType::Handover;
                    t.object = e.object;
                    t.robot = e.robot;
                    t.grasp = e.grasp;
                    t.robot2 = r2;
                    t.grasp2 = g2;
                    t.cost = caps.baseDistance(e.robot, r2);
                    addTransition({v, robotElement(r2)}, {received, robotElement(e.robot)}, t);
                    open.push_back(received);
                }
            }
        }
    }
}

TaskHypergraph TaskHypergraph::build(const Capabilities &caps) {
    const Scene &scene = caps.scene();
    TaskHypergraph hg;
    const int R = caps.numRobots();
    const int O = static_cast<int>(scene.objects.size());
    for (int r = 0; r < R; ++r)
        hg.element({ElementKind::RobotOnly, r});

    std::deque<int> open;
    for (int o = 0; o < O; ++o) {
        const auto &spec = scene.objects[static_cast<std::size_t>(o)];
        const int start = hg.addLocation({o, Location::Kind::Start, {spec.start}});
        const int goal = hg.addLocation({o, Location::Kind::Goal, spec.goals});
        open.push_back(hg.element({ElementKind::ObjectAt, -1, o, -1, start}));
        hg.element({ElementKind::ObjectAt, -1, o, -1, goal});
    }
    hg.close(caps, std::move(open));
    hg.finalize(R, O);
    return hg;
}

int TaskHypergraph::expandMoveOut(const Capabilities &caps, int object, const Eigen::Isometry3d &pose) {
    const int location = addLocation({object, Location::Kind::MoveOut, {pose}});
    const int at = element({ElementKind::ObjectAt, -1, object, -1, location});
    // Elements expanded before this location existed get their place here;
    // elements reached from now on get it in the closure.
    for (int v = 0; v < static_cast<int>(expanded_.size()); ++v) {
        const auto &e = graph_.vertex(v);
        if (expanded_[static_cast<std::size_t>(v)] && e.kind == ElementKind::RobotObject && e.object == object)
            addPlace(caps, v, location);
    }
    move_out_elements_[static_cast<std::size_t>(object)].push_back(at);
    close(caps, {at});
    return at;
}

const std::vector<int> &TaskHypergraph::moveOutElements(int object) const {
    return move_out_elements_.at(static_cast<std::size_t>(object));
}

std::string TaskHypergraph::describe(int element) const {
    const auto &e = graph_.vertex(element);
    std::ostringstream s;
    switch (e.kind) {
    case ElementKind::Source: s << "source"; break;
    case ElementKind::Sink: s << "sink"; break;
    case ElementKind::RobotOnly: s << "r" << e.robot; break;
    case ElementKind::RobotObject: s << "r" << e.robot << "+o" << e.object << "/g" << e.grasp; break;
    case ElementKind::ObjectAt:
        s << "o" << e.object << "@";
        switch (locations_[static_cast<std::size_t>(e.location)].kind) {
        case Location::Kind::Start: s << "start"; break;
        case Location::Kind::Goal: s << "goal"; break;
        case Location::Kind::MoveOut: s << "moveout" << e.location; break;
        }
        break;
    }
    return s.str();
}

std::string TaskHypergraph::describeArc(int arc) const {
    const auto &a = graph_.arc(arc);
    std::ostringstream s;
    switch (a.data.type) {
    case TransitionType::Source: s << "source"; break;
    case TransitionType::Sink: s << "sink"; break;
    case TransitionType::Pick: s << "pick"; break;
    case TransitionType::Place: s << "place"; break;
    case TransitionType::Handover: s << "handover"; break;
    }
    s << " {";
    for (std::size_t i = 0; i < a.tail.size(); ++i)
        s << (i ? ", " : "") << describe(a.tail[i]);
    s << "} -> {";
    for (std::size_t i = 0; i < a.head.size(); ++i)
        s << (i ? ", " : "") << describe(a.head[i]);
    s << "}";
    return s.str();
}

} // namespace lazydash
