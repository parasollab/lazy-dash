#pragma once

#include "lazydash/hypergraph/Hypergraph.h"
#include "lazydash/task/Capabilities.h"

#include <Eigen/Geometry>
#include <deque>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace lazydash {

/// Kinds of task space element.
enum class ElementKind { Source, Sink, RobotOnly, RobotObject, ObjectAt };

/// A task space element T_i = (B_i, W_i, C_i): RobotOnly(r) hand-free,
/// RobotObject(r, o, g) r holds o with grasp g, ObjectAt(o, l) o rests at l.
struct TaskElement {
    ElementKind kind = ElementKind::Source;
    int robot = -1;
    int object = -1;
    int grasp = -1;
    int location = -1;

    auto key() const { return std::make_tuple(kind, robot, object, grasp, location); }
};

/// A place where an object can rest: a start (one pose), a goal (acceptable
/// alternative poses), or a move-out (one sampled pose).
struct Location {
    enum class Kind { Start, Goal, MoveOut };
    int object = -1;
    Kind kind = Kind::Start;
    std::vector<Eigen::Isometry3d> poses;
};

enum class TransitionType { Source, Sink, Pick, Place, Handover };

/// A transition hyperarc E_T (tail = preconditions, head = postconditions).
/// Handover: giver (`robot`, `grasp`) to receiver (`robot2`, `grasp2`).
/// `cost` is the transition cost estimate.
struct Transition {
    TransitionType type = TransitionType::Source;
    int object = -1, location = -1, pose = -1;
    int robot = -1, grasp = -1;
    int robot2 = -1, grasp2 = -1;
    double cost = 0.0;
};

/// The task space hypergraph H_T = (V_T, E_T): task elements and the
/// transitions between them, with no motion detail and no ordering.
class TaskHypergraph {
public:
    using Graph = Hypergraph<TaskElement, Transition>;

    /// Closure from the start and goal elements under pick, place and
    /// handover, admitting transitions by capability only.
    static TaskHypergraph build(const Capabilities &capabilities);

    /// Task-space expansion: adds a move-out location for `object` at `pose`
    /// and the closure it makes reachable. Existing ids stay valid; returns
    /// the new ObjectAt element.
    int expandMoveOut(const Capabilities &capabilities, int object, const Eigen::Isometry3d &pose);
    /// The move-out elements of an object, in order of expansion.
    const std::vector<int> &moveOutElements(int object) const;

    // Construction API (also used to build small graphs by hand).
    TaskHypergraph();
    int addLocation(Location location);
    /// The element's id, adding it if new.
    int element(const TaskElement &e);
    int findElement(const TaskElement &e) const; // -1 if absent
    int addTransition(std::vector<int> tail, std::vector<int> head, Transition t);
    /// Add the source arc (to robot-only and start elements) and the sink arc
    /// (from goal elements). Call once, last.
    void finalize(int num_robots, int num_objects);

    const Graph &graph() const { return graph_; }
    const std::vector<Location> &locations() const { return locations_; }
    int source() const { return source_; }
    int sourceArc() const { return source_arc_; }
    int sinkArc() const { return sink_arc_; }
    int robotElement(int robot) const;
    int startElement(int object) const;
    int goalElement(int object) const;
    int numObjects() const { return static_cast<int>(start_location_.size()); }
    int numRobots() const { return static_cast<int>(robot_element_.size()); }

    /// The object-carrying element in an arc's tail / head (-1 if none):
    /// ObjectAt or RobotObject of the arc's object.
    int objectTail(int arc) const;
    int objectHead(int arc) const;

    std::string describe(int element) const;
    std::string describeArc(int arc) const;

private:
    Graph graph_;
    std::vector<Location> locations_;
    std::map<std::tuple<ElementKind, int, int, int, int>, int> index_;
    std::vector<int> robot_element_, start_location_, goal_location_;
    std::vector<std::vector<int>> move_out_elements_; // per object
    std::vector<char> expanded_;                      // closure: element expanded
    int source_ = -1, sink_ = -1, source_arc_ = -1, sink_arc_ = -1;

    /// Expand every element in `open` (and what it reaches) once.
    void close(const Capabilities &caps, std::deque<int> open);
    void addPlace(const Capabilities &caps, int held, int location);
};

} // namespace lazydash
