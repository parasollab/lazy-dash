// Task space hypergraph H_T and the task query over H_TE (layer 1).
#include "lazydash/scene/Scene.h"
#include "lazydash/task/Capabilities.h"
#include "lazydash/task/DurationModel.h"
#include "lazydash/task/Heuristic.h"
#include "lazydash/task/TaskHypergraph.h"
#include "lazydash/task/TaskSearch.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <set>

using namespace lazydash;

namespace {

const auto kScenes = std::filesystem::path(LAZYDASH_RESOURCE_DIR) / "scenes";

/// Replays S_T: every tail is on the frontier when its step fires, every H_TE
/// vertex is consumed at most once, tails/heads match the arc's elements, and
/// the sink fires with every goal on the frontier.
void checkHyperpath(const TaskHypergraph &hg, const TaskSchedule &st) {
    std::set<int> frontier{0};
    std::vector<int> consumed(st.vertices.size(), 0);
    REQUIRE(st.steps.front().arc == hg.sourceArc());
    REQUIRE(st.steps.back().arc == hg.sinkArc());
    for (const auto &step : st.steps) {
        const auto &arc = hg.graph().arc(step.arc);
        REQUIRE(step.tail.size() == arc.tail.size());
        for (std::size_t i = 0; i < step.tail.size(); ++i) {
            const int te = step.tail[i];
            REQUIRE(frontier.count(te) == 1);
            REQUIRE(st.vertices[static_cast<std::size_t>(te)].element == arc.tail[i]);
            REQUIRE(++consumed[static_cast<std::size_t>(te)] == 1);
            frontier.erase(te);
        }
        for (std::size_t i = 0; i < step.head.size(); ++i) {
            REQUIRE(st.vertices[static_cast<std::size_t>(step.head[i])].element == arc.head[i]);
            frontier.insert(step.head[i]);
        }
    }
}

std::vector<int> middleArcs(const TaskSchedule &st) {
    std::vector<int> arcs;
    for (std::size_t s = 1; s + 1 < st.steps.size(); ++s)
        arcs.push_back(st.steps[s].arc);
    return arcs;
}

Transition makeTransition(TransitionType type, int object, int robot, double cost, int robot2 = -1) {
    Transition t;
    t.type = type;
    t.object = object;
    t.robot = robot;
    t.robot2 = robot2;
    t.cost = cost;
    return t;
}

/// Hand-built H_T: robots and objects with start/goal locations; arcs added by
/// tests. Every base, home and pose is at the origin, so the duration
/// estimates are stage times only.
struct Builder {
    TaskHypergraph hg;
    Scene scene;
    std::unique_ptr<DurationModel> durations_;
    int robots, objects;
    std::vector<int> start, goal;
    Builder(int r, int o) : robots(r), objects(o) {
        scene.arms.resize(static_cast<std::size_t>(r));
        for (int i = 0; i < r; ++i)
            hg.element({ElementKind::RobotOnly, i});
        for (int i = 0; i < o; ++i) {
            const int s = hg.addLocation({i, Location::Kind::Start, {Eigen::Isometry3d::Identity()}});
            const int g = hg.addLocation({i, Location::Kind::Goal, {Eigen::Isometry3d::Identity()}});
            start.push_back(hg.element({ElementKind::ObjectAt, -1, i, -1, s}));
            goal.push_back(hg.element({ElementKind::ObjectAt, -1, i, -1, g}));
        }
    }
    int held(int r, int o) { return hg.element({ElementKind::RobotObject, r, o, 0}); }
    int pick(int r, int o, double c = 1) {
        return hg.addTransition({hg.robotElement(r), start[static_cast<std::size_t>(o)]}, {held(r, o)},
                                makeTransition(TransitionType::Pick, o, r, c));
    }
    int place(int r, int o, double c = 1) {
        return hg.addTransition({held(r, o)}, {hg.robotElement(r), goal[static_cast<std::size_t>(o)]},
                                makeTransition(TransitionType::Place, o, r, c));
    }
    int handover(int r1, int r2, int o, double c = 1) {
        return hg.addTransition({held(r1, o), hg.robotElement(r2)}, {held(r2, o), hg.robotElement(r1)},
                                makeTransition(TransitionType::Handover, o, r1, c, r2));
    }
    void done() {
        hg.finalize(robots, objects);
        durations_ = std::make_unique<DurationModel>(
            scene, hg, std::vector<Eigen::Vector3d>(static_cast<std::size_t>(robots), Eigen::Vector3d::Zero()));
    }
    const DurationModel &durations() const { return *durations_; }
};

} // namespace

TEST_CASE("A handover chain is found in hyperpath order") {
    Builder b(2, 1);
    const int pick = b.pick(0, 0), give = b.handover(0, 1, 0), place = b.place(1, 0);
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());
    const auto st = search.query({});
    REQUIRE(st.has_value());
    checkHyperpath(b.hg, *st);
    CHECK(middleArcs(*st) == std::vector<int>{pick, give, place});
}

TEST_CASE("One robot serves two objects in turn: no element is consumed twice") {
    Builder b(1, 2);
    b.pick(0, 0);
    b.place(0, 0);
    b.pick(0, 1);
    b.place(0, 1);
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());
    const auto st = search.query({});
    REQUIRE(st.has_value());
    checkHyperpath(b.hg, *st);
    CHECK(st->steps.size() == 6); // source, 2 x (pick, place), sink
}

TEST_CASE("Frontier and history constraints are scoped and can make the query fail") {
    Builder b(1, 2);
    const int pick0 = b.pick(0, 0), place0 = b.place(0, 0), pick1 = b.pick(0, 1), place1 = b.place(0, 1);
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());

    // C_f(o1@start, r0+o0): o0 may not be picked while o1 still rests at its start.
    TaskConstraints cf;
    cf.add({TaskConstraint::Kind::Frontier, b.start[1], b.held(0, 0), "test"});
    auto st = search.query(cf);
    REQUIRE(st.has_value());
    checkHyperpath(b.hg, *st);
    CHECK(middleArcs(*st) == std::vector<int>{pick1, place1, pick0, place0});

    // C_h(r0+o1, o0@goal): o0 may not be placed with r0+o1 in its history, so o0 goes first.
    TaskConstraints ch;
    ch.add({TaskConstraint::Kind::History, b.held(0, 1), b.goal[0], "test"});
    st = search.query(ch);
    REQUIRE(st.has_value());
    CHECK(middleArcs(*st) == std::vector<int>{pick0, place0, pick1, place1});

    // Both together contradict: H_T is exhausted under C_T (not a budget failure).
    TaskConstraints both = cf;
    both.add(ch.at(0));
    CHECK_FALSE(search.query(both).has_value());
    CHECK_FALSE(search.budgetExhausted());
    // Duplicates are not added twice.
    CHECK_FALSE(both.add(ch.at(0)));
}

TEST_CASE("Backtracking leaves a dead end the greedy order walks into") {
    // One robot, two objects: the greedy order picks o0 (longer chain) first,
    // but C_h forbids placing o1 once r0 has held o0 (r0+o0 enters o1's
    // history). Only backtracking finds that o1 must go first.
    Builder b(1, 2);
    const int pick0 = b.pick(0, 0, 2), place0 = b.place(0, 0);
    const int pick1 = b.pick(0, 1), place1 = b.place(0, 1);
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());
    TaskConstraints ch;
    ch.add({TaskConstraint::Kind::History, b.held(0, 0), b.goal[1], "test"});
    const auto st = search.query(ch);
    REQUIRE(st.has_value());
    checkHyperpath(b.hg, *st);
    CHECK(middleArcs(*st) == std::vector<int>{pick1, place1, pick0, place0});
    CHECK(search.stats().backtracks > 0);
}

TEST_CASE("The greedy order moves a blocking element first") {
    // C_f(o1@start, o0@goal): o0 may not be placed while o1 rests at its
    // start. The order prefers arcs that consume o1@start, so no dead end.
    Builder b(1, 2);
    const int pick0 = b.pick(0, 0, 0.5), place0 = b.place(0, 0);
    const int pick1 = b.pick(0, 1), place1 = b.place(0, 1);
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());
    TaskConstraints cf;
    cf.add({TaskConstraint::Kind::Frontier, b.start[1], b.goal[0], "test"});
    const auto st = search.query(cf);
    REQUIRE(st.has_value());
    CHECK(middleArcs(*st) == std::vector<int>{pick1, place1, pick0, place0});
    CHECK(search.stats().backtracks == 0);
}

TEST_CASE("C_f orders the hyperpath in time: TaskStep::after") {
    // Two threads: r0 serves o0, r1 serves o1. C_f(r1, r0+o0): r0 may not pick
    // o0 while r1 is idle. So the pick of o0 waits for r1's pick, and r1's
    // place (r1 idle again) waits for the pick of o0.
    Builder b(2, 2);
    const int pick0 = b.pick(0, 0), place0 = b.place(0, 0), pick1 = b.pick(1, 1), place1 = b.place(1, 1);
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());
    TaskConstraints cf;
    cf.add({TaskConstraint::Kind::Frontier, b.hg.robotElement(1), b.held(0, 0), "test"});
    const auto st = search.query(cf);
    REQUIRE(st.has_value());
    checkHyperpath(b.hg, *st);
    const auto stepOf = [&](int arc) {
        for (std::size_t s = 0; s < st->steps.size(); ++s)
            if (st->steps[s].arc == arc)
                return static_cast<int>(s);
        return -1;
    };
    const int s_pick0 = stepOf(pick0), s_pick1 = stepOf(pick1), s_place1 = stepOf(place1);
    REQUIRE(s_pick0 >= 0);
    REQUIRE(s_pick1 >= 0);
    REQUIRE(s_place1 >= 0);
    CHECK(s_pick1 < s_pick0);
    const auto has = [](const std::vector<int> &v, int x) { return std::find(v.begin(), v.end(), x) != v.end(); };
    CHECK(has(st->steps[static_cast<std::size_t>(s_pick0)].after, s_pick1));
    if (s_place1 > s_pick0)
        CHECK(has(st->steps[static_cast<std::size_t>(s_place1)].after, s_pick0));
    // No constraint, no ordering.
    const auto free = search.query({});
    REQUIRE(free.has_value());
    for (const auto &step : free->steps)
        CHECK(step.after.empty());
    (void)place0;
}

TEST_CASE("A move-out pose: C_f scoped by via, and an object may leave its move-out") {
    // o1 may only be placed while o0 is off the shelf (neither at its start
    // nor at its goal), and o0 may not be picked from the shelf once o1 is
    // there. Only a move-out of o0 satisfies both.
    Builder b(1, 2);
    const int mo = b.hg.addLocation({0, Location::Kind::MoveOut, {Eigen::Isometry3d::Identity()}});
    const int out = b.hg.element({ElementKind::ObjectAt, -1, 0, -1, mo});
    const int pick0 = b.pick(0, 0), place0 = b.place(0, 0);
    const int set_down = b.hg.addTransition({b.held(0, 0)}, {b.hg.robotElement(0), out},
                                            makeTransition(TransitionType::Place, 0, 0, 1));
    const int take_up = b.hg.addTransition({b.hg.robotElement(0), out}, {b.held(0, 0)},
                                           makeTransition(TransitionType::Pick, 0, 0, 1));
    const int pick1 = b.pick(0, 1), place1 = b.place(0, 1);
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());

    TaskConstraints ct;
    ct.add({TaskConstraint::Kind::Frontier, b.start[0], b.goal[1], "o0 on the shelf blocks o1"});
    ct.add({TaskConstraint::Kind::Frontier, b.goal[0], b.goal[1], "o0 at its goal blocks o1"});
    TaskConstraint from_shelf{TaskConstraint::Kind::Frontier, b.goal[1], b.held(0, 0), "o1 blocks picking o0"};
    from_shelf.via = b.start[0];
    ct.add(from_shelf);
    const auto st = search.query(ct);
    REQUIRE(st.has_value());
    checkHyperpath(b.hg, *st);
    CHECK(middleArcs(*st) == std::vector<int>{pick0, set_down, pick1, place1, take_up, place0});

    // Unscoped, the last C_f also forbids taking o0 up from its move-out.
    TaskConstraints unscoped;
    for (auto c : ct.all()) {
        c.via = -1;
        unscoped.add(c);
    }
    CHECK_FALSE(search.query(unscoped).has_value());
}

TEST_CASE("A goal that must wait for another goal is not reached first (dead-end prune)") {
    // o1 at its goal blocks placing o0 (o0 is the rear block): o0 must go
    // first. o1 is cheaper, so the greedy order would place it first and
    // only backtracking would recover; the prune never fires that place.
    Builder b(1, 2);
    const int pick0 = b.pick(0, 0, 2), place0 = b.place(0, 0, 2);
    const int pick1 = b.pick(0, 1, 0.5), place1 = b.place(0, 1, 0.5);
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());
    TaskConstraints ct;
    ct.add({TaskConstraint::Kind::Frontier, b.goal[1], b.goal[0], "o1 at its goal blocks placing o0"});
    const auto st = search.query(ct);
    REQUIRE(st.has_value());
    checkHyperpath(b.hg, *st);
    CHECK(middleArcs(*st) == std::vector<int>{pick0, place0, pick1, place1});
    CHECK(search.stats().backtracks == 0);
}

TEST_CASE("expandMoveOut adds a location and its transitions, keeping every id") {
    const auto scene = Scene::load((kScenes / "paper" / "stocking-2layers-4objects.json").string());
    const Capabilities caps(scene, scene.makeArms());
    auto hg = TaskHypergraph::build(caps);
    const int V = hg.graph().numVertices(), E = hg.graph().numArcs();
    std::vector<std::string> before;
    for (int a = 0; a < E; ++a)
        before.push_back(hg.describeArc(a));
    // A table pose next to o0's arm (scene region passive_0).
    const Eigen::Isometry3d pose = poseFromRpy(Eigen::Vector3d(-0.45, 0.35, 0.75), Eigen::Vector3d::Zero());
    const int out = hg.expandMoveOut(caps, 0, pose);
    CHECK(out == V);
    CHECK(hg.moveOutElements(0) == std::vector<int>{out});
    CHECK(hg.locations()[static_cast<std::size_t>(hg.graph().vertex(out).location)].kind == Location::Kind::MoveOut);
    for (int a = 0; a < E; ++a)
        CHECK(hg.describeArc(a) == before[static_cast<std::size_t>(a)]);
    int into = 0, from = 0;
    for (int a = E; a < hg.graph().numArcs(); ++a) {
        into += hg.objectHead(a) == out;
        from += hg.objectTail(a) == out;
    }
    CHECK(into > 0);
    CHECK(from > 0);
    const Heuristic h(hg);
    CHECK(h(out) < Heuristic::kInfinity);
}

TEST_CASE("H_T of the Sorting: 4 robots scenes: size and scaling") {
    // The paper's runs: 50/83, 90/158, 170/311, 410/770 for 4/8/16/40 objects.
    const std::vector<std::pair<int, std::pair<int, int>>> paper = {
        {4, {50, 83}}, {8, {90, 158}}, {16, {170, 311}}, {40, {410, 770}}};
    std::vector<int> ours;
    for (const auto &[n, sizes] : paper) {
        const auto scene = Scene::load(
            (kScenes / "paper" / ("sorting-4robots-" + std::to_string(n) + "objects.json")).string());
        const Capabilities caps(scene, scene.makeArms());
        const auto t0 = std::chrono::steady_clock::now();
        const auto hg = TaskHypergraph::build(caps);
        const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const int V = hg.graph().numVertices(), E = hg.graph().numArcs();
        // Count rule with one robot-only element per robot: V <= 2 + R + 2O + |G|RO.
        const int R = 4, G = 2;
        CHECK(V <= 2 + R + 2 * n + G * R * n);
        WARN(n << " objects: H_T " << V << "/" << E << " (paper " << sizes.first << "/" << sizes.second
             << "), built in " << s << " s");
        ours.push_back(V);

        const Heuristic h(hg);
        const DurationModel durations(caps, hg);
        TaskSearch search(hg, h, durations);
        const auto st = search.query({});
        REQUIRE(st.has_value());
        checkHyperpath(hg, *st);
        WARN("  S_T: " << st->steps.size() << " steps, " << search.stats().expansions
             << " expansions, " << search.stats().backtracks << " backtracks, "
             << search.stats().seconds << " s");
    }
    // Growth with objects close to the paper's (410 / 50 = 8.2 from 4 to 40 objects).
    const double growth = double(ours.back()) / ours.front();
    CHECK(growth > 0.7 * 8.2);
    CHECK(growth < 1.3 * 8.2);
}

TEST_CASE("Every generated scene has a task plan") {
    for (const auto &e : std::filesystem::directory_iterator(kScenes / "generated")) {
        if (e.path().extension() != ".json")
            continue;
        INFO(e.path().filename().string());
        const auto scene = Scene::load(e.path().string());
        const Capabilities caps(scene, scene.makeArms());
        const auto hg = TaskHypergraph::build(caps);
        const Heuristic h(hg);
        for (int o = 0; o < hg.numObjects(); ++o)
            REQUIRE(h(hg.startElement(o)) < Heuristic::kInfinity);
        const DurationModel durations(caps, hg);
        TaskSearch search(hg, h, durations);
        const auto st = search.query({});
        REQUIRE(st.has_value());
        checkHyperpath(hg, *st);
        WARN(e.path().filename().string() << ": H_T " << hg.graph().numVertices() << "/"
             << hg.graph().numArcs() << ", S_T " << st->steps.size() << " steps in "
             << search.stats().seconds << " s (" << search.stats().backtracks << " backtracks)");
    }
}

TEST_CASE("The task query: a hyperpath in estimated time") {
    const auto scene = Scene::load((kScenes / "paper" / "sorting-4robots-16objects.json").string());
    const Capabilities caps(scene, scene.makeArms());
    const auto hg = TaskHypergraph::build(caps);
    const Heuristic h(hg);
    const DurationModel durations(caps, hg);
    TaskSearch search(hg, h, durations);
    const auto st = search.query({});
    REQUIRE(st.has_value());
    checkHyperpath(hg, *st);
    for (std::size_t s = 1; s + 1 < st->steps.size(); ++s) {
        const auto &step = st->steps[s];
        for (int te : step.tail)
            CHECK(step.finish >= st->vertices[static_cast<std::size_t>(te)].ready + durations.stageTime() - 1e-9);
        for (int te : step.head)
            CHECK(st->vertices[static_cast<std::size_t>(te)].ready == step.finish);
    }
}

TEST_CASE("A block can be handed there and back to free a hand") {
    // Only r0 picks and places; r1 only takes handovers. Each goal is blocked
    // while the other block rests at its start, so r0 parks o0 with r1, serves
    // o1, and takes o0 back: o0 returns to r0+o0, already in its history.
    Builder b(2, 2);
    for (int o : {0, 1}) {
        b.pick(0, o);
        b.place(0, o);
        b.handover(0, 1, o);
        b.handover(1, 0, o);
    }
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());
    TaskConstraints cf;
    cf.add({TaskConstraint::Kind::Frontier, b.start[1], b.goal[0], "test"});
    cf.add({TaskConstraint::Kind::Frontier, b.start[0], b.goal[1], "test"});
    const auto st = search.query(cf);
    REQUIRE(st.has_value());
    checkHyperpath(b.hg, *st);
    std::multiset<int> held;
    for (const auto &v : st->vertices)
        if (b.hg.graph().vertex(v.element).kind == ElementKind::RobotObject)
            held.insert(v.element);
    CHECK((held.count(b.held(0, 0)) == 2 || held.count(b.held(0, 1)) == 2));
}

TEST_CASE("A set-aside transition blocks only its own goal pose") {
    // Two places of o0 into the same goal element (two poses) share tail and
    // head. Setting one aside (no anchors) leaves the other.
    Builder b(1, 1);
    const int pick = b.pick(0, 0), place_a = b.place(0, 0), place_b = b.place(0, 0, 2);
    b.done();
    const Heuristic h(b.hg);
    TaskSearch search(b.hg, h, b.durations());
    TaskConstraints ct;
    TaskConstraint aside{TaskConstraint::Kind::Frontier, b.held(0, 0), b.goal[0], "test", true, 2, place_a};
    REQUIRE(ct.add(aside));
    const auto st = search.query(ct);
    REQUIRE(st.has_value());
    CHECK(middleArcs(*st) == std::vector<int>{pick, place_b});
    aside.arc = place_b;
    CHECK(ct.add(aside)); // a different arc: a different constraint
    CHECK_FALSE(search.query(ct).has_value());
}
