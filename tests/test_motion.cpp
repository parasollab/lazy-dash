// Layer 2 (anchors, task conflict detection) and layer 3 (lazy roadmaps,
// motion hypergraph, sequential motion query).
#include "lazydash/motion/LazyRoadmap.h"
#include "lazydash/motion/MotionLayer.h"
#include "lazydash/robot/BodyModels.h"
#include "lazydash/scene/Scene.h"
#include "lazydash/schedule/MeshVerifier.h"
#include "lazydash/task/Capabilities.h"
#include "lazydash/task/Heuristic.h"
#include "lazydash/task/TaskConflictDetection.h"
#include "lazydash/task/TaskHypergraph.h"
#include "lazydash/task/TaskSearch.h"
#include "lazydash/task/TransitionSampler.h"

#include "comotion/collision/CollisionChecker.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <set>

using namespace lazydash;

namespace {

const auto kPaper = std::filesystem::path(LAZYDASH_RESOURCE_DIR) / "scenes" / "paper";

/// Layers 1-3 wired together for one scene.
struct Pipeline {
    Scene scene;
    BodyModels models;
    Capabilities caps;
    TaskHypergraph hg;
    Heuristic h;
    DurationModel durations;
    MeshVerifier mesh;
    TransitionSampler sampler;
    TaskConflictDetection layer2;
    MotionLayer layer3;

    explicit Pipeline(Scene s, std::uint64_t seed = 1)
        : scene(std::move(s)), models(scene), caps(scene, scene.makeArms()),
          hg(TaskHypergraph::build(caps)), h(hg), durations(caps, hg), mesh(scene),
          sampler(hg, caps, models, mesh, seed), layer2(hg, sampler, models),
          layer3(hg, models, mesh, RoadmapParams{}, seed) {}

    TaskSchedule taskPlan(const TaskConstraints &ct = {}) {
        TaskSearch search(hg, h, durations);
        auto st = search.query(ct);
        REQUIRE(st.has_value());
        return *st;
    }
};

Scene load(const std::string &name) { return Scene::load((kPaper / name).string()); }

bool same(const std::vector<double> &a, const std::vector<double> &b) {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > 1e-9)
            return false;
    return true;
}

} // namespace

TEST_CASE("Anchors hold the object where the transition says, on valid configurations") {
    Pipeline p(load("sorting-4robots-4objects.json"));
    const auto st = p.taskPlan();
    int handovers = 0;
    for (const auto &step : st.steps) {
        const auto &t = p.hg.graph().arc(step.arc).data;
        if (t.type != TransitionType::Pick && t.type != TransitionType::Place &&
            t.type != TransitionType::Handover)
            continue;
        const auto a = p.sampler.sample(step.arc);
        REQUIRE(a.has_value());
        UrIk ik(p.models.arm(a->robot));
        const auto ee = p.models.graspSet(a->robot).eePose(static_cast<std::size_t>(a->grasp), a->object_pose);
        CHECK(ik.linkPose(a->q_grasp, kUr5eGripperLink).isApprox(ee, 1e-6));
        if (t.type == TransitionType::Handover) {
            ++handovers;
            UrIk ik2(p.models.arm(a->robot2));
            const auto ee2 =
                p.models.graspSet(a->robot2).eePose(static_cast<std::size_t>(a->grasp2), a->object_pose);
            CHECK(ik2.linkPose(a->q2_grasp, kUr5eGripperLink).isApprox(ee2, 1e-6));
            CHECK(p.mesh.pathValid(*p.models.holdingMesh(a->robot, a->grasp), {a->q_grasp}));
            CHECK(p.mesh.pathValid(*p.models.armMesh(a->robot2), {a->q2_approach, a->q2_grasp}));
        } else {
            CHECK(p.mesh.pathValid(*p.models.armMesh(a->robot), {a->q_approach, a->q_grasp}));
        }
    }
    CHECK(handovers > 0);
}

TEST_CASE("Task conflict detection collects every conflict and maps it to C_f") {
    auto scene = load("sorting-4robots-4objects.json");
    // Two floating blockers, 8 cm above blocks 0 and 1, where the grippers come down.
    for (int b : {0, 1}) {
        ObjectSpec blocker = scene.objects[static_cast<std::size_t>(b)];
        blocker.name = "blocker_" + std::to_string(b);
        blocker.start.translation().z() += 0.08;
        blocker.goals = scene.objects[2].goals;
        for (auto &g : blocker.goals)
            g.translation().y() += 0.1 * (b + 1);
        scene.objects.push_back(blocker);
    }
    Pipeline p(std::move(scene));
    // Force the order that conflicts: blocks 0 and 1 are picked while their
    // blockers still rest above them (no blocker may be picked before).
    TaskConstraints order;
    for (int v = 0; v < p.hg.graph().numVertices(); ++v) {
        const auto &e = p.hg.graph().vertex(v);
        if (e.kind == ElementKind::RobotObject && e.object >= 4)
            order.add({TaskConstraint::Kind::Frontier, p.hg.startElement(e.object - 4), v, "test order"});
    }
    const auto st = p.taskPlan(order);
    const auto result = p.layer2.run(st);
    std::set<int> blocked_objects;
    for (const auto &c : result.constraints) {
        REQUIRE(c.kind == TaskConstraint::Kind::Frontier);
        const auto &pre = p.hg.graph().vertex(c.pre);
        if (pre.kind == ElementKind::ObjectAt && pre.object >= 4)
            blocked_objects.insert(pre.object);
    }
    WARN(result.constraints.size() << " constraints, " << result.conflicts << " conflicts, "
         << result.resamples << " resamples:\n" << [&] {
             std::string s;
             for (const auto &l : result.log)
                 s += "  " + l + "\n";
             return s;
         }());
    // Both blockers are reported in one pass, not only the first.
    CHECK(blocked_objects == std::set<int>{4, 5});

    // A pick/place conflict is permanent only if every IK branch of the
    // transition has it; else it is tentative.
    const auto blockedOn = [&](const TransitionAnchors &a, int object, const Eigen::Vector3d &at) {
        const bool pick = a.type == TransitionType::Pick;
        return !p.sampler.clearOfObject(a.robot, pick ? -1 : a.grasp, a.q_approach, object, at) ||
               !p.sampler.clearOfObject(a.robot, pick ? -1 : a.grasp, a.q_grasp, object, at) ||
               !p.sampler.clearOfObject(a.robot, pick ? a.grasp : -1, a.q_approach, object, at);
    };
    int geometric = 0;
    for (const auto &c : result.constraints) {
        const auto &pre = p.hg.graph().vertex(c.pre);
        const Eigen::Vector3d at =
            p.hg.locations()[static_cast<std::size_t>(pre.location)].poses[0].translation();
        for (const auto &step : st.steps) {
            const auto &t = p.hg.graph().arc(step.arc).data;
            if ((t.type != TransitionType::Pick && t.type != TransitionType::Place) ||
                p.hg.objectTail(step.arc) != c.via || p.hg.objectHead(step.arc) != c.post)
                continue;
            const auto branches = p.sampler.pickPlaceBranches(step.arc);
            REQUIRE(!branches.empty());
            bool every = true;
            for (const auto &a : branches)
                every = every && (blockedOn(a, pre.object, at) ||
                                  (t.type == TransitionType::Place &&
                                   (a.object_pose.translation() - at).norm() <
                                       2 * p.scene.objects[0].collision_radius));
            CHECK(c.tentative == !every);
            geometric += every;
        }
    }
    WARN(geometric << " of " << result.constraints.size() << " C_f geometric");

    // Under the constraints the task plan moves the blockers first.
    TaskConstraints ct;
    for (const auto &c : result.constraints)
        ct.add(c);
    const auto st2 = p.taskPlan(ct);
    CHECK(p.layer2.run(st2).constraints.size() < result.constraints.size());
}

TEST_CASE("Lazy roadmap: edges are validated only when used, C_M regions bind per query") {
    auto scene = load("sorting-4robots-4objects.json");
    BodyModels models(scene);
    MeshVerifier mesh(scene);
    comotion::CollisionChecker spheres(comotion::CollisionChecker::Backend::Spheres);
    scene.applyObstacles(spheres);
    LazyRoadmap rm(models.arm(0), models.armMesh(0), spheres, mesh, scene.arms[0].lower,
                   scene.arms[0].upper, 5, RoadmapParams{50, 10});
    const auto from = scene.arms[0].start;
    auto to = from;
    to[0] += 2.0; // pan two radians around
    LazyRoadmap::QueryStats qs;
    const auto path = rm.query(from, to, {}, &qs);
    REQUIRE(path.has_value());
    CHECK(same(path->front(), from));
    CHECK(same(path->back(), to));
    CHECK(mesh.pathValid(*models.armMesh(0), *path));
    WARN("query: " << qs.edges_checked << " of " << rm.numEdges() << " edges checked, "
         << qs.local_samples + qs.global_samples << " samples added, rrt " << qs.rrt_used);
    CHECK(qs.edges_checked < rm.numEdges());

    // A C_M region around the tool halfway along the first motion: the next
    // query avoids it.
    auto mid = (*path)[0];
    for (std::size_t j = 0; j < mid.size(); ++j)
        mid[j] = 0.5 * ((*path)[0][j] + (*path)[1][j]);
    const auto tool = models.arm(0)->getLinkTransforms(mid)[static_cast<std::size_t>(
        models.arm(0)->linkIndex(kUr5eGripperLink))].translation();
    const std::vector<comotion::ObstacleSphere> region{{tool, 0.1}};
    if (rm.configValid(from, region) && rm.configValid(to, region)) {
        const auto around = rm.query(from, to, region);
        if (around)
            for (std::size_t i = 0; i + 1 < around->size(); ++i)
                CHECK(rm.motionValid((*around)[i], (*around)[i + 1], region));
    }
    // The region binds that query only: the plain query still succeeds.
    CHECK(rm.query(from, to).has_value());
}

TEST_CASE("Lazy roadmap: the found path is shortcut, never longer, every motion valid") {
    auto scene = load("sorting-4robots-4objects.json");
    BodyModels models(scene);
    MeshVerifier mesh(scene);
    comotion::CollisionChecker spheres(comotion::CollisionChecker::Backend::Spheres);
    scene.applyObstacles(spheres);
    const auto length = [](const comotion::Path &p) {
        double d = 0.0;
        for (std::size_t i = 0; i + 1 < p.size(); ++i) {
            double s = 0.0;
            for (std::size_t j = 0; j < p[i].size(); ++j)
                s += (p[i + 1][j] - p[i][j]) * (p[i + 1][j] - p[i][j]);
            d += std::sqrt(s);
        }
        return d;
    };
    RoadmapParams on{300, 10}, off{300, 10}; // dense: the endpoints are not neighbours
    off.shortcut_checks = 0;
    const auto from = scene.arms[0].start;
    auto to = from;
    to[0] += 2.0;
    to[1] -= 0.5;
    LazyRoadmap rm_on(models.arm(0), models.armMesh(0), spheres, mesh, scene.arms[0].lower, scene.arms[0].upper, 5, on);
    LazyRoadmap rm_off(models.arm(0), models.armMesh(0), spheres, mesh, scene.arms[0].lower, scene.arms[0].upper, 5, off);
    const auto p_on = rm_on.query(from, to), p_off = rm_off.query(from, to);
    REQUIRE(p_on.has_value());
    REQUIRE(p_off.has_value());
    CHECK(same(p_on->front(), from));
    CHECK(same(p_on->back(), to));
    for (std::size_t i = 0; i + 1 < p_on->size(); ++i)
        CHECK(rm_on.motionValid((*p_on)[i], (*p_on)[i + 1]));
    CHECK(p_on->size() <= p_off->size());
    CHECK(length(*p_on) <= length(*p_off) + 1e-9);
    WARN("shortcut: " << p_off->size() << " -> " << p_on->size() << " vertices, " << length(*p_off) << " -> "
                      << length(*p_on) << " rad");
}

TEST_CASE("Layers 1-3 give a continuous optimistic schedule, valid per body on meshes") {
    Pipeline p(load("sorting-4robots-4objects.json"));
    const auto st = p.taskPlan();
    const auto l2 = p.layer2.run(st);
    REQUIRE(l2.valid());
    const auto l3 = p.layer3.plan(st, l2, {});
    for (const auto &line : l3.log)
        WARN(line);
    REQUIRE(l3.schedule.has_value());
    const auto &sm = *l3.schedule;
    WARN("S_M: " << sm.segments.size() << " segments, H_M " << l3.hm.numVertices() << "/"
         << l3.hm.numArcs() << ", " << l3.stats.queries << " queries, " << l3.stats.edges_checked
         << " edges checked, " << l3.stats.mesh_rejections << " mesh rejections, rrt "
         << l3.stats.rrt << ", " << l3.stats.seconds << " s");
    for (std::size_t r = 0; r < sm.by_robot.size(); ++r) {
        std::vector<double> at = sm.start[r];
        for (int id : sm.by_robot[r]) {
            const auto &seg = sm.segments[static_cast<std::size_t>(id)];
            CHECK(same(seg.path.front(), at)); // each segment starts where the last ended
            const auto model = seg.holding_grasp < 0 ? std::shared_ptr<const comotion::RobotModel>(p.models.armMesh(seg.robot))
                                                     : p.models.holdingMesh(seg.robot, seg.holding_grasp);
            CHECK(p.mesh.pathValid(*model, seg.path));
            at = seg.path.back();
        }
    }
}

TEST_CASE("A move blocked under C_M emits C_f(context, head of the blocked transition)") {
    Pipeline p(load("sorting-4robots-4objects.json"));
    const auto st = p.taskPlan();
    const auto l2 = p.layer2.run(st);
    REQUIRE(l2.valid());
    // The first pick: forbid the region around its approach configuration's
    // gripper, in the context of block 1 resting at its start.
    std::size_t s = 1;
    while (p.hg.graph().arc(st.steps[s].arc).data.type != TransitionType::Pick)
        ++s;
    const auto &a = *l2.anchors[s];
    const auto tool = p.models.arm(a.robot)->getLinkTransforms(a.q_approach)[static_cast<std::size_t>(
        p.models.arm(a.robot)->linkIndex(kUr5eGripperLink))].translation();
    MotionConstraints cm;
    MotionConstraint c;
    c.element = p.hg.robotElement(a.robot);
    c.consumer_arc = st.steps[s].arc;
    c.context = p.hg.startElement(a.object == 1 ? 2 : 1);
    c.region = {{tool, 0.08}};
    c.origin = "test";
    cm.add(c);
    const auto l3 = p.layer3.plan(st, l2, cm);
    REQUIRE_FALSE(l3.schedule.has_value());
    REQUIRE(l3.constraints.size() == 1);
    CHECK(l3.constraints[0].pre == c.context);
    CHECK(l3.constraints[0].post == p.hg.objectHead(st.steps[s].arc));
    CHECK(l3.constraints[0].tentative); // the roadmap ran out of budget: rests on sampling

    // Under that C_f, the next S_T is ordered so that the C_M no longer
    // applies (C_f and C_M share the partial order): layer 3 plans the move.
    TaskConstraints ct;
    ct.add(l3.constraints[0]);
    TaskSearch search(p.hg, p.h, p.durations);
    const auto st2 = search.query(ct);
    REQUIRE(st2.has_value());
    const auto l2b = p.layer2.run(*st2);
    REQUIRE(l2b.valid());
    const auto l3c = p.layer3.plan(*st2, l2b, cm);
    for (const auto &k : l3c.constraints)
        CHECK(k.origin.find("blocked under C_M") == std::string::npos);

    // A context the transition consumes (here the block being picked) cannot
    // be ordered away: a tentative C_f(element, head), no ban of the pick.
    c.context = p.hg.startElement(a.object);
    MotionConstraints own;
    own.add(c);
    const auto l3b = p.layer3.plan(st, l2, own);
    REQUIRE_FALSE(l3b.schedule.has_value());
    REQUIRE(l3b.constraints.size() == 1);
    CHECK(l3b.constraints[0].pre == c.element);
    CHECK(l3b.constraints[0].tentative);
}

TEST_CASE("Tentative task constraints are lifted, the others kept") {
    TaskConstraints ct;
    ct.add({TaskConstraint::Kind::Frontier, 1, 2, "geometric", false});
    ct.add({TaskConstraint::Kind::Frontier, 3, 4, "sampling", true});
    CHECK(ct.numTentative() == 1);
    CHECK(ct.liftTentative() == 1);
    REQUIRE(ct.size() == 1);
    CHECK(ct.onPost(2).size() == 1);
    CHECK(ct.onPost(4).empty());
}
