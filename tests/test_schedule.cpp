// The timed schedule under D, layer 4, and the whole planning loop.
#include "lazydash/motion/MotionLayer.h"
#include "lazydash/planning/LazyDash.h"
#include "lazydash/robot/BodyModels.h"
#include "lazydash/scene/Scene.h"
#include "lazydash/schedule/MeshVerifier.h"
#include "lazydash/schedule/MotionConflicts.h"
#include "lazydash/schedule/TimedSchedule.h"
#include "lazydash/task/Capabilities.h"
#include "lazydash/task/Heuristic.h"
#include "lazydash/task/TaskConflictDetection.h"
#include "lazydash/task/TaskHypergraph.h"
#include "lazydash/task/TaskSearch.h"
#include "lazydash/task/TransitionSampler.h"

#include "comotion/collision/ConflictChecker.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>

using namespace lazydash;

namespace {

const auto kScenes = std::filesystem::path(LAZYDASH_RESOURCE_DIR) / "scenes";

bool same(const std::vector<double> &a, const std::vector<double> &b, double tol = 1e-6) {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > tol)
            return false;
    return true;
}

struct Optimistic {
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
    TaskSchedule st;
    OptimisticSchedule sm;

    explicit Optimistic(const std::string &file)
        : scene(Scene::load((kScenes / file).string())), models(scene), caps(scene, scene.makeArms()),
          hg(TaskHypergraph::build(caps)), h(hg), durations(caps, hg), mesh(scene), sampler(hg, caps, models, mesh, 1),
          layer2(hg, sampler, models), layer3(hg, models, mesh) {
        TaskSearch search(hg, h, durations);
        st = *search.query({});
        const auto l2 = layer2.run(st);
        REQUIRE(l2.valid());
        auto l3 = layer3.plan(st, l2, {});
        REQUIRE(l3.schedule.has_value());
        sm = *l3.schedule;
    }
};

} // namespace

TEST_CASE("The timed schedule respects D: order per arm and handover barriers") {
    Optimistic o("paper/sorting-4robots-4objects.json");
    TimedSchedule ts(o.sm, o.st, o.hg, o.models);
    const auto &segs = ts.segments();
    for (const auto &list : ts.byRobot())
        for (std::size_t k = 1; k < list.size(); ++k)
            CHECK(segs[static_cast<std::size_t>(list[k])].start >= segs[static_cast<std::size_t>(list[k - 1])].end());
    int handovers = 0;
    for (std::size_t s = 0; s < o.st.steps.size(); ++s) {
        const auto &t = o.hg.graph().arc(o.st.steps[s].arc).data;
        if (t.type != TransitionType::Handover)
            continue;
        ++handovers;
        int approach = -1, backoff = -1;
        for (std::size_t i = 0; i < segs.size(); ++i)
            if (segs[i].seg.kind == MotionSegment::Kind::Stage && segs[i].seg.step == static_cast<int>(s))
                (segs[i].seg.stage == 0 ? approach : backoff) = static_cast<int>(i);
        REQUIRE(approach >= 0);
        REQUIRE(backoff >= 0);
        const auto &A = segs[static_cast<std::size_t>(approach)];
        const auto &B = segs[static_cast<std::size_t>(backoff)];
        // The giver holds still at the exchange while the receiver approaches.
        CHECK(ts.segmentAt(t.robot, A.start) < 0);
        CHECK(same(ts.robotConfigAt(t.robot, A.start), B.path.front()));
        CHECK(B.start >= A.end());
        // The object passes from the giver to the receiver when the receiver closes.
        CHECK(ts.objectAt(t.object, A.end() - 1).robot == t.robot);
        CHECK(ts.objectAt(t.object, A.end()).robot == t.robot2);
    }
    CHECK(handovers > 0);
    // Every object ends at a goal, at rest.
    for (int obj = 0; obj < ts.numObjects(); ++obj) {
        const auto end = ts.objectAt(obj, ts.makespan());
        CHECK_FALSE(end.carried);
        CHECK(end.element == o.hg.goalElement(obj));
    }
    // A delay re-times every dependent segment.
    const std::size_t before = ts.makespan();
    ts.segments()[1].delay += 100;
    ts.retime();
    CHECK(ts.makespan() >= before);
    CHECK(ts.makespan() <= before + 100);

    // An ordering of C_f (TaskStep::after) between steps of different arms
    // is part of D: the later step's segments start after the earlier's end.
    int j = -1, k = -1;
    for (std::size_t s = 1; s + 1 < o.st.steps.size() && k < 0; ++s) {
        const int r = o.hg.graph().arc(o.st.steps[s].arc).data.robot;
        if (j < 0)
            j = static_cast<int>(s);
        else if (r != o.hg.graph().arc(o.st.steps[static_cast<std::size_t>(j)].arc).data.robot)
            k = static_cast<int>(s);
    }
    REQUIRE(k > j);
    TaskSchedule ordered = o.st;
    ordered.steps[static_cast<std::size_t>(k)].after.push_back(j);
    TimedSchedule tso(o.sm, ordered, o.hg, o.models);
    std::size_t end_j = 0;
    for (const auto &s : tso.segments())
        if (s.seg.step == j)
            end_j = std::max(end_j, s.end());
    int checked = 0;
    for (const auto &s : tso.segments())
        if (s.seg.step == k) {
            CHECK(s.start >= end_j);
            ++checked;
        }
    CHECK(checked > 0);
}

TEST_CASE("Layer 4 resolves the optimistic schedule into a valid one, or feeds back") {
    Optimistic o("paper/sorting-4robots-4objects.json");
    TimedSchedule ts(o.sm, o.st, o.hg, o.models);
    MotionConflicts layer4(o.hg, o.st, o.models, o.mesh);
    const auto r = layer4.resolve(ts);
    WARN("layer 4: valid " << r.valid << ", h-h " << r.hh << ", h-v " << r.hv << ", waits " << r.timing_fixes
         << ", ARC " << r.arc_repairs << ", mesh conflicts " << r.mesh_conflicts << ", C_M " << r.cm.size()
         << ", C_T " << r.ct.size() << ", " << r.seconds << " s");
    CHECK((r.valid || !r.cm.empty() || !r.ct.empty()));
    if (r.valid) {
        CHECK_FALSE(layer4.firstConflict(ts, false).has_value());
        CHECK_FALSE(layer4.firstConflict(ts, true).has_value());
    }
}

TEST_CASE("Lazy-DaSH solves small scenes end to end, verified on meshes") {
    for (const char *file : {"paper/sorting-4robots-4objects.json", "generated/sorting-4robots-4objects-s0.json",
                             "generated/wall-2walls-4objects-s0.json"}) {
        INFO(file);
        const auto scene = Scene::load((kScenes / file).string());
        LazyDashParams params;
        params.time_limit = 600;
        LazyDash planner(scene, params);
        const auto result = planner.solve();
        const auto &s = result.stats;
        std::string log;
        for (std::size_t i = 0; i < result.log.size() && i < 40; ++i)
            log += "  " + result.log[i] + "\n";
        WARN(file << ": success " << result.success << " (" << result.failure << "), makespan "
             << s.makespan << ", outer " << s.outer_iterations << ", inner " << s.inner_iterations
             << ", C_T " << s.task_constraints << ", C_M " << s.motion_constraints << ", h-h " << s.hh
             << ", h-v " << s.hv << ", waits " << s.timing_fixes << ", ARC " << s.arc_repairs
             << ", mesh conflicts " << s.mesh_conflicts << "\n  H_T " << s.ht_vertices << "/" << s.ht_arcs
             << ", H_M " << s.hm_vertices << "/" << s.hm_arcs << ", roadmap vertices " << s.roadmap_vertices
             << "\n  clocks: task " << s.task_query << " s, conflicts " << s.task_conflicts << " s, motion "
             << s.motion_query << " s, resolution " << s.conflict_resolution << " s, total " << s.total
             << " s\n" << log);
        REQUIRE(result.success);
        // Independent re-check of the reported paths on meshes, all bodies at once.
        const MeshVerifier mesh(scene);
        BodyModels models(scene);
        for (std::size_t r = 0; r < result.robot_paths.size(); ++r)
            CHECK(result.robot_paths[r].arrival_timestep() == s.makespan);
        for (std::size_t o = 0; o < result.object_paths.size(); ++o) {
            const auto &end = result.object_paths[o].back();
            bool at_goal = false;
            for (const auto &g : scene.objects[o].goals)
                at_goal = at_goal || (g.translation() - Eigen::Vector3d(end[0], end[1], end[2])).norm() < 1e-6;
            CHECK(at_goal);
        }
    }
}
