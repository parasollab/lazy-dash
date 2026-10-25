#include "lazydash/motion/MotionLayer.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace lazydash {

namespace {

bool sameConfig(const std::vector<double> &a, const std::vector<double> &b) {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > 1e-9)
            return false;
    return true;
}

} // namespace

MotionLayer::MotionLayer(const TaskHypergraph &hg, const BodyModels &models,
                         const MeshVerifier &mesh, RoadmapParams params, std::uint64_t seed)
    : hg_(hg), models_(models), mesh_(mesh), params_(params), seed_(seed),
      scene_spheres_(comotion::CollisionChecker::Backend::Spheres) {
    models_.scene().applyObstacles(scene_spheres_);
}

LazyRoadmap &MotionLayer::roadmap(int robot, int grasp) {
    auto &slot = roadmaps_[{robot, grasp}];
    if (!slot) {
        const auto &spec = models_.scene().arms[static_cast<std::size_t>(robot)];
        std::shared_ptr<const comotion::RobotModel> planning =
            grasp < 0 ? std::shared_ptr<const comotion::RobotModel>(models_.arm(robot)) : models_.holding(robot, grasp);
        std::shared_ptr<const comotion::RobotModel> verification =
            grasp < 0 ? std::shared_ptr<const comotion::RobotModel>(models_.armMesh(robot)) : models_.holdingMesh(robot, grasp);
        slot = std::make_unique<LazyRoadmap>(planning, verification, scene_spheres_, mesh_, spec.lower,
                                             spec.upper, seed_ * 1000003ull + robot * 97ull + (grasp + 1),
                                             params_);
        slot->setBudgetScale(scale_);
    }
    return *slot;
}

void MotionLayer::setBudgetScale(double scale) {
    scale_ = scale;
    for (auto &[key, rm] : roadmaps_)
        rm->setBudgetScale(scale);
}

std::size_t MotionLayer::roadmapVertices() const {
    std::size_t n = 0;
    for (const auto &[key, rm] : roadmaps_)
        n += static_cast<std::size_t>(rm->numVertices());
    return n;
}

MotionLayer::Precedence MotionLayer::precedence(const TaskSchedule &st) {
    // Steps are in topological order, so one pass closes the relation.
    const std::size_t n = st.steps.size();
    Precedence before(n, std::vector<char>(n, 0));
    // Orderings committed by C_f (TaskStep::after) are edges too.
    for (std::size_t s = 0; s < n; ++s) {
        before[s][s] = 1;
        std::vector<int> preds;
        for (int te : st.steps[s].tail)
            preds.push_back(st.vertices[static_cast<std::size_t>(te)].producer);
        preds.insert(preds.end(), st.steps[s].after.begin(), st.steps[s].after.end());
        for (int p : preds) {
            if (p < 0)
                continue;
            for (std::size_t a = 0; a < n; ++a)
                if (before[a][static_cast<std::size_t>(p)])
                    before[a][s] = 1;
        }
    }
    return before;
}

std::vector<comotion::ObstacleSphere>
MotionLayer::regions(const TaskSchedule &st, const std::vector<int> &consumers, const Precedence &before,
                     int te, int step, const MotionConstraints &cm, int *context) const {
    std::vector<comotion::ObstacleSphere> out;
    const int element = st.vertices[static_cast<std::size_t>(te)].element;
    const int arc = st.steps[static_cast<std::size_t>(step)].arc;
    const int motion_start = st.vertices[static_cast<std::size_t>(te)].producer;
    for (const auto &c : cm.all()) {
        if (c.element != element || c.consumer_arc != arc)
            continue;
        // Pi_1: the context is alive during the motion in the hyperpath's partial
        // order (S_T is one linearization; independent threads overlap in time).
        bool alive = false;
        for (std::size_t u = 0; u < st.vertices.size() && !alive; ++u) {
            if (st.vertices[u].element != c.context)
                continue;
            const int produced = st.vertices[u].producer, consumed = consumers[u];
            // Consumed before the move starts (also via TaskStep::after).
            bool ends_before = consumed >= 0 && motion_start >= 0 &&
                               before[static_cast<std::size_t>(consumed)][static_cast<std::size_t>(motion_start)];
            for (int p : st.steps[static_cast<std::size_t>(step)].after)
                ends_before = ends_before ||
                              (consumed >= 0 && before[static_cast<std::size_t>(consumed)][static_cast<std::size_t>(p)]);
            const bool starts_after = produced >= 0 &&
                                      before[static_cast<std::size_t>(step)][static_cast<std::size_t>(produced)];
            alive = !ends_before && !starts_after;
        }
        if (!alive)
            continue;
        out.insert(out.end(), c.region.begin(), c.region.end());
        if (context)
            *context = c.context;
    }
    return out;
}

MotionLayerResult MotionLayer::plan(const TaskSchedule &st, const TaskConflictResult &layer2,
                                    const MotionConstraints &cm) {
    const auto t0 = std::chrono::steady_clock::now();
    MotionLayerResult result;
    OptimisticSchedule sm;
    const auto &scene = models_.scene();
    const int R = models_.numRobots();
    sm.by_robot.resize(static_cast<std::size_t>(R));
    sm.anchors = layer2.anchors;
    for (const auto &arm : scene.arms)
        sm.start.push_back(arm.start);
    std::vector<std::vector<double>> config = sm.start;
    std::vector<int> mvertex(static_cast<std::size_t>(R), -1); // arm -> its current H_M vertex
    const auto consumers = st.consumers();
    const auto before = precedence(st);
    auto &hm = result.hm;

    const auto addSegment = [&](MotionSegment seg) {
        const int id = static_cast<int>(sm.segments.size());
        sm.by_robot[static_cast<std::size_t>(seg.robot)].push_back(id);
        sm.segments.push_back(std::move(seg));
        return id;
    };
    const auto finish = [&] {
        result.stats.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return result;
    };

    for (std::size_t s = 0; s < st.steps.size(); ++s) {
        const auto &step = st.steps[s];
        const auto &t = hg_.graph().arc(step.arc).data;
        if (t.type == TransitionType::Source) {
            // E^comp: the initial elements, arms at their start configurations.
            std::vector<int> heads;
            for (int te : step.head) {
                const auto &e = hg_.graph().vertex(st.vertices[static_cast<std::size_t>(te)].element);
                MotionVertex v{te, st.vertices[static_cast<std::size_t>(te)].element, e.robot, {}};
                if (e.kind == ElementKind::RobotOnly)
                    v.q = config[static_cast<std::size_t>(e.robot)];
                heads.push_back(hm.addVertex(v));
                if (e.kind == ElementKind::RobotOnly)
                    mvertex[static_cast<std::size_t>(e.robot)] = heads.back();
            }
            hm.addArc({}, heads, {MotionArc::Kind::Composition, static_cast<int>(s), {}});
            continue;
        }
        if (t.type == TransitionType::Sink) {
            hm.addArc({}, {}, {MotionArc::Kind::Composition, static_cast<int>(s), {}});
            continue;
        }
        const auto &anchors = layer2.anchors[s];
        if (!anchors) {
            result.log.push_back("layer3: no anchors for step " + std::to_string(s));
            return finish();
        }
        const auto &a = *anchors;
        const int head = hg_.objectHead(step.arc);

        // Arms entering this transition, their H_TE vertex and start configuration.
        struct Role {
            int robot, te, grasp;
            const std::vector<double> *start;
        };
        std::vector<Role> roles;
        const auto teOf = [&](int robot) {
            for (int te : step.tail)
                if (hg_.graph().vertex(st.vertices[static_cast<std::size_t>(te)].element).robot == robot)
                    return te;
            return -1;
        };
        if (t.type == TransitionType::Pick)
            roles.push_back({t.robot, teOf(t.robot), -1, &a.q_approach});
        else if (t.type == TransitionType::Place)
            roles.push_back({t.robot, teOf(t.robot), t.grasp, &a.q_approach});
        else {
            roles.push_back({t.robot, teOf(t.robot), t.grasp, &a.q_grasp});
            roles.push_back({t.robot2, teOf(t.robot2), -1, &a.q2_approach});
        }

        // E^move of each arm into the transition (QueryMotionPlan).
        std::vector<int> transition_tail;
        for (const auto &role : roles) {
            auto &q = config[static_cast<std::size_t>(role.robot)];
            if (!sameConfig(q, *role.start)) {
                int context = -1;
                const auto extra = regions(st, consumers, before, role.te, static_cast<int>(s), cm, &context);
                LazyRoadmap::QueryStats qs;
                ++result.stats.queries;
                std::vector<long long> key{role.robot, role.grasp};
                for (const std::vector<double> *v : {static_cast<const std::vector<double> *>(&q), role.start})
                    for (double x : *v)
                        key.push_back(std::llround(x * 1e9));
                for (const auto &sph : extra) {
                    for (int k = 0; k < 3; ++k)
                        key.push_back(std::llround(sph.center[k] * 1e6));
                    key.push_back(std::llround(sph.radius * 1e6));
                }
                std::optional<comotion::Path> path;
                if (auto hit = query_cache_.find(key); hit != query_cache_.end()) {
                    path = hit->second;
                } else {
                    path = roadmap(role.robot, role.grasp).query(q, *role.start, extra, &qs);
                    if (path)
                        query_cache_.emplace(key, *path);
                }
                result.stats.edges_checked += qs.edges_checked;
                result.stats.mesh_rejections += qs.mesh_rejections;
                result.stats.rrt += qs.rrt_used;
                if (!path) {
                    const int element = st.vertices[static_cast<std::size_t>(role.te)].element;
                    TaskConstraint own;
                    own.kind = TaskConstraint::Kind::Frontier;
                    own.pre = element;
                    own.post = head;
                    own.tentative = true;
                    own.origin = "layer3: no motion for " + hg_.describe(element) + " into " +
                                 hg_.describeArc(step.arc);
                    // A C_M context consumed by the transition itself (handover partner) cannot
                    // be ordered away, so it is a plain failure.
                    const auto &tail = hg_.graph().arc(step.arc).tail;
                    if (context >= 0 && std::find(tail.begin(), tail.end(), context) == tail.end()) {
                        TaskConstraint c = own;
                        c.pre = context; // tentative: the roadmap ran out of budget
                        c.origin = "layer3: move of " + hg_.describe(element) + " into " +
                                   hg_.describeArc(step.arc) + " blocked under C_M by " + hg_.describe(context);
                        result.log.push_back(c.origin);
                        result.constraints.push_back(std::move(c));
                    } else {
                        if (context >= 0)
                            own.origin += " (blocked under C_M by its partner " + hg_.describe(context) + ")";
                        result.log.push_back(own.origin);
                        result.constraints.push_back(std::move(own));
                    }
                    result.failed_arcs.push_back(step.arc);
                    const int producer = st.vertices[static_cast<std::size_t>(role.te)].producer;
                    if (producer > 0)
                        result.failed_arcs.push_back(st.steps[static_cast<std::size_t>(producer)].arc);
                    return finish();
                }
                MotionSegment seg;
                seg.kind = MotionSegment::Kind::Move;
                seg.robot = role.robot;
                seg.step = static_cast<int>(s);
                seg.te = role.te;
                seg.element = st.vertices[static_cast<std::size_t>(role.te)].element;
                seg.holding_grasp = role.grasp;
                seg.object = role.grasp >= 0 ? t.object : -1;
                seg.path = *path;
                const int id = addSegment(std::move(seg));
                const int v = hm.addVertex({role.te, st.vertices[static_cast<std::size_t>(role.te)].element,
                                            role.robot, *role.start});
                hm.addArc({mvertex[static_cast<std::size_t>(role.robot)]}, {v},
                          {MotionArc::Kind::Move, static_cast<int>(s), {id}});
                mvertex[static_cast<std::size_t>(role.robot)] = v;
                q = *role.start;
            }
            transition_tail.push_back(mvertex[static_cast<std::size_t>(role.robot)]);
        }

        // E^tran: the stage segments of this transition.
        const auto headTe = [&](int robot) {
            for (int te : step.head)
                if (hg_.graph().vertex(st.vertices[static_cast<std::size_t>(te)].element).robot == robot)
                    return te;
            return -1;
        };
        const auto stage = [&](int robot, int index, int te, int grasp, const std::vector<double> &from,
                               const std::vector<double> &to, int contact_object, int contact_robot,
                               int contact_grasp) {
            MotionSegment seg;
            seg.kind = MotionSegment::Kind::Stage;
            seg.robot = robot;
            seg.step = static_cast<int>(s);
            seg.stage = index;
            seg.te = te;
            seg.element = st.vertices[static_cast<std::size_t>(te)].element;
            seg.holding_grasp = grasp;
            seg.object = t.object;
            seg.contact_object = contact_object;
            seg.contact_robot = contact_robot;
            seg.contact_grasp = contact_grasp;
            seg.path = comotion::Path{from, to};
            return addSegment(std::move(seg));
        };
        std::vector<int> stage_ids;
        std::vector<int> transition_head;
        if (t.type == TransitionType::Pick) {
            stage_ids.push_back(stage(t.robot, 0, teOf(t.robot), -1, a.q_approach, a.q_grasp, t.object, -1, t.grasp));
            stage_ids.push_back(stage(t.robot, 1, headTe(t.robot), t.grasp, a.q_grasp, a.q_approach, -1, -1, -1));
            config[static_cast<std::size_t>(t.robot)] = a.q_approach;
        } else if (t.type == TransitionType::Place) {
            stage_ids.push_back(stage(t.robot, 0, teOf(t.robot), t.grasp, a.q_approach, a.q_grasp, -1, -1, -1));
            stage_ids.push_back(stage(t.robot, 1, headTe(t.robot), -1, a.q_grasp, a.q_approach, t.object, -1, t.grasp));
            config[static_cast<std::size_t>(t.robot)] = a.q_approach;
        } else {
            stage_ids.push_back(stage(t.robot2, 0, teOf(t.robot2), -1, a.q2_approach, a.q2_grasp, -1, t.robot, t.grasp2));
            stage_ids.push_back(stage(t.robot, 1, headTe(t.robot), -1, a.q_grasp, a.q_approach, -1, t.robot2, t.grasp));
            config[static_cast<std::size_t>(t.robot)] = a.q_approach;
            config[static_cast<std::size_t>(t.robot2)] = a.q2_grasp;
        }
        for (const auto &role : roles) {
            const int te = headTe(role.robot);
            const int v = hm.addVertex({te, st.vertices[static_cast<std::size_t>(te)].element, role.robot,
                                        config[static_cast<std::size_t>(role.robot)]});
            transition_head.push_back(v);
            mvertex[static_cast<std::size_t>(role.robot)] = v;
        }
        hm.addArc(transition_tail, transition_head, {MotionArc::Kind::Transition, static_cast<int>(s), stage_ids});
    }
    result.schedule = std::move(sm);
    return finish();
}

} // namespace lazydash
