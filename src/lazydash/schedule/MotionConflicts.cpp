#include "lazydash/schedule/MotionConflicts.h"

#include "lazydash/schedule/ScheduledARC.h"

#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/robot/FlyingSphere.h"

#include <algorithm>
#include <chrono>

namespace lazydash {

namespace {

constexpr int kRestingContactId = 999;

/// Every object resting during an epoch as one static body (a fixed link per
/// object), which drops object-object pairs from the scan.
class RestingObjects : public comotion::FlyingSphere {
public:
    RestingObjects(const std::vector<std::pair<int, Eigen::Vector3d>> &objects, double radius)
        : comotion::FlyingSphere(1e-6, -1.0, 1.0) {
        auto &end = links_.back();
        end.collision_spheres.clear();
        const std::string parent = end.name;
        for (const auto &[o, position] : objects) {
            comotion::LinkInfo link;
            link.name = linkName(o);
            link.index = static_cast<int>(links_.size());
            link.collision_spheres.push_back({Eigen::Vector3d::Zero(), radius, link.index});
            comotion::JointInfo joint;
            joint.name = link.name + "_joint";
            joint.type = "fixed";
            joint.parent_link = parent;
            joint.child_link = link.name;
            joint.origin = Eigen::Affine3d(Eigen::Translation3d(position));
            link_name_to_index_[link.name] = link.index;
            links_.push_back(std::move(link));
            joints_.push_back(std::move(joint));
        }
        for (std::size_t i = 0; i < links_.size(); ++i)
            for (std::size_t j = i + 1; j < links_.size(); ++j)
                disabled_collisions_.insert(std::minmax(links_[i].name, links_[j].name));
        buildKinematicChain();
        setContactId(kRestingContactId);
    }
    static std::string linkName(int o) { return "object_" + std::to_string(o); }
};

} // namespace

MotionConflicts::MotionConflicts(const TaskHypergraph &hg, const TaskSchedule &st,
                                 const BodyModels &models, const MeshVerifier &mesh,
                                 Layer4Params params, std::uint64_t seed)
    : hg_(hg), st_(st), models_(models), mesh_(mesh), params_(params), seed_(seed),
      spheres_(comotion::CollisionChecker::Backend::Spheres) {
    models_.scene().applyObstacles(spheres_);
}

MotionConflicts::Bodies MotionConflicts::epochBodies(const TimedSchedule &schedule, std::size_t t0,
                                                     std::size_t t1, bool mesh) const {
    Bodies b;
    const auto armModel = [&](int r, int grasp) -> std::shared_ptr<const comotion::RobotModel> {
        if (grasp < 0)
            return mesh ? models_.armMesh(r) : models_.arm(r);
        return mesh ? models_.holdingMesh(r, grasp) : models_.holding(r, grasp);
    };
    // Resting objects: one static body.
    std::vector<std::pair<int, Eigen::Vector3d>> resting;
    for (int o = 0; o < schedule.numObjects(); ++o) {
        const auto s = schedule.objectAt(o, t0);
        if (!s.carried)
            resting.emplace_back(o, s.position);
    }
    std::shared_ptr<RestingObjects> resting_body;
    if (!resting.empty()) {
        resting_body = std::make_shared<RestingObjects>(
            resting, models_.scene().objects.empty() ? 0.0 : models_.scene().objects[0].collision_radius);
        b.owned.push_back(resting_body);
    }
    // Arms, with the contact rules of the stage they are in.
    std::vector<std::shared_ptr<const comotion::RobotModel>> arm_models;
    for (int r = 0; r < schedule.numRobots(); ++r)
        arm_models.push_back(armModel(r, schedule.heldGraspAt(r, t0)));
    for (int r = 0; r < schedule.numRobots(); ++r) {
        const int seg_id = schedule.segmentAt(r, t0);
        const comotion::RobotModel *model = arm_models[static_cast<std::size_t>(r)].get();
        if (seg_id >= 0) {
            const auto &seg = schedule.segments()[static_cast<std::size_t>(seg_id)].seg;
            if (seg.kind == MotionSegment::Kind::Stage && seg.contact_grasp >= 0 &&
                (seg.contact_object >= 0 || seg.contact_robot >= 0)) {
                auto copy = std::make_shared<comotion::RobotModel>(*model);
                const auto &grasp = models_.graspSet(r).grasp(static_cast<std::size_t>(seg.contact_grasp));
                if (seg.contact_object >= 0) {
                    if (resting_body)
                        allowGraspContact(*copy, grasp, *resting_body, RestingObjects::linkName(seg.contact_object));
                } else {
                    const int other_grasp = schedule.heldGraspAt(seg.contact_robot, t0);
                    const auto &other = *arm_models[static_cast<std::size_t>(seg.contact_robot)];
                    if (other_grasp >= 0)
                        allowExchangeContact(*copy, grasp, other,
                                             models_.graspSet(seg.contact_robot)
                                                 .grasp(static_cast<std::size_t>(other_grasp)));
                    else
                        allowGraspContact(*copy, grasp, other, kHeldObjectLink);
                }
                b.owned.push_back(copy);
                model = copy.get();
            }
        }
        b.robots.push_back(model);
        b.arm.push_back(r);
        b.paths.push_back(schedule.robotSlice(r, t0, t1));
    }
    if (resting_body) {
        b.robots.push_back(resting_body.get());
        b.arm.push_back(-1);
        b.paths.push_back(comotion::Path{{0.0, 0.0, 0.0}});
    }
    // Arm models are owned by BodyModels (shared) or by `b.owned` (contact copies).
    return b;
}

std::optional<MotionConflicts::Found> MotionConflicts::firstConflict(const TimedSchedule &schedule,
                                                                      bool mesh, std::size_t from) const {
    auto boundaries = schedule.epochBoundaries();
    boundaries.push_back(schedule.makespan() + 1);
    const comotion::ConflictChecker checker(mesh ? mesh_.checker() : spheres_);
    comotion::CompositePathValidationOptions options;
    options.check_environment = false; // segments are valid against the scene (layer 3)
    for (std::size_t e = 0; e + 1 < boundaries.size(); ++e) {
        const std::size_t t0 = boundaries[e], t1 = boundaries[e + 1] - 1;
        if (t1 < t0 || t1 < from)
            continue;
        const auto bodies = epochBodies(schedule, t0, t1, mesh);
        options.t_begin = from > t0 ? from - t0 : 0; // clean before `from`
        const auto conflict = checker.findConflict(bodies.paths, bodies.robots, options);
        if (!conflict)
            continue;
        Found f;
        f.t = t0 + static_cast<std::size_t>(std::max(conflict->timestep, 0));
        f.mesh = mesh;
        bool hit_resting = false;
        for (int idx : {conflict->robot_i, conflict->robot_j}) {
            if (idx < 0)
                continue;
            const auto i = static_cast<std::size_t>(idx);
            if (bodies.arm[i] >= 0)
                (f.arm_a < 0 ? f.arm_a : f.arm_b) = bodies.arm[i];
            else
                hit_resting = true;
        }
        if (hit_resting && f.arm_a >= 0) {
            // Which resting object: the one the arm (with its contact rules,
            // so its own target is exempt) cannot clear at that time.
            const auto &arm_model = *bodies.robots[static_cast<std::size_t>(f.arm_a)];
            const auto q = schedule.robotConfigAt(f.arm_a, f.t);
            const auto &cc = mesh ? mesh_.checker() : spheres_;
            const int seg_id = schedule.segmentAt(f.arm_a, f.t);
            const int contact = seg_id >= 0 ? schedule.segments()[static_cast<std::size_t>(seg_id)].seg.contact_object : -1;
            for (int o = 0; o < schedule.numObjects() && f.object < 0; ++o) {
                const auto s = schedule.objectAt(o, f.t);
                if (s.carried || o == contact)
                    continue;
                if (!cc.isValidPair(arm_model, q, *models_.object(o), {s.position.x(), s.position.y(), s.position.z()}))
                    f.object = o;
            }
            if (f.object < 0) // the resting body's spheres are those objects: should not happen
                continue;
        }
        return f;
    }
    return std::nullopt;
}

std::vector<comotion::ObstacleSphere> MotionConflicts::armRegion(const TimedSchedule &schedule, int arm,
                                                                 std::size_t t) const {
    const int grasp = schedule.heldGraspAt(arm, t);
    const auto model = grasp < 0 ? std::shared_ptr<const comotion::RobotModel>(models_.arm(arm))
                                 : models_.holding(arm, grasp);
    std::vector<comotion::ObstacleSphere> region;
    for (const auto &s : model->getCollisionSpheres(schedule.robotConfigAt(arm, t)))
        region.push_back({s.center, s.radius + model->collisionPadding() + params_.region_margin});
    return region;
}

bool MotionConflicts::tryTiming(TimedSchedule &schedule, const Found &c, Layer4Result &out,
                                std::size_t &changed) {
    auto &tries = timing_tries_[{std::min(c.arm_a, c.arm_b), std::max(c.arm_a, c.arm_b)}];
    if (tries >= params_.max_timing_tries)
        return false;
    // Delay the arm that started its current segment later until the other
    // has finished its segment (the one it is in, or its next one).
    const int sa = schedule.segmentAt(c.arm_a, c.t), sb = schedule.segmentAt(c.arm_b, c.t);
    if (sa < 0 && sb < 0)
        return false;
    int delayed = -1, other = -1;
    if (sa >= 0 && sb >= 0) {
        const bool a_later = schedule.segments()[static_cast<std::size_t>(sa)].start >=
                             schedule.segments()[static_cast<std::size_t>(sb)].start;
        delayed = a_later ? sa : sb;
        other = a_later ? sb : sa;
    } else {
        // One arm holds still: the mover waits for it to leave, if it ever moves again.
        const int still_arm = sa < 0 ? c.arm_a : c.arm_b;
        delayed = sa < 0 ? sb : sa;
        for (int id : schedule.byRobot()[static_cast<std::size_t>(still_arm)])
            if (schedule.segments()[static_cast<std::size_t>(id)].start > c.t) {
                other = id;
                break;
            }
        if (other < 0)
            return false;
    }
    auto &d = schedule.segments()[static_cast<std::size_t>(delayed)];
    const auto &o = schedule.segments()[static_cast<std::size_t>(other)];
    const std::size_t wait = o.end() > d.start ? o.end() - d.start + 1 : 1;
    if (d.delay + wait > params_.max_delay)
        return false;
    changed = d.start; // the delayed arm's path differs from its old start on
    d.delay += wait;
    ++tries;
    ++out.timing_fixes;
    schedule.retime();
    out.log.push_back("layer4: h-h r" + std::to_string(c.arm_a) + "/r" + std::to_string(c.arm_b) +
                      " at t=" + std::to_string(c.t) + ": wait " + std::to_string(wait));
    return true;
}

bool MotionConflicts::tryArc(TimedSchedule &schedule, const Found &c, Layer4Result &out,
                             std::size_t &changed) {
    const int sa = schedule.segmentAt(c.arm_a, c.t), sb = schedule.segmentAt(c.arm_b, c.t);
    if (sa < 0 || sb < 0)
        return false;
    const auto &A = schedule.segments()[static_cast<std::size_t>(sa)];
    const auto &B = schedule.segments()[static_cast<std::size_t>(sb)];
    if (A.seg.kind != MotionSegment::Kind::Move || B.seg.kind != MotionSegment::Kind::Move)
        return false;
    const std::size_t t0 = std::max(A.start, B.start), t1 = std::min(A.end(), B.end());
    if (t1 <= t0 + 1)
        return false;
    const bool mesh = c.mesh;
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        mesh ? comotion::CollisionChecker::Backend::Fcl : comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(schedule.resolution());
    problem->setVmax(schedule.vmax());
    std::vector<comotion::Path> paths;
    for (const auto *seg : {&A, &B}) {
        const int r = seg->seg.robot, g = seg->seg.holding_grasp;
        std::shared_ptr<const comotion::RobotModel> model =
            g < 0 ? std::shared_ptr<const comotion::RobotModel>(mesh ? models_.armMesh(r) : models_.arm(r))
                  : (mesh ? models_.holdingMesh(r, g) : models_.holding(r, g));
        paths.push_back(schedule.robotSlice(r, t0, t1));
        problem->addRobot(std::make_shared<comotion::RobotModel>(*model), paths.back().front(),
                          paths.back().back());
    }
    // Static for the subproblem: the scene and every object resting at any time in the window.
    auto obstacles = models_.scene().spheres;
    const double radius = models_.scene().objects.empty() ? 0.0 : models_.scene().objects[0].collision_radius;
    for (int o = 0; o < schedule.numObjects(); ++o)
        for (std::size_t t : {t0, (t0 + t1) / 2, t1}) {
            const auto s = schedule.objectAt(o, t);
            if (!s.carried) {
                obstacles.push_back({s.position, radius});
                break;
            }
        }
    problem->setObstacles(obstacles);
    problem->setCylinderObstacles(models_.scene().cylinders);
    ScheduledARC arc;
    arc.setProblem(problem);
    arc.setPlanningSeed(static_cast<unsigned>(seed_ + out.arc_repairs));
    const auto outcome = arc.repairPaths(paths, params_.arc_time, params_.arc_max_repairs);
    if (!outcome.conflict_free)
        return false;
    const std::size_t a_start = A.start, b_start = B.start;
    changed = t0;
    for (int k = 0; k < 2; ++k) {
        comotion::Path dense;
        for (std::size_t t = 0; t <= paths[static_cast<std::size_t>(k)].arrival_timestep(); ++t)
            dense.push_back(paths[static_cast<std::size_t>(k)].config_at_timestep(t));
        const int id = k == 0 ? sa : sb;
        const std::size_t base = k == 0 ? a_start : b_start;
        schedule.spliceSegment(id, t0 - base, t1 - base, dense);
    }
    schedule.retime();
    ++out.arc_repairs;
    out.log.push_back("layer4: h-h r" + std::to_string(c.arm_a) + "/r" + std::to_string(c.arm_b) +
                      " at t=" + std::to_string(c.t) + ": ScheduledARC repair" + (mesh ? " (meshes)" : ""));
    return true;
}

Layer4Result MotionConflicts::resolve(TimedSchedule &schedule) {
    const auto t_start = std::chrono::steady_clock::now();
    Layer4Result out;
    const auto done = [&] {
        out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
        return out;
    };
    // Both levels are clean before these times (a repair only changes paths
    // from its start time on), so scans resume there.
    std::size_t sphere_from = 0, mesh_from = 0;
    for (int iter = 0; iter < params_.max_iterations; ++iter) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() > params_.time_limit) {
            out.log.push_back("layer4: time budget spent");
            return done();
        }
        auto c = firstConflict(schedule, false, sphere_from);
        if (c) {
            sphere_from = c->t;
        } else {
            sphere_from = schedule.makespan() + 1;
            c = firstConflict(schedule, true, mesh_from);
            if (c) {
                ++out.mesh_conflicts;
                mesh_from = c->t;
            }
        }
        if (!c) {
            out.valid = true;
            return done();
        }
        if (c->arm_a >= 0 && c->arm_b >= 0) { // hyperarc-hyperarc
            ++out.hh;
            std::size_t changed = 0;
            if (tryTiming(schedule, *c, out, changed) || tryArc(schedule, *c, out, changed)) {
                sphere_from = std::min(sphere_from, changed);
                mesh_from = std::min(mesh_from, changed);
                continue;
            }
            // Replanning in layer 3: the moving arm avoids the other arm there.
            const int sa = schedule.segmentAt(c->arm_a, c->t), sb = schedule.segmentAt(c->arm_b, c->t);
            const auto isMove = [&](int id) {
                return id >= 0 && schedule.segments()[static_cast<std::size_t>(id)].seg.kind == MotionSegment::Kind::Move;
            };
            int mover_seg = -1, other_arm = -1;
            if (isMove(sa) && (!isMove(sb) || schedule.segments()[static_cast<std::size_t>(sa)].start >=
                                                  schedule.segments()[static_cast<std::size_t>(sb)].start)) {
                mover_seg = sa;
                other_arm = c->arm_b;
            } else if (isMove(sb)) {
                mover_seg = sb;
                other_arm = c->arm_a;
            }
            // Both arms holding still between segments: the one that arrived
            // last made the overlap; its arrival is the segment to repair.
            int stage_seg = sa >= 0 ? sa : sb;
            int stage_arm = sa >= 0 ? c->arm_a : c->arm_b;
            if (sa < 0 && sb < 0) {
                const auto lastBefore = [&](int r) {
                    int best = -1;
                    for (int id : schedule.byRobot()[static_cast<std::size_t>(r)])
                        if (schedule.segments()[static_cast<std::size_t>(id)].end() <= c->t &&
                            (best < 0 || schedule.segments()[static_cast<std::size_t>(id)].end() >
                                             schedule.segments()[static_cast<std::size_t>(best)].end()))
                            best = id;
                    return best;
                };
                const int la = lastBefore(c->arm_a), lb = lastBefore(c->arm_b);
                const auto endOf = [&](int id) { return schedule.segments()[static_cast<std::size_t>(id)].end(); };
                const bool a_last = la >= 0 && (lb < 0 || endOf(la) >= endOf(lb));
                const int last = a_last ? la : lb;
                if (last >= 0) {
                    if (isMove(last)) {
                        mover_seg = last;
                        other_arm = a_last ? c->arm_b : c->arm_a;
                    } else {
                        stage_seg = last;
                        stage_arm = a_last ? c->arm_a : c->arm_b;
                    }
                }
            }
            if (mover_seg >= 0) {
                const auto &seg = schedule.segments()[static_cast<std::size_t>(mover_seg)].seg;
                MotionConstraint m;
                m.element = seg.element;
                m.consumer_arc = st_.steps[static_cast<std::size_t>(seg.step)].arc;
                m.context = schedule.robotElementAt(other_arm, c->t);
                m.region = armRegion(schedule, other_arm, c->t);
                m.origin = "layer4: h-h " + hg_.describe(seg.element) + " vs " + hg_.describe(m.context) +
                           " at t=" + std::to_string(c->t);
                out.log.push_back(m.origin);
                out.cm.push_back(std::move(m));
                return done();
            }
            // Two anchored stages (or a stage against an arm holding still).
            const int other = stage_arm == c->arm_a ? c->arm_b : c->arm_a;
            const int step = stage_seg >= 0 ? schedule.segments()[static_cast<std::size_t>(stage_seg)].seg.step : -1;
            if (step < 0) {
                out.log.push_back("layer4: unresolvable h-h between still arms");
                return done();
            }
            TaskConstraint t;
            t.kind = TaskConstraint::Kind::Frontier;
            t.pre = schedule.robotElementAt(other, c->t);
            t.post = hg_.objectHead(st_.steps[static_cast<std::size_t>(step)].arc);
            t.tentative = true;
            t.origin = "layer4: stage of " + hg_.describeArc(st_.steps[static_cast<std::size_t>(step)].arc) +
                       " blocked by " + hg_.describe(t.pre);
            out.log.push_back(t.origin);
            out.ct.push_back(std::move(t));
            return done();
        }
        // hyperarc-vertex: an arm against a resting object.
        ++out.hv;
        const int arm = c->arm_a >= 0 ? c->arm_a : c->arm_b;
        if (arm < 0 || c->object < 0) {
            out.log.push_back("layer4: vertex-vertex conflict between resting objects (layer 2 miss)");
            return done();
        }
        const auto rest = schedule.objectAt(c->object, c->t);
        const int seg_id = schedule.segmentAt(arm, c->t);
        const double radius = models_.scene().objects[static_cast<std::size_t>(c->object)].collision_radius;
        if (seg_id >= 0 && schedule.segments()[static_cast<std::size_t>(seg_id)].seg.kind == MotionSegment::Kind::Move) {
            const auto &seg = schedule.segments()[static_cast<std::size_t>(seg_id)].seg;
            MotionConstraint m;
            m.element = seg.element;
            m.consumer_arc = st_.steps[static_cast<std::size_t>(seg.step)].arc;
            m.context = rest.element;
            m.region = {{rest.position, radius + params_.region_margin}};
            m.origin = "layer4: h-v " + hg_.describe(seg.element) + " vs " + hg_.describe(rest.element) +
                       " at t=" + std::to_string(c->t);
            out.log.push_back(m.origin);
            out.cm.push_back(std::move(m));
            return done();
        }
        // An anchored stage (or an arm holding still) cannot be replanned under
        // C_M, so the order changes: tentative C_f(B@q, head).
        TaskConstraint t;
        t.kind = TaskConstraint::Kind::Frontier;
        t.pre = rest.element;
        t.tentative = true;
        if (seg_id >= 0) {
            const int step = schedule.segments()[static_cast<std::size_t>(seg_id)].seg.step;
            t.post = hg_.objectHead(st_.steps[static_cast<std::size_t>(step)].arc);
        } else {
            t.post = schedule.robotElementAt(arm, c->t);
        }
        t.origin = "layer4: h-v " + hg_.describe(schedule.robotElementAt(arm, c->t)) + " (anchored) vs " +
                   hg_.describe(rest.element) + " at t=" + std::to_string(c->t);
        out.log.push_back(t.origin);
        out.ct.push_back(std::move(t));
        return done();
    }
    out.log.push_back("layer4: iteration budget spent");
    return done();
}

} // namespace lazydash
