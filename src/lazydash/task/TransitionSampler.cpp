#include "lazydash/task/TransitionSampler.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lazydash {

namespace {

double maxJointDiff(const std::vector<double> &a, const std::vector<double> &b) {
    double d = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        d = std::max(d, std::abs(a[i] - b[i]));
    return d;
}

int checksFor(const std::vector<double> &a, const std::vector<double> &b) {
    return std::max(1, static_cast<int>(std::ceil(maxJointDiff(a, b) / MeshVerifier::kCheckStep)));
}

/// Same-branch IK solutions for poses 7 cm apart differ by much less than this.
constexpr double kSameBranch = 1.0;

/// Probability that a sample tries IK branches nearest the start configuration first.
constexpr double kNearestBranchFirst = 0.9;

} // namespace

TransitionSampler::TransitionSampler(const TaskHypergraph &hg, const Capabilities &caps,
                                     const BodyModels &models, const MeshVerifier &mesh,
                                     std::uint64_t seed)
    : hg_(hg), caps_(caps), models_(models), mesh_(mesh),
      spheres_(comotion::CollisionChecker::Backend::Spheres), rng_(seed), move_out_rng_(seed ^ 0x6d6f76656f7574ull) {
    models_.scene().applyObstacles(spheres_);
}

bool TransitionSampler::lineValid(const comotion::RobotModel &model,
                                  const comotion::RobotModel &mesh_model,
                                  const std::vector<double> &a,
                                  const std::vector<double> &b) const {
    return spheres_.isMotionValid(model, a, b, checksFor(a, b)) &&
           mesh_.pathValid(mesh_model, {a, b});
}

bool TransitionSampler::pairLineValid(const comotion::RobotModel &moving,
                                      const std::vector<double> &a, const std::vector<double> &b,
                                      const comotion::RobotModel &fixed,
                                      const std::vector<double> &q_fixed,
                                      const comotion::CollisionChecker &checker) const {
    const int checks = checksFor(a, b);
    std::vector<double> q(a.size());
    for (int i = 0; i <= checks; ++i) {
        const double t = static_cast<double>(i) / checks;
        for (std::size_t j = 0; j < q.size(); ++j)
            q[j] = a[j] + t * (b[j] - a[j]);
        if (!checker.isValidPair(moving, q, fixed, q_fixed))
            return false;
    }
    return true;
}

std::vector<std::pair<std::vector<double>, std::vector<double>>>
TransitionSampler::stagePairs(int robot, int grasp, const Eigen::Isometry3d &object_pose) {
    auto approach = caps_.holdConfigs(robot, grasp, object_pose, GraspSet::Stage::Approach);
    const auto at_grasp = caps_.holdConfigs(robot, grasp, object_pose, GraspSet::Stage::Grasp);
    std::vector<std::pair<std::vector<double>, std::vector<double>>> pairs;
    for (auto &a : approach) {
        const std::vector<double> *best = nullptr;
        for (const auto &g : at_grasp)
            if (maxJointDiff(a, g) < kSameBranch && (!best || maxJointDiff(a, g) < maxJointDiff(a, *best)))
                best = &g;
        if (best)
            pairs.emplace_back(std::move(a), *best);
    }
    // Mostly nearest the start configuration first (short moves between
    // anchors), otherwise random so every branch stays possible.
    std::shuffle(pairs.begin(), pairs.end(), rng_);
    if (std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < kNearestBranchFirst) {
        const auto &home = models_.scene().arms.at(static_cast<std::size_t>(robot)).start;
        const auto dist = [&](const std::vector<double> &q) {
            double d = 0.0;
            for (std::size_t j = 0; j < q.size(); ++j)
                d += (q[j] - home[j]) * (q[j] - home[j]);
            return d;
        };
        std::stable_sort(pairs.begin(), pairs.end(),
                         [&](const auto &x, const auto &y) { return dist(x.first) < dist(y.first); });
    }
    return pairs;
}

std::optional<TransitionAnchors> TransitionSampler::sample(int arc) {
    const auto type = hg_.graph().arc(arc).data.type;
    if (type == TransitionType::Pick || type == TransitionType::Place) {
        auto first = pickPlace(arc, 1);
        if (first.empty())
            return std::nullopt;
        return std::move(first.front());
    }
    if (type == TransitionType::Handover)
        return sampleHandover(arc);
    return std::nullopt;
}

std::vector<TransitionAnchors> TransitionSampler::pickPlaceBranches(int arc) {
    return pickPlace(arc, std::numeric_limits<std::size_t>::max());
}

std::vector<TransitionAnchors> TransitionSampler::pickPlace(int arc, std::size_t limit) {
    const auto &t = hg_.graph().arc(arc).data;
    TransitionAnchors out;
    out.arc = arc;
    out.type = t.type;
    out.object = t.object;
    out.robot = t.robot;
    out.grasp = t.grasp;
    out.object_pose = hg_.locations()[static_cast<std::size_t>(t.location)].poses[static_cast<std::size_t>(t.pose)];
    const auto &bare = *models_.arm(t.robot);
    const auto &bare_mesh = *models_.armMesh(t.robot);
    const auto held = models_.holding(t.robot, t.grasp);
    const auto held_mesh = models_.holdingMesh(t.robot, t.grasp);
    std::vector<TransitionAnchors> found;
    for (auto &[a, g] : stagePairs(t.robot, t.grasp, out.object_pose)) {
        if (found.size() >= limit)
            break;
        // Pick: approach hand free, lift holding. Place: descend holding, retreat hand free.
        const bool free_ok = lineValid(bare, bare_mesh, a, g);
        const bool held_ok = lineValid(*held, *held_mesh, g, a);
        if (free_ok && held_ok) {
            out.q_approach = a;
            out.q_grasp = g;
            found.push_back(out);
        }
    }
    return found;
}

Eigen::Isometry3d TransitionSampler::sampleHandoverPose(int giver, int giver_grasp, int receiver) {
    const auto &scene = models_.scene();
    const auto &p = scene.params;
    const Eigen::Vector3d b1 = scene.arms[static_cast<std::size_t>(giver)].base.position;
    const Eigen::Vector3d b2 = scene.arms[static_cast<std::size_t>(receiver)].base.position;
    std::uniform_real_distribution<double> unit(0.0, 1.0), sym(-1.0, 1.0);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = 0.5 * (b1 + b2) +
                         Eigen::Vector3d(p.handover_xy_half_width * sym(rng_), p.handover_xy_half_width * sym(rng_),
                                         p.handover_z_min + (p.handover_z_max - p.handover_z_min) * unit(rng_));
    if (unit(rng_) < 0.1) { // anywhere in SO(3), so every orientation stays possible
        Eigen::Quaterniond q(Eigen::Vector4d(sym(rng_), sym(rng_), sym(rng_), sym(rng_)));
        pose.linear() = q.normalized().toRotationMatrix();
        return pose;
    }
    // Mostly: the giver's tool points from the giver toward the receiver, tilted up to 30 degrees.
    Eigen::Vector3d d = b2 - b1;
    d.z() = 0.0;
    d.normalize();
    Eigen::Vector3d tilt_axis = Eigen::Vector3d(sym(rng_), sym(rng_), sym(rng_)).cross(d);
    if (tilt_axis.norm() > 1e-6)
        d = Eigen::AngleAxisd(0.52 * unit(rng_), tilt_axis.normalized()) * d;
    const auto &grasp = models_.graspSet(giver).grasp(static_cast<std::size_t>(giver_grasp));
    const Eigen::Vector3d a1 = grasp.ee_T_object.inverse().linear() * Eigen::Vector3d::UnitZ();
    const Eigen::Matrix3d align = Eigen::Quaterniond::FromTwoVectors(a1, d).toRotationMatrix();
    pose.linear() = Eigen::AngleAxisd(M_PI * sym(rng_), d).toRotationMatrix() * align;
    return pose;
}

std::optional<TransitionAnchors> TransitionSampler::sampleHandover(int arc) {
    const auto &t = hg_.graph().arc(arc).data;
    const auto key = std::make_tuple(t.robot, t.grasp, t.robot2, t.grasp2);
    if (auto it = handover_failed_.find(key); it != handover_failed_.end() && it->second >= budget_scale_)
        return std::nullopt; // already failed at this budget
    TransitionAnchors out;
    out.arc = arc;
    out.type = t.type;
    out.object = t.object;
    out.robot = t.robot;
    out.grasp = t.grasp;
    out.robot2 = t.robot2;
    out.grasp2 = t.grasp2;
    const auto &giver = *models_.arm(t.robot);
    const auto &giver_mesh = *models_.armMesh(t.robot);
    const auto &receiver = *models_.arm(t.robot2);
    const auto &receiver_mesh = *models_.armMesh(t.robot2);
    const auto giver_held = models_.holding(t.robot, t.grasp);
    const auto giver_held_mesh = models_.holdingMesh(t.robot, t.grasp);
    const auto receiver_held = models_.holding(t.robot2, t.grasp2);
    const auto receiver_held_mesh = models_.holdingMesh(t.robot2, t.grasp2);
    const auto &receiver_grasp = models_.graspSet(t.robot2).grasp(static_cast<std::size_t>(t.grasp2));
    const auto &giver_grasp = models_.graspSet(t.robot).grasp(static_cast<std::size_t>(t.grasp));

    // Exchange contacts: the receiver's gripper may touch the giver's held
    // object while closing, the giver's the receiver's while backing off.
    comotion::RobotModel receiver_c(receiver), receiver_mesh_c(receiver_mesh);
    allowExchangeContact(receiver_c, receiver_grasp, *giver_held, giver_grasp);
    allowExchangeContact(receiver_mesh_c, receiver_grasp, *giver_held_mesh, giver_grasp);
    comotion::RobotModel giver_c(giver), giver_mesh_c(giver_mesh);
    allowExchangeContact(giver_c, giver_grasp, *receiver_held, receiver_grasp);
    allowExchangeContact(giver_mesh_c, giver_grasp, *receiver_held_mesh, receiver_grasp);

    // Checks run cheapest first: object pose, sphere checks, then mesh checks
    // (each stage pair at most once).
    const auto &object = *models_.object(t.object);
    const int attempts = static_cast<int>(models_.scene().params.handover_attempts * budget_scale_);
    for (int attempt = 0; attempt < attempts; ++attempt) {
        out.object_pose = sampleHandoverPose(t.robot, t.grasp, t.robot2);
        const Eigen::Vector3d at = out.object_pose.translation();
        // No robot_range test here: IK decides whether both arms reach.
        if (!spheres_.isValidSingle(object, {at.x(), at.y(), at.z()}))
            continue;
        auto giver_pairs = stagePairs(t.robot, t.grasp, out.object_pose);
        std::erase_if(giver_pairs, [&](const auto &p) {
            return !spheres_.isValidSingleFull(*giver_held, p.second) ||
                   !spheres_.isMotionValid(giver, p.second, p.first, checksFor(p.second, p.first));
        });
        if (giver_pairs.empty())
            continue;
        auto receiver_pairs = stagePairs(t.robot2, t.grasp2, out.object_pose);
        std::erase_if(receiver_pairs, [&](const auto &p) {
            return !spheres_.isValidSingleFull(*receiver_held, p.second) ||
                   !spheres_.isMotionValid(receiver, p.first, p.second, checksFor(p.first, p.second));
        });
        // Mesh verdicts per stage pair: -1 unknown, 0 invalid, 1 valid.
        std::vector<signed char> giver_mesh_ok(giver_pairs.size(), -1), receiver_mesh_ok(receiver_pairs.size(), -1);
        for (std::size_t i = 0; i < giver_pairs.size(); ++i) {
            const auto &[g_back, g_hold] = giver_pairs[i];
            for (std::size_t j = 0; j < receiver_pairs.size() && giver_mesh_ok[i] != 0; ++j) {
                const auto &[r_app, r_grasp] = receiver_pairs[j];
                if (receiver_mesh_ok[j] == 0)
                    continue;
                // The receiver approaches the giver holding still; then the
                // giver backs off from the receiver holding still.
                if (!pairLineValid(receiver_c, r_app, r_grasp, *giver_held, g_hold, spheres_) ||
                    !pairLineValid(giver_c, g_hold, g_back, *receiver_held, r_grasp, spheres_))
                    continue;
                if (giver_mesh_ok[i] < 0)
                    giver_mesh_ok[i] = mesh_.pathValid(*giver_held_mesh, {g_hold}) &&
                                       mesh_.pathValid(giver_mesh, {g_hold, g_back});
                if (!giver_mesh_ok[i])
                    break;
                if (receiver_mesh_ok[j] < 0)
                    receiver_mesh_ok[j] = mesh_.pathValid(receiver_mesh, {r_app, r_grasp}) &&
                                          mesh_.pathValid(*receiver_held_mesh, {r_grasp});
                if (!receiver_mesh_ok[j])
                    continue;
                if (!pairLineValid(receiver_mesh_c, r_app, r_grasp, *giver_held_mesh, g_hold, mesh_.checker()) ||
                    !pairLineValid(giver_mesh_c, g_hold, g_back, *receiver_held_mesh, r_grasp, mesh_.checker()))
                    continue;
                out.q_grasp = g_hold;
                out.q_approach = g_back;
                out.q2_approach = r_app;
                out.q2_grasp = r_grasp;
                return out;
            }
        }
    }
    handover_failed_[key] = budget_scale_;
    return std::nullopt;
}

std::optional<Eigen::Isometry3d> TransitionSampler::sampleMoveOut(int object,
                                                                  const std::vector<Eigen::Vector3d> &occupied,
                                                                  int attempts) {
    const auto &scene = models_.scene();
    std::vector<const PlacementRegion *> regions;
    for (const auto &r : scene.placement_regions)
        if (r.object == object)
            regions.push_back(&r);
    if (regions.empty()) // no region of its own: the shared ones
        for (const auto &r : scene.placement_regions)
            if (r.object < 0)
                regions.push_back(&r);
    if (regions.empty()) // nor shared ones: any surface the scene offers
        for (const auto &r : scene.placement_regions)
            regions.push_back(&r);
    if (regions.empty())
        return std::nullopt;
    const double clearance = 2 * scene.objects[static_cast<std::size_t>(object)].collision_radius + 0.01;
    const auto &body = *models_.object(object);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int attempt = 0; attempt < attempts; ++attempt) {
        const auto &region = *regions[static_cast<std::size_t>(move_out_rng_() % regions.size())];
        Eigen::Vector3d p;
        for (int k = 0; k < 3; ++k)
            p[k] = region.min[k] + (region.max[k] - region.min[k]) * unit(move_out_rng_);
        const double yaw = region.yaw_min + (region.yaw_max - region.yaw_min) * unit(move_out_rng_);
        if (!spheres_.isValidSingle(body, {p.x(), p.y(), p.z()}))
            continue;
        bool clear = true;
        for (const auto &q : occupied)
            clear = clear && (q - p).norm() >= clearance;
        if (!clear)
            continue;
        const Eigen::Isometry3d pose = poseFromRpy(p, Eigen::Vector3d(0.0, 0.0, yaw));
        // Require pick/place anchors for some robot and grasp, not only IK.
        for (int r = 0; r < caps_.numRobots(); ++r)
            for (int g = 0; g < caps_.numGrasps(); ++g) {
                if (!caps_.canHold(r, g, pose))
                    continue;
                const auto held = models_.holding(r, g);
                const auto held_mesh = models_.holdingMesh(r, g);
                for (const auto &[a, q] : stagePairs(r, g, pose))
                    if (lineValid(*models_.arm(r), *models_.armMesh(r), a, q) && lineValid(*held, *held_mesh, q, a))
                        return pose;
            }
    }
    return std::nullopt;
}

bool TransitionSampler::clearOfObject(int robot, int holding_grasp, const std::vector<double> &q,
                                      int object, const Eigen::Vector3d &position) const {
    const std::vector<double> at{position.x(), position.y(), position.z()};
    const auto &obj = *models_.object(object);
    const std::shared_ptr<const comotion::RobotModel> planning =
        holding_grasp < 0 ? models_.arm(robot) : models_.holding(robot, holding_grasp);
    const std::shared_ptr<const comotion::RobotModel> verification =
        holding_grasp < 0 ? models_.armMesh(robot) : models_.holdingMesh(robot, holding_grasp);
    return spheres_.isValidPair(*planning, q, obj, at) && mesh_.checker().isValidPair(*verification, q, obj, at);
}

} // namespace lazydash
