// Accuracy of the sphere planning model against the mesh verification model
// near contact: configurations perturbed from the mesh collision boundary.
#include "lazydash/robot/Ur5eModel.h"

#include "comotion/collision/CollisionChecker.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <functional>
#include <random>
#include <sstream>

using comotion::CollisionChecker;

namespace {

using Config = std::vector<double>;
using Valid = std::function<bool(const Config &)>;

Config randomConfig(std::mt19937 &rng, std::size_t n) {
    std::uniform_real_distribution<double> u(-M_PI, M_PI);
    Config q(n);
    for (auto &v : q)
        v = u(rng);
    return q;
}

Config lerp(const Config &a, const Config &b, double t) {
    Config q(a.size());
    for (std::size_t i = 0; i < a.size(); ++i)
        q[i] = a[i] + t * (b[i] - a[i]);
    return q;
}

/// Configurations within `sigma` (rad, per joint) of the boundary of `valid`.
std::vector<Config> nearBoundary(const Valid &valid, std::size_t dims, int boundary_points,
                                 int per_point, double sigma, std::mt19937 &rng) {
    std::vector<Config> out;
    std::normal_distribution<double> noise(0.0, sigma);
    while (static_cast<int>(out.size()) < boundary_points * per_point) {
        Config free = randomConfig(rng, dims), hit = randomConfig(rng, dims);
        if (!valid(free) || valid(hit))
            continue;
        double lo = 0.0, hi = 1.0; // valid at lo, invalid at hi
        for (int k = 0; k < 30; ++k) {
            const double mid = 0.5 * (lo + hi);
            (valid(lerp(free, hit, mid)) ? lo : hi) = mid;
        }
        const Config boundary = lerp(free, hit, lo);
        for (int p = 0; p < per_point; ++p) {
            Config q = boundary;
            for (auto &v : q)
                v += noise(rng);
            out.push_back(q);
        }
    }
    return out;
}

struct Agreement {
    int mesh_valid = 0, rejected = 0, mesh_invalid = 0, missed = 0;
    double missRate() const { return mesh_invalid ? double(missed) / mesh_invalid : 0.0; }
    double rejectRate() const { return mesh_valid ? double(rejected) / mesh_valid : 0.0; }
};

Agreement compare(const std::vector<Config> &configs, const Valid &mesh, const Valid &spheres) {
    Agreement a;
    for (const auto &q : configs) {
        const bool s = spheres(q);
        if (mesh(q)) {
            ++a.mesh_valid;
            a.rejected += !s;
        } else {
            ++a.mesh_invalid;
            a.missed += s;
        }
    }
    return a;
}

Config head(const Config &q) { return {q.begin(), q.begin() + 6}; }
Config tail(const Config &q) { return {q.begin() + 6, q.end()}; }

/// A copy of `robot` with collision padding `pad` (environment and other
/// robots only; self-collision checks ignore padding).
std::shared_ptr<comotion::RobotModel> padded(const comotion::RobotModel &robot, double pad) {
    auto copy = std::make_shared<comotion::RobotModel>(robot);
    copy->setCollisionPadding(pad);
    return copy;
}

void addTable(CollisionChecker &cc) {
    cc.setCylinderObstacles({{Eigen::Vector3d(0.0, 0.0, -0.03), Eigen::Vector3d::UnitZ(), 3.0, 0.02}});
    cc.setObstacles({{Eigen::Vector3d(0.45, 0.25, 0.3), 0.08}, {Eigen::Vector3d(-0.3, 0.5, 0.5), 0.06}});
}

} // namespace

TEST_CASE("Sphere model near contact: safety and cost by pad", "[.experimental]") {
    std::mt19937 rng(21);
    const double sigma = 0.01; // rad per joint: millimetres at the tool
    // Arms stand 2 cm above the table top, as in the scenes.
    const lazydash::BasePose at_origin{Eigen::Vector3d(0.0, 0.0, 0.02), 0.0};
    const auto mesh = lazydash::makeUr5e(at_origin, lazydash::Ur5eGeometry::Meshes);
    const auto sph = lazydash::makeUr5e(at_origin);
    CollisionChecker fcl(CollisionChecker::Backend::Fcl), spheres(CollisionChecker::Backend::Spheres);
    addTable(fcl);
    addTable(spheres);

    // Two arms 1.4 m apart, facing each other (as neighbours in the sort scenes).
    const lazydash::BasePose left{Eigen::Vector3d(-0.7, 0.0, 0.0), 0.0};
    const lazydash::BasePose right{Eigen::Vector3d(0.7, 0.0, 0.0), M_PI};
    const auto mesh_a = lazydash::makeUr5e(left, lazydash::Ur5eGeometry::Meshes);
    const auto mesh_b = lazydash::makeUr5e(right, lazydash::Ur5eGeometry::Meshes);
    const auto sph_a = lazydash::makeUr5e(left), sph_b = lazydash::makeUr5e(right);

    const Valid single_mesh = [&](const Config &q) { return fcl.isValidSingleFull(*mesh, q); };
    const Valid env_mesh = [&](const Config &q) { return fcl.isValidSingle(*mesh, q); };
    const Valid self_mesh = [&](const Config &q) { return fcl.isSelfCollisionFree(*mesh, q); };
    const Valid pair_mesh = [&](const Config &q) {
        return fcl.isValidPair(*mesh_a, head(q), *mesh_b, tail(q));
    };
    const auto single_near = nearBoundary(single_mesh, 6, 600, 5, sigma, rng);
    const auto env_near = nearBoundary(env_mesh, 6, 600, 5, sigma, rng);
    const auto self_near = nearBoundary(self_mesh, 6, 600, 5, sigma, rng);
    const auto pair_near = nearBoundary(pair_mesh, 12, 600, 5, sigma, rng);
    std::vector<Config> single_random, pair_random;
    for (int i = 0; i < 3000; ++i) {
        single_random.push_back(randomConfig(rng, 6));
        pair_random.push_back(randomConfig(rng, 12));
    }

    // unsafe: P(mesh collision | spheres accept) near contact.
    // cost: mesh-valid random configurations the spheres reject.
    const auto unsafe = [](const Agreement &a) {
        const int accepted = a.mesh_valid - a.rejected + a.missed;
        return accepted ? 100.0 * a.missed / accepted : 0.0;
    };
    std::ostringstream table;
    table << "pad (env + inter-robot; self unpadded)\n"
          << "pad    | single: unsafe near contact, cost random | pair: unsafe near contact, cost random\n";
    for (double pad : {0.0, 0.003, 0.006, 0.01, 0.015}) {
        const auto planning = padded(*sph, pad);
        const auto pa = padded(*sph_a, pad), pb = padded(*sph_b, pad);
        const Valid single_sph = [&](const Config &q) {
            return spheres.isValidSingleFull(*planning, q);
        };
        const Valid pair_sph = [&](const Config &q) {
            return spheres.isValidPair(*pa, head(q), *pb, tail(q));
        };
        const auto sn = compare(single_near, single_mesh, single_sph);
        const auto sr = compare(single_random, single_mesh, single_sph);
        const auto pn = compare(pair_near, pair_mesh, pair_sph);
        const auto pr = compare(pair_random, pair_mesh, pair_sph);
        const auto en = compare(env_near, env_mesh, [&](const Config &q) {
            return spheres.isValidSingle(*planning, q);
        });
        const auto fn = compare(self_near, self_mesh, [&](const Config &q) {
            return spheres.isSelfCollisionFree(*planning, q);
        });
        char line[240];
        std::snprintf(line, sizeof line,
                      "%.3f  | %6.1f%% %6.1f%%                        | %6.1f%% %6.1f%%"
                      "   | env %5.1f%%  self %5.1f%%\n",
                      pad, unsafe(sn), 100 * sr.rejectRate(), unsafe(pn), 100 * pr.rejectRate(),
                      unsafe(en), unsafe(fn));
        table << line;
    }
    WARN("sigma " << sigma << " rad, " << single_near.size() << " near-contact configurations each\n"
         << table.str());
}
