#include "lazydash/motion/LazyRoadmap.h"

#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/MultiRobotProblem.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

namespace lazydash {

namespace {

using Clock = std::chrono::steady_clock;

double seconds(Clock::time_point since) {
    return std::chrono::duration<double>(Clock::now() - since).count();
}

} // namespace

LazyRoadmap::LazyRoadmap(std::shared_ptr<const comotion::RobotModel> model,
                         std::shared_ptr<const comotion::RobotModel> mesh_model,
                         const comotion::CollisionChecker &scene_checker, const MeshVerifier &mesh,
                         std::vector<double> lower, std::vector<double> upper, std::uint64_t seed,
                         RoadmapParams params)
    : model_(std::move(model)), mesh_model_(std::move(mesh_model)),
      scene_checker_(scene_checker), mesh_(mesh), lower_(std::move(lower)),
      upper_(std::move(upper)), rng_(seed), params_(params) {
    Config q;
    for (int i = 0, added = 0; added < params_.initial_samples && i < 50 * params_.initial_samples; ++i)
        if (sampleValid(q, nullptr, 0.0)) {
            addVertex(q);
            ++added;
        }
}

double LazyRoadmap::distance(const Config &a, const Config &b) {
    double d = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        d += (a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(d);
}

bool LazyRoadmap::sphereMotionValid(const Config &a, const Config &b,
                                    const comotion::CollisionChecker &checker) const {
    double span = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        span = std::max(span, std::abs(b[i] - a[i]));
    const int checks = std::max(1, static_cast<int>(std::ceil(span / params_.edge_step)));
    return checker.isMotionValid(*model_, a, b, checks);
}

bool LazyRoadmap::configValid(const Config &q,
                              const std::vector<comotion::ObstacleSphere> &extra) const {
    if (!scene_checker_.isValidSingleFull(*model_, q) ||
        !mesh_.checker().isValidSingleFull(*mesh_model_, q))
        return false;
    if (extra.empty())
        return true;
    comotion::CollisionChecker with_extra(scene_checker_);
    auto obstacles = scene_checker_.obstacles();
    obstacles.insert(obstacles.end(), extra.begin(), extra.end());
    with_extra.setObstacles(obstacles);
    return with_extra.isValidSingle(*model_, q);
}

bool LazyRoadmap::motionValid(const Config &a, const Config &b,
                              const std::vector<comotion::ObstacleSphere> &extra) const {
    if (!sphereMotionValid(a, b, scene_checker_) || !mesh_.pathValid(*mesh_model_, {a, b}))
        return false;
    if (extra.empty())
        return true;
    comotion::CollisionChecker with_extra(scene_checker_);
    auto obstacles = scene_checker_.obstacles();
    obstacles.insert(obstacles.end(), extra.begin(), extra.end());
    with_extra.setObstacles(obstacles);
    return sphereMotionValid(a, b, with_extra);
}

comotion::Path LazyRoadmap::shortcut(const std::vector<int> &vertex_path,
                                     const comotion::CollisionChecker *with_extra) {
    std::vector<int> kept{vertex_path.front()};
    int checks = params_.shortcut_checks;
    for (std::size_t i = 0; i + 1 < vertex_path.size();) {
        std::size_t j = vertex_path.size() - 1;
        for (; j > i + 1 && checks > 0; --j, --checks) {
            const auto &qa = vertices_[static_cast<std::size_t>(vertex_path[i])];
            const auto &qb = vertices_[static_cast<std::size_t>(vertex_path[j])];
            if (sphereMotionValid(qa, qb, scene_checker_) && mesh_.pathValid(*mesh_model_, {qa, qb})) {
                // Valid on both levels: a roadmap edge, whatever `extra` says.
                const int e = edgeBetween(vertex_path[i], vertex_path[j]);
                if (e >= 0)
                    edges_[static_cast<std::size_t>(e)].status = Status::Valid;
                else
                    addEdge(vertex_path[i], vertex_path[j], Status::Valid);
                if (!with_extra || sphereMotionValid(qa, qb, *with_extra))
                    break;
            }
        }
        if (checks == 0 && j > i + 1)
            j = i + 1;
        kept.push_back(vertex_path[j]);
        i = j;
    }
    comotion::Path path;
    for (int v : kept)
        path.push_back(vertices_[static_cast<std::size_t>(v)]);
    return path;
}

bool LazyRoadmap::sampleValid(Config &q, const Config *around, double sigma) {
    std::normal_distribution<double> noise(0.0, sigma);
    q.assign(lower_.size(), 0.0);
    for (int attempt = 0; attempt < 100; ++attempt) {
        for (std::size_t i = 0; i < q.size(); ++i) {
            if (around)
                q[i] = std::clamp((*around)[i] + noise(rng_), lower_[i], upper_[i]);
            else
                q[i] = std::uniform_real_distribution<double>(lower_[i], upper_[i])(rng_);
        }
        if (configValid(q))
            return true;
    }
    return false;
}

int LazyRoadmap::findVertex(const Config &q) const {
    for (std::size_t v = 0; v < vertices_.size(); ++v)
        if (distance(vertices_[v], q) < 1e-9)
            return static_cast<int>(v);
    return -1;
}

int LazyRoadmap::edgeBetween(int a, int b) const {
    for (int e : adjacency_[static_cast<std::size_t>(a)]) {
        const auto &edge = edges_[static_cast<std::size_t>(e)];
        if ((edge.a == a && edge.b == b) || (edge.a == b && edge.b == a))
            return e;
    }
    return -1;
}

void LazyRoadmap::addEdge(int a, int b, Status status) {
    if (a == b || edgeBetween(a, b) >= 0)
        return;
    const int id = static_cast<int>(edges_.size());
    edges_.push_back({a, b, distance(vertices_[static_cast<std::size_t>(a)],
                                     vertices_[static_cast<std::size_t>(b)]),
                      status});
    adjacency_[static_cast<std::size_t>(a)].push_back(id);
    adjacency_[static_cast<std::size_t>(b)].push_back(id);
}

int LazyRoadmap::addVertex(const Config &q) {
    const int id = static_cast<int>(vertices_.size());
    std::vector<std::pair<double, int>> near;
    for (std::size_t v = 0; v < vertices_.size(); ++v)
        near.emplace_back(distance(vertices_[v], q), static_cast<int>(v));
    std::sort(near.begin(), near.end());
    vertices_.push_back(q);
    adjacency_.emplace_back();
    for (std::size_t i = 0; i < near.size() && i < static_cast<std::size_t>(params_.k); ++i)
        addEdge(id, near[i].second, Status::Unknown);
    return id;
}

std::optional<std::vector<int>>
LazyRoadmap::shortestPath(int from, int to, const std::vector<char> &blocked) const {
    const std::size_t n = vertices_.size();
    std::vector<double> g(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
    g[static_cast<std::size_t>(from)] = 0.0;
    open.push({distance(vertices_[static_cast<std::size_t>(from)], vertices_[static_cast<std::size_t>(to)]), from});
    while (!open.empty()) {
        const auto [f, v] = open.top();
        open.pop();
        if (v == to)
            break;
        const double gv = g[static_cast<std::size_t>(v)];
        if (f - distance(vertices_[static_cast<std::size_t>(v)], vertices_[static_cast<std::size_t>(to)]) > gv + 1e-12)
            continue;
        for (int e : adjacency_[static_cast<std::size_t>(v)]) {
            const auto &edge = edges_[static_cast<std::size_t>(e)];
            if (edge.status == Status::Invalid || (static_cast<std::size_t>(e) < blocked.size() && blocked[static_cast<std::size_t>(e)]))
                continue;
            const int w = edge.a == v ? edge.b : edge.a;
            const double gw = gv + edge.length;
            if (gw < g[static_cast<std::size_t>(w)]) {
                g[static_cast<std::size_t>(w)] = gw;
                parent[static_cast<std::size_t>(w)] = v;
                open.push({gw + distance(vertices_[static_cast<std::size_t>(w)], vertices_[static_cast<std::size_t>(to)]), w});
            }
        }
    }
    if (!std::isfinite(g[static_cast<std::size_t>(to)]))
        return std::nullopt;
    std::vector<int> path{to};
    while (path.back() != from)
        path.push_back(parent[static_cast<std::size_t>(path.back())]);
    std::reverse(path.begin(), path.end());
    return path;
}

std::optional<comotion::Path> LazyRoadmap::rrt(const Config &from, const Config &to,
                                               const std::vector<comotion::ObstacleSphere> &extra) {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(comotion::CollisionChecker::Backend::Spheres);
    problem->addRobot(std::make_shared<comotion::RobotModel>(*model_), from, to);
    auto obstacles = scene_checker_.obstacles();
    obstacles.insert(obstacles.end(), extra.begin(), extra.end());
    problem->setObstacles(obstacles);
    problem->setCylinderObstacles(scene_checker_.cylinders());
    problem->setCspaceBoundsForRobot(0, lower_, upper_);
    comotion::CompositeRRT planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(static_cast<unsigned>(rng_()));
    if (planner.solve(params_.rrt_time) != ompl::base::PlannerStatus::EXACT_SOLUTION)
        return std::nullopt;
    auto paths = planner.getSolutionPaths();
    if (paths.empty() || !mesh_.pathValid(*mesh_model_, paths[0]))
        return std::nullopt;
    comotion::Path path;
    for (const auto &q : paths[0])
        path.push_back(q);
    return path;
}

std::optional<comotion::Path> LazyRoadmap::query(const Config &from, const Config &to,
                                                 const std::vector<comotion::ObstacleSphere> &extra,
                                                 QueryStats *stats) {
    const auto t0 = Clock::now();
    QueryStats local;
    QueryStats &st = stats ? *stats : local;
    st = {};
    if (!configValid(from, extra) || !configValid(to, extra)) {
        st.seconds = seconds(t0);
        return std::nullopt;
    }
    int s = findVertex(from), g = findVertex(to);
    if (s < 0)
        s = addVertex(from);
    if (g < 0)
        g = addVertex(to);

    std::optional<comotion::CollisionChecker> with_extra;
    if (!extra.empty()) {
        with_extra.emplace(scene_checker_);
        auto obstacles = scene_checker_.obstacles();
        obstacles.insert(obstacles.end(), extra.begin(), extra.end());
        with_extra->setObstacles(obstacles);
    }
    std::vector<char> blocked; // edges failing only against `extra`, this query
    std::vector<Config> trouble; // midpoints of edges invalidated in this query
    std::vector<char> extra_checked;
    int round = 0;
    bool rrt_tried = false;

    const int rounds = static_cast<int>(params_.rounds * scale_);
    while (seconds(t0) < params_.time_limit * scale_) {
        blocked.resize(edges_.size(), 0);
        extra_checked.resize(edges_.size(), 0);
        const auto vertex_path = shortestPath(s, g, blocked);
        if (vertex_path) {
            bool valid = true;
            // Rung 1: validate the candidate's edges lazily, sphere level first.
            for (std::size_t i = 0; i + 1 < vertex_path->size() && valid; ++i) {
                const int e = edgeBetween((*vertex_path)[i], (*vertex_path)[i + 1]);
                auto &edge = edges_[static_cast<std::size_t>(e)];
                const auto &qa = vertices_[static_cast<std::size_t>(edge.a)];
                const auto &qb = vertices_[static_cast<std::size_t>(edge.b)];
                if (edge.status == Status::Unknown) {
                    ++st.edges_checked;
                    edge.status = sphereMotionValid(qa, qb, scene_checker_) ? Status::Valid : Status::Invalid;
                }
                if (edge.status == Status::Valid && with_extra && !extra_checked[static_cast<std::size_t>(e)]) {
                    extra_checked[static_cast<std::size_t>(e)] = 1;
                    if (!sphereMotionValid(qa, qb, *with_extra))
                        blocked[static_cast<std::size_t>(e)] = 1;
                }
                if (edge.status == Status::Invalid || blocked[static_cast<std::size_t>(e)]) {
                    Config mid(qa.size());
                    for (std::size_t j = 0; j < mid.size(); ++j)
                        mid[j] = 0.5 * (qa[j] + qb[j]);
                    trouble.push_back(mid);
                    valid = false;
                }
            }
            if (!valid)
                continue;
            // Mesh acceptance of the sphere-valid path, edge by edge.
            comotion::Path path;
            for (int v : *vertex_path)
                path.push_back(vertices_[static_cast<std::size_t>(v)]);
            const int bad = mesh_.firstInvalidSegment(*mesh_model_, path);
            if (bad >= 0) {
                ++st.mesh_rejections;
                edges_[static_cast<std::size_t>(edgeBetween((*vertex_path)[static_cast<std::size_t>(bad)],
                                                            (*vertex_path)[static_cast<std::size_t>(bad) + 1]))]
                    .status = Status::Invalid;
                continue;
            }
            st.seconds = seconds(t0);
            return shortcut(*vertex_path, with_extra ? &*with_extra : nullptr);
        }
        // No candidate path: climb the ladder.
        if (round < rounds) {
            ++round;
            Config q;
            if (trouble.empty()) // nothing invalidated yet: sample along the straight line
                for (int i = 1; i <= 4; ++i) {
                    Config mid(from.size());
                    for (std::size_t j = 0; j < mid.size(); ++j)
                        mid[j] = from[j] + (to[j] - from[j]) * i / 5.0;
                    trouble.push_back(mid);
                }
            for (int i = 0; i < params_.local_samples; ++i) {
                const auto &around = trouble[std::uniform_int_distribution<std::size_t>(0, trouble.size() - 1)(rng_)];
                if (sampleValid(q, &around, params_.local_sigma)) {
                    addVertex(q);
                    ++st.local_samples;
                }
            }
            for (int i = 0; i < params_.global_samples; ++i)
                if (sampleValid(q, nullptr, 0.0)) {
                    addVertex(q);
                    ++st.global_samples;
                }
            continue;
        }
        if (!rrt_tried) {
            rrt_tried = true;
            st.rrt_used = true;
            if (auto path = rrt(from, to, extra)) {
                std::vector<int> ids{s};
                for (std::size_t i = 1; i < path->size(); ++i) {
                    int v = findVertex((*path)[i]);
                    if (v < 0) {
                        v = static_cast<int>(vertices_.size());
                        vertices_.push_back((*path)[i]);
                        adjacency_.emplace_back();
                    }
                    addEdge(ids.back(), v, Status::Valid);
                    ids.push_back(v);
                }
                st.seconds = seconds(t0);
                return shortcut(ids, with_extra ? &*with_extra : nullptr);
            }
            continue;
        }
        break;
    }
    st.seconds = seconds(t0);
    return std::nullopt;
}

} // namespace lazydash
