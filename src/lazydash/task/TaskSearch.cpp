#include "lazydash/task/TaskSearch.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace lazydash {

namespace {

std::uint64_t mix(std::uint64_t x) {
    x += 0x9e3779b97f4a7c15ull;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
    return x ^ (x >> 31);
}

} // namespace

std::vector<int> TaskSchedule::consumers() const {
    std::vector<int> consumer(vertices.size(), -1);
    for (std::size_t s = 0; s < steps.size(); ++s)
        for (int v : steps[s].tail)
            consumer[static_cast<std::size_t>(v)] = static_cast<int>(s);
    return consumer;
}

TaskSearch::TaskSearch(const TaskHypergraph &hg, const Heuristic &h, const DurationModel &durations)
    : hg_(hg), base_h_(h), h_ptr_(&h), durations_(durations) {
    const int n = hg_.graph().numVertices();
    words_ = static_cast<std::size_t>((n + 63) / 64);
    std::mt19937_64 rng(0x1a2b3c4d);
    for (int i = 0; i < n; ++i)
        element_salt_.push_back(rng());
}

void TaskSearch::reset() {
    schedule_ = {};
    te_on_element_.assign(static_cast<std::size_t>(hg_.graph().numVertices()), -1);
    tail_on_frontier_.assign(static_cast<std::size_t>(hg_.graph().numArcs()), 0);
    frontier_key_ = 0;
    stats_ = {};
    budget_exhausted_ = false;
}

int TaskSearch::addVertex(int element, int producer, std::vector<std::uint64_t> history) {
    TEVertex v;
    v.element = element;
    v.producer = producer;
    std::uint64_t hash = 0;
    for (std::size_t w = 0; w < history.size(); ++w)
        hash = mix(hash ^ mix(history[w] + w));
    v.history = std::move(history);
    v.history_hash = hash;
    schedule_.vertices.push_back(std::move(v));
    return static_cast<int>(schedule_.vertices.size()) - 1;
}

void TaskSearch::putOnFrontier(int te) {
    const auto &v = schedule_.vertices[static_cast<std::size_t>(te)];
    te_on_element_[static_cast<std::size_t>(v.element)] = te;
    for (int a : hg_.graph().outArcs(v.element))
        ++tail_on_frontier_[static_cast<std::size_t>(a)];
    frontier_key_ += mix(element_salt_[static_cast<std::size_t>(v.element)] ^ (key_histories_ ? v.history_hash : 0));
}

void TaskSearch::takeFromFrontier(int te) {
    const auto &v = schedule_.vertices[static_cast<std::size_t>(te)];
    te_on_element_[static_cast<std::size_t>(v.element)] = -1;
    for (int a : hg_.graph().outArcs(v.element))
        --tail_on_frontier_[static_cast<std::size_t>(a)];
    frontier_key_ -= mix(element_salt_[static_cast<std::size_t>(v.element)] ^ (key_histories_ ? v.history_hash : 0));
}

std::vector<std::uint64_t> TaskSearch::mergedHistory(const std::vector<int> &tail_te) const {
    std::vector<std::uint64_t> history(words_, 0);
    for (int te : tail_te) {
        const auto &v = schedule_.vertices[static_cast<std::size_t>(te)];
        for (std::size_t w = 0; w < v.history.size(); ++w)
            history[w] |= v.history[w];
        history[static_cast<std::size_t>(v.element) / 64] |= 1ull << (v.element % 64);
    }
    return history;
}

double TaskSearch::finishTime(int arc, double not_before) const {
    const auto &d = durations_;
    const auto &t = hg_.graph().arc(arc).data;
    const Eigen::Vector3d p = d.point(arc);
    // An arm moves once free; its stages start when it and the object are there.
    const auto arrival = [&](int robot, const TEVertex &arm) {
        return std::max(arm.ready, not_before) + d.travel(robot, arm.point, p);
    };
    double ready = 0.0;
    switch (t.type) {
    case TransitionType::Pick:
        ready = std::max(arrival(t.robot, onFrontier(hg_.robotElement(t.robot))), onFrontier(hg_.objectTail(arc)).ready);
        break;
    case TransitionType::Place: ready = arrival(t.robot, onFrontier(hg_.objectTail(arc))); break;
    case TransitionType::Handover:
        ready = std::max(arrival(t.robot, onFrontier(hg_.objectTail(arc))),
                         arrival(t.robot2, onFrontier(hg_.robotElement(t.robot2))));
        break;
    default: return not_before;
    }
    return ready + d.stageTime();
}

void TaskSearch::fire(int arc) {
    const auto &a = hg_.graph().arc(arc);
    TaskStep step;
    step.arc = arc;
    for (int e : a.tail)
        step.tail.push_back(te_on_element_[static_cast<std::size_t>(e)]);
    const auto history = mergedHistory(step.tail);
    // C_f as an order in time: the steps this one must follow.
    if (constraints_) {
        const auto &steps = schedule_.steps;
        for (int e : a.head)
            for (int ci : constraints_->onPost(e)) {
                const auto &c = constraints_->at(ci);
                if (c.kind != TaskConstraint::Kind::Frontier || !c.appliesTo(hg_.objectTail(arc), arc))
                    continue;
                for (std::size_t s = 0; s < steps.size(); ++s)
                    for (int te : steps[s].tail)
                        if (schedule_.vertices[static_cast<std::size_t>(te)].element == c.pre)
                            step.after.push_back(static_cast<int>(s));
            }
        for (const auto &c : constraints_->all()) {
            if (c.kind != TaskConstraint::Kind::Frontier ||
                std::find(a.head.begin(), a.head.end(), c.pre) == a.head.end())
                continue;
            for (std::size_t s = 0; s < steps.size(); ++s) {
                if (!c.appliesTo(hg_.objectTail(steps[s].arc), steps[s].arc))
                    continue;
                for (int te : steps[s].head)
                    if (schedule_.vertices[static_cast<std::size_t>(te)].element == c.post)
                        step.after.push_back(static_cast<int>(s));
            }
        }
        std::sort(step.after.begin(), step.after.end());
        step.after.erase(std::unique(step.after.begin(), step.after.end()), step.after.end());
    }
    double not_before = 0.0;
    for (int s : step.after)
        not_before = std::max(not_before, schedule_.steps[static_cast<std::size_t>(s)].finish);
    step.finish = finishTime(arc, not_before);
    const Eigen::Vector3d point = durations_.point(arc);
    for (int te : step.tail)
        takeFromFrontier(te);
    const int index = static_cast<int>(schedule_.steps.size());
    for (int e : a.head) {
        const int te = addVertex(e, index, history);
        auto &v = schedule_.vertices[static_cast<std::size_t>(te)];
        v.ready = step.finish;
        v.point = point;
        if (a.data.type == TransitionType::Source) {
            const auto &element = hg_.graph().vertex(e);
            v.point = element.kind == ElementKind::RobotOnly
                          ? durations_.home(element.robot)
                          : hg_.locations()[static_cast<std::size_t>(element.location)].poses[0].translation();
        }
        step.head.push_back(te);
        putOnFrontier(te);
    }
    schedule_.steps.push_back(std::move(step));
}

void TaskSearch::undo() {
    const TaskStep step = schedule_.steps.back();
    schedule_.steps.pop_back();
    for (auto it = step.head.rbegin(); it != step.head.rend(); ++it) {
        takeFromFrontier(*it);
        schedule_.vertices.pop_back(); // heads are the last vertices added
    }
    for (int te : step.tail)
        putOnFrontier(te);
}

bool TaskSearch::goalsReached() const {
    for (int o = 0; o < hg_.numObjects(); ++o)
        if (te_on_element_[static_cast<std::size_t>(hg_.goalElement(o))] < 0)
            return false;
    return true;
}

std::vector<int> TaskSearch::candidates(const TaskConstraints &constraints) const {
    const auto &g = hg_.graph();
    // Resting objects on the frontier that hold some C_f back ("move B first").
    std::vector<char> blocking(static_cast<std::size_t>(g.numVertices()), 0);
    for (const auto &c : constraints.all())
        if (c.kind == TaskConstraint::Kind::Frontier && te_on_element_[static_cast<std::size_t>(c.pre)] >= 0 &&
            g.vertex(c.pre).kind == ElementKind::ObjectAt)
            blocking[static_cast<std::size_t>(c.pre)] = 1;
    std::mt19937_64 noise(perturbation_ ^ (frontier_key_ * 0x9e3779b97f4a7c15ull));
    std::uniform_real_distribution<double> jitter(0.0, perturbation_ ? 1e-6 : 0.0);
    // Objects that must wait: C_f(goal(B), goal(A)) with A not at its goal
    // ranks B's moves last (ranking only).
    std::vector<char> is_goal(static_cast<std::size_t>(g.numVertices()), 0);
    for (int o = 0; o < hg_.numObjects(); ++o)
        is_goal[static_cast<std::size_t>(hg_.goalElement(o))] = 1;
    std::vector<char> must_wait(static_cast<std::size_t>(hg_.numObjects()), 0);
    std::map<int, std::vector<int>> goal_orders; // goal(B) -> C_f(goal(B), goal(A)) with A not there yet
    for (int ci = 0; ci < static_cast<int>(constraints.size()); ++ci) {
        const auto &c = constraints.at(ci);
        if (c.kind == TaskConstraint::Kind::Frontier && is_goal[static_cast<std::size_t>(c.pre)] &&
            is_goal[static_cast<std::size_t>(c.post)] && te_on_element_[static_cast<std::size_t>(c.post)] < 0) {
            must_wait[static_cast<std::size_t>(g.vertex(c.pre).object)] = 1;
            goal_orders[c.pre].push_back(ci);
        }
    }
    // Exact pruning: placing B at goal(B) is excluded when some A could then
    // never reach goal(A), since goal elements stay on the frontier for good.
    std::map<int, std::vector<char>> reach_of; // object -> elements it can still reach (ignoring C_T)
    const auto reachable = [&](int object) -> const std::vector<char> & {
        auto it = reach_of.find(object);
        if (it != reach_of.end())
            return it->second;
        std::vector<char> seen(static_cast<std::size_t>(g.numVertices()), 0);
        std::vector<int> open;
        for (int v = 0; v < g.numVertices(); ++v)
            if (te_on_element_[static_cast<std::size_t>(v)] >= 0 && g.vertex(v).object == object &&
                (g.vertex(v).kind == ElementKind::ObjectAt || g.vertex(v).kind == ElementKind::RobotObject)) {
                seen[static_cast<std::size_t>(v)] = 1;
                open.push_back(v);
            }
        while (!open.empty()) {
            const int v = open.back();
            open.pop_back();
            for (int a : g.outArcs(v)) {
                if (hg_.objectTail(a) != v)
                    continue;
                const int w = hg_.objectHead(a);
                if (w >= 0 && !seen[static_cast<std::size_t>(w)]) {
                    seen[static_cast<std::size_t>(w)] = 1;
                    open.push_back(w);
                }
            }
        }
        return reach_of.emplace(object, std::move(seen)).first->second;
    };
    const auto deadEnd = [&](const auto &a) {
        for (int e : a.head) {
            const auto orders = goal_orders.find(e);
            if (orders == goal_orders.end())
                continue;
            for (int ci : orders->second) {
                const int target = constraints.at(ci).post;
                const auto &seen = reachable(g.vertex(target).object);
                bool open_route = false;
                for (int b : g.inArcs(target)) {
                    if (hg_.objectHead(b) != target || !seen[static_cast<std::size_t>(hg_.objectTail(b))])
                        continue;
                    bool forever = false;
                    for (int cj : constraints.onPost(target)) {
                        const auto &c = constraints.at(cj);
                        forever = forever ||
                                  (c.kind == TaskConstraint::Kind::Frontier && is_goal[static_cast<std::size_t>(c.pre)] &&
                                   (c.pre == e || te_on_element_[static_cast<std::size_t>(c.pre)] >= 0) &&
                                   c.appliesTo(hg_.objectTail(b), b));
                    }
                    open_route = open_route || !forever;
                }
                if (!open_route)
                    return true;
            }
        }
        return false;
    };
    struct Timed {
        int tier, object, detour, last_arm, fresh;
        double start, completion, criticality;
        int arc;
    };
    std::vector<Timed> timed;
    std::vector<int> returns; // round trips (unlimited round only), offered last
    // A pick taking the last free arm while others hold objects leaves no
    // handover partner (gridlock risk), so it counts as a detour.
    int free_arms = 0, held = 0;
    for (int v = 0; v < g.numVertices(); ++v)
        if (te_on_element_[static_cast<std::size_t>(v)] >= 0) {
            free_arms += g.vertex(v).kind == ElementKind::RobotOnly;
            held += g.vertex(v).kind == ElementKind::RobotObject;
        }
    for (int arc = 0; arc < g.numArcs(); ++arc) {
        if (arc == hg_.sourceArc() || arc == hg_.sinkArc())
            continue;
        const auto &a = g.arc(arc);
        if (tail_on_frontier_[static_cast<std::size_t>(arc)] != static_cast<int>(a.tail.size()))
            continue;
        const int object_tail = hg_.objectTail(arc), object_head = hg_.objectHead(arc);
        const double h_head = h(object_head);
        if (h_head == Heuristic::kInfinity)
            continue;
        // Round trips (return to an element in the history) are offered only
        // in the last round, after every other candidate.
        const auto &tail_vertex =
            schedule_.vertices[static_cast<std::size_t>(te_on_element_[static_cast<std::size_t>(object_tail)])];
        const auto &tail_element = g.vertex(object_tail);
        const bool round_trip = isReturn(arc);
        if (round_trip && !allow_returns_)
            continue;

        bool blocked = false;
        std::vector<std::uint64_t> history;
        for (int head : a.head) {
            for (int ci : constraints.onPost(head)) {
                const auto &c = constraints.at(ci);
                if (c.kind == TaskConstraint::Kind::Frontier) {
                    // `pre` on the frontier before firing, so C_f(v, head(b))
                    // also forbids b from consuming v.
                    blocked = c.appliesTo(object_tail, arc) && te_on_element_[static_cast<std::size_t>(c.pre)] >= 0;
                } else if (c.kind == TaskConstraint::Kind::History) {
                    if (history.empty()) {
                        std::vector<int> tail_te;
                        for (int e : a.tail)
                            tail_te.push_back(te_on_element_[static_cast<std::size_t>(e)]);
                        history = mergedHistory(tail_te);
                    }
                    blocked = (history[static_cast<std::size_t>(c.pre) / 64] >> (c.pre % 64)) & 1u;
                }
                if (blocked)
                    break;
            }
            if (blocked)
                break;
        }
        if (blocked || deadEnd(a))
            continue;
        if (round_trip) {
            returns.push_back(arc); // last, and out of the objects' ranking
            continue;
        }
        bool unblocks = false;
        for (int e : a.tail)
            unblocks = unblocks || blocking[static_cast<std::size_t>(e)];
        const bool waits = a.data.object >= 0 && must_wait[static_cast<std::size_t>(a.data.object)];
        const int tier = unblocks ? 0 : waits ? 2 : 1;
        const int last_arm = a.data.type == TransitionType::Pick && free_arms <= 1 && held > 0;
        double start = 0.0;
        for (int e : a.tail)
            start = std::max(start, onFrontier(e).ready);
        const double completion = finishTime(arc, 0.0) + (*time_h_)(object_head) + (perturbation_ ? jitter(noise) : 0.0);
        // Transitions beyond the object's shortest route.
        const double route_regret = durations_.costScale() * a.data.cost + durations_.stageTime() +
                                    (*time_h_)(object_head) - (*time_h_)(object_tail);
        const int detour = static_cast<int>(std::floor(route_regret / durations_.stageTime() + 0.5)) + last_arm;
        // A last-arm pick never gets tier 0; at equal detours, objects in a
        // hand come before new picks.
        const int fresh = tail_element.kind == ElementKind::RobotObject ? 0 : 1;
        timed.push_back({last_arm && tier == 0 ? 1 : tier, a.data.object, detour, last_arm, fresh, start, completion,
                         tail_vertex.ready + (*time_h_)(object_tail), arc});
    }
    // Per (tier, object): the start of its best candidate (fewest detours,
    // then not the last arm, then earliest completion).
    std::map<std::pair<int, int>, std::tuple<int, int, double, double>> best; // -> (detour, last arm, completion, start)
    for (const auto &c : timed) {
        const auto key = std::make_tuple(c.detour, c.last_arm, c.completion, c.start);
        auto [it, fresh] = best.try_emplace({c.tier, c.object}, key);
        if (!fresh && key < it->second)
            it->second = key;
    }
    std::sort(timed.begin(), timed.end(), [&](const Timed &x, const Timed &y) {
        const auto &bx = best.at({x.tier, x.object}), &by = best.at({y.tier, y.object});
        return std::make_tuple(x.tier, std::get<0>(bx), std::get<1>(bx), x.fresh, std::get<3>(bx), -x.criticality, x.object,
                               x.detour, x.last_arm, x.completion, x.arc) <
               std::make_tuple(y.tier, std::get<0>(by), std::get<1>(by), y.fresh, std::get<3>(by), -y.criticality, y.object,
                               y.detour, y.last_arm, y.completion, y.arc);
    });
    std::vector<int> out;
    for (const auto &c : timed)
        out.push_back(c.arc);
    out.insert(out.end(), returns.begin(), returns.end());
    return out;
}

bool TaskSearch::isReturn(int arc) const {
    const int tail = hg_.objectTail(arc), head = hg_.objectHead(arc);
    if (tail < 0 || head < 0 || te_on_element_[static_cast<std::size_t>(tail)] < 0)
        return false;
    const auto &e = hg_.graph().vertex(tail);
    if (e.kind == ElementKind::ObjectAt &&
        hg_.locations()[static_cast<std::size_t>(e.location)].kind == Location::Kind::MoveOut)
        return false; // leaving a move-out is the planned way back
    return onFrontier(tail).inHistory(head);
}

std::vector<char> TaskSearch::bannedArcs(const TaskConstraints &constraints) const {
    const auto &g = hg_.graph();
    std::vector<char> banned(static_cast<std::size_t>(g.numArcs()), 0);
    for (int a = 0; a < g.numArcs(); ++a) {
        const auto &arc = g.arc(a);
        for (int post : arc.head) {
            for (int ci : constraints.onPost(post)) {
                const auto &c = constraints.at(ci);
                if (c.kind == TaskConstraint::Kind::Frontier && c.appliesTo(hg_.objectTail(a), a) &&
                    std::find(arc.tail.begin(), arc.tail.end(), c.pre) != arc.tail.end())
                    banned[static_cast<std::size_t>(a)] = 1;
            }
        }
    }
    return banned;
}

bool TaskSearch::routesCut(const TaskConstraints &constraints) const {
    const auto &g = hg_.graph();
    const auto banned = bannedArcs(constraints);
    for (int o = 0; o < hg_.numObjects(); ++o) {
        std::vector<char> seen(static_cast<std::size_t>(g.numVertices()), 0);
        std::vector<int> open{hg_.startElement(o)};
        seen[static_cast<std::size_t>(open.back())] = 1;
        bool reached = false;
        while (!open.empty() && !reached) {
            const int v = open.back();
            open.pop_back();
            reached = v == hg_.goalElement(o);
            for (int a : g.outArcs(v)) {
                if (banned[static_cast<std::size_t>(a)] || hg_.objectTail(a) != v)
                    continue;
                const int w = hg_.objectHead(a);
                if (w >= 0 && !seen[static_cast<std::size_t>(w)]) {
                    seen[static_cast<std::size_t>(w)] = 1;
                    open.push_back(w);
                }
            }
        }
        if (!reached)
            return true;
    }
    return false;
}

std::optional<TaskSchedule> TaskSearch::query(const TaskConstraints &constraints,
                                              const TaskSearchBudget &budget) {
    const auto t0 = std::chrono::steady_clock::now();
    const auto elapsed = [&] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };
    reset();
    constraints_ = &constraints;
    perturbation_ = budget.perturbation;
    const bool has_ch = std::any_of(constraints.all().begin(), constraints.all().end(),
                                    [](const TaskConstraint &c) { return c.kind == TaskConstraint::Kind::History; });
    if (routesCut(constraints)) {
        stats_.seconds = elapsed();
        return std::nullopt; // exhausted, proven without search
    }
    // Heuristic without the arcs C_T forbids outright (exact pruning).
    const auto banned = bannedArcs(constraints);
    if (std::find(banned.begin(), banned.end(), 1) != banned.end()) {
        query_h_.emplace(hg_, banned);
        h_ptr_ = &*query_h_;
    } else {
        h_ptr_ = &base_h_;
    }
    time_h_.emplace(hg_, banned, durations_.costScale(), durations_.stageTime());
    // Limited discrepancy search: at most d deviations from the greedy order,
    // d = 0..kMaxDiscrepancy, then complete depth-first search.
    constexpr int kMaxDiscrepancy = 4;
    // Round trips (isReturn) come only in a last unlimited round.
    for (int limit = 0; limit <= kMaxDiscrepancy + 2; ++limit) {
        const bool unlimited = limit > kMaxDiscrepancy;
        allow_returns_ = limit > kMaxDiscrepancy + 1;
        // State key: frontier vertices with histories; without C_h the
        // unlimited rounds drop the histories.
        key_histories_ = has_ch || !unlimited;
        schedule_ = {};
        std::fill(te_on_element_.begin(), te_on_element_.end(), -1);
        std::fill(tail_on_frontier_.begin(), tail_on_frontier_.end(), 0);
        frontier_key_ = 0;
        putOnFrontier(addVertex(hg_.source(), -1, std::vector<std::uint64_t>(words_, 0)));
        fire(hg_.sourceArc());

        // State -> fewest discrepancies it was reached with in this round.
        std::unordered_map<std::uint64_t, int> visited{{frontier_key_, 0}};
        std::vector<Choice> stack;
        const auto finish = [&]() -> std::optional<TaskSchedule> {
            if (goalsReached()) {
                fire(hg_.sinkArc());
                stats_.seconds = elapsed();
                return schedule_;
            }
            return std::nullopt;
        };
        if (auto done = finish())
            return done;
        stack.push_back({candidates(constraints), 0, 0});

        while (!stack.empty()) {
            if (stats_.expansions >= budget.max_expansions || elapsed() > budget.time_limit) {
                budget_exhausted_ = true;
                stats_.seconds = elapsed();
                return std::nullopt;
            }
            Choice &choice = stack.back();
            if (choice.next == choice.candidates.size()) {
                stack.pop_back();
                ++stats_.backtracks;
                if (!stack.empty())
                    undo();
                continue;
            }
            const int used = choice.used + (choice.next > 0 ? 1 : 0);
            if (!unlimited && used > limit) {
                stack.pop_back();
                ++stats_.backtracks;
                if (!stack.empty())
                    undo();
                continue;
            }
            fire(choice.candidates[choice.next++]);
            auto [it, fresh] = visited.try_emplace(frontier_key_, used);
            if (!fresh) {
                if (it->second <= used) {
                    undo();
                    continue;
                }
                it->second = used; // reached with fewer discrepancies: explore again
            }
            ++stats_.expansions;
            if (auto done = finish())
                return done;
            stack.push_back({candidates(constraints), 0, used});
        }
    }
    stats_.seconds = elapsed();
    return std::nullopt;
}

} // namespace lazydash
