#include "lazydash/task/Heuristic.h"

#include <functional>
#include <queue>
#include <utility>

namespace lazydash {

Heuristic::Heuristic(const TaskHypergraph &hg) : Heuristic(hg, {}) {}

Heuristic::Heuristic(const TaskHypergraph &hg, const std::vector<char> &banned) : Heuristic(hg, banned, 1.0, 0.0) {}

Heuristic::Heuristic(const TaskHypergraph &hg, const std::vector<char> &banned, double scale,
                     double per_transition) {
    const auto &g = hg.graph();
    h_.assign(static_cast<std::size_t>(g.numVertices()), kInfinity);
    for (int v = 0; v < g.numVertices(); ++v)
        if (g.vertex(v).kind == ElementKind::RobotOnly || g.vertex(v).kind == ElementKind::Sink)
            h_[static_cast<std::size_t>(v)] = 0.0;

    // Dijkstra backward over the object flow of each transition.
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> queue;
    for (int o = 0; o < hg.numObjects(); ++o) {
        const int goal = hg.goalElement(o);
        h_[static_cast<std::size_t>(goal)] = 0.0;
        queue.push({0.0, goal});
    }
    while (!queue.empty()) {
        const auto [d, v] = queue.top();
        queue.pop();
        if (d > h_[static_cast<std::size_t>(v)])
            continue;
        for (int a : g.inArcs(v)) {
            if (hg.objectHead(a) != v || (!banned.empty() && banned[static_cast<std::size_t>(a)]))
                continue;
            const int tail = hg.objectTail(a);
            if (tail < 0)
                continue;
            const double candidate = d + scale * g.arc(a).data.cost + per_transition;
            if (candidate < h_[static_cast<std::size_t>(tail)]) {
                h_[static_cast<std::size_t>(tail)] = candidate;
                queue.push({candidate, tail});
            }
        }
    }
}

} // namespace lazydash
