#pragma once

#include "lazydash/task/TaskHypergraph.h"

#include <limits>
#include <vector>

namespace lazydash {

/// Cost-to-go h(v_T) in H_T: the cheapest chain of transition cost estimates
/// from element v to its object's goal, computed backward from the goals.
/// Robot-only elements have h = 0; elements with no path to a goal have h = inf.
class Heuristic {
public:
    static constexpr double kInfinity = std::numeric_limits<double>::infinity();

    explicit Heuristic(const TaskHypergraph &hg);
    /// Without the arcs C_T bans outright (`banned[a]` != 0), so dead ends
    /// get h = inf and are pruned early.
    Heuristic(const TaskHypergraph &hg, const std::vector<char> &banned);
    /// Each transition counted as `scale` x cost + `per_transition` (time-to-go).
    Heuristic(const TaskHypergraph &hg, const std::vector<char> &banned, double scale, double per_transition);

    double operator()(int element) const { return h_.at(static_cast<std::size_t>(element)); }

private:
    std::vector<double> h_;
};

} // namespace lazydash
