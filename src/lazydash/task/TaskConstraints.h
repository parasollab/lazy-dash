#pragma once

#include <map>
#include <string>
#include <vector>

namespace lazydash {

/// A scoped task constraint over H_T elements: frontier C_f(pre, post) blocks
/// `post` while `pre` is on the frontier; history C_h(pre, post) blocks it if
/// `pre` is in the expansion's history. Tentative ones rest on sampling.
struct TaskConstraint {
    enum class Kind { Frontier, History };
    Kind kind = Kind::Frontier;
    int pre = -1;
    int post = -1;
    std::string origin;
    bool tentative = false;
    /// Lift order of a tentative constraint: 1 = motion (cheap to retry),
    /// 2 = anchors (costly to retry).
    int tier = 1;
    /// The single H_T arc a tier-2 constraint blocks, or -1.
    int arc = -1;
    /// C_f only: the object tail of the transitions it blocks, or -1 (any).
    int via = -1;

    /// Whether this C_f applies to transition `a`, whose object tail is `tail`.
    bool appliesTo(int tail, int a) const { return (via < 0 || via == tail) && (arc < 0 || arc == a); }
};

/// The set C_T used by the task query (layer 1). Constraints accumulate within
/// a run, except tentative ones, which `liftTentative` removes.
class TaskConstraints {
public:
    /// Add a constraint; returns false if an identical one exists.
    bool add(TaskConstraint c);
    /// Remove the tentative constraints of tier <= `max_tier` (default: all);
    /// returns how many were removed.
    std::size_t liftTentative(int max_tier = 2);
    std::size_t numTentative(int tier = 0) const; // 0: any tier
    /// Remove the constraints for which `drop` is true; returns how many.
    template <class Pred> std::size_t removeIf(Pred drop) {
        std::vector<TaskConstraint> kept;
        for (auto &c : all_)
            if (!drop(c))
                kept.push_back(std::move(c));
        const std::size_t removed = all_.size() - kept.size();
        all_.clear();
        by_post_.clear();
        for (auto &c : kept)
            add(std::move(c));
        return removed;
    }

    /// Constraints whose `post` is `element`.
    const std::vector<int> &onPost(int element) const;
    const TaskConstraint &at(int i) const { return all_.at(static_cast<std::size_t>(i)); }
    const std::vector<TaskConstraint> &all() const { return all_; }
    std::size_t size() const { return all_.size(); }

private:
    std::vector<TaskConstraint> all_;
    std::map<int, std::vector<int>> by_post_;
};

} // namespace lazydash
