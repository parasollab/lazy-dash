#include "lazydash/task/TaskConstraints.h"

#include <utility>

namespace lazydash {

bool TaskConstraints::add(TaskConstraint c) {
    for (int i : onPost(c.post))
        if (all_[static_cast<std::size_t>(i)].kind == c.kind &&
            all_[static_cast<std::size_t>(i)].pre == c.pre && all_[static_cast<std::size_t>(i)].via == c.via &&
            all_[static_cast<std::size_t>(i)].arc == c.arc)
            return false;
    by_post_[c.post].push_back(static_cast<int>(all_.size()));
    all_.push_back(std::move(c));
    return true;
}

std::size_t TaskConstraints::liftTentative(int max_tier) {
    std::vector<TaskConstraint> kept;
    for (auto &c : all_)
        if (!c.tentative || c.tier > max_tier)
            kept.push_back(std::move(c));
    const std::size_t removed = all_.size() - kept.size();
    all_.clear();
    by_post_.clear();
    for (auto &c : kept)
        add(std::move(c));
    return removed;
}

std::size_t TaskConstraints::numTentative(int tier) const {
    std::size_t n = 0;
    for (const auto &c : all_)
        n += c.tentative && (tier == 0 || c.tier == tier);
    return n;
}

const std::vector<int> &TaskConstraints::onPost(int element) const {
    static const std::vector<int> none;
    const auto it = by_post_.find(element);
    return it == by_post_.end() ? none : it->second;
}

} // namespace lazydash
