#include "lazydash/task/TaskConflictDetection.h"

#include <algorithm>
#include <iterator>
#include <set>
#include <sstream>

namespace lazydash {

TaskConflictDetection::TaskConflictDetection(const TaskHypergraph &hg, TransitionSampler &sampler,
                                             const BodyModels &models, int resample_budget)
    : hg_(hg), sampler_(sampler), models_(models), resample_budget_(resample_budget) {}

std::vector<int> TaskConflictDetection::blockers(const TransitionAnchors &a,
                                                 const std::map<int, Resting> &resting) const {
    // Static configurations of the transition (-1 = hand free, else the grasp held).
    std::vector<std::tuple<int, int, const std::vector<double> *>> statics;
    switch (a.type) {
    case TransitionType::Pick: // approach start, grasp (closing), lift end (holding)
        statics = {{a.robot, -1, &a.q_approach}, {a.robot, -1, &a.q_grasp}, {a.robot, a.grasp, &a.q_approach}};
        break;
    case TransitionType::Place: // descend start (holding), at the goal, retreat end
        statics = {{a.robot, a.grasp, &a.q_approach}, {a.robot, a.grasp, &a.q_grasp}, {a.robot, -1, &a.q_approach}};
        break;
    case TransitionType::Handover:
        statics = {{a.robot, a.grasp, &a.q_grasp}, {a.robot, -1, &a.q_approach},
                   {a.robot2, -1, &a.q2_approach}, {a.robot2, a.grasp2, &a.q2_grasp}};
        break;
    default:
        break;
    }
    std::vector<int> hit;
    const double r = models_.scene().objects.empty() ? 0.0 : models_.scene().objects[0].collision_radius;
    for (const auto &[object, rest] : resting) {
        if (object == a.object)
            continue;
        bool blocked = false;
        for (const auto &[robot, grasp, q] : statics)
            blocked = blocked || !sampler_.clearOfObject(robot, grasp, *q, object, rest.position);
        // Two resting objects overlapping (the placed one and another).
        if (a.type == TransitionType::Place &&
            (a.object_pose.translation() - rest.position).norm() < 2 * r)
            blocked = true;
        if (blocked)
            hit.push_back(object);
    }
    return hit;
}

TaskConflictResult TaskConflictDetection::run(const TaskSchedule &st) {
    TaskConflictResult out;
    out.anchors.resize(st.steps.size());
    std::map<int, Resting> resting; // object -> where it rests now
    for (std::size_t s = 0; s < st.steps.size(); ++s) {
        const auto &step = st.steps[s];
        const auto &arc = hg_.graph().arc(step.arc);
        if (arc.data.type == TransitionType::Source) {
            for (int o = 0; o < hg_.numObjects(); ++o)
                resting[o] = {hg_.startElement(o),
                              hg_.locations()[static_cast<std::size_t>(
                                                  hg_.graph().vertex(hg_.startElement(o)).location)]
                                  .poses[0]
                                  .translation()};
            continue;
        }
        if (arc.data.type == TransitionType::Sink)
            continue;
        const int object = arc.data.object;
        const int head = hg_.objectHead(step.arc);

        // Cached anchors first, then fresh samples free of the current frontier.
        std::optional<TransitionAnchors> anchors;
        std::vector<int> hit;
        std::set<int> geometric; // blockers on every choice of the transition
        if (auto it = cache_.find(step.arc); it != cache_.end()) {
            anchors = it->second;
            hit = blockers(*anchors, resting);
        }
        const bool pick_place = arc.data.type == TransitionType::Pick || arc.data.type == TransitionType::Place;
        if (pick_place && (!anchors || !hit.empty())) {
            // First IK branch free of the frontier, else the fewest blockers.
            // A blocker is geometric only if every branch has it.
            auto cand = branches_.find(step.arc);
            if (cand == branches_.end())
                cand = branches_.emplace(step.arc, sampler_.pickPlaceBranches(step.arc)).first;
            const bool had = anchors.has_value();
            std::optional<std::size_t> best;
            std::vector<int> best_hit;
            for (std::size_t k = 0; k < cand->second.size(); ++k) {
                auto h = blockers(cand->second[k], resting);
                if (k == 0)
                    geometric.insert(h.begin(), h.end());
                else
                    for (auto g = geometric.begin(); g != geometric.end();)
                        g = std::find(h.begin(), h.end(), *g) == h.end() ? geometric.erase(g) : std::next(g);
                if (!best || h.size() < best_hit.size()) {
                    best = k;
                    best_hit = std::move(h);
                }
                if (best_hit.empty())
                    break; // free of the frontier: no conflict to classify
            }
            if (best) {
                anchors = cand->second[*best];
                hit = std::move(best_hit);
                if (had)
                    ++out.resamples;
            } else {
                anchors.reset();
            }
        }
        for (int attempt = 0; !pick_place && (!anchors || !hit.empty()) && attempt <= resample_budget_; ++attempt) {
            auto sample = sampler_.sample(step.arc);
            if (!sample)
                break;
            if (anchors)
                ++out.resamples;
            anchors = std::move(sample);
            hit = blockers(*anchors, resting);
        }
        if (!anchors) {
            // No anchors within budget: a tentative tier-2 constraint. Handover
            // anchors do not depend on the object, so all its copies are set aside.
            std::vector<int> arcs{step.arc};
            if (arc.data.type == TransitionType::Handover)
                for (int a = 0; a < hg_.graph().numArcs(); ++a) {
                    const auto &d = hg_.graph().arc(a).data;
                    if (a != step.arc && d.type == TransitionType::Handover && d.robot == arc.data.robot &&
                        d.grasp == arc.data.grasp && d.robot2 == arc.data.robot2 && d.grasp2 == arc.data.grasp2)
                        arcs.push_back(a);
                }
            for (int a : arcs) {
                TaskConstraint c{TaskConstraint::Kind::Frontier, hg_.objectTail(a), hg_.objectHead(a),
                                 "layer2: no anchors for " + hg_.describeArc(a), true, 2, a};
                out.constraints.push_back(c);
            }
            out.log.push_back("layer2: no anchors for " + hg_.describeArc(step.arc) +
                              (arcs.size() > 1 ? " (and " + std::to_string(arcs.size() - 1) + " other blocks)" : ""));
            continue;
        }
        cache_[step.arc] = *anchors;
        for (int b : hit) {
            ++out.conflicts;
            const bool branch_only = pick_place && !geometric.count(b);
            TaskConstraint c{TaskConstraint::Kind::Frontier, resting.at(b).element, head,
                             "layer2: " + hg_.describeArc(step.arc) + " blocked by " +
                                 hg_.describe(resting.at(b).element) + (branch_only ? " (this IK branch)" : ""),
                             anchors->resampleable() || branch_only};
            c.via = hg_.objectTail(step.arc); // this transition's source, not every way to `head`
            out.log.push_back(c.origin);
            out.constraints.push_back(std::move(c));
        }
        out.anchors[s] = anchors;

        // Apply the action to the frontier of resting objects.
        if (arc.data.type == TransitionType::Pick)
            resting.erase(object);
        else if (arc.data.type == TransitionType::Place)
            resting[object] = {head, anchors->object_pose.translation()}; // goal or move-out
    }
    return out;
}

} // namespace lazydash
