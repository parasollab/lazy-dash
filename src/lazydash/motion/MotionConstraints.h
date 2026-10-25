#pragma once

#include "comotion/collision/ObstacleShapes.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace lazydash {

/// A motion constraint C_M = (Pi_1, Pi_2, p), emitted by layer 4 and enforced by
/// layer 3: the E^move of `element` into H_T arc `consumer_arc` must avoid
/// `region` while H_T element `context` is alive in the hyperpath's partial order.
struct MotionConstraint {
    int element = -1;
    int consumer_arc = -1;
    int context = -1;
    std::vector<comotion::ObstacleSphere> region;
    std::string origin;
};

/// The set C_M (accumulates within a run).
class MotionConstraints {
public:
    /// Add `c`; one with the same (element, consumer_arc, context) gets the
    /// new region added to its own instead. Returns false if nothing changed.
    bool add(MotionConstraint c) {
        for (auto &old : all_) {
            if (old.element != c.element || old.consumer_arc != c.consumer_arc || old.context != c.context)
                continue;
            bool grew = false;
            for (const auto &s : c.region) {
                bool known = false;
                for (const auto &o : old.region)
                    known = known || ((o.center - s.center).norm() < 1e-6 && std::abs(o.radius - s.radius) < 1e-6);
                if (!known) {
                    old.region.push_back(s);
                    grew = true;
                }
            }
            return grew;
        }
        all_.push_back(std::move(c));
        return true;
    }
    const std::vector<MotionConstraint> &all() const { return all_; }
    std::size_t size() const { return all_.size(); }

private:
    std::vector<MotionConstraint> all_;
};

} // namespace lazydash
