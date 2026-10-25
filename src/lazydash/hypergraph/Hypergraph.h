#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lazydash {

/// A directed hypergraph (H_T, H_M): hyperarcs connect tail vertices (preconditions)
/// to head vertices (postconditions). Ids are dense; nothing is removed.
template <class V, class E>
class Hypergraph {
public:
    struct Arc {
        E data;
        std::vector<int> tail;
        std::vector<int> head;
    };

    int addVertex(V v) {
        vertices_.push_back(std::move(v));
        out_.emplace_back();
        in_.emplace_back();
        return static_cast<int>(vertices_.size()) - 1;
    }

    int addArc(std::vector<int> tail, std::vector<int> head, E data) {
        const int id = static_cast<int>(arcs_.size());
        for (int v : tail)
            out_.at(static_cast<std::size_t>(v)).push_back(id);
        for (int v : head)
            in_.at(static_cast<std::size_t>(v)).push_back(id);
        arcs_.push_back({std::move(data), std::move(tail), std::move(head)});
        return id;
    }

    int numVertices() const { return static_cast<int>(vertices_.size()); }
    int numArcs() const { return static_cast<int>(arcs_.size()); }

    const V &vertex(int v) const { return vertices_.at(static_cast<std::size_t>(v)); }
    V &vertex(int v) { return vertices_.at(static_cast<std::size_t>(v)); }
    const Arc &arc(int a) const { return arcs_.at(static_cast<std::size_t>(a)); }
    Arc &arc(int a) { return arcs_.at(static_cast<std::size_t>(a)); }

    /// Arcs that have `v` in their tail (that consume `v`).
    const std::vector<int> &outArcs(int v) const { return out_.at(static_cast<std::size_t>(v)); }
    /// Arcs that have `v` in their head (that produce `v`).
    const std::vector<int> &inArcs(int v) const { return in_.at(static_cast<std::size_t>(v)); }

private:
    std::vector<V> vertices_;
    std::vector<Arc> arcs_;
    std::vector<std::vector<int>> out_, in_;
};

} // namespace lazydash
