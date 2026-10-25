#pragma once

#include "lazydash/task/DurationModel.h"
#include "lazydash/task/Heuristic.h"
#include "lazydash/task/TaskConstraints.h"
#include "lazydash/task/TaskHypergraph.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lazydash {

/// A vertex of the task-extended hypergraph H_TE: v_TE = <v_T, Pi>, with
/// `history` a bitset of the H_T elements on its hyperpath from the source.
/// `ready` (s) and `point` are when and where the element holds (estimated).
struct TEVertex {
    int element = -1;
    int producer = -1;
    std::vector<std::uint64_t> history;
    std::uint64_t history_hash = 0;
    double ready = 0.0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();

    bool inHistory(int element_id) const {
        const auto w = static_cast<std::size_t>(element_id) / 64;
        return w < history.size() && (history[w] >> (element_id % 64)) & 1u;
    }
};

/// One fired hyperarc E_TE = <Tail, Head, E_T> (tail/head: H_TE vertex ids).
/// `after`: extra orderings committed by C_f; `finish`: estimated end (s).
struct TaskStep {
    int arc = -1;
    std::vector<int> tail, head;
    std::vector<int> after;
    double finish = 0.0;
};

/// The unvalidated schedule S_T: a source-to-sink hyperpath in H_TE, as steps
/// in firing order (one linearization of the partial order).
struct TaskSchedule {
    std::vector<TEVertex> vertices;
    std::vector<TaskStep> steps;

    /// The step that consumes H_TE vertex `v` (-1: still on the final frontier).
    std::vector<int> consumers() const;
};

struct TaskSearchBudget {
    std::size_t max_expansions = 500000;
    double time_limit = 60.0; // seconds
    /// Nonzero: seed for breaking exact ties of the greedy order randomly.
    std::uint64_t perturbation = 0;
};

struct TaskSearchStats {
    std::size_t expansions = 0;
    std::size_t backtracks = 0;
    double seconds = 0.0;
};

/// The task query (layer 1): greedy search with backtracking over H_TE under
/// the task constraints C_T, as limited discrepancy search in a list-schedule
/// order of estimated time. Each frontier state is visited once.
class TaskSearch {
public:
    /// `durations` estimates when each candidate would run (the greedy order).
    TaskSearch(const TaskHypergraph &hg, const Heuristic &h, const DurationModel &durations);

    /// S_T, or nothing when H_T is exhausted under C_T (or the budget ends).
    std::optional<TaskSchedule> query(const TaskConstraints &constraints,
                                      const TaskSearchBudget &budget = {});

    const TaskSearchStats &stats() const { return stats_; }
    /// True if the last query failed because the budget ran out.
    bool budgetExhausted() const { return budget_exhausted_; }

private:
    struct Choice {
        std::vector<int> candidates;
        std::size_t next = 0;
        int used = 0; // discrepancies taken on the path to this choice
    };

    void reset();
    /// True if some object has no start-to-goal route in H_T without the
    /// banned arcs: a proof of exhaustion without searching.
    bool routesCut(const TaskConstraints &constraints) const;
    /// Arcs that C_T forbids in every state: C_f(pre, post) with pre in the
    /// arc's tail and post in its head.
    std::vector<char> bannedArcs(const TaskConstraints &constraints) const;
    int addVertex(int element, int producer, std::vector<std::uint64_t> history);
    void putOnFrontier(int te);
    void takeFromFrontier(int te);
    void fire(int arc);
    void undo();
    bool goalsReached() const;
    std::vector<int> candidates(const TaskConstraints &constraints) const;
    /// `arc` returns its object to an element already in its history (a round
    /// trip, offered only in the last round).
    bool isReturn(int arc) const;
    std::vector<std::uint64_t> mergedHistory(const std::vector<int> &tail_te) const;
    /// Estimated finish of `arc` fired now, starting no earlier than `not_before`.
    double finishTime(int arc, double not_before) const;
    const TEVertex &onFrontier(int element) const {
        return schedule_.vertices[static_cast<std::size_t>(te_on_element_[static_cast<std::size_t>(element)])];
    }

    const TaskHypergraph &hg_;
    const Heuristic &base_h_;
    std::optional<Heuristic> query_h_; // with this query's banned arcs removed
    const Heuristic *h_ptr_ = nullptr;
    double h(int element) const { return (*h_ptr_)(element); }
    const DurationModel &durations_;
    std::optional<Heuristic> time_h_; // time-to-go (s), this query's banned arcs removed
    std::size_t words_ = 0;
    std::vector<std::uint64_t> element_salt_;

    TaskSchedule schedule_;
    const TaskConstraints *constraints_ = nullptr; // of the running query
    std::vector<int> te_on_element_;   // element -> H_TE vertex on the frontier, or -1
    std::vector<int> tail_on_frontier_; // arc -> number of tail elements on the frontier
    std::uint64_t frontier_key_ = 0;
    bool key_histories_ = true; // state key includes histories (only C_h reads them)
    bool allow_returns_ = true;  // candidates include round trips (isReturn)
    TaskSearchStats stats_;
    bool budget_exhausted_ = false;
    std::uint64_t perturbation_ = 0;
};

} // namespace lazydash
