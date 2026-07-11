#ifndef PPL_CBS_H_
#define PPL_CBS_H_

#include "Utilities/MPUtils.h"

#include <functional>
#include <map>
#include <queue>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <vector>

template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
struct CBSNode {

  /// Map of task to its constraints
  //std::unordered_map<IndividualTask*,std::unordered_set<ConstraintType>> constraintMap;
  std::map<IndividualTask*,std::set<ConstraintType>> constraintMap;

  /// Map of task to its solution
  std::unordered_map<IndividualTask*,IndividualSolution*> solutionMap;

  /// Cost of current solution
  double cost;

  /// Number of conflicts in node
  size_t conflicts = MAX_INT;

  std::vector<std::pair<IndividualTask*,ConstraintType>> cachedNextConstraintSet;

  CBSNode() {
    cost = 0;
  }

  CBSNode(std::vector<IndividualTask*> _tasks) {
    for(auto& task : _tasks) {
      constraintMap[task] = {};
      solutionMap[task] = nullptr;
    }
    cost = 0;
  }

  CBSNode(const CBSNode& _node) {
    constraintMap = _node.constraintMap;
    solutionMap = _node.solutionMap;
    cost = _node.cost;
  }

  CBSNode& operator=(const CBSNode& _node) {
    if(this != &_node) {
      constraintMap.clear();
      constraintMap = _node.constraintMap;
      solutionMap.clear();
      solutionMap = _node.solutionMap;
      cost = _node.cost;
    }
    return *this;
  }

  CBSNode& operator=(CBSNode&& _node) {
    if(this != &_node) {
      constraintMap.clear();
      constraintMap = std::move(_node.constraintMap);
      solutionMap.clear();
      solutionMap = std::move(_node.solutionMap);
      cost = std::move(_node.cost);
    }
    return *this;
  }

  bool operator==(const CBSNode& _node) const {

    // Check if costs are equal
    if(cost != _node.cost)
      return false;

    for(auto& kv : constraintMap) {
      auto task = kv.first;

      // Check if other node has the task in the constraint map
      auto iter = _node.constraintMap.find(task);
      if(iter == _node.constraintMap.end())
        return false;

      // Check if constraint sets match
      if(kv.second != *iter)
        return false;
    }

    for(auto& kv : solutionMap) {
      auto task = kv.first;

      // Check if other node has the task in the solution map
      auto iter = _node.solutionMap.find(task);
      if(iter == _node.solutionMap.end())
        return false;

      // Check if solutions match
      if(kv.second != *(iter->second))
        return false;
    }
    return true;
  }

  bool operator>(const CBSNode& _node) const noexcept {
    if(cost != _node.cost)
      return cost > _node.cost;

    return conflicts > _node.conflicts;
  }

};


template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
using CBSLowLevelPlanner =
  std::function<bool(CBSNode<IndividualTask, ConstraintType, IndividualSolution>& _node,
    IndividualTask* _task)>;

template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
using CBSLowLevelPlannerWithQueryStrategy =
  std::function<bool(CBSNode<IndividualTask, ConstraintType, IndividualSolution>& _node,
    IndividualTask* _task, std::string _queryStrategy)>;

template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
using CBSValidationFunction =
  std::function<std::vector<std::pair<IndividualTask*, ConstraintType>>(
    CBSNode<IndividualTask, ConstraintType, IndividualSolution>& _node)>;

template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
using CBSCostFunction =
  std::function<double(CBSNode<IndividualTask, ConstraintType, IndividualSolution>& _node)>;

template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
using CBSSplitNodeFunction =
  std::function<std::vector<CBSNode<IndividualTask,ConstraintType,IndividualSolution>>(
    CBSNode<IndividualTask, ConstraintType, IndividualSolution>& _node,
    std::vector<std::pair<IndividualTask*, ConstraintType>> _constraints,
    CBSLowLevelPlanner<IndividualTask, ConstraintType, IndividualSolution>& _lowlevel,
    CBSCostFunction<IndividualTask, ConstraintType, IndividualSolution>& _cost)>;

template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
using CBSSplitNodeFunctionWithQueryStrategy =
  std::function<std::vector<CBSNode<IndividualTask,ConstraintType,IndividualSolution>>(
    CBSNode<IndividualTask, ConstraintType, IndividualSolution>& _node,
    std::vector<std::pair<IndividualTask*, ConstraintType>> _constraints,
    CBSLowLevelPlannerWithQueryStrategy<IndividualTask, ConstraintType, IndividualSolution>& _lowlevel,
    CBSCostFunction<IndividualTask, ConstraintType, IndividualSolution>& _cost)>;

template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
using CBSInitialFunction =
  std::function<void(
      std::vector<CBSNode<IndividualTask, ConstraintType, IndividualSolution>>& _root,
      std::vector<IndividualTask*> _tasks,
      CBSLowLevelPlanner<IndividualTask, ConstraintType, IndividualSolution>& _lowlevel,
      CBSCostFunction<IndividualTask, ConstraintType, IndividualSolution>& _cost)>;

template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
using CBSInitialFunctionWithQueryStrategy =
  std::function<void(
      std::vector<CBSNode<IndividualTask, ConstraintType, IndividualSolution>>& _root,
      std::vector<IndividualTask*> _tasks,
      CBSLowLevelPlannerWithQueryStrategy<IndividualTask, ConstraintType, IndividualSolution>& _lowlevel,
      CBSCostFunction<IndividualTask, ConstraintType, IndividualSolution>& _cost)>;

template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
CBSInitialFunction<IndividualTask, ConstraintType, IndividualSolution>
CBSDefaultInitialFunction() {
  return [](
      std::vector<CBSNode<IndividualTask, ConstraintType, IndividualSolution>>& _root,
      std::vector<IndividualTask*> _tasks,
      CBSLowLevelPlanner<IndividualTask, ConstraintType, IndividualSolution>& _lowlevel,
      CBSCostFunction<IndividualTask, ConstraintType, IndividualSolution>& _cost){

    CBSNode<IndividualTask, ConstraintType, IndividualSolution> single(_tasks);
    for(auto task : _tasks) {
      if (!_lowlevel(single, task))
        return;
    }
    single.cost = _cost(single);
    _root.push_back(single);
  };
}

template <typename IndividualTask, typename ConstraintType>
using CBSEarlyTerminationFunction = 
  std::function<bool(const size_t& _numNodes,
                    std::vector<std::pair<IndividualTask*, ConstraintType>> _constraints)>;

template <typename IndividualTask, typename ConstraintType>
CBSEarlyTerminationFunction<IndividualTask,ConstraintType>
CBSDefaultEarlyTermination() {
  return [](const size_t& _numNodes, 
            std::vector<std::pair<IndividualTask*, ConstraintType>> _constraints) {
    return false;
  };
}


template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
CBSNode<IndividualTask, ConstraintType, IndividualSolution>
CBS(
  std::vector<IndividualTask*> _tasks,
  CBSValidationFunction<IndividualTask,ConstraintType,IndividualSolution>& _validate,
  CBSSplitNodeFunction<IndividualTask,ConstraintType,IndividualSolution>& _split,
  CBSLowLevelPlanner<IndividualTask,ConstraintType,IndividualSolution>& _lowlevel,
  CBSCostFunction<IndividualTask,ConstraintType,IndividualSolution>& _cost)
{
  auto initial = CBSDefaultInitialFunction<IndividualTask, ConstraintType, IndividualSolution>();
  auto termination = CBSDefaultEarlyTermination<IndividualTask, ConstraintType>();
  return CBS(_tasks, _validate, _split, _lowlevel, _cost, initial, termination);
}


template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
CBSNode<IndividualTask, ConstraintType, IndividualSolution>
CBS(
  std::vector<IndividualTask*> _tasks,
  CBSValidationFunction<IndividualTask,ConstraintType,IndividualSolution>& _validate,
  CBSSplitNodeFunction<IndividualTask,ConstraintType,IndividualSolution>& _split,
  CBSLowLevelPlanner<IndividualTask,ConstraintType,IndividualSolution>& _lowlevel,
  CBSCostFunction<IndividualTask,ConstraintType,IndividualSolution>& _cost,
  CBSInitialFunction<IndividualTask,ConstraintType,IndividualSolution>& _initial)
{
  auto termination = CBSDefaultEarlyTermination<IndividualTask, ConstraintType>();
  return CBS(_tasks, _validate, _split, _lowlevel, _cost, _initial, termination);
}


template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
CBSNode<IndividualTask, ConstraintType, IndividualSolution>
CBS(
  std::vector<IndividualTask*> _tasks,
  CBSValidationFunction<IndividualTask,ConstraintType,IndividualSolution>& _validate,
  CBSSplitNodeFunction<IndividualTask,ConstraintType,IndividualSolution>& _split,
  CBSLowLevelPlanner<IndividualTask,ConstraintType,IndividualSolution>& _lowlevel,
  CBSCostFunction<IndividualTask,ConstraintType,IndividualSolution>& _cost,
  CBSEarlyTerminationFunction<IndividualTask,ConstraintType>& _termination)
{
  auto initial = CBSDefaultInitialFunction<IndividualTask, ConstraintType, IndividualSolution>();
  return CBS(_tasks, _validate, _split, _lowlevel, _cost, initial, _termination);
}


template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
CBSNode<IndividualTask, ConstraintType, IndividualSolution>
CBS(
  std::vector<IndividualTask*> _tasks,
  CBSValidationFunction<IndividualTask,ConstraintType,IndividualSolution>& _validate,
  CBSSplitNodeFunctionWithQueryStrategy<IndividualTask,ConstraintType,IndividualSolution>& _split,
  CBSLowLevelPlannerWithQueryStrategy<IndividualTask,ConstraintType,IndividualSolution>& _lowlevel,
  CBSCostFunction<IndividualTask,ConstraintType,IndividualSolution>& _cost,
  CBSInitialFunctionWithQueryStrategy<IndividualTask,ConstraintType,IndividualSolution>& _initial,
  CBSEarlyTerminationFunction<IndividualTask,ConstraintType>& _termination)
{
  using CBSNodeType = CBSNode<IndividualTask,ConstraintType,IndividualSolution>;

  // Create the conflict tree
  std::priority_queue<CBSNodeType,
                      std::vector<CBSNodeType>,
                      std::greater<CBSNodeType>> ct;

  // Create root node with initial plans
  std::vector<CBSNodeType> root;
  std::unordered_map<const CBSNodeType*,size_t> nodeIdMap;

  // Total number of CT nodes
  size_t numNodes = 0;

  _initial(root, _tasks, _lowlevel, _cost);

  for (const auto& node : root) {
    numNodes++;
    nodeIdMap[&node] = numNodes;
    ct.push(node);
  }

  // Search conflict tree
  while(!ct.empty()) {

    // Grab minimum cost node
    auto node = ct.top();
    ct.pop();

    // size_t id = 0;
    // size_t cost = 0;
    // for(auto kv : nodeIdMap) {
    //   if(kv.first->constraintMap == node.constraintMap) {
    //     id = kv.second;
    //     cost = node.cost;
    //     break;
    //   }
    // }
    // std::cout << "Popped node: " << id << " (" << cost << ")" << std::endl;
    // std::cout << "\tConstraints: " << std::endl;
    // for(auto kv : node.constraintMap) {
    //   for(auto c : kv.second) {
    //     std::cout << "\t\t" << kv.first->GetLabel() << ": " << c << std::endl; 
    //   }
    // }

    // Validate solution in node
    auto constraints = _validate(node);

    // If there are no conflicts, return the valid solution
    if(constraints.empty())
     return node;

    // Check early termination
    if(_termination(numNodes, constraints)) {
      std::cout << "break" << std::endl;
      break;
    }

    // Create child nodes
    std::cout << "split" << std::endl;
    auto children = _split(node, constraints, _lowlevel, _cost);
    std::cout << "split done" << std::endl;

    // Add child nodes to the tree
    for(const auto& child : children) {
      numNodes++;
      ct.push(child);
      nodeIdMap[&child] = numNodes;

      // std::cout << "\tChild node: " << nodeIdMap[&child] << " (" << child.cost << ")" << std::endl;
      // std::cout << "\t\tConstraints: " << std::endl;
      // for(auto kv : child.constraintMap) {
      //   for(auto c : kv.second) {
      //     std::cout << "\t\t\t" << kv.first->GetLabel() << ": " << c << std::endl; 
      //   }
      // }
    }
  }

  auto empty = CBSNodeType();
  empty.cost = std::numeric_limits<double>::infinity();
  return empty;
}

















template <typename IndividualTask, typename ConstraintType, typename IndividualSolution>
CBSNode<IndividualTask, ConstraintType, IndividualSolution>
CBS(
  std::vector<IndividualTask*> _tasks,
  CBSValidationFunction<IndividualTask,ConstraintType,IndividualSolution>& _validate,
  CBSSplitNodeFunction<IndividualTask,ConstraintType,IndividualSolution>& _split,
  CBSLowLevelPlanner<IndividualTask,ConstraintType,IndividualSolution>& _lowlevel,
  CBSCostFunction<IndividualTask,ConstraintType,IndividualSolution>& _cost,
  CBSInitialFunction<IndividualTask,ConstraintType,IndividualSolution>& _initial,
  CBSEarlyTerminationFunction<IndividualTask,ConstraintType>& _termination)
{

  using CBSNodeType = CBSNode<IndividualTask,ConstraintType,IndividualSolution>;

  // Create the conflict tree
  std::priority_queue<CBSNodeType,
                      std::vector<CBSNodeType>,
                      std::greater<CBSNodeType>> ct;

  // Create root node with initial plans
  std::vector<CBSNodeType> root;
  std::unordered_map<const CBSNodeType*,size_t> nodeIdMap;

  // Total number of CT nodes
  size_t numNodes = 0;

  _initial(root, _tasks, _lowlevel, _cost);

  for (auto node : root) {
    numNodes++;
    nodeIdMap[&node] = numNodes;
    ct.push(node);
  }

  // Search conflict tree
  while(!ct.empty()) {

    // Grab minimum cost node
    auto node = ct.top();
    ct.pop();

    // std::cout << "\tPopped node: " << nodeIdMap[&node] << std::endl;
    // std::cout << "\tCost: " << node.cost << std::endl; // Replace with actual property
    // std::cout << "\tConstraints: " << std::endl;
    // for(auto kv : node.constraintMap) {
    //   for(auto c : kv.second) {
    //     std::cout << "\t\t" << c << std::endl; 
    //   }
    // }
    // Validate solution in node
    auto constraints = _validate(node);

    // If there are no conflicts, return the valid solution
    if(constraints.empty())
     return node;

    // Check early termination
    if(_termination(numNodes, constraints)) {
      std::cout << "break" << std::endl;
      break;
    }

    std::cout << "split" << std::endl;
    // Create child nodes
    auto children = _split(node, constraints, _lowlevel, _cost);

    // Add child nodes to the tree
    for(const auto& child : children) {
      numNodes++;
      nodeIdMap[&child] = numNodes;
      ct.push(child);
      // std::cout << "\tPopped node: " << nodeIdMap[&child] << std::endl;
      // std::cout << "\tCost: " << node.cost << std::endl; // Replace with actual property
      // std::cout << "\tConstraints: " << std::endl;
      // for(auto kv : node.constraintMap) {
      //   for(auto c : kv.second) {
      //     std::cout << "\t\t" << c << std::endl; 
      //   }
      // }
    }
    std::cout << "split done" << std::endl;
  }

  auto empty = CBSNodeType();
  empty.cost = std::numeric_limits<double>::infinity();
  return empty;
}


#endif
