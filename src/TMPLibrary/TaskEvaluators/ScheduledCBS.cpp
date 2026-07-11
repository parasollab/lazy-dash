#include "ScheduledCBS.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "MPLibrary/PathModifiers/ShortcuttingPathModifier.h"

#include "TMPLibrary/StateGraphs/ModeGraph.h"
#include "TMPLibrary/StateGraphs/LazyModeGraph.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"
#include "TMPLibrary/TaskEvaluators/ModeQuery.h"
#include "TMPLibrary/TaskEvaluators/ModeHyperpathQuery.h"

#include <map>
#include <set>


/*----------------------- Construction -----------------------*/

ScheduledCBS::
ScheduledCBS() {
  this->SetName("ScheduledCBS");
}

ScheduledCBS::
ScheduledCBS(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("ScheduledCBS");

  m_vcLabel = _node.Read("vcLabel",true,"",
         "Validity checker to use for multi-robot collision checking.");

  m_queryLabel = _node.Read("queryLabel",true,"",
         "Map Evaluator to use to query individual solutions.");

  m_queryStrategy = _node.Read("queryStrategy",true,"",
         "MPStrategy to sue to query individial paths.");

  m_initialQueryStrategy = _node.Read("initialQueryStrategy",false,"CompositeEval",
         "MPStrategy to sue to query individial paths.");

  m_bypass = _node.Read("bypass",false,m_bypass,
          "Flag to use bypass strategy.");

  m_sqLabel = _node.Read("sqLabel",true,"","SubmodeQuery label.");

  m_upperBound = std::numeric_limits<double>::infinity();

  m_buffer = _node.Read("buffer",false,m_buffer,size_t(0),SIZE_MAX,
        "Number of timesteps to buffer after a conflict.");

  m_alpha = _node.Read("alpha", false, double(0), double(0), std::numeric_limits<double>::max(), 
    "User-defined hyperparameter for the probability of quitting");

  m_X = _node.Read("X", false, double(1), double(0), std::numeric_limits<double>::max(),
    "User-defined hyperparameter for the probability of quitting");  

  m_writeSolution = _node.Read("writeSolution",false,m_writeSolution,
          "Flag to use bypass strategy.");
  
  m_pmLabel = _node.Read("pmLabel", false, "", "Path Modifier Method");

  // m_robotType = _node.Read("robotType",false,m_robotType,
  //         "Robot Type. Only [ur5e, gantry] are supported.");

}

ScheduledCBS::
~ScheduledCBS() {

}

/*------------------------ Overrides -------------------------*/
void
ScheduledCBS::
Initialize() {
 
  m_startTimes.clear();
  m_endTimes.clear();
 
  m_unsafeVertexIntervalMap.clear();
  m_unsafeEdgeIntervalMap.clear();

  m_vertexIntervals.clear();
  m_edgeIntervals.clear();

  m_scheduleAtomicDistances.clear();

  m_geometricConstraintSet.clear();
  m_motionConstraintSet.clear();
}
    
void
ScheduledCBS::
SetUpperBound(double _upperBound) {
  m_upperBound = _upperBound;
}

bool
ScheduledCBS::
Run(Plan* _plan) {
  if(!_plan)
    _plan = this->GetPlan();

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  MethodTimer mt_ind(stats,this->GetNameAndLabel() + "::Run_ScheduledCBS");
  

  m_quit = false;
  BuildScheduleGraph(_plan);

  // Configure CBS Functions
  CBSLowLevelPlannerWithQueryStrategy<SemanticTask,Constraint,GroupPathType> lowLevel(
    [this](Node& _node, SemanticTask* _task, std::string _queryStrategy) {
      return LowLevelPlanner(_node,_task,_queryStrategy);
    }
  );

  CBSValidationFunction<SemanticTask,Constraint,GroupPathType> validation(
    [this](Node& _node) {
      return this->ValidationFunction(_node);
    }
  );

  CBSCostFunction<SemanticTask,Constraint,GroupPathType> cost(
    [this](Node& _node) {
      return this->CostFunction(_node);
    }
  );

  CBSSplitNodeFunctionWithQueryStrategy<SemanticTask,Constraint,GroupPathType> splitNode(
    [this](Node& _node, std::vector<std::pair<SemanticTask*,Constraint>> _constraints,
           CBSLowLevelPlannerWithQueryStrategy<SemanticTask,Constraint,GroupPathType>& _lowLevel,
           CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost) {
      return this->SplitNodeFunction(_node,_constraints,_lowLevel,_cost);
    }
  );

  CBSInitialFunctionWithQueryStrategy<SemanticTask,Constraint,GroupPathType> initial(
    [this](std::vector<Node>& _root, std::vector<SemanticTask*> _task,
           CBSLowLevelPlannerWithQueryStrategy<SemanticTask,Constraint,GroupPathType>& _lowLevel,
           CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost) {
      return this->InitialSolutionFunction(_root,_task,_lowLevel,_cost);
    }
  );

  CBSEarlyTerminationFunction<SemanticTask,Constraint> termination(
    [this](const size_t& _numNodes, 
          std::vector<std::pair<SemanticTask*,Constraint>> _constraints) {
      return this->m_quit or this->EarlyTermination(_numNodes,_constraints);
    }
  );

  // Collect tasks
  auto decomp = _plan->GetDecomposition();
  auto tasks = decomp->GetGroupMotionTasks();


  // Call CBS
  std::cout << "Start CBS" << std::endl;
  // return true;
  Node solution = CBS(tasks,validation,splitNode,lowLevel,cost,initial,termination);
  std::cout << "Done CBS" << std::endl;

  if(m_motionConstraintSet.size() > 0)
    return false;

  // Check if solution was found
  if(solution.cost == std::numeric_limits<double>::infinity()) {
    _plan->SetCost(std::numeric_limits<double>::infinity());
    return false;
  }

  ConvertToPlan(solution,_plan);

  if(m_pmLabel != "")
    ConvertToModifiedPlan(solution,_plan);

  return true;
}
/*----------------------- CBS Functors -----------------------*/

bool
ScheduledCBS::
LowLevelPlanner(Node& _node, SemanticTask* _task, std::string _queryStrategy) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");
  // Initialize maps
  std::map<SemanticTask*,size_t> startTimes;
  std::map<SemanticTask*,size_t> endTimes;
  std::set<SemanticTask*> solved;

  // Collect start and end times for tasks starting at 0
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    auto path = kv.second;
    startTimes[task] = 0;

    if(task->GetDependencies().empty() and task != _task) {
      if(path) {
        auto timesteps = path->TimeSteps();
        if(timesteps > 0) {
          endTimes[task] = timesteps-1;
        }
        else {
          endTimes[task] = 0;
        }
        solved.insert(task);
      }
    }
  }

  // Add all tasks with solutions to solved set
  auto size = solved.size();
  do {
    size = solved.size();

    // Iterate through set of tasks
    for(auto kv : _node.solutionMap) {
      auto task = kv.first;
      auto path = kv.second;

      if(solved.count(task))
        continue;

      size_t startTime = FindStartTime(task, solved, endTimes);
      if(startTime == MAX_UINT)
        continue;      

      startTimes[task] = startTime;

      if(task == _task or !path)
        continue;

      solved.insert(task);

      size_t timesteps = path->TimeSteps();
      size_t offset = timesteps;// > 0 ? timesteps - 1 : 0;
      endTimes[task] = startTime + offset;
    }
  } while (solved.size() != size);

  // Initialize queue of tasks to replan
  std::priority_queue<std::pair<size_t,SemanticTask*>,
                      std::vector<std::pair<size_t,SemanticTask*>>,
                      std::greater<std::pair<double,SemanticTask*>>> pq;

  pq.push(std::make_pair(startTimes[_task],_task));

  std::set<SemanticTask*> unsolved;
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    if(solved.count(task) or task == _task)
      continue;
    unsolved.insert(kv.first);
  }

  // Plan unsolved tasks
  while(!pq.empty()) {
    auto current = pq.top();
    pq.pop();
    SemanticTask* task = current.second;
    size_t startTime = current.first;

    if(m_debug) {
      std::cout << "\n\nStart time for " 
                << task->GetLabel() 
                << ": " 
                << startTime 
                << std::endl;
    }

    // Compute new path
    auto path = QueryPath(task,startTime,_node,_queryStrategy);

    // Check if path was found
    if(!path) {
      auto t = task->GetGroupMotionTask();
      for(auto iter = t->begin(); iter != t->end(); iter++) {
        // auto g = dynamic_cast<BoundaryConstraint*>(iter->GetGoalConstraints()[0].get());
        // auto s = dynamic_cast<BoundaryConstraint*>(iter->GetStartConstraint());
        auto g = iter->GetGoalConstraints()[0]->Clone();
        auto s = iter->GetStartConstraint()->Clone();

        auto gb = g->GetBoundary()->GetCenter();
        auto sb = s->GetBoundary()->GetCenter();
        std::cout << "Start: " << sb << std::endl;
        std::cout << "Goal: " << gb << std::endl;
        std::cout << "Start time: " << startTime << std::endl;
      }
      std::cout << "Failed to find a path for "
                << task->GetLabel()
                << std::endl;
      return false;
    }
    else {
      if(m_debug) {
        std::cout << "Initial path" << std::endl;
        std::cout << path->VIDs() << std::endl;
        std::cout << path->GetWaitTimes() << std::endl;
        std::cout << path->TimeSteps() << std::endl;
      }
    }

    // Save path to solution
    _node.solutionMap[task] = path;
    solved.insert(task);
    
    size_t timesteps = path->TimeSteps();
    size_t offset = timesteps;// > 0 ? timesteps - 1 : 0;
    endTimes[task] = startTime + offset;

    if(m_debug) {
      std::cout << "Found path for "
                << task->GetLabel()
                << " from "
                << startTime
                << " to " 
                << startTime + offset
                << std::endl;
    }

    if(startTime > endTimes[task]) {
      std::cout << path->VIDs() << std::endl;
      std::cout << path->GetWaitTimes() << std::endl;
      std::cout << startTime << ", " <<  timesteps << ", " << (startTime + timesteps) << std::endl;
      throw RunTimeException(WHERE) << " BAD STUFF";
    }

    // Check if new tasks are available to plan
    std::vector<SemanticTask*> toRemove;
    for(auto t : unsolved) {
      
      size_t st = FindStartTime(t, solved, endTimes);
      if(st == MAX_UINT)
        continue;

      // If the task is ready, move it to the pq

      startTimes[t] = st;
      toRemove.push_back(t);
      pq.push(std::make_pair(st,t));
    }

    // Remove newly available tasks from unsolved queue
    for(auto t : toRemove) {
      unsolved.erase(t);
    }
  }

  if(solved.size() != _node.solutionMap.size())
    throw RunTimeException(WHERE) << "Did not solve all the tasks.";


  m_passiveEndTaskMap.clear();
  for(auto vit = m_scheduleGraph->begin(); vit != m_scheduleGraph->end(); vit++) {
    if(vit->property() == nullptr)
      continue;
    auto task = vit->property();
    auto group = task->GetGroupMotionTask()->GetRobotGroup();
    Robot* passive = nullptr;
    for(auto r : group->GetRobots()) {
      if(r->GetMultiBody()->IsPassive()) {
        passive = r;
        break;
      }
    }
    if(passive == nullptr) 
      continue;
    
    m_passiveEndTaskMap[passive] = task;
    // std::cout << passive->GetLabel() << " ends at " << endTimes[task] << std::endl;
  }



  m_currentScheduleGraph.clear();
  for(auto vit = m_scheduleGraph->begin(); vit != m_scheduleGraph->end(); vit++) {
    m_currentScheduleGraph[vit->property()] = std::make_pair(startTimes[vit->property()],endTimes[vit->property()]);
  }
  

  if(m_debug) {
    for(auto vit = m_scheduleGraph->begin(); vit != m_scheduleGraph->end(); vit++) {
      std::cout << (vit->property() != nullptr ? vit->property()->GetLabel() : "root")
                << " ... " << startTimes[vit->property()] << ":" << endTimes[vit->property()]
                << std::endl;
    }
  }

  return true;
}

std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>>
ScheduledCBS::
ValidationFunction(Node& _node) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  if(!_node.cachedNextConstraintSet.empty())
    return _node.cachedNextConstraintSet;
  auto constraintSets = FindConflicts(_node,m_bypass);

  _node.conflicts = constraintSets.size();

  if(constraintSets.empty())
    return {};

  return constraintSets[0];
}
 
std::vector<ScheduledCBS::Node>
ScheduledCBS::
SplitNodeFunction(Node& _node, 
          std::vector<std::pair<SemanticTask*,Constraint>> _constraints, 
          CBSLowLevelPlannerWithQueryStrategy<SemanticTask,Constraint,GroupPathType>& _lowLevel,
          CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost) {
  
  std::vector<Node> newNodes;

  for(auto pair : _constraints) {
    // Unpack constraint info
    auto task = pair.first;
    auto constraint = pair.second;

    // Copy parent node
    Node child = _node;

    // Add new constraint
    child.constraintMap[task].insert(constraint);

    // Replan tasks affected by constraint. Skip if no valid replanned path is found
    if(!_lowLevel(child,task,m_queryStrategy)) {
      std::cout << "no valid plan for task " << task->GetLabel() << std::endl;
      continue;
    }

    // Update the cost and add to set of new nodes
    double cost = _cost(child);
    child.cost = cost;

    // if(child.cost < _node.cost) {

    //   for(auto kv : _node.solutionMap) {
    //     auto st = kv.first;
    //     auto pp = kv.second;
    //     auto pc = pp->TimeSteps();
    //     auto cp = child.solutionMap[st];
    //     auto cc = cp->TimeSteps();
    //     std::cout << st->GetLabel() << " parent: " << pc << ", child: " << cc << std::endl;
    //     if(pc > cc) {
    //       std::cout << "CHILD COSTS LESS THAN PARENT" << std::endl;
    //       std::cout << "Parent" << std::endl;
    //       std::cout << pp->VIDs() << std::endl;
    //       std::cout << pp->GetWaitTimes() << std::endl;
    //       std::cout << "Child" << std::endl;
    //       std::cout << cp->VIDs() << std::endl;
    //       std::cout << cp->GetWaitTimes() << std::endl;
    //     }
    //   }

    //   throw RunTimeException(WHERE) << "Child cost less than parent.";
    // }

    if(child.cost > m_upperBound)
      continue;

    if(m_bypass and child.cost == _node.cost) {
      auto newConstraints = FindConflicts(child,true);
      child.conflicts = newConstraints.size();
      if(!newConstraints.empty())
        child.cachedNextConstraintSet = newConstraints[0];

      if(child.conflicts < _node.conflicts) {
        child.constraintMap = _node.constraintMap;
        return {child};
      }

    }

    newNodes.push_back(child);
  }

  // if(newNodes.empty()) {
  //   std::vector<SemanticTask*> tasks;
  //   for(auto pair : _constraints) {
  //     tasks.push_back(pair.first);
  //   }

  //   m_quit = HandleFailure(tasks);
  // }

  return newNodes;
}

double
ScheduledCBS::
CostFunction(Node& _node) {

  double cost = 0;
  for(auto kv : _node.solutionMap) {
    auto path = kv.second;
    cost = std::max(cost,double(m_endTimes[path]));
  }

  return cost;
}

void
ScheduledCBS::
InitialSolutionFunction(std::vector<Node>& _root, std::vector<SemanticTask*> _tasks,
                        CBSLowLevelPlannerWithQueryStrategy<SemanticTask,Constraint,GroupPathType>& _lowLevel,
                        CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost) {
                        
  std::cout << "initial Solution Function " << std::endl;
  Node node; 
  // m_initial = true;

  SemanticTask* initTask = nullptr;
  for(auto task : _tasks) {
    node.solutionMap[task] = nullptr;
    node.constraintMap[task] = {};

    if(!initTask and task->GetDependencies().empty()) {
      initTask = task;
    }
  }
  
  // Plan tasks
  if(!_lowLevel(node,initTask,m_queryStrategy)) {
    auto task = initTask->GetGroupMotionTask().get();
    for(auto iter = task->begin(); iter != task->end(); iter++) {
      // auto g = dynamic_cast<BoundaryConstraint*>(iter->GetGoalConstraints()[0].get());
      // auto s = dynamic_cast<BoundaryConstraint*>(iter->GetStartConstraint());
      auto g = iter->GetGoalConstraints()[0]->Clone();
      auto s = iter->GetStartConstraint()->Clone();

      auto gb = g->GetBoundary()->GetCenter();
      auto sb = s->GetBoundary()->GetCenter();
      std::cout << "Start: " << sb << std::endl;
      std::cout << "Goal: " << gb << std::endl;
    }

    throw RunTimeException(WHERE) << "No initial plan for task " << initTask->GetLabel();
  }

  // m_initial = false;

  // Set node cost
  node.cost = _cost(node);

  _root.push_back(node);
}


std::set<std::pair<size_t,size_t>> 
ScheduledCBS::
GetGeometricConstraints() {
  return m_geometricConstraintSet;
}

std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> 
ScheduledCBS::
GetMotionConstraints() {
  return m_motionConstraintSet;
}


bool
ScheduledCBS::
EarlyTermination(const size_t& _numNodes,
                std::vector<std::pair<SemanticTask*,Constraint>> _constraints) {
  
  // std::cout << "Early Termination. constraints size: " << _constraints.size() << std::endl;

  std::set<std::set<std::pair<SemanticTask*,std::pair<size_t,size_t>>>> info;

  if(m_motionConstraintSet.size() > 0) {
    std::cout << "Motion Constraint detected " << m_motionConstraintSet.size() << std::endl;
    return true;
  }

  // for(auto pair : _constraints) {
  //   std::vector<SemanticTask*> q;
  //   std::set<SemanticTask*> taskSets;
  //   std::string prefix = pair.first->GetLabel().substr(0,9);
  //   q.push_back(pair.first);
  //   while (!q.empty()) {
  //     auto current = q.front();
  //     q.erase(q.begin());
  //     auto taskSet = current; // Process the current node
  //     taskSets.insert(taskSet);
  //     auto children = current->GetDependencies();
  //     for(auto child : children) {
  //       for(auto t : child.second) {
  //         q.push_back(t); // Add each child to the queue
  //       }
  //     }
  //   }

  //   std::cout << "Collecting tasks start with " << prefix << std::endl;
  //   std::set<std::pair<SemanticTask*,std::pair<size_t,size_t>>> selectedTaskSet;
  //   for(auto st : taskSets) {
  //     std::pair<SemanticTask*,std::pair<size_t,size_t>> selectedTask;
  //     if (st->GetLabel().find(prefix) != std::string::npos) {
  //       selectedTask.first = st;
  //       selectedTask.second = m_currentScheduleGraph[st]; // Process the current node
  //       std::cout << st->GetLabel() << " ... " 
  //                 << m_currentScheduleGraph[st].first << ":" << m_currentScheduleGraph[st].second << std::endl;
  //       selectedTaskSet.insert(selectedTask);
  //     }
  //   }

  //   info.insert(selectedTaskSet);
  //   // std::cout << "Same: " ;
  //   // for(auto t : taskSets) {
  //   //   std::cout << t.first->GetLabel() << std::endl;
  //   // }

  //   std::cout << "---" << std::endl;
  // }
  

  // bool overlap = false;
  // for (auto outerIt1 = info.begin(); outerIt1 != info.end(); ++outerIt1) {
  //   for (auto outerIt2 = std::next(outerIt1); outerIt2 != info.end(); ++outerIt2) {
  //     // Compare every task in outerIt1 with every task in outerIt2
  //     for (const auto& innerPair1 : *outerIt1) {
  //       for (const auto& innerPair2 : *outerIt2) {
  //         if (innerPair1.second.first < innerPair2.second.second &&
  //             innerPair2.second.first < innerPair1.second.second) {
  //           std::cout << "SemanticTasks overlap: " << std::endl;
  //           std::cout << innerPair1.first->GetLabel() << ": " << innerPair1.second.first << ":" << innerPair1.second.second << std::endl;
  //           std::cout << innerPair2.first->GetLabel() << ": " << innerPair2.second.first << ":" << innerPair2.second.second << std::endl;
  //           // Handle overlap
  //           overlap = true;
  //         }
  //       }
  //     }
  //   }
  // }



  // if(!overlap) {
  //   std::cout << "not overlap" << std::endl;
  //   std::vector<std::pair<RobotGroup*,std::pair<size_t,size_t>>> aggregatedTimes;
  //   for (auto taskSet : info) {
  //     size_t minStartTime = std::numeric_limits<size_t>::max();
  //     size_t maxEndTime = 0;

  //     std::set<RobotGroup*> groupSet;
  //     for (auto pair : taskSet) {
  //       auto task = pair.first;
  //       auto range = pair.second;
  //       std::cout << task->GetLabel() << std::endl;
  //       groupSet.insert(task->GetGroupMotionTask()->GetRobotGroup());
  //       std::cout << task->GetGroupMotionTask()->GetRobotGroup()->GetLabel() << std::endl;
  //       if (range.first < minStartTime) {
  //           minStartTime = range.first;
  //       }
  //       if (range.second > maxEndTime) {
  //           maxEndTime = range.second;
  //       }
  //     }

  //     RobotGroup* group;
  //     for(auto g : groupSet) {
  //       bool active = false;
  //       for(auto r : g->GetRobots()) {
  //         if(!r->GetMultiBody()->IsPassive()) {
  //           active = true;
  //           break;
  //         }
  //       }
  //       if(!active)
  //         group = g;
  //     }
  //     std::cout << "Geo Constraint for: " << group->GetLabel() << std::endl;

  //     std::cout << group->GetLabel() << " ... " << minStartTime << ":" << maxEndTime << std::endl;

  //     aggregatedTimes.emplace_back(std::make_pair(group, std::make_pair(minStartTime, maxEndTime)));
  //   }

  //   // Sort by the start time of each aggregated time range
  //   std::sort(aggregatedTimes.begin(), aggregatedTimes.end(), 
  //             [](const std::pair<RobotGroup*,std::pair<size_t,size_t>>& a, const std::pair<RobotGroup*,std::pair<size_t,size_t>>& b) {
  //                 return a.second.first < b.second.first;
  //             });

  //   if(aggregatedTimes.size() > 2) {
  //     throw RunTimeException(WHERE) << "Handling multiple constraints at once not implemented yet";
  //   }
  //   // Output the order
  //   std::cout << "Order of task sets by start time:" << std::endl;
  //   for (auto item : aggregatedTimes) {
  //       std::cout << item.first->GetLabel() << " Task Set starting at " << item.second.first << " and ending at " << item.second.second << std::endl;
  //   }

  //   m_geometricConstraintSet.insert(std::make_pair(aggregatedTimes.back().first,aggregatedTimes.front().first));
  //   std::cout << "gc constraint set size: " << m_geometricConstraintSet.size() << std::endl;

  //   return true;
  // }
  // for(auto stSet : info) {
  //   for(auto st : stSet)
  //     auto start = m_currentScheduleGraph[st].first;
  //     auto end = m_currentScheduleGraph[st].second;

  //     start = std::min(m_currentScheduleGraph[st].first,start);
  //     start = std::max(m_currentScheduleGraph[st].first,start);
  // }

  double prob = 1 - pow(m_X, m_alpha * _numNodes / (m_quitTimes+1));
  double rand = DRand();
  if(rand < prob) {
    m_quitTimes++;

    if(this->m_debug) {
      std::cout << "Quitting CBS search with "
                << _numNodes
                << " nodes with "
                << rand 
                << " (rand) < "
                << prob << " (probability)."
                << std::endl;
    }

    return true;
  }
  else {
    if(this->m_debug) {
      std::cout << "Running CBS search with "
                << _numNodes
                << " nodes with "
                << rand 
                << " (rand) > "
                << prob << " (probability)."
                << std::endl;
    }

    return false;
  }
}

/*--------------------- Helper Functions ---------------------*/

ScheduledCBS::GroupPathType*
ScheduledCBS::
QueryPath(SemanticTask* _task, const size_t _startTime,
                         const Node& _node,
                         std::string _queryStrategy) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::QueryPath");

  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto solution = mg->GetMPSolution();
  auto lib = this->GetMPLibrary();
  lib->SetTask(nullptr);
  lib->SetGroupTask(_task->GetGroupMotionTask().get());
  auto problem = this->GetMPProblem();
  auto group = _task->GetGroupMotionTask()->GetRobotGroup();

  // Set formations
  auto grm = solution->GetGroupRoadmap(group);
  grm->SetAllFormationsInactive();
  for(auto f : _task->GetFormations()) {
    grm->SetFormationActive(f);
  }

  // Get last timestep of constraints
  const auto& constraints = _node.constraintMap.at(_task);
  size_t lastTimestep = 0;


  for(auto c : constraints) {
    //lastTimestep = std::max(c.second.max,lastTimestep);
    lastTimestep = std::max(c.second.min,lastTimestep);
  }
  // std::cout << "QueryPath: " << constraints.size() << ", " << lastTimestep << std::endl;
  
  for(auto& robot : problem->GetRobots()) {
    robot->SetVirtual(true);
  }
  for(auto& robot : group->GetRobots()) {
    robot->SetVirtual(false);
  }

  // Compute Intervals
  ComputeIntervals(_task,_node);

  auto q = dynamic_cast<SIPPMethod<MPTraits<Cfg>>*>(
    this->GetMPLibrary()->GetMapEvaluator(m_queryLabel)
  );
  q->SetMinEndTime(lastTimestep);
  q->SetStartTime(_startTime);
  q->SetVertexIntervals(m_vertexIntervals);
  q->SetEdgeIntervals(m_edgeIntervals);

  lib->Solve(problem,_task->GetGroupMotionTask().get(),solution,_queryStrategy,
            LRand(),this->GetNameAndLabel()+"::"+_task->GetLabel());

  q->SetMinEndTime(0);
  q->SetStartTime(0);
  q->SetVertexIntervals({});
  q->SetEdgeIntervals({});

  for(auto& robot : problem->GetRobots())
    robot->SetVirtual(false);

  auto path = solution->GetGroupPath(group);
  if(path->VIDs().size() == 0)
    return nullptr;

  // Make path copy
  auto newPath = new GroupPathType(solution->GetGroupRoadmap(group));
  *newPath = *path;

  size_t timesteps = newPath->TimeSteps();
  size_t offset = timesteps;// > 0 ? timesteps - 1 : 0;
  m_startTimes[newPath] = _startTime;
  m_endTimes[newPath] = _startTime + offset;

  return newPath;
}

std::vector<std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>>>
ScheduledCBS::
FindConflicts(Node& _node, bool _getAll) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();

  std::vector<std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>>> constraintSets;

  auto vc = static_cast<CollisionDetectionValidityMethod<MPTraits<Cfg>>*>(
              this->GetMPLibrary()->GetValidityChecker(m_vcLabel));

  // Sort tasks based on slack
  auto slack = ComputeScheduleSlack(_node);

  if(m_debug) {
    std::cout << "Slack values" << std::endl;
    for(auto kv : slack) {
      auto task = m_scheduleGraph->GetVertex(kv.first);
      auto slack = kv.second;
      std::cout << (task ? task->GetLabel() : "root") 
                << " : " 
                << slack 
                << std::endl;
    }
  }

  std::vector<std::pair<size_t,SemanticTask*>> slackOrderedTasks;
  for(auto kv : slack) {
    auto task = m_scheduleGraph->GetVertex(kv.first);
    if(!task)
      continue;

    auto pair = std::make_pair(size_t(kv.second),task);
    slackOrderedTasks.push_back(pair);
  }

  std::sort(slackOrderedTasks.begin(),slackOrderedTasks.end(),
            [this,_node](const std::pair<size_t,SemanticTask*> _elem1,
                   const std::pair<size_t,SemanticTask*> _elem2) {
    
    if(_elem1.first != _elem2.first)
      return _elem1.first < _elem2.first;

    auto path1 = _node.solutionMap.at(_elem1.second);
    auto path2 = _node.solutionMap.at(_elem2.second);

    return this->m_startTimes[path1] < this->m_startTimes[path2];
  });

  // Find max timestep
  size_t maxTimestep = 0;
  for(auto kv : _node.solutionMap) {
    auto path = kv.second;
    maxTimestep = std::max(maxTimestep,m_endTimes[path]);
  }

  // Collect cfgs
  auto lib = this->GetMPLibrary();
  std::map<SemanticTask*,std::vector<GroupCfgType>> cfgPaths;
  for(auto kv : _node.solutionMap) {
    auto task = kv.first;
    auto path = kv.second;
    cfgPaths[task] = path->FullCfgsWithWait(lib);
  }

  // TODO::Make sure collision checking happens until start of next task, 
  //      not just end of current path.

  // for(size_t t = 0; t <= maxTimestep; t++) {
  //   for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end(); iter1++) {
  
  auto sq = dynamic_cast<ModeHyperpathQuery*>(this->GetTaskEvaluator(m_sqLabel).get());

  for(auto kv : m_passiveEndTaskMap) {
    auto passive = kv.first;
    auto passiveTask = kv.second;
    auto passiveCfgs = cfgPaths[passiveTask];
    auto passiveGCfg = passiveCfgs.back();
    auto passivePath = _node.solutionMap.at(passiveTask);
    size_t passiveEndTime = m_endTimes[passivePath];

    for(size_t i = 0; i < slackOrderedTasks.size(); i++) {
      auto task = slackOrderedTasks[i].second;
      auto path = _node.solutionMap.at(task);

      size_t taskStartTime = m_startTimes[path];
      if(taskStartTime < passiveEndTime)
        continue;
      
      if(m_debug) {
        std::cout << "=== Task " << task->GetLabel() 
                  << " ... " << m_startTimes[path] << ":" << m_endTimes[path] << std::endl;
      }
      const auto& cfgs = cfgPaths[task];

      bool collision = false;
      for(auto gcfg : cfgs) {
        gcfg.ConfigureRobot();
        auto group = gcfg.GetGroupRoadmap()->GetGroup();

        for(auto robot1 : group->GetRobots()) {
          if(robot1 == passive) 
            continue;

          // if(m_debug) {
          //   std::cout << "Collision Check between tasks " 
          //             << "(" << passive->GetLabel() << " ... " << passiveEndTime << ") and "
          //             << "(" << robot1->GetLabel() << " ... " << task->GetLabel() << " ... " << m_startTimes[path] << ":" << m_endTimes[path] << ") " << std::endl; 
          // }
          auto mb1 = robot1->GetMultiBody();
          auto mb2 = passive->GetMultiBody();

          CDInfo cdInfo;
          collision = collision or vc->IsMultiBodyCollision(cdInfo,
            mb1,mb2,this->GetNameAndLabel());
          
          if(collision)
            break;
        }

        if(collision) {
          // Identify conflict type
          auto vertexTasks = sq->GetVertexTasks();
          auto hyperarcTasks = sq->GetHyperarcTasks();
          
          bool isVertex1 = false;
          bool isHyperarc1 = false;
          bool isVertex2 = false;
          bool isHyperarc2 = false;

          auto vi1 = vertexTasks.find(passiveTask);
          if(vi1 != vertexTasks.end()) {
            isVertex1 = true;
          }

          auto hi2 = hyperarcTasks.find(task);
          if(hi2 != hyperarcTasks.end()) {
            isHyperarc2 = true;
          }

          if(m_debug)
            std::cout << isVertex1 << isHyperarc1 << isVertex2 << isHyperarc2 << std::endl;

          if((isVertex1 and isHyperarc2)) {
            size_t conflictVID = vi1->second;
            size_t conflictHID = hi2->second;
            size_t passiveEndTime = m_endTimes[passivePath];
            size_t activeEndTime = m_endTimes[path];

            std::string prefix = task->GetLabel().substr(0,9);
            std::set<SemanticTask*> selectedTask;
            for(size_t k = 0; k < slackOrderedTasks.size(); k++) {
              auto t = slackOrderedTasks[k].second;
              if (t->GetLabel().find(prefix) != std::string::npos) {
                selectedTask.insert(t);
              }
            }

            RobotGroup* stGroup = nullptr;
            bool hasObject = false;
            for(auto st : selectedTask) {
              if(st->GetGroupMotionTask()->GetRobotGroup()->IsPassive())
                continue;
              auto group = st->GetGroupMotionTask()->GetRobotGroup();
              for(auto r : group->GetRobots()) {
                if(r->GetMultiBody()->IsPassive()) {
                  hasObject = true;
                  stGroup = group;
                  break;
                }
              }
              if(hasObject)
                break;
            }

            if(m_debug) {
              std::cout << "vh: Possibly a geometric constraint" << std::endl;
              std::cout << "\twith " << passive->GetLabel() << " (vertex) " << group->GetLabel() << " (hyperarc) " << std::endl;
            }

            if(hasObject and stGroup != nullptr) {
              if(m_debug)
                std::cout << "\t     " << passive->GetLabel() << " (vertex) " << stGroup->GetLabel() << " (hyperarc) " << std::endl;


              if(passiveEndTime < activeEndTime) {
                if(m_debug)
                  std::cout << "Passive ends at " << passiveEndTime << " and Active ends at " << activeEndTime << std::endl;
                std::cout << conflictVID << std::endl;
                std::cout << conflictHID << std::endl;
                std::cout << "Add motion constraint between " << passive->GetLabel() << " (vid " << conflictVID << ")" 
                          << " and " << group->GetLabel() << " (hid " << conflictHID << ")" << std::endl; 
                SubmodeQuery::SchedulingConstraint v;
                SubmodeQuery::SchedulingConstraint h;
                v.vertex = true;
                v.id = conflictVID;
                h.vertex = false;
                h.id = conflictHID;

                m_motionConstraintSet.insert(std::make_pair(std::make_pair(v,h),passiveGCfg.GetRobotCfg(passive)));

                std::cout << std::endl;
                std::cout << std::endl;
              
                std::cout << "Collision found between "
                          << group->GetLabel()
                          << " and "
                          << passive->GetLabel() << " (passive) "
                          << " at positions\n\t1: "
                          << gcfg.PrettyPrint()
                          << "\n\t2: "
                          << passiveGCfg.PrettyPrint() << " (passive)"
                          << "\nduring task "
                          << task->GetLabel()
                          << std::endl;
                break;
              }
            }
          }
        }
      }
    }
  }

  

  for(size_t i = 0; i < slackOrderedTasks.size(); i++) {
    auto task1 = slackOrderedTasks[i].second;
    auto slack1 = slackOrderedTasks[i].first;
    auto path1 = _node.solutionMap.at(task1);
 
    const size_t start1 = m_startTimes[path1];

    // Compute final timestep for path after waiting
    size_t end1 = m_endTimes[path1];
    auto vid1 = m_scheduleGraph->GetVID(task1);
    for(auto dep : m_scheduleGraph->GetPredecessors(vid1)) {
      auto depTask = m_scheduleGraph->GetVertex(dep);
      if(depTask) {
        end1 = std::min(end1,m_startTimes[_node.solutionMap.at(depTask)]);
      }
      else {
        end1 = maxTimestep;
      }
    }
    const auto& cfgs1 = cfgPaths[task1];

    std::set<SemanticTask*> collidingTasks;

    for(size_t t = start1; t < end1; t++) {
      const size_t index1 = std::min(t - start1,cfgs1.size()-1);
      const auto gcfg1 = cfgs1[index1];
      gcfg1.ConfigureRobot();
      auto group1 = gcfg1.GetGroupRoadmap()->GetGroup();

      bool collision = false;
      //auto iter2 = iter1;
      //iter2++;
      //for(; iter2 != cfgPaths.end(); iter2++) {
      for(size_t j = i+1; j < slackOrderedTasks.size(); j++) {
        auto task2 = slackOrderedTasks[j].second;

        if(collidingTasks.count(task2))
          continue;

        auto slack2 = slackOrderedTasks[j].first;
        auto path2 = _node.solutionMap.at(task2);
        
        const size_t start2 = m_startTimes[path2];
        if(t < start2)
          continue;

        // Compute final timestep for path after waiting
        size_t end2 = m_endTimes[path2];
        auto vid2 = m_scheduleGraph->GetVID(task2);

        // Check if this task has dependencies or is terminal
        auto depTasks = m_scheduleGraph->GetPredecessors(vid2);
        bool terminal = true;
        if(!depTasks.empty()) {
          // If it has dependencies, account for waiting time
          for(auto dep : depTasks) {
            auto depTask = m_scheduleGraph->GetVertex(dep);
            if(!depTask) 
              continue;
              
            end2 = std::min(end2,m_startTimes[_node.solutionMap.at(depTask)]);
            auto gt1 = depTask->GetGroupMotionTask();
            auto grp1 = gt1->GetRobotGroup();
            auto gt2 = task2->GetGroupMotionTask();
            auto grp2 = gt2->GetRobotGroup();
            bool overlap = false;
            for(auto r1 : grp1->GetRobots()) {
              for(auto r2 : grp2->GetRobots()) {
                if(r1 == r2) {
                  overlap = true;
                  break;
                }
              }
              if(overlap)
                break;
            }

            // If there is overlap in the robots, then task1 is not terminal
            if(overlap) {
              terminal = false;
            }
          }
        }
        
        if(terminal) {
          // Otherwise, it is terminal, and needs to be collision checked through
          // the end of path1.
          end2 = end1;
        }

        if(t > end2)
          continue;
        


        // std::string prefix = task1->GetLabel().substr(0,9);
        // std::set<SemanticTask*> selectedTask;
        // for(size_t k = 0; k < slackOrderedTasks.size(); k++) {
        //   auto task = slackOrderedTasks[k].second;
        //   if (task->GetLabel().find(prefix) != std::string::npos) {
        //     selectedTask.insert(task);
        //   }
        // }
        // std::cout << "\nCheck Collision between \n\t1:" << std::endl;
        // for(auto st : selectedTask) {
        //   std::cout << "\t\t" << st->GetLabel() << std::endl;
        // }

        // prefix = task2->GetLabel().substr(0,9);
        // selectedTask.clear();
        // for(size_t k = 0; k < slackOrderedTasks.size(); k++) {
        //   auto task = slackOrderedTasks[k].second;
        //   if (task->GetLabel().find(prefix) != std::string::npos) {
        //     selectedTask.insert(task);
        //   }
        // }
        // std::cout << "\n\t2:" << std::endl;
        // for(auto st : selectedTask) {
        //   std::cout << "\t\t" << st->GetLabel() << std::endl;
        // }


        const auto& cfgs2 = cfgPaths[task2];
        const size_t index2 = std::min(t - start2,cfgs2.size()-1);
        const auto gcfg2 = cfgs2[index2];
        gcfg2.ConfigureRobot();
        auto group2 = gcfg2.GetGroupRoadmap()->GetGroup();

        // Check for collision
        for(auto robot1 : group1->GetRobots()) {
          for(auto robot2 : group2->GetRobots()) {

            if(robot1 == robot2)
              continue;

            auto mb1 = robot1->GetMultiBody();
            auto mb2 = robot2->GetMultiBody();
            // auto baseTrans1 = mb1->GetBase()->GetWorldTransformation().translation();
            // auto baseTrans2 = mb2->GetBase()->GetWorldTransformation().translation();
            // double distanceSquared = 0.0;
            // for (size_t i = 0; i < 3; ++i) {
            //     double diff = baseTrans1[i] - baseTrans2[i];
            //     distanceSquared += diff * diff;
            // }
            // double distance = std::sqrt(distanceSquared);

            // if(distance > 2.6)
            //   continue;

            CDInfo cdInfo;
            collision = collision or vc->IsMultiBodyCollision(cdInfo,
              mb1,mb2,this->GetNameAndLabel());
            
            if(collision) {
              if(m_debug) {
                std::cout << std::endl;
                std::cout << std::endl;
              
                std::cout << "Collision found between "
                          << robot1->GetLabel()
                          << " and "
                          << robot2->GetLabel()
                          << ", at timestep "
                          << t
                          << " and positions\n\t1: "
                          << gcfg1.GetRobotCfg(robot1).PrettyPrint()
                          << "\n\t2: "
                          << gcfg2.GetRobotCfg(robot2).PrettyPrint()
                          << "\nduring tasks "
                          << task1->GetLabel()
                          << " and "
                          << task2->GetLabel()
                          << std::endl;

                std::cout << "Task starts: " << start1  
                          << ", " << start2 << std::endl;

                std::cout << "Task1:" << std::endl
                          << "Path1 VIDs: " << path1->VIDs() << std::endl
                          << "Path1 Wait Times: " << path1->GetWaitTimes() << std::endl << std::endl;
                // for(auto cfg : path1->FullCfgsWithWait(lib)) {
                //   std::cout << "\t" << cfg.PrettyPrint() << std::endl;
                // }

                std::cout << "Task2:" << std::endl
                          << path2->VIDs() << std::endl
                          << path2->GetWaitTimes() << std::endl << std::endl;
                // for(auto cfg : path2->FullCfgsWithWait(lib)) {
                //   std::cout << "\t" << cfg.PrettyPrint() << std::endl;
                // }


                std::cout << "COLLISION BETWEEN SLACKS OF "
                          << slack1 << " AND " << slack2 << std::endl; 
              }




              // Identify conflict type
              auto vertexTasks = sq->GetVertexTasks();
              auto hyperarcTasks = sq->GetHyperarcTasks();
              
              bool isVertex1 = false;
              bool isHyperarc1 = false;
              bool isVertex2 = false;
              bool isHyperarc2 = false;

              auto vi1 = vertexTasks.find(task1);
              if(vi1 != vertexTasks.end()) {
                isVertex1 = true;
              }
              auto hi1 = hyperarcTasks.find(task1);
              if(hi1 != hyperarcTasks.end()) {
                isHyperarc1 = true;
              }

              auto vi2 = vertexTasks.find(task2);
              if(vi2 != vertexTasks.end()) {
                isVertex2 = true;
              }
              auto hi2 = hyperarcTasks.find(task2);
              if(hi2 != hyperarcTasks.end()) {
                isHyperarc2 = true;
              }
              if(m_debug)
                std::cout << isVertex1 << isHyperarc1 << isVertex2 << isHyperarc2 << std::endl;
              std::string conflictType;
              if(isVertex1 and isVertex2) {
                if(m_debug) {
                  std::cout << "VV: Should resample the object's location" << std::endl;
                  std::cout << "\twith " << group1->GetLabel() << " (vertex) " << group2->GetLabel() << " (vertex) " << std::endl;
                }
                conflictType = "vv";
              }
              else if((isVertex1 and isHyperarc2) or (isHyperarc1 and isVertex2)) {
                RobotGroup* passiveGroup;
                RobotGroup* activeGroup;
                SemanticTask* activeTask;
                GroupCfgType passiveGCfg;
                size_t conflictVID = MAX_UINT;
                size_t conflictHID = MAX_UINT;
                size_t passiveEndTime;
                size_t activeEndTime;
                if(isVertex1) {
                  passiveGroup = group1;
                  activeGroup = group2;
                  activeTask = task2;
                  passiveGCfg = gcfg1;
                  conflictVID = vi1->second;
                  conflictHID = hi2->second;
                  passiveEndTime = m_endTimes[path1];
                  activeEndTime = m_endTimes[path2];
                }
                else {
                  passiveGroup = group2;
                  activeGroup = group1;
                  activeTask = task1;
                  passiveGCfg = gcfg2;
                  conflictVID = vi2->second;
                  conflictHID = hi1->second;
                  passiveEndTime = m_endTimes[path2];
                  activeEndTime = m_endTimes[path1];
                }

                if(!passiveGroup->IsPassive() or activeGroup->IsPassive())
                  continue;

                conflictType = "vh";



                std::string prefix = activeTask->GetLabel().substr(0,9);
                std::set<SemanticTask*> selectedTask;
                for(size_t k = 0; k < slackOrderedTasks.size(); k++) {
                  auto task = slackOrderedTasks[k].second;
                  if (task->GetLabel().find(prefix) != std::string::npos) {
                    selectedTask.insert(task);
                  }
                }

                RobotGroup* stGroup = nullptr;
                bool hasObject = false;
                for(auto st : selectedTask) {
                  if(st->GetGroupMotionTask()->GetRobotGroup()->IsPassive())
                    continue;
                  auto group = st->GetGroupMotionTask()->GetRobotGroup();
                  for(auto r : group->GetRobots()) {
                    if(r->GetMultiBody()->IsPassive()) {
                      hasObject = true;
                      stGroup = group;
                      break;
                    }
                  }
                  if(hasObject)
                    break;
                }

                if(m_debug) {
                  std::cout << "VH: Possibly a geometric constraint" << std::endl;
                  std::cout << "\twith " << passiveGroup->GetLabel() << " (vertex) " << activeGroup->GetLabel() << " (hyperarc) " << std::endl;
                }

                if(hasObject and stGroup != nullptr) {
                  if(m_debug)
                    std::cout << "\t     " << passiveGroup->GetLabel() << " (vertex) " << stGroup->GetLabel() << " (hyperarc) " << std::endl;
                  Robot* passive = nullptr;
                  for(auto r : passiveGroup->GetRobots()) {
                    if(r->GetMultiBody()->IsPassive()) {
                      passive = r;
                      break;
                    }
                  }

                  // Robot* passive2 = nullptr;
                  // for(auto r : stGroup->GetRobots()) {
                  //   if(r->GetMultiBody()->IsPassive()) {
                  //     passive2 = r;
                  //     break;
                  //   }
                  // }


                  if(passiveEndTime < activeEndTime) {
                    if(m_debug)
                      std::cout << "Passive ends at " << passiveEndTime << " and Active ends at " << activeEndTime << std::endl;
                    // m_geometricConstraintSet.insert(std::make_pair(passive2,passive));
                    std::cout << conflictVID << std::endl;
                    std::cout << conflictHID << std::endl;
                    std::cout << "Add motion constraint between " << passiveGroup->GetLabel() << " (vid " << conflictVID << ")" 
                              << " and " << activeGroup->GetLabel() << " (hid " << conflictHID << ")" << std::endl; 
                    SubmodeQuery::SchedulingConstraint v;
                    SubmodeQuery::SchedulingConstraint h;
                    v.vertex = true;
                    v.id = conflictVID;
                    h.vertex = false;
                    h.id = conflictHID;
                    // for(auto pair : m_motionConstraintCount) {
                    //   size_t vid = pair.first.first.id;
                    //   size_t hid = pair.first.second.id;
                    //   std::cout << "Check! " << vid << " " << hid << std::endl;
                    //   std::cout << "\twith " << v.id << " " << h.id << std::endl;
                    //   if(vid == v.id and hid == h.id) {
                    //     m_geometricConstraintSet.insert(std::make_pair(passive2,passive));
                    //   } 
                    // }
                    m_motionConstraintSet.insert(std::make_pair(std::make_pair(v,h),passiveGCfg.GetRobotCfg(passive)));
                    m_motionConstraintCount.insert(std::make_pair(std::make_pair(v,h),passiveGCfg.GetRobotCfg(passive)));
                  }
                }
              }
              else if(isHyperarc1 and isHyperarc2) {
                if(m_debug) {
                  std::cout << "HH: wait if the scheduledCBS can solve this" << std::endl;
                  std::cout << "\twith " << group1->GetLabel() << " (hyperarc) " << group2->GetLabel() << " (hyperarc) " << std::endl;
                }
                conflictType = "hh";
              }
              if(m_debug) {
                std::cout << "Motion constraint size: " << m_motionConstraintSet.size() << std::endl;
              }

              collidingTasks.insert(task2);

              auto endT = t;
              bool group1Passive = true;
              bool group2Passive = true;

              for(auto r : group1->GetRobots()) {
                if(!r->GetMultiBody()->IsPassive()) {
                  group1Passive = false;
                  break;
                }
              }

              for(auto r : group2->GetRobots()) {
                if(!r->GetMultiBody()->IsPassive()) {
                  group2Passive = false;
                  break;
                }
              }

              if(group1Passive) {
                if(terminal) {
                  endT = SIZE_MAX;
                }
                else {
                  endT = end1;
                }
              }
              else if(group2Passive) {
                if(terminal) {
                  endT = SIZE_MAX;
                }
                else {
                  endT = end2;
                }
              }

              stats->IncStat(this->GetNameAndLabel()+"::CollisionFound");

              auto edge1 = path1->GetEdgeAtTimestep(index1);
              auto edge2 = path2->GetEdgeAtTimestep(index2);

              if(m_debug) {
                std::cout << "Edge 1: " << edge1 << std::endl; 
                std::cout << "Edge 2: " << edge2 << std::endl; 
              }

              size_t duration1 = 0;
              size_t duration2 = 0;

              if(edge1.first != edge1.second) {
                duration1 = path1->GetRoadmap()->GetEdge(
                  edge1.first,edge1.second).GetTimeSteps();
              }

              if(edge2.first != edge2.second) {
                duration2 = path2->GetRoadmap()->GetEdge(
                  edge2.first,edge2.second).GetTimeSteps();
              }

              size_t zero = 0;
              Range<size_t> interval1(t < duration1 ? zero : t-duration1,endT);
              Range<size_t> interval2(t < duration2 ? zero : t-duration2,endT);

              std::vector<std::pair<SemanticTask*,Constraint>> constraints;
              constraints.push_back(std::make_pair(task1,
                                    std::make_pair(edge1,interval1)));
              constraints.push_back(std::make_pair(task2,
                                    std::make_pair(edge2,interval2)));

              bool exists = false;
              for(auto constraint : constraints) {
                for(auto c : _node.constraintMap[constraint.first]) {
                  if(c == constraint.second) {
                    exists = true;
                    // throw RunTimeException(WHERE) << "Adding constraint that already exists.";

                  }
                }
              }
              if(exists)
                continue;

              constraintSets.push_back(constraints);

              if(!_getAll)
                return constraintSets;
            }

            if(collision)
              break;
          }
          if(collision)
            break;
        }
        if(collision)
          break;
      }
    }
  }


  // Double check for the conflict between the static objects and moving robots

  // for(auto kv : m_passiveEndTaskMap) {
  //   auto passive = kv.first;
  //   auto passiveTask = kv.second;
  //   auto passiveCfgs = cfgPaths[passiveTask];
  //   auto passiveGCfg = passiveCfgs.back();
  //   auto passivePath = _node.solutionMap.at(passiveTask);
  //   size_t passiveEndTime = m_endTimes[passivePath];

  //   for(size_t i = 0; i < slackOrderedTasks.size(); i++) {
  //     auto task = slackOrderedTasks[i].second;
  //     auto path = _node.solutionMap.at(task);

  //     size_t taskStartTime = m_startTimes[path];
  //     if(taskStartTime < passiveEndTime)
  //       continue;
      
  //     if(m_debug) {
  //       std::cout << "=== Task " << task->GetLabel() 
  //                 << " ... " << m_startTimes[path] << ":" << m_endTimes[path] << std::endl;
  //     }
  //     const auto& cfgs = cfgPaths[task];

  //     bool collision = false;
  //     for(auto gcfg : cfgs) {
  //       gcfg.ConfigureRobot();
  //       auto group = gcfg.GetGroupRoadmap()->GetGroup();

  //       for(auto robot1 : group->GetRobots()) {
  //         if(robot1 == passive) 
  //           continue;

  //         if(m_debug) {
  //           std::cout << "Collision Check between tasks " 
  //                     << "(" << passive->GetLabel() << " ... " << passiveEndTime << ") and "
  //                     << "(" << robot1->GetLabel() << " ... " << task->GetLabel() << " ... " << m_startTimes[path] << ":" << m_endTimes[path] << ") " << std::endl; 
  //         }
  //         auto mb1 = robot1->GetMultiBody();
  //         auto mb2 = passive->GetMultiBody();

  //         CDInfo cdInfo;
  //         collision = collision or vc->IsMultiBodyCollision(cdInfo,
  //           mb1,mb2,this->GetNameAndLabel());
          
  //         if(collision)
  //           break;
  //       }

  //       if(collision) {
  //         std::string prefix = task->GetLabel().substr(0,9);
  //         if(m_debug)
  //           std::cout << "Collecting tasks start with " << prefix << std::endl;
  //         std::set<SemanticTask*> selectedTask;
  //         for(size_t k = 0; k < slackOrderedTasks.size(); k++) {
  //           auto task3 = slackOrderedTasks[k].second;
  //           if (task3->GetLabel().find(prefix) != std::string::npos) {
  //             selectedTask.insert(task3);
  //           }
  //         }

  //         Robot* passive2 = nullptr;
  //         Robot* active2 = nullptr;
  //         for(auto st : selectedTask) {
  //           for(auto r : st->GetGroupMotionTask()->GetRobotGroup()->GetRobots()) {
  //             if(r->GetMultiBody()->IsPassive()) {
  //               passive2 = r;
  //             }
  //             else {
  //               active2 = r;
  //             }
  //           }
  //         }

  //         if(active2 == nullptr and passive2 == nullptr)
  //           continue;
          
  //         std::cout << "Active: " << active2->GetLabel() << std::endl;
  //         std::cout << "Passive: " << passive->GetLabel() << std::endl;
  //         assert(10==1);

  //         m_motionConstraintSet.insert(std::make_pair(std::make_pair(vid1,vid2),passiveGCfg));
  //         m_motionConstraintCount[std::make_pair<passive2,passive>]++;
  //         std::cout << "count: " << m_motionConstraintCount[std::make_pair<passive2,passive>] << std::endl;
  //         if(m_motionConstraintCount[std::make_pair<passive2,passive1>] > 3) {
  //           m_geometricConstraintSet.insert(std::make_pair(passive2,passive));
  //         }

  //         std::cout << std::endl;
  //         std::cout << std::endl;
        
  //         std::cout << "Collision found between "
  //                   << group->GetLabel()
  //                   << " and "
  //                   << passive->GetLabel() << " (passive) "
  //                   << " at positions\n\t1: "
  //                   << gcfg.PrettyPrint()
  //                   << "\n\t2: "
  //                   << passiveGCfg.PrettyPrint() << " (passive)"
  //                   << "\nduring task "
  //                   << task->GetLabel()
  //                   << std::endl;
  //         break;
  //       }
  //     }
  //   }
  // }





  std::sort(constraintSets.begin(), constraintSets.end(), [slack,this](
              const std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>> _elem1,
              const std::vector<std::pair<SemanticTask*,ScheduledCBS::Constraint>> _elem2) {
    for(size_t i = 0; i < _elem1.size() and i < _elem2.size(); i++) {
      if(i >= _elem2.size()){
        return true;
      }

      auto task1 = _elem1[i].first;
      auto task2 = _elem2[i].first;

      auto vid1 = this->m_scheduleGraph->GetVID(task1);
      auto vid2 = this->m_scheduleGraph->GetVID(task2);

      auto slack1 = slack.at(vid1);
      auto slack2 = slack.at(vid2);

      if(slack1 == slack2){
        return false;
      }
      return slack1 < slack2;
    }
    return true;
  });

  return constraintSets;
}

bool
ScheduledCBS::
HandleFailure(std::vector<SemanticTask*> _tasks) {
  auto sq = dynamic_cast<ModeHyperpathQuery*>(this->GetTaskEvaluator(m_sqLabel).get());

  // TODO::Remove assumption of two tasks 
  auto task1 = _tasks[0];
  auto task2 = _tasks[1];

  std::cout << "Handle failures: " << std::endl;
  std::cout << task1->GetLabel() << " and " << task2->GetLabel() << std::endl;

  sq->AddSchedulingConstraint(task1,task2);
  sq->AddSchedulingConstraint(task2,task1);


  return true;
}

void
ScheduledCBS::
ConvertToPlan(const Node& _node, Plan* _plan) {
  _plan->SetCost(_node.cost);

  // if(!m_writeSolution)
  //   return;
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();

  MethodTimer mt(stats,this->GetNameAndLabel() + "::SaveSolution");

  // Collect all of the robots
  std::unordered_map<Robot*,std::vector<Cfg>> robotPaths;
  std::unordered_map<Robot*,std::unordered_map<SemanticTask*,pair<size_t,size_t>>> timeTable;
  for(auto kv : _node.solutionMap) {
    auto group = kv.first->GetGroupMotionTask()->GetRobotGroup();
    for(auto robot : group->GetRobots()) {
      robotPaths[robot] = {};
      timeTable[robot] = {};
    }
  }

  
  std::vector<SemanticTask*> ordering;

  auto lib = this->GetMPLibrary();

  std::unordered_map<SemanticTask*,std::vector<GroupCfgType>> cfgPaths;

  std::unordered_map<SemanticTask*,size_t> startTimes;
  std::unordered_map<SemanticTask*,size_t> endTimes;
  size_t finalTime = 0;

  // Build in order sequence of tasks
  while(ordering.size() < _node.solutionMap.size()) {

    // Find the set of tasks that are ready to be validated
    for(auto kv :_node.solutionMap) {
      auto task = kv.first;
      lib->SetGroupTask(task->GetGroupMotionTask().get()); 

      // Skip if already validated
      if(std::find(ordering.begin(),ordering.end(),task) != ordering.end())
        continue;

      // Check if dependencies have been validated
      size_t startTime = 0;
      bool ready = true;
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {
          if(std::find(ordering.begin(),ordering.end(),t) == ordering.end()) {
            ready = false;
            break;
          }

          startTime = std::max(endTimes[t],startTime);
        }
      }

      if(!ready)
        continue;

      // Recreate the paths at resolution level
      const auto& path = kv.second;
      const auto cfgs = path->FullCfgsWithWait(lib);

      for(size_t i = 0; i < cfgs.size(); i++) {
        cfgPaths[task].push_back(cfgs[i]);
      }

      startTimes[task] = startTime;
      auto timesteps = path->TimeSteps();
      if(timesteps > 0) {
        endTimes[task] = startTime + timesteps; //- 1;
        if(startTime == 0) {
          endTimes[task] = endTimes[task] - 1;
        }
      }
      else {
        endTimes[task] = startTimes[task];
      }

      finalTime = std::max(endTimes[task],finalTime);
      ordering.push_back(task);

      // Update the end times of the preceeding tasks
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {
          endTimes[t] = startTime;

          if(m_debug) {
            std::cout << "Updating end time of " << t->GetLabel()
                      << " to " << startTime
                      << " because of " << task->GetLabel()
                      << std::endl;
          } 
        }
      }
    
      if(m_debug) {
        std::cout << ">>>>>> SCHEDULE <<<<<<" << std::endl;
        std::cout << task->GetLabel() << ": " << startTimes[task] << " --->>> " << endTimes[task] << std::endl;
        for(auto dep : task->GetDependencies()) {
          for(auto t : dep.second) {
            std::cout << "  " << t->GetLabel() << ": " << startTimes[t] << " --->>> " << endTimes[t] << std::endl;
          }
        }
      }
    }
  }



  // Add the cfgs to the paths
  for(size_t t = 0; t <= finalTime; t++) {
    std::unordered_set<Robot*> used;

    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end(); iter1++) {
      auto t1 = iter1->first;

      // std::cout << t << " | " << t1->GetLabel() << ": " << startTimes[t1] << ", " << endTimes[t1] << std::endl;
      if(startTimes[t1] > t or endTimes[t1] < t)
        continue;

      // Configure first group at timestep
      const auto& path1  = iter1->second;
      // for (auto c : path1) {
      //   std::cout << "    " << c.PrettyPrint() << std::endl;
      // }
      const size_t step1 = std::min(t - startTimes[t1],path1.size()-1);
      const auto& cfg1   = path1[step1];
      const auto group1  = cfg1.GetGroupRoadmap()->GetGroup();
      for(auto robot : group1->GetRobots()) {
        // Account for overlap at start/end points in tasks
        if(used.count(robot))
          continue;
        used.insert(robot);

        // Hack bc of direspect to path constraints
        auto cfg = cfg1.GetRobotCfg(robot);
        size_t gripperIdx = 0;
        if(robot->GetLabel().find("ur5e") != std::string::npos) {
          if(!robot->GetMultiBody()->IsPassive() &&
                (robot->GetMultiBody()->PosDOF() != 0 or robot->GetMultiBody()->OrientationDOF() != 0) &&
                    group1->Size() > 1) {
            gripperIdx = 1 + robot->GetMultiBody()->PosDOF() + robot->GetMultiBody()->OrientationDOF();
            cfg[gripperIdx] = .0001;
          }
        }
        
        robotPaths[robot].push_back(cfg);
        timeTable[robot].insert({t1, {startTimes[t1], path1.size()}});
      }
    }
  }




  // // Path modification
  // if(m_pmLabel != "") {
  //   std::cout << "\n** Path Modifier **\n" << std::endl;
  //   std::vector<Robot*> activeRobots;
  //   std::string activeRobotsLabel = "";

  //   // Collect active robots
  //   for(auto kv : robotPaths) {
  //     auto robot = kv.first;
  //     if(!robot->GetMultiBody()->IsPassive())
  //       activeRobots.push_back(robot);
  //   }
    
  //   for(auto robot : activeRobots) {
  //     activeRobotsLabel += (robot->GetLabel() + "--");
  //   }

  //   auto problem = this->GetMPProblem();
  //   auto activeGroup = problem->AddRobotGroup(activeRobots, activeRobotsLabel);
  //   auto newMPSolution = new MPSolution(activeGroup);
  //   auto activeGroupRoadmap = newMPSolution->GetGroupRoadmap(activeGroup);

  //   // Pad shorter paths
  //   for(auto& kv : robotPaths) {
  //     auto& path = kv.second;
  //     if(path.empty()) continue;
  //     while(path.size() < finalTime) {
  //       path.push_back(path.back());  // Repeat last configuration
  //     }
  //   }
    
  //   auto coord = this->GetPlan()->GetCoordinator();
  //   for(auto kv : coord->GetInitialRobotGroups()) {
  //     for(auto robot : kv.first->GetRobots()) {
  //       if(robot->GetMultiBody()->IsPassive())
  //         robot->SetVirtual(true);
  //       else {
  //         auto base = robot->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
  //         for(auto t : base) {
  //           std::cout << t << " ";
  //         }
  //         std::cout << std::endl;
  //       }
  //     }
  //   } 

  //   std::vector<GroupCfgType> activeGroupPath{};
  //   size_t prevGvid = MAX_UINT;
  //   auto vc = lib->GetValidityChecker("pqp_solid");
    
  //   for(size_t t = 0; t < finalTime; t++) {
  //     std::cout << t << "/" << finalTime << " ";
  //     GroupCfgType activeGcfg(activeGroupRoadmap);
  //     for(auto robot : activeGroup->GetRobots()) {
  //       auto cfg = robotPaths[robot][t];
  //       std::cout << cfg.PrettyPrint();
  //       if(vc->IsValid(cfg,""))
  //         std::cout << " valid " << std::endl;
  //       else
  //         std::cout << " invalid " << std::endl;

  //       activeGcfg.SetRobotCfg(robot, std::move(cfg));
  //     }
      
  //     auto gvid = activeGroupRoadmap->AddVertex(activeGcfg);
      
  //     if(t > 0) {
  //       activeGroupRoadmap->AddEdge(prevGvid,gvid);
  //     }
  //     prevGvid = gvid;
  //     activeGroupPath.push_back(activeGcfg);
  //   }

  //   std::cout << " Start: path length " << activeGroupPath.size() << std::endl;
  //   std::vector<GroupCfgType> newPath;
  //   lib->GetPathModifier(m_pmLabel)->Modify(activeGroupRoadmap, activeGroupPath, newPath);
  //   std::cout << "\n Done: path length " << newPath.size() << std::endl;
  // }





  // Find changes in gripper dof and add buffers
  std::vector<size_t> switches;
  for(size_t t = 1; t < finalTime; t++) {
    for(auto& kv : robotPaths) {
      if(kv.first->GetMultiBody()->IsPassive())
        continue;

      const auto& path = kv.second;

      if(t >= path.size())
        continue;

      auto cfg1 = path[t-1];
      auto cfg2 = path[t];
      
      size_t gripperIdx = 0;
      if(kv.first->GetLabel().find("ur5e") != std::string::npos) {
        if(kv.first->GetLabel().find("mobile") != std::string::npos) {
          gripperIdx = 4;
        }
        else {
          gripperIdx = 1;
        }
      }

      if(abs(cfg1[gripperIdx] - cfg2[gripperIdx]) >= .000001) {
        if(switches.empty() or switches.back() != t-1)
          switches.push_back(t-1);
        switches.push_back(t);
        // std::cout << "Found switch at " << t-1 << std::endl;
        break;
      }
    }
  }

  const size_t numCopies = 5;

  for(size_t i = 0; i < switches.size(); i++) {
    size_t t = switches[i] + numCopies*i;
    for(auto& kv : robotPaths) {
      auto& path = kv.second;

      if(t >= path.size())
        continue;

      auto cfg = path[t];

      path.insert(path.begin()+t,numCopies,cfg);
    }
  }

  for(auto kv : robotPaths) {
    if(m_debug) {
      std::cout << "PATH FOR: " << kv.first->GetLabel() << std::endl;
      std::cout << "PATH LENGTH: " << kv.second.size() << std::endl;
    }

    const std::string filename = this->GetMPProblem()->GetBaseFilename() 
                               + "::FinalPath::" + kv.first->GetLabel();

    std::ofstream ofs(filename);

    for(size_t i = 0; i < kv.second.size(); i++) {
      auto cfg = kv.second[i];
      if(m_debug) {
        std::cout << "\t" << i << ": " << cfg.PrettyPrint() << std::endl;
      }
      ofs << cfg << "\n";
    }
    ofs.close();
    //::WritePath("hypergraph-"+kv.first->GetLabel()+".rdmp.path",kv.second);
  }


  for(auto kv : timeTable) {
    std::cout << "TIMETABLE FOR: " << kv.first->GetLabel() << std::endl;

    const std::string filename = this->GetMPProblem()->GetBaseFilename() 
                               + "::TimeTable::" + kv.first->GetLabel();

    std::ofstream ofs(filename);

    for(auto kv2 : kv.second) {
      ofs << kv2.first->GetLabel() << " (" << kv2.second.first << ":" << kv2.second.second << ")\n";
    }
    ofs.close();
  }











  // Naive way to create paths - will lose synchronization
  // Initialize a decomposition
  auto top = std::shared_ptr<SemanticTask>(new SemanticTask());
  Decomposition* decomp = new Decomposition(top);
  plan->SetDecomposition(decomp);
  
  for(auto kv : robotPaths) {
    auto robot = kv.first;
    auto cfgs = kv.second;

    // Create a motion task
    auto mpTask = std::shared_ptr<MPTask>(new MPTask(robot));

    // Create a semantic task
    const std::string label = robot->GetLabel() + ":PATH";
    auto task = new SemanticTask(label,top.get(),decomp,
                 SemanticTask::SubtaskRelation::AND,false,true,mpTask);

    // Create a task solution
    auto sol = std::shared_ptr<TaskSolution>(new TaskSolution(task));
    sol->SetRobot(robot);

    // Initialize mp solution and path
    auto mpsol = new MPSolution(robot);
    auto rm = mpsol->GetRoadmap(robot);

    std::vector<size_t> vids;
    for(auto cfg : cfgs) {
      auto vid = rm->AddVertex(cfg);
      vids.push_back(vid);
    }
    
    auto path = mpsol->GetPath(robot);
    *path += vids;

    // Save mp solution in task solution
    sol->SetMotionSolution(mpsol);

    // Save task solution in plan
    plan->SetTaskSolution(task,sol);
  }
  plan->Print();
}










void
ScheduledCBS::
ConvertToModifiedPlan(const Node& _node, Plan* _plan) {
  _plan->SetCost(_node.cost);

  // if(!m_writeSolution)
  //   return;
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto solution = mg->GetMPSolution();

  MethodTimer mt(stats,this->GetNameAndLabel() + "::SaveSolution");

  // Collect all of the robots
  std::unordered_map<Robot*,std::vector<Cfg>> robotPaths;
  for(auto kv : _node.solutionMap) {
    auto group = kv.first->GetGroupMotionTask()->GetRobotGroup();
    for(auto robot : group->GetRobots()) {
      robotPaths[robot] = {};
    }
  }

  
  std::vector<SemanticTask*> ordering;

  auto lib = this->GetMPLibrary();

  std::unordered_map<SemanticTask*,std::vector<GroupCfgType>> cfgPaths;

  std::unordered_map<SemanticTask*,size_t> startTimes;
  std::unordered_map<SemanticTask*,size_t> endTimes;
  size_t finalTime = 0;

  // Build in order sequence of tasks
  while(ordering.size() < _node.solutionMap.size()) {

    // Find the set of tasks that are ready to be validated
    for(auto kv :_node.solutionMap) {
      auto task = kv.first;
      lib->SetGroupTask(task->GetGroupMotionTask().get()); 

      // Skip if already validated
      if(std::find(ordering.begin(),ordering.end(),task) != ordering.end())
        continue;

      // Check if dependencies have been validated
      size_t startTime = 0;
      bool ready = true;
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {
          if(std::find(ordering.begin(),ordering.end(),t) == ordering.end()) {
            ready = false;
            break;
          }

          startTime = std::max(endTimes[t],startTime);
        }
      }

      if(!ready)
        continue;

      // Recreate the paths at resolution level
      const auto& path = kv.second;
      const auto cfgs = path->FullCfgsWithWait(lib);
      std::vector<GroupCfgType> oldPath;
      std::vector<GroupCfgType> newPath;
      for(size_t i = 0; i < cfgs.size(); i++) {
        oldPath.push_back(cfgs[i]);
      }

      auto group = task->GetGroupMotionTask()->GetRobotGroup();
      auto grm = solution->GetGroupRoadmap(group);
      lib->GetPathModifier(m_pmLabel)->Modify(grm, oldPath, newPath);
      if(newPath.size() == 0)
        newPath = oldPath;

      std::cout << "size change: " << oldPath.size() << " --> " << newPath.size() << std::endl;

      for(size_t i = 0; i < newPath.size(); i++) {
        cfgPaths[task].push_back(newPath[i]);
      }

      startTimes[task] = startTime;
      // auto timesteps = path->TimeSteps();
      auto timesteps = newPath.size();
      if(timesteps > 0) {
        endTimes[task] = startTime + timesteps; //- 1;
        if(startTime == 0) {
          endTimes[task] = endTimes[task] - 1;
        }
      }
      else {
        endTimes[task] = startTimes[task];
      }

      finalTime = std::max(endTimes[task],finalTime);
      ordering.push_back(task);

      // Update the end times of the preceeding tasks
      for(auto dep : task->GetDependencies()) {
        for(auto t : dep.second) {
          endTimes[t] = startTime;

          if(m_debug) {
            std::cout << "Updating end time of " << t->GetLabel()
                      << " to " << startTime
                      << " because of " << task->GetLabel()
                      << std::endl;
          } 
        }
      }
    
      if(m_debug) {
        std::cout << ">>>>>> SCHEDULE <<<<<<" << std::endl;
        std::cout << task->GetLabel() << ": " << startTimes[task] << " --->>> " << endTimes[task] << std::endl;
        for(auto dep : task->GetDependencies()) {
          for(auto t : dep.second) {
            std::cout << "  " << t->GetLabel() << ": " << startTimes[t] << " --->>> " << endTimes[t] << std::endl;
          }
        }
      }
    }
  }



  // Add the cfgs to the paths
  for(size_t t = 0; t <= finalTime; t++) {
    std::unordered_set<Robot*> used;

    for(auto iter1 = cfgPaths.begin(); iter1 != cfgPaths.end(); iter1++) {
      auto t1 = iter1->first;

      // std::cout << t << " | " << t1->GetLabel() << ": " << startTimes[t1] << ", " << endTimes[t1] << std::endl;
      if(startTimes[t1] > t or endTimes[t1] < t)
        continue;

      // Configure first group at timestep
      const auto& path1  = iter1->second;
      // for (auto c : path1) {
      //   std::cout << "    " << c.PrettyPrint() << std::endl;
      // }
      const size_t step1 = std::min(t - startTimes[t1],path1.size()-1);
      const auto& cfg1   = path1[step1];
      const auto group1  = cfg1.GetGroupRoadmap()->GetGroup();
      for(auto robot : group1->GetRobots()) {
        // Account for overlap at start/end points in tasks
        if(used.count(robot))
          continue;
        used.insert(robot);

        // Hack bc of direspect to path constraints
        auto cfg = cfg1.GetRobotCfg(robot);
        size_t gripperIdx = 0;
        if(robot->GetLabel().find("ur5e") != std::string::npos) {
          if(!robot->GetMultiBody()->IsPassive() &&
                (robot->GetMultiBody()->PosDOF() != 0 or robot->GetMultiBody()->OrientationDOF() != 0) &&
                    group1->Size() > 1) {
            gripperIdx = 1 + robot->GetMultiBody()->PosDOF() + robot->GetMultiBody()->OrientationDOF();
            cfg[gripperIdx] = .0001;
          }
        }
        
        robotPaths[robot].push_back(cfg);
      }
    }
  }





  // Find changes in gripper dof and add buffers
  std::vector<size_t> switches;
  for(size_t t = 1; t < finalTime; t++) {
    for(auto& kv : robotPaths) {
      if(kv.first->GetMultiBody()->IsPassive())
        continue;

      const auto& path = kv.second;

      if(t >= path.size())
        continue;

      auto cfg1 = path[t-1];
      auto cfg2 = path[t];
      
      size_t gripperIdx = 0;
      if(kv.first->GetLabel().find("ur5e") != std::string::npos) {
        if(kv.first->GetLabel().find("mobile") != std::string::npos) {
          gripperIdx = 4;
        }
        else {
          gripperIdx = 1;
        }
      }

      if(abs(cfg1[gripperIdx] - cfg2[gripperIdx]) >= .000001) {
        if(switches.empty() or switches.back() != t-1)
          switches.push_back(t-1);
        switches.push_back(t);
        // std::cout << "Found switch at " << t-1 << std::endl;
        break;
      }
    }
  }

  const size_t numCopies = 5;

  for(size_t i = 0; i < switches.size(); i++) {
    size_t t = switches[i] + numCopies*i;
    for(auto& kv : robotPaths) {
      auto& path = kv.second;

      if(t >= path.size())
        continue;

      auto cfg = path[t];

      path.insert(path.begin()+t,numCopies,cfg);
    }
  }

  for(auto kv : robotPaths) {
    if(m_debug) {
      std::cout << "PATH FOR: " << kv.first->GetLabel() << std::endl;
      std::cout << "PATH LENGTH: " << kv.second.size() << std::endl;
    }

    const std::string filename = this->GetMPProblem()->GetBaseFilename() 
                               + "::FinalPath::" + kv.first->GetLabel() + "::Shortcut";

    std::ofstream ofs(filename);

    for(size_t i = 0; i < kv.second.size(); i++) {
      auto cfg = kv.second[i];
      if(m_debug) {
        std::cout << "\t" << i << ": " << cfg.PrettyPrint() << std::endl;
      }
      ofs << cfg << "\n";
    }
    ofs.close();
  }




  // Naive way to create paths - will lose synchronization
  // Initialize a decomposition
  auto top = std::shared_ptr<SemanticTask>(new SemanticTask());
  Decomposition* decomp = new Decomposition(top);
  plan->SetDecomposition(decomp);
  
  for(auto kv : robotPaths) {
    auto robot = kv.first;
    auto cfgs = kv.second;

    // Create a motion task
    auto mpTask = std::shared_ptr<MPTask>(new MPTask(robot));

    // Create a semantic task
    const std::string label = robot->GetLabel() + ":PATH";
    auto task = new SemanticTask(label,top.get(),decomp,
                 SemanticTask::SubtaskRelation::AND,false,true,mpTask);

    // Create a task solution
    auto sol = std::shared_ptr<TaskSolution>(new TaskSolution(task));
    sol->SetRobot(robot);

    // Initialize mp solution and path
    auto mpsol = new MPSolution(robot);
    auto rm = mpsol->GetRoadmap(robot);

    std::vector<size_t> vids;
    for(auto cfg : cfgs) {
      auto vid = rm->AddVertex(cfg);
      vids.push_back(vid);
    }
    
    auto path = mpsol->GetPath(robot);
    *path += vids;

    // Save mp solution in task solution
    sol->SetMotionSolution(mpsol);

    // Save task solution in plan
    plan->SetTaskSolution(task,sol);
  }
  plan->Print();
}








size_t
ScheduledCBS::
FindStartTime(SemanticTask* _task, std::set<SemanticTask*> _solved, 
              std::map<SemanticTask*,size_t> _endTimes) {
  
  size_t startTime = 0;

  // Check if all dependencies have been solved
  bool missingDep = false;
  for(auto dep : _task->GetDependencies()) {
    for(auto t : dep.second) {
      // Check if dependency has been solved
      if(!_solved.count(t)) {
        // If not, break
        missingDep = true;
        break;
      }
      // Otherwise, update start time
      startTime = std::max(startTime,_endTimes[t]);
    }

    if(missingDep)
      break;
  }

  // Check that task and all dependencies have been solved
  if(missingDep)
    return MAX_UINT;

  return startTime;
}

void
ScheduledCBS::
ComputeIntervals(SemanticTask* _task, const Node& _node) {
  // std::cout << "compute intervals" << std::endl;
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ComputeIntervals");

  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto solution = mg->GetMPSolution();
  auto group = _task->GetGroupMotionTask()->GetRobotGroup();
  auto grm = solution->GetGroupRoadmap(group);

  m_vertexIntervals.clear();
  m_edgeIntervals.clear();

  const auto& constraints = _node.constraintMap.at(_task);

  UnsafeVertexIntervals vertexUnsafeIntervals;
  UnsafeEdgeIntervals edgeUnsafeIntervals;

  for(auto c : constraints) {
    auto edge = c.first;
    if(edge.first == edge.second) {
      vertexUnsafeIntervals[edge.first].push_back(c.second);
    }
    else {
      edgeUnsafeIntervals[edge].push_back(c.second);
    }
  }

  for(auto vit = grm->begin(); vit != grm->end(); vit++) {
    m_vertexIntervals[vit->descriptor()] = ConstructSafeIntervals(vertexUnsafeIntervals[vit->descriptor()]);

    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      m_edgeIntervals[std::make_pair(eit->source(),eit->target())] = ConstructSafeIntervals(
        edgeUnsafeIntervals[std::make_pair(eit->source(),eit->target())]);
    }
  }
}

std::vector<Range<size_t>>
ScheduledCBS::
ConstructSafeIntervals(std::vector<Range<size_t>>& _unsafeIntervals) {
  
  //const size_t m_buffer = 2;

  // Return infinite interval if there are no unsafe intervals
  if(_unsafeIntervals.empty())
    return {Range<size_t>(0,MAX_UINT)};

  // Merge unsafe intervals
  std::vector<Range<size_t>> unsafeIntervals = _unsafeIntervals;

  bool firstTime = true;
  do {

    std::set<size_t> merged;
    
    std::vector<Range<size_t>> copyIntervals = unsafeIntervals;
    const size_t size = copyIntervals.size();

    unsafeIntervals.clear();

    for(size_t i = 0; i < size; i++) {
      if(merged.count(i))
        continue;

      const auto& interval1 = copyIntervals[i];

      size_t zero = 0;
      size_t min = std::max(zero,std::min(interval1.min,interval1.min - (firstTime ? m_buffer : 0)));
      size_t max = std::max(interval1.max,interval1.max + (firstTime ? m_buffer : 0));


      for(size_t j = i+1; j < copyIntervals.size(); j++) {

        const auto& interval2 = copyIntervals[j];

        // Check if there is no overlap
        if(interval2.min > max + m_buffer or min > interval2.max + m_buffer)
          continue;

        
        size_t intervalMin = std::min(interval2.min,interval2.min-m_buffer);
        size_t intervalMax = std::max(interval2.max,interval2.max+m_buffer);

        // If there is, merge the intervals
        min = std::min(min,intervalMin);
        max = std::max(max,intervalMax);

        if(min > max)
          throw RunTimeException(WHERE) << "Upside down interval.";

        merged.insert(j);
      }

      Range<size_t> interval(min,max);
      unsafeIntervals.push_back(interval);
    }

    firstTime = false;

    if(copyIntervals.size() == unsafeIntervals.size())
      break;

  } while(true);

  struct less_than {
    inline bool operator()(const Range<size_t>& _r1, const Range<size_t>& _r2) {
      return _r1.min < _r2.min;
    }
  };

  std::sort(unsafeIntervals.begin(), unsafeIntervals.end(), less_than());
  
  for(size_t i = 1; i < unsafeIntervals.size(); i++) {
    if(unsafeIntervals[i-1].max > unsafeIntervals[i].max
      or unsafeIntervals[i-1].max > unsafeIntervals[i].min)
      throw RunTimeException(WHERE) << "THIS IS VERY BAD AND WILL RESULT IN AN INFINITE LOOP WITH NO ANSWERS.";
  }

  // Construct set of intervals
  std::vector<Range<size_t>> intervals;

  size_t min = 0;
  size_t max = std::numeric_limits<size_t>::infinity();

  auto iter = unsafeIntervals.begin();
  while(iter != unsafeIntervals.end()) {
    max = std::min(iter->min - 1,iter->min);
    if(min < max)
      intervals.emplace_back(min,max);
  
    min = std::max(iter->max + 1,iter->max);
    iter++;
  }

  if(min == SIZE_MAX) {
    if(m_debug) {
      for(auto inter : intervals) {
        std::cout << inter << std::endl;
      }
    }
    return intervals;
  }

  max = MAX_UINT;
  intervals.push_back(Range<size_t>(min,max));

  if(m_debug) {
    for(auto inter : intervals) {
      std::cout << inter << std::endl;
    }
  }

  return intervals;
}

/*------------------ Critical Path Functions -----------------*/

void
ScheduledCBS::
BuildScheduleGraph(Plan* _plan) {

  // Clear old schedule
  m_scheduleGraph.reset(new ScheduleGraph(_plan->GetCoordinator()->GetRobot()));

  // Add root node
  auto root = m_scheduleGraph->AddVertex(nullptr);

  // Collect tasks with dependents
  std::set<size_t> parentTasks;

  // Add all tasks to graph
  for(auto task : _plan->GetDecomposition()->GetGroupMotionTasks()) {
    auto vid = m_scheduleGraph->AddVertex(task);
    // std::cout << task->GetLabel() << std::endl;
    // std::cout << "is dependent on" << std::endl;
    for(auto dep : task->GetDependencies()) {
      for(auto t : dep.second) {
        // std::cout << t->GetLabel() << std::endl;
        auto depVID = m_scheduleGraph->AddVertex(t);
        parentTasks.insert(depVID);

        m_scheduleGraph->AddEdge(vid,depVID,size_t(1));
      }
    }
    // std::cout << "-=-=-=" << std::endl;
  }

  // Add edges from root to tasks with no dependencies
  for(auto vit = m_scheduleGraph->begin(); vit != m_scheduleGraph->end(); vit++) {
    auto vid = vit->descriptor();
    if(parentTasks.count(vid) or vid == root)
      continue;

    m_scheduleGraph->AddEdge(root,vid,size_t(1));
  }

  // Compute atomic distances
  ComputeScheduleAtomicDistances();

  if(m_debug) {
    std::cout << "Built Schedule Graph" << std::endl;

    for(auto vit = m_scheduleGraph->begin(); vit != m_scheduleGraph->end(); vit++) {

      std::cout << vit->descriptor() 
                << ":"
                << m_scheduleAtomicDistances[vit->descriptor()]
                << "\tname: " 
                << (vit->property() != nullptr ? vit->property()->GetLabel() : "root")
                << std::endl
                << "\t";

      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        std::cout << eit->target() << ", ";
      }
    }
  }
}

void
ScheduledCBS::
ComputeScheduleAtomicDistances() {

  m_scheduleAtomicDistances.clear();

  SSSPPathWeightFunction<ScheduleGraph> weight = [this](
      typename ScheduleGraph::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {
    return 1.;
    // return _sourceDistance + 1.;
  };
  
  auto sssp = DijkstraSSSP(m_scheduleGraph.get(),{0},weight);

  for(auto kv : sssp.distance) {
    m_scheduleAtomicDistances[kv.first] = kv.second;
  }
}

std::vector<std::vector<size_t>>
ScheduledCBS::
ComputeCriticalPaths(const Node& _node) {

  auto slack = ComputeScheduleSlack(_node);

  if(m_debug) {
    std::cout << "Slack values" << std::endl;
    for(auto kv : slack) {
      auto task = m_scheduleGraph->GetVertex(kv.first);
      auto slack = kv.second;
      std::cout << (task ? task->GetLabel() : "root") 
                << " : " 
                << slack 
                << std::endl;
    }
  }

  return {};
}

std::unordered_map<size_t,double>
ScheduledCBS::
ComputeScheduleSlack(const Node& _node) {

  SSSPPathWeightFunction<ScheduleGraph> weight = [this,_node](
      typename ScheduleGraph::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {
    
    auto source = this->m_scheduleGraph->GetVertex(_ei->source());
    // auto target = this->m_scheduleGraph->GetVertex(_ei->target());

    // Check if source is root vertex
    if(!source)
      return 0.;

    // Otherwise compute slack between parent task (target) and child task (source)
    // auto child = _node.solutionMap.at(source);
    // auto parent = _node.solutionMap.at(target);

    // auto start = m_startTimes[child];
    // auto end = m_startTimes[parent] + parent->TimeSteps();

    // auto additionalSlack = start - end;

    return 1.;
    // return double(additionalSlack) + _sourceDistance;
  };

  auto sssp = DijkstraSSSP(m_scheduleGraph.get(),{0},weight);
  return sssp.distance;
}

/*------------------------------------------------------------*/
