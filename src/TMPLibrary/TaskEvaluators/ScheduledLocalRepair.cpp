#include "ScheduledLocalRepair.h"

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

ScheduledLocalRepair::
ScheduledLocalRepair() {
  this->SetName("ScheduledLocalRepair");
}

ScheduledLocalRepair::
ScheduledLocalRepair(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("ScheduledLocalRepair");

  m_vcLabel = _node.Read("vcLabel",true,"",
         "Validity checker to use for multi-robot collision checking.");
    
  m_queryStrategy = _node.Read("queryStrategy",true,"",
         "MPStrategy to sue to query individial paths.");
  
  m_initialQueryStrategy = _node.Read("initialQueryStrategy",true,"",
          "MPStrategy to sue to query individial paths.");
  
  m_repairQueryStrategy = _node.Read("repairQueryStrategy",true,"",
          "MPStrategy to sue to query individial paths.");
  
  m_repairExpansionStrategy = _node.Read("repairExpansionStrategy",true,"",
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
  
  m_pathValidationMode = _node.Read("pathValidationMode",false,m_pathValidationMode,
          "Flag to use bypass strategy.");
  
  m_pmLabel = _node.Read("pmLabel", false, "", "Path Modifier Method");

  m_windowSize = _node.Read("windowSize", true, m_windowSize,
    size_t(1), std::numeric_limits<size_t>::max(),
    "The window size");

  m_queriesIncrement = _node.Read("increment",true, m_queriesIncrement, size_t(0), std::numeric_limits<size_t>::max(), "The increment");

}

ScheduledLocalRepair::
~ScheduledLocalRepair() {

}

/*------------------------ Overrides -------------------------*/
void
ScheduledLocalRepair::
Initialize() {
  std::cout << "Initialize scheduler" << std::endl;
  m_startTimes.clear();
  m_endTimes.clear();
 
  m_scheduleAtomicDistances.clear();

  m_geometricConstraintSet.clear();
  m_motionConstraintSet.clear();
  std::cout << "Initialize scheduler" << std::endl;
}
    
void
ScheduledLocalRepair::
SetUpperBound(double _upperBound) {
  m_upperBound = _upperBound;
}

bool
ScheduledLocalRepair::
Run(Plan* _plan) {
  std::cout << "Start ScheduledLR" << std::endl;
  if(!_plan)
    _plan = this->GetPlan();

  m_conflictCount.clear();

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  MethodTimer mt_ind(stats,this->GetNameAndLabel() + "::Run_ScheduledLocalRepair");

  m_quit = false;
  BuildScheduleGraph(_plan);

  // Collect tasks
  auto decomp = _plan->GetDecomposition();
  auto tasks = decomp->GetGroupMotionTasks();

  // Call CBS
  std::cout << "Start Repair" << std::endl;
  auto solution = Repair(_plan);

  if(m_motionConstraintSet.size() > 0)
    return false;

  // Check if solution was found
  if(!solution) {
    // if(solution.cost == std::numeric_limits<double>::infinity()) {
    std::cout << "Failed to repair conflicts" << std::endl;
    _plan->SetCost(std::numeric_limits<double>::infinity());
    return false;
  }

  std::cout << "Repair Successful" << std::endl;
  ConvertToPlan(_plan);

  // if(m_pmLabel != "")
  //   ConvertToModifiedPlan(solution,_plan);

  return true;
}


/*----------------------- CBS Functors -----------------------*/


bool
ScheduledLocalRepair::
Repair(Plan* _plan) {
  // _plan->SetCost(_node.cost);
  // auto lib = this->GetMPLibrary();
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  // auto sq = dynamic_cast<ModeHyperpathQuery*>(this->GetTaskEvaluator(m_sqLabel).get());

  MethodTimer mt(stats,this->GetNameAndLabel() + "::Repair");

  // Query all paths for initialization
  InitialSolutionFunction();

  if(m_pathValidationMode)
    return true;


  // Validate paths
  auto constraintSet = ValidationFunction();

  // Check for geometric constraints (v-v or v-h) before resolving h-h conflicts
  auto termination = EarlyTermination();
  if(termination) {
    // Pass it down to task planner to resolve the geometric conflicts
    return false;
  }

  // h-h conflicts detected
  while(constraintSet.size() > 0) {
    // UpdateLocalProblem();

    std::cout << "\n\n++++++++++ Validate the path ++++++++++" << std::endl;
    constraintSet = ValidationFunction();
    std::cout << "\nFound constraints " << constraintSet.size() << std::endl;
    for(auto c : constraintSet) {
      std::cout << "At " << c.second << std::endl;
      for(auto t : c.first) {
        std::cout << "\t" << t->GetLabel() << std::endl;
      } 
    }

    // Repair;
    std::cout << "\n\n++++++++++ Solve subproblems ++++++++++" << std::endl;
    if(SolveSubProblem(constraintSet)) {
      std::cout << "Successfully solve all subproblems!" << std::endl;
    }
    std::cout << "Start validating new path" << std::endl;
  }

  m_motionConstraintSet.clear();
  std::cout << "All Conflict have resolved" << std::endl;
  return true;
}


bool 
ScheduledLocalRepair::
SolveSubProblem(std::vector<std::pair<std::set<SemanticTask*>,size_t>> _conflicts) {
  if(!_conflicts.size())
    return true;
  
  std::vector<std::string> taskLabels;
  for(auto pair : _conflicts) {
    for(auto st : pair.first) {
      taskLabels.push_back(st->GetLabel());
    }
  }
  std::sort(taskLabels.begin(), taskLabels.end());

  std::string labels = "";
  for(std::string label : taskLabels) {
    labels += label;
  }

  if(m_conflictCount.count(labels))
    m_conflictCount[labels] += 1;
  else
    m_conflictCount[labels] = 1;

  bool repairFull = false;
  if(m_conflictCount[labels] >= 3) {
    repairFull = true;
  }
  if(m_conflictCount[labels] >= 20) {
    assert(10==1);
  }

  repairFull = true;
  
  auto problem = this->GetMPProblem();
  auto solution = this->GetMPSolution();
  auto lib = this->GetMPLibrary();
  // auto vc = lib->GetValidityChecker(m_vcLabel);
  auto sm = lib->GetSampler("UniformRandomFree");
  auto dm = lib->GetDistanceMetric("euclidean");
  bool includePassive = true;
  lib->SetTask(nullptr);
  

  std::vector<Robot*> robots;
  std::set<RobotGroup*> groups;
  // std::unordered_map<SemanticTask*,size_t> startTimes;
  std::string robotLabel = "";
  for(auto pair : _conflicts) {
    for(auto st : pair.first) {
      auto groupTask = st->GetGroupMotionTask();
      auto group = groupTask->GetRobotGroup();
      groups.insert(group);
      for(auto iter = groupTask->begin(); iter != groupTask->end(); iter++) {
        if(!includePassive and iter->GetRobot()->GetMultiBody()->IsPassive())
          continue;
        robots.push_back(iter->GetRobot());
        robotLabel += iter->GetRobot()->GetLabel() + "::";
      }
    }
  }
  RobotGroup* compositeRobotGroup = problem->AddRobotGroup(robots, robotLabel);
  solution->AddRobotGroup(compositeRobotGroup);
  auto compositeGroupRoadmap = solution->GetGroupRoadmap(compositeRobotGroup);
  compositeGroupRoadmap->SetAllFormationsInactive();
  auto compositePath = new GroupPathType(compositeGroupRoadmap);


  // std::cout << "Composite group is: " << robotLabel << std::endl;
  size_t lowerBound = -MAX_UINT;
  size_t upperBound = MAX_UINT;
  for(auto pair : _conflicts) {
    for(auto st : pair.first) {
      auto path = m_solutionMap.at(st);
      // std::cout << m_startTimes[path] << ":" << m_endTimes[path] << std::endl;
      lowerBound = std::max(lowerBound, m_startTimes[path]);
      upperBound = std::min(upperBound, m_endTimes[path]);
    }
  }
  // std::cout << "Collect min and max timesteps: " << lowerBound << ":" << upperBound << std::endl;

  // bool solved = false;

  size_t iteration = 0;
  size_t collisionTime = 0;
  size_t windowSize = m_windowSize;
  std::unordered_map<SemanticTask*,std::pair<size_t,size_t>> idxMap;

  while(true) {
    std::cout << "================= Iteration " << iteration << " with window size " << windowSize << " =================" << endl;
    iteration++;

    
    // auto compositeGroupTask = new GroupTask(compositeRobotGroup);
    // auto compositeGroupTask = std::shared_ptr<GroupTask>(new GroupTask(compositeRobotGroup));
    GroupTask compositeGroupTask(compositeRobotGroup);
    lib->SetGroupTask(&compositeGroupTask);
    
    // Set robots not virtual
    for(auto& robot : problem->GetRobots()) 
      robot->SetVirtual(true);
    for(auto robot : compositeRobotGroup->GetRobots())
      robot->SetVirtual(false);
    
    size_t count = 1;
    bool increaseWindow = false;

    size_t maxLength = 0;

    std::map<Robot*,const Boundary*> startConstraintMap;
    std::map<Robot*,const Boundary*> goalConstraintMap;
  
    for(auto pair : _conflicts) {
      std::cout << "Start Solving (" << count << "/" << _conflicts.size() << ") problem with window size " << windowSize << std::endl;
      count++;

      collisionTime = pair.second;

      for(auto st : pair.first) {
        auto groupTask = st->GetGroupMotionTask();
        auto group = groupTask->GetRobotGroup();
        auto path = m_solutionMap.at(st);
        size_t startTime = std::max(lowerBound, std::max(size_t(0),collisionTime-windowSize));
        // size_t goalTime = std::min(upperBound, collisionTime+windowSize);
        // size_t goalTime = upperBound;
        size_t goalTime = std::min(m_endTimes[path], collisionTime+windowSize);

        size_t startIdx = std::max(startTime - m_startTimes[path], size_t(0));
        size_t goalIdx = std::min(goalTime - m_startTimes[path], path->FullCfgsWithWait(lib).size()-1);

        maxLength = std::max(maxLength, path->FullCfgsWithWait(lib).size()-1);

        if(repairFull) {
          std::cout << "\nFull Repair\n " << std::endl;
          startIdx = 0;
          goalIdx = path->FullCfgsWithWait(lib).size() - 1;
        }

        idxMap[st] = std::make_pair(startIdx,goalIdx);
        
        std::cout << "Global Timestep: " << lowerBound << " ... " << startTime << " (" << collisionTime << ") " << goalTime << " ... " << m_endTimes[path] << std::endl;
        std::cout << "Path Timestep: 0 ... " << startIdx << ":" << goalIdx << " ... " << path->FullCfgsWithWait(lib).size()-1 << std::endl;

        std::cout << group->GetLabel() << std::endl;
    

        // auto sampleVID = path->VIDs()[0];
        // auto formations = solution->GetGroupRoadmap(group)->GetVertex(sampleVID).GetFormations();
        // std::cout << "Formation size: " << formations.size() << std::endl;
        // for(auto f : formations) {
        //   compositeGroupRoadmap->AddFormation(f);
        //   compositeGroupRoadmap->SetFormationActive(f);
        // }

        auto exampleGcfg = path->FullCfgsWithWait(lib)[0];
        auto formations = exampleGcfg.GetFormations();
        for(auto f : formations) {
          compositeGroupRoadmap->AddFormation(f);
          compositeGroupRoadmap->SetFormationActive(f);
        }
        


        // TODO: Rather finding a boundary map ?
        for(auto iter = groupTask->begin(); iter != groupTask->end(); iter++) {
          auto robot = iter->GetRobot();
          std::cout << robot->GetLabel() << std::endl;
          if(robot->GetMultiBody()->IsPassive()) // Passive cfg is determined by formation constraints in the roadmap. Skip configuring tasks
            continue;

          auto startCfg = path->FullCfgsWithWait(lib)[startIdx].GetRobotCfg(robot);
          auto goalCfg = path->FullCfgsWithWait(lib)[goalIdx].GetRobotCfg(robot);
          auto startBoundary = new CSpaceConstraint(robot,startCfg);
          auto goalBoundary = new CSpaceConstraint(robot,goalCfg);
          startConstraintMap[robot] = startBoundary->GetBoundary();
          goalConstraintMap[robot] = goalBoundary->GetBoundary();

          // Get Robot and its dependent robot's cfgs based on the leader boundary
          for(auto f : formations) {
            auto startCfgs = f->GetRandomFormationCfg(startConstraintMap[robot]);
            auto goalCfgs = f->GetRandomFormationCfg(goalConstraintMap[robot]);
            std::cout << "\tCheck start formation" << std::endl;            
            for(auto c : startCfgs) {
              auto r = c.GetRobot();
              if(r == robot)
                continue;
              auto cSpace = new CSpaceConstraint(r,c);
              std::cout << "\t\t" << r->GetLabel() << ": " << cSpace->GetBoundary()->GetCenter() << std::endl;
              startConstraintMap[r] = cSpace->GetBoundary();
            }
            std::cout << "\tCheck goal formation" << std::endl;            
            for(auto c : goalCfgs) {
              auto r = c.GetRobot();
              if(r == robot)
                continue;
              auto cSpace = new CSpaceConstraint(r,c);
              std::cout << "\t\t" << r->GetLabel() << ": " << cSpace->GetBoundary()->GetCenter() << std::endl;
              goalConstraintMap[r] = cSpace->GetBoundary();
            }
          }

          std::cout << "\tUpdated Start" << std::endl;
          for(auto kv : startConstraintMap) {
            std::cout << "\t\t" << kv.first->GetLabel() << ": " << kv.second->GetCenter() << std::endl;
          }  
          std::cout << "\tUpdated Goal" << std::endl;
          for(auto kv : goalConstraintMap) {
            std::cout << "\t\t" << kv.first->GetLabel() << ": " << kv.second->GetCenter() << std::endl;
          }  

        }
      }
    }

    std::cout << "********** " << maxLength << " " << windowSize << " ****************" << std::endl;
    if(maxLength < windowSize) {
      std::cout << "full repair" << std::endl;
      repairFull = true;
    }

    std::cout << "Done setting boundaries" << std::endl;

    std::vector<GroupCfgType> startSamples;
    sm->Sample(1,1,startConstraintMap,std::back_inserter(startSamples));
    // std::cout << startSamples.size() << std::endl;
    if(startSamples.size() == 0) {
      std::cout << "Start invalid" << std::endl;
      increaseWindow = true;
    }
    

    std::vector<GroupCfgType> goalSamples;
    sm->Sample(1,1,goalConstraintMap,std::back_inserter(goalSamples));
    // std::cout << goalSamples.size() << std::endl;
    if(goalSamples.size() == 0) {
      std::cout << "Goal invalid" << std::endl;
      increaseWindow = true;
    }


    if(increaseWindow) {
      windowSize += m_windowSize;
      increaseWindow = false;
      // std::cout << "Increasing window size" << std::endl;
      continue;
    }

    auto startCompositeGcfg = startSamples[0];
    auto goalCompositeGcfg = goalSamples[0];

    compositeGroupRoadmap->AddVertex(startCompositeGcfg);
    compositeGroupRoadmap->AddVertex(goalCompositeGcfg);
    
    std::cout << "Done sampling" << std::endl;

    std::cout << "Setting Group Task Info" << std::endl;
    for(auto pair : _conflicts) {
      for(auto st : pair.first) {
        auto groupTask = st->GetGroupMotionTask();

        auto group = groupTask->GetRobotGroup();
        std::cout << group->GetLabel() << std::endl;

        for(auto iter = groupTask->begin(); iter != groupTask->end(); iter++) {
          auto robot = iter->GetRobot();
          auto startBoundary = startConstraintMap[robot];
          auto goalBoundary = goalConstraintMap[robot];
          auto satrtConstraint = std::unique_ptr<BoundaryConstraint>(
                      new BoundaryConstraint(robot,std::move(startBoundary->Clone())));
          auto goalConstraint = std::unique_ptr<BoundaryConstraint>(
                      new BoundaryConstraint(robot,std::move(goalBoundary->Clone())));

          std::cout << "\t" << robot->GetLabel() << std::endl;
          std::cout << "\t\tStart " << startBoundary->GetCenter() << std::endl;
          std::cout << "\t\tGoal " << goalBoundary->GetCenter() << std::endl;

          MPTask t(robot);
          if(iter->GetPathConstraints().size() > 0) {
            for(const auto& c : iter->GetPathConstraints()) {
              if(c->GetRobot() != robot)
                continue;
              auto pathConstraint = c->Clone();
              t.AddPathConstraint(std::move(pathConstraint));
            }
          }
          t.SetStartConstraint(std::move(satrtConstraint));
          t.AddGoalConstraint(std::move(goalConstraint));

          compositeGroupTask.AddTask(t);
        }
      }
    }
    // Ensure that all groups are in solution
    solution->AddRobotGroup(compositeGroupTask.GetRobotGroup());

    std::cout << "Start and Goal" << std::endl;
    std::cout << "\tStart: " << startCompositeGcfg.PrettyPrint() << std::endl;
    std::cout << "\tGoal: " << goalCompositeGcfg.PrettyPrint() << std::endl;

    // lib->SetGroupTask(compositeGroupTask);
    lib->SetTask(nullptr);

    // Call the MPLibrary solve function to expand the roadmap
    std::cout << "Generating Roadmap ... " << std::endl;
    lib->SetPreserveHooks(true);
    lib->Solve(problem, &compositeGroupTask,solution, m_repairExpansionStrategy, LRand(), 
            "ExpandModeRoadmap");

    std::cout << "Solving Problem ... " << std::endl;
    lib->Solve(problem, &compositeGroupTask, solution, m_repairQueryStrategy, LRand(), "");

    lib->SetPreserveHooks(false);

    compositePath = solution->GetGroupPath(compositeRobotGroup);
    if(compositePath->VIDs().size() > 0) {
      std::cout << "Found a solution with window size " << windowSize << std::endl;
      for(auto cfg : compositePath->Cfgs()) 
        std::cout << "\t" << cfg.PrettyPrint() << std::endl;
      std::cout << "Intervals: " << std::endl;
      compositePath->PrintIntervals();
      
      std::cout << "Timesteps: " << compositePath->TimeSteps() << std::endl;
      break;
    }

    // std::cout << "Failed to solve the subproblem with window size " << windowSize << std::endl;
    windowSize += m_windowSize;
  }

  for(auto& robot : problem->GetRobots())
    robot->SetVirtual(false);




  // std::cout << "\nRepair paths" << std::endl;
  std::unordered_map<Robot*,std::vector<std::pair<size_t,size_t>>> individualVIDs;
  for(auto kv : idxMap) {
    auto task = kv.first;
    auto group = task->GetGroupMotionTask()->GetRobotGroup();
    auto grm = solution->GetGroupRoadmap(group);
    auto path = m_solutionMap[task];
    std::cout << "\n== Group: " << group->GetLabel() << std::endl;
    std::cout << "Original vids: " << path->VIDs() << std::endl;
    std::cout << "Original Path: " << std::endl;
    for(auto cfg : path->Cfgs()) {
      std::cout << cfg.PrettyPrint() << std::endl;
    }
    std::cout << "Intervals" << std::endl;
    path->PrintIntervals();

    auto segment = kv.second;
    auto segmentStartVID = path->GetEdgeAtTimestep(segment.first).first; // vid
    auto segmentEndVID = path->GetEdgeAtTimestep(segment.second).second; // vid
    std::cout << "Repair happens between " << segmentStartVID << " " << segmentEndVID << std::endl;
    
    
    auto segmentStartIdx = path->GetEdgeAtTimestepInIndices(segment.first).first; // index
    auto segmentEndIdx = path->GetEdgeAtTimestepInIndices(segment.second).second; // index
    std::cout << "Indices are " << segmentStartIdx << " " << segmentEndIdx << std::endl;
    std::cout << "Adding " << compositePath->VIDs() << " between them" << std::endl;

    
    // Check if the repair starts and ends at the very first and end of the original path 
    size_t startBuffer = 0;
    size_t endBuffer = 0;
    auto intervals = path->GetIntervals();
    // size_t currentTimestep = 0;
    if(!repairFull and intervals[segmentStartIdx].first != segment.first) {
      startBuffer = segment.first - intervals[segmentStartIdx].first;
      std::cout << segmentStartVID << " at time step " << intervals[segmentStartIdx].first << " and segment start " << segment.first << std::endl;
    }
    else {
      std::cout << "start coincides" << std::endl;
    }
    if(!repairFull and intervals[segmentEndIdx].first != segment.second) {
      endBuffer = intervals[segmentEndIdx].first - segment.second;
      std::cout << segmentEndVID << " at time step " << intervals[segmentEndIdx].first << " and segment end " << segment.second << std::endl;
    }
    else {
      std::cout << "end coincides" << std::endl;
    }

    std::cout << "segmentStartIdx: " << segmentStartIdx 
                    << ", segmentEndIdx: " << segmentEndIdx
                    << ", startBuffer: " << startBuffer
                    << ", endBuffer: " << endBuffer << std::endl;
    for(size_t i = 0 ; i < path->VIDs().size() ; i++) {
      std::cout << i << " -----  " << std::endl;
      bool add = true;

      if(segmentStartIdx < i and i < segmentEndIdx) {
        std::cout << "\tSkip" << std::endl;
        continue;
      }

      if(i==segmentStartIdx and startBuffer == 0) {
        std::cout << "\tSkip start is true" << std::endl;
        add = false;
      }
      if(i==segmentEndIdx and endBuffer == 0) {
        std::cout << "\tSkip end is true" << std::endl;
        add = false;
      }
      
      auto gcfg = path->Cfgs()[i];
      GroupCfgType prevGcfg(grm);

      if(add) {
        // std::cout << "\tAdding Original vid: " << std::endl;
        size_t duration = path->GetDurations()[i];
        if(i==segmentStartIdx)
            duration = startBuffer;

        for(auto robot : grm->GetGroup()->GetRobots()) {
          auto vid = gcfg.GetVID(robot);
          std::cout << "\t\t" << robot->GetLabel() 
                    << " ( vid " << vid << ", duration " << duration << " ) " 
                    << gcfg.GetRobotCfg(robot).PrettyPrint() << std::endl;
          individualVIDs[robot].push_back(std::make_pair(vid,duration)); 
          prevGcfg = gcfg;
        }
      }


      if(i == segmentStartIdx) {
        // std::cout << "\tAdding intermediate points: " << compositePath->Cfgs().size() << std::endl;
        for(size_t j = 0 ; j < compositePath->Cfgs().size() ; j ++) {
          auto gCfg = compositePath->Cfgs()[j];
          auto extraCfg = gCfg;
          size_t duration = compositePath->GetDurations()[j];
          if(j == compositePath->Cfgs().size() - 1) {
            duration = endBuffer;
            std::cout << "oh! " <<  duration << std::endl;
          }

          for(auto robot : grm->GetGroup()->GetRobots()) {
            auto vid = extraCfg.GetVID(robot);
            // std::cout << "\t\t" << robot->GetLabel() 
            //           << " ( vid " << vid << ", duration " << duration << " ) " 
            //           << extraCfg.GetRobotCfg(robot).PrettyPrint() << std::endl;
            individualVIDs[robot].push_back(std::make_pair(vid,duration));
            // intervalMap[robot].push_back(std::pair(currentTimestep,currentTimestep+));
          }

          prevGcfg = extraCfg;
        }
      }
    }
  }


  // std::cout << "\nOriginal Full Path" << std::endl;
  // for(auto pair : _conflicts) {
  //   for(auto st : pair.first) {
  //     auto path = m_solutionMap[st];
  //     auto group = st->GetGroupMotionTask()->GetRobotGroup();
  //     std::cout << "\n" << group->GetLabel() << ": Timesteps " << path->TimeSteps() << std::endl;
  //     for(auto gcfg : path->Cfgs()) {
  //       std::cout << "\t" << gcfg.PrettyPrint() << std::endl;
  //     }
  //     std::cout << "Intervals: " << std::endl;
  //     path->PrintIntervals();
  //   }
  // }
  
  // std::cout << "\nNew Path" << std::endl;
  // for(auto kv : individualVIDs) {
  //   std::cout << kv.first->GetLabel() << ": " << std::endl;
  //   for(auto pair : kv.second) {
  //     std::cout << "\tvid " << pair.first << ": duration " << pair.second << std::endl;
  //   }
  // }

  std::unordered_map<Robot*, std::vector<size_t>> resultingVIDs;
  std::unordered_map<Robot*, std::vector<size_t>> edgeDurations;
  std::unordered_map<Robot*, std::vector<size_t>> waitTimes;
  ProcessVIDsWithRepeatSupport(individualVIDs, resultingVIDs, edgeDurations, waitTimes, groups);

  // std::cout << "\nOrganized Path" << std::endl;
  // for (const auto& kv : resultingVIDs) {
  //   Robot* robot = kv.first;
  //   const auto& vids = kv.second;
  //   const auto& durations = edgeDurations[robot];
  //   const auto& waits = waitTimes[robot];
  
  //   std::cout << robot->GetLabel() << ":\n";
  //   std::cout << "\tvids: " << vids << std::endl;
  //   std::cout << "\tdurations: " << durations << std::endl;
  //   std::cout << "\twaits: " << waits << std::endl;
  
  //   for (size_t i = 0; i < durations.size(); ++i) {
  //     std::cout << "\t\t" << vids[i] << " (" << waits[i] << ") --> " 
  //               << vids[i + 1] << " (" << waits[i + 1] << "): " 
  //               << " duration " << durations[i] << std::endl;
  //   }
  // }
  


  // Composite Path
  // std::cout << "\nConstructing Composite path" << std::endl;
  std::unordered_map<Robot*,size_t> timeStepMap;
  for(auto kv : resultingVIDs) {
    auto robot = kv.first;
    timeStepMap[robot] = kv.second.size();
  }

  for(auto pair : _conflicts) {
    for(auto st : pair.first) {
      auto groupTask = st->GetGroupMotionTask();
      auto group = groupTask->GetRobotGroup();
      auto robots = group->GetRobots();
      auto grm = solution->GetGroupRoadmap(group);

      // Merge individual cfgs into group cfgs.
      size_t previousVID = MAX_UINT;
      GroupCfgType previousCfg;
      std::vector<size_t> compositeVIDs;

      size_t maxLength = 0;
      for(auto robot : robots) {
        if(robot->GetMultiBody()->IsPassive())
          continue;
        maxLength = std::max(maxLength, timeStepMap[robot]);
      }

      // std::cout << "\n" << group->GetLabel() << " " << maxLength << std::endl;
      Robot* keyRobot = nullptr;
      for(size_t i = 0; i < maxLength; i++) {
        GroupCfgType gcfg(grm);
        
        for(auto kv : resultingVIDs) {
          auto robot = kv.first;
          if(std::find(robots.begin(), robots.end(), robot) == robots.end())
            continue;
          size_t vid = kv.second[i];
          // std::cout << "\t" << i << ") " << robot->GetLabel() << ", vid " << vid << std::endl;
          gcfg.SetRobotCfg(robot,vid);

          keyRobot = robot;
        }

        auto gvid = grm->AddVertex(gcfg);
        compositeVIDs.push_back(gvid);

        if(i == 0) {
          previousVID = gvid;
          previousCfg = gcfg;
          continue;
        }

        // Connect gcfg to previous
        auto distance = dm->Distance(previousCfg,gcfg);
        GroupLocalPlanType glp(grm, "sl");
        glp.SetWeight(distance);
        glp.SetTimeSteps(edgeDurations[keyRobot][i-1]);

        for(auto robot : group->GetRobots()) {
          std::vector<Cfg> intermediates = {previousCfg.GetRobotCfg(robot),
                                          gcfg.GetRobotCfg(robot)};

          DefaultWeight<Cfg> individualEdge("",distance,intermediates);
          glp.SetEdge(robot,std::move(individualEdge));
          // std::cout << "\t\t" << robot->GetLabel() << " " << previousCfg.GetRobotCfg(robot).PrettyPrint() 
          //                     << " --> " << gcfg.GetRobotCfg(robot).PrettyPrint() << std::endl;
        }
        if(gvid != previousVID) {
          grm->AddEdge(previousVID,gvid,glp);
        }
        
        previousVID = gvid;
        previousCfg = gcfg;
      }

      // std::cout << "Done" << std::endl;
      
      auto originalPath = m_solutionMap[st];
      originalPath->Clear();
      *originalPath += compositeVIDs;
      originalPath->SetWaitTimes(waitTimes[keyRobot]);
      std::cout << "wait times " << originalPath->GetWaitTimes() << std::endl;
      
      size_t timesteps = originalPath->FullCfgsWithWait(lib).size();
      m_endTimes[originalPath] = m_startTimes[originalPath] + timesteps;



      // std::cout << "Composite Path for " << group->GetLabel() << ": " << m_startTimes[originalPath] << ":" << m_endTimes[originalPath] << std::endl;
      // for(size_t i = 0 ; i < originalPath->Cfgs().size() ; ++i) {
      //   auto cfg = originalPath->Cfgs()[i];
      //   std::cout << "\t" << m_startTimes[originalPath] + i << ": " << cfg.PrettyPrint() << std::endl;
      // }
      // std::cout << "Intervals:" << std::endl;
      // originalPath->PrintIntervals();
    }
  }

  std::cout << "\nConflict resolution at " << collisionTime << std::endl;
  std::cout << "Conflict count" << std::endl;
  for(auto kv : m_conflictCount) {
    std::cout << kv.first << ": " << kv.second << std::endl;
  }

  return true;
}


void 
ScheduledLocalRepair::
ProcessVIDsWithRepeatSupport(
  const std::unordered_map<Robot*, std::vector<std::pair<size_t, size_t>>>& _originalVIDs,
  std::unordered_map<Robot*, std::vector<size_t>>& _resultingVIDs,
  std::unordered_map<Robot*, std::vector<size_t>>& _edgeDurations,
  std::unordered_map<Robot*, std::vector<size_t>>& _waitTimes,
  std::set<RobotGroup*> _groups) {

  std::unordered_map<Robot*,std::vector<size_t>> activeIndexMap;

  for (const auto& kv : _originalVIDs) {
    Robot* robot = kv.first;
    if(robot->GetMultiBody()->IsPassive())
      continue;
    const auto& original = kv.second;

    std::vector<size_t> vids;
    std::vector<size_t> durations;
    std::vector<size_t> waits;

    if(original.size() < 2) {
      std::cout << "size < 2" << std::endl;
      continue;
    } 

    size_t current_vid = original[0].first;
    size_t wait_sum = 0;

    vids.push_back(current_vid);
    activeIndexMap[robot].push_back(0);

    for (size_t i = 1; i < original.size(); ++i) {
      size_t next_vid = original[i].first;
      size_t duration = original[i-1].second;
      std::cout << next_vid << ": " << duration << std::endl;

      if(next_vid == current_vid) {
        wait_sum += duration;
      } 
      else {
        durations.push_back(duration);   // edge from current_vid to next_vid
        waits.push_back(wait_sum);       // wait time at current_vid
        current_vid = next_vid;
        vids.push_back(current_vid);
        activeIndexMap[robot].push_back(i);
        wait_sum = 0;
      }
    }

    // Terminal node
    waits.push_back(wait_sum);

    _resultingVIDs[robot] = std::move(vids);
    _edgeDurations[robot] = std::move(durations);
    _waitTimes[robot] = std::move(waits);
  }

  for(auto group : _groups) {
    Robot* active = nullptr;
    Robot* passive = nullptr;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive()) {
        passive = robot;
      }
      else {
        active = robot;
      }
    }

    if(active and passive) {
      std::vector<size_t> vids;
      std::vector<size_t> durations;
      std::vector<size_t> waits;

      // Active and Passive in the same group must be paired
      for (size_t val : _waitTimes[active]) {
        waits.push_back(val);
      }
      
      for (size_t val : _edgeDurations[active]) {
        durations.push_back(val);
      }

        // But vids must be different
      for(size_t idx : activeIndexMap[active]) {
        vids.push_back(_originalVIDs.at(passive)[idx].first);
      }

      _resultingVIDs[passive] = std::move(vids);
      _edgeDurations[passive] = std::move(durations);
      _waitTimes[passive] = std::move(waits);
    }
  }
}




bool
ScheduledLocalRepair::
LowLevelPlanner(SemanticTask* _task, std::string _queryStrategy) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::LowLevelPlanner");
  // Initialize maps
  std::map<SemanticTask*,size_t> startTimes;
  std::map<SemanticTask*,size_t> endTimes;
  std::set<SemanticTask*> solved;

  // Collect start and end times for tasks starting at 0
  for(auto kv : m_solutionMap) {
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
    for(auto kv : m_solutionMap) {
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
  for(auto kv : m_solutionMap) {
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

    // TODO: Compute new path
    auto path = QueryPath(task,startTime,_queryStrategy);

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
    m_solutionMap[task] = path;
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

  if(solved.size() != m_solutionMap.size())
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

std::vector<std::pair<std::set<SemanticTask*>,size_t>>
ScheduledLocalRepair::
ValidationFunction() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ValidationFunction");

  auto constraints = FindConflicts();

  if(constraints.empty())
    return {};

  return constraints;
}
 

double
ScheduledLocalRepair::
CostFunction(Node& _node) {

  double cost = 0;
  for(auto kv : _node.solutionMap) {
    auto path = kv.second;
    cost = std::max(cost,double(m_endTimes[path]));
  }

  return cost;
}

void
ScheduledLocalRepair::
InitialSolutionFunction() {
                        
  std::cout << "initial Solution Function " << std::endl;

  // Collect Tasks
  std::vector<SemanticTask*> tasks;
  for(auto vit = m_scheduleGraph->begin(); vit != m_scheduleGraph->end(); vit++) {
    if(vit->property() == nullptr)
      continue;
    auto task = vit->property();
    tasks.push_back(task);
  }

  SemanticTask* initTask = nullptr;
  for(auto task : tasks) {
    m_solutionMap[task] = nullptr;

    if(!initTask and task->GetDependencies().empty())
      initTask = task;
  }
  
  // Plan tasks
  if(!LowLevelPlanner(initTask,m_initialQueryStrategy))
    throw RunTimeException(WHERE) << "No initial plan for task " << initTask->GetLabel();

  // Set node cost
  // node.cost = _cost(node);

  // _root.push_back(node);
}


std::set<std::pair<size_t,size_t>> 
ScheduledLocalRepair::
GetGeometricConstraints() {
  return m_geometricConstraintSet;
}

std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> 
ScheduledLocalRepair::
GetMotionConstraints() {
  return m_motionConstraintSet;
}


bool
ScheduledLocalRepair::
EarlyTermination() {
  if(m_internalMotionConstraintSet.size()) {
    for(size_t i = 0 ; i < 10 ; i++) {
      InitialSolutionFunction();
    }
    return false;
  }
  if(m_motionConstraintSet.size()) {
    std::cout << "Motion Constraint detected " << m_motionConstraintSet.size() << std::endl;
    return true;
  }
  return false;
}

/*--------------------- Helper Functions ---------------------*/

ScheduledLocalRepair::GroupPathType*
ScheduledLocalRepair::
QueryPath(SemanticTask* _task, const size_t _startTime,
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
  // auto grm = solution->GetGroupRoadmap(group);
  // grm->SetAllFormationsInactive();
  // for(auto f : _task->GetFormations()) {
  //   grm->SetFormationActive(f);
  // }

  
  for(auto& robot : problem->GetRobots()) {
    robot->SetVirtual(true);
  }
  for(auto& robot : group->GetRobots()) {
    robot->SetVirtual(false);
  }

  if (m_internalMotionConstraintSet.count(_task)) {
    std::set<size_t> virtualObstacles;
    for (const auto& gcfg : m_internalMotionConstraintSet[_task]) {
      auto grp = gcfg.GetGroupRoadmap()->GetGroup();
      for (auto r : grp->GetRobots()) {
        r->SetVirtual(false);
        auto data = gcfg.GetRobotCfg(r).GetData();
        EulerAngle rotation(0, 0, 0);
        const Vector3d translation(data[0], data[1], data[2]);
        Transformation transformation(std::move(translation), std::move(rotation));

        auto fileName = r->GetMultiBody()->GetBody(0)->GetFileName();
        auto id = this->GetMPProblem()->GetEnvironment()->AddObstacle("",fileName,transformation);
        std::cout << "Adding obstacle idx: " << id << " -- filename is " << fileName << std::endl;
        virtualObstacles.insert(id);
      }
    }

    lib->Solve(problem,_task->GetGroupMotionTask().get(),solution,_queryStrategy,
            LRand(),this->GetNameAndLabel()+"::"+_task->GetLabel());
    for(size_t i = 0 ; i < virtualObstacles.size() ; i++) {
      size_t last = this->GetMPProblem()->GetEnvironment()->NumObstacles() - 1;
      this->GetMPProblem()->GetEnvironment()->RemoveObstacle(last);
    }

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
  else {
    lib->Solve(problem,_task->GetGroupMotionTask().get(),solution,_queryStrategy,
            LRand(),this->GetNameAndLabel()+"::"+_task->GetLabel());

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

}

std::vector<std::pair<std::set<SemanticTask*>,size_t>>
ScheduledLocalRepair::
FindConflicts() {
  // return {};
  std::vector<std::pair<std::set<SemanticTask*>,size_t>> constraints;

  auto vc = static_cast<CollisionDetectionValidityMethod<MPTraits<Cfg>>*>(
              this->GetMPLibrary()->GetValidityChecker(m_vcLabel));

  // Sort tasks based on slack
  auto slack = ComputeScheduleSlack();

  std::vector<std::pair<size_t,SemanticTask*>> slackOrderedTasks;
  for(auto kv : slack) {
    auto task = m_scheduleGraph->GetVertex(kv.first);
    if(!task)
      continue;

    auto pair = std::make_pair(size_t(kv.second),task);
    slackOrderedTasks.push_back(pair);
  }

  std::sort(slackOrderedTasks.begin(),slackOrderedTasks.end(),
            [this](const std::pair<size_t,SemanticTask*> _elem1,
                   const std::pair<size_t,SemanticTask*> _elem2) {
    
    if(_elem1.first != _elem2.first)
      return _elem1.first < _elem2.first;

    auto path1 = m_solutionMap.at(_elem1.second);
    auto path2 = m_solutionMap.at(_elem2.second);

    return this->m_startTimes[path1] < this->m_startTimes[path2];
  });

  // Find max timestep
  size_t maxTimestep = 0;
  for(auto kv : m_solutionMap) {
    auto path = kv.second;
    maxTimestep = std::max(maxTimestep,m_endTimes[path]);
  }

  // Collect cfgs
  auto lib = this->GetMPLibrary();
  std::map<SemanticTask*,std::vector<GroupCfgType>> cfgPaths;
  for(auto kv : m_solutionMap) {
    auto task = kv.first;
    auto path = kv.second;
    cfgPaths[task] = path->FullCfgsWithWait(lib);
  }

  
  auto sq = dynamic_cast<ModeHyperpathQuery*>(this->GetTaskEvaluator(m_sqLabel).get());

  for(auto kv : m_passiveEndTaskMap) {
    auto passive = kv.first;
    auto passiveTask = kv.second;
    auto passiveCfgs = cfgPaths[passiveTask];
    auto passiveGCfg = passiveCfgs.back();
    auto passivePath = m_solutionMap.at(passiveTask);
    size_t passiveEndTime = m_endTimes[passivePath];

    for(size_t i = 0; i < slackOrderedTasks.size(); i++) {
      auto task = slackOrderedTasks[i].second;
      auto path = m_solutionMap.at(task);

      size_t taskStartTime = m_startTimes[path];
      if(taskStartTime < passiveEndTime)
        continue;
      
      const auto& cfgs = cfgPaths[task];

      bool collision = false;
      for(auto gcfg : cfgs) {
        gcfg.ConfigureRobot();
        auto group = gcfg.GetGroupRoadmap()->GetGroup();

        for(auto robot1 : group->GetRobots()) {
          if(robot1 == passive) 
            continue;

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
          // bool isHyperarc1 = false;
          // bool isVertex2 = false;
          bool isHyperarc2 = false;

          auto vi1 = vertexTasks.find(passiveTask);
          if(vi1 != vertexTasks.end()) {
            isVertex1 = true;
          }

          auto hi2 = hyperarcTasks.find(task);
          if(hi2 != hyperarcTasks.end()) {
            isHyperarc2 = true;
          }

          // if(m_debug)
          //   std::cout << isVertex1 << isHyperarc1 << isVertex2 << isHyperarc2 << std::endl;

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

            if(hasObject and stGroup != nullptr) {
              if(std::find(stGroup->GetRobots().begin(), stGroup->GetRobots().end(), passive) != stGroup->GetRobots().end())
                continue;
              if(m_debug)
              std::cout << "\t     " << passive->GetLabel() << " (vertex) " << stGroup->GetLabel() << " (hyperarc) " << std::endl;
            
              std::cout << "vh: Motion Constraints" << std::endl;

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

                m_internalMotionConstraintSet[task].insert(passiveGCfg);

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




  std::vector<SemanticTask*> tasks;
  for(auto vit = m_scheduleGraph->begin(); vit != m_scheduleGraph->end(); vit++) {
    if(vit->property() == nullptr)
      continue;
    auto task = vit->property();
    tasks.push_back(task);
  }


  size_t lastTimestep = MAX_UINT;
  for(auto c : m_previousConstraints) {
    auto taskSet = c.first;
    for(auto st : taskSet) {
      auto path = m_solutionMap.at(st);
      auto start = m_startTimes[path];
      lastTimestep = std::min(lastTimestep,start);
    }
  }
  if(lastTimestep == MAX_UINT) {
    lastTimestep = 0;
  }



  for(size_t i = 0; i < tasks.size(); i++) {
    auto task1 = tasks[i];
    auto path1 = m_solutionMap.at(task1);

    const size_t start1 = m_startTimes[path1];
    size_t end1 = m_endTimes[path1];


    if(end1 < lastTimestep) {
      std::cout << "Already computed: endTime " << end1 << ", lastTime " << lastTimestep << std::endl;
      continue;
    }


    auto vid1 = m_scheduleGraph->GetVID(task1);
    for(auto dep : m_scheduleGraph->GetPredecessors(vid1)) {
      auto depTask = m_scheduleGraph->GetVertex(dep);
      if(depTask) {
        end1 = std::min(end1,m_startTimes[m_solutionMap.at(depTask)]);
      }
      else {
        end1 = maxTimestep;
      }
    }
    const auto& cfgs1 = cfgPaths[task1];

    std::set<SemanticTask*> collidingTasks{task1};
    
    for(size_t t = start1; t < end1; t++) {
      const size_t index1 = std::min(t - start1,cfgs1.size()-1);
      const auto gcfg1 = cfgs1[index1];
      gcfg1.ConfigureRobot();
      auto group1 = gcfg1.GetGroupRoadmap()->GetGroup();

      bool collision = false;
      for(size_t j = i+1; j < tasks.size(); j++) {
        auto task2 = tasks[j];
        auto path2 = m_solutionMap.at(task2);

        if(collidingTasks.count(task2))
          continue;

        
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
              
            end2 = std::min(end2,m_startTimes[m_solutionMap.at(depTask)]);
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

            CDInfo cdInfo;
            collision = collision or vc->IsMultiBodyCollision(cdInfo,
              mb1,mb2,this->GetNameAndLabel());
            
            if(collision) {
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
                        << "Path1 Wait Times: " << path1->GetWaitTimes() << std::endl
                        << "Path1 Timesteps: " << std::endl;
                        path1->PrintIntervals();

              std::cout << "Task2:" << std::endl
                        << "Path2 VIDs: " << path2->VIDs() << std::endl
                        << "Path1 Wait Times: " << path2->GetWaitTimes() << std::endl
                        << "Path2 Timesteps: " << std::endl;
                        path2->PrintIntervals();




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
              // if(m_debug)
              //   std::cout << isVertex1 << isHyperarc1 << isVertex2 << isHyperarc2 << std::endl;
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
                for(size_t k = 0; k < tasks.size(); k++) {
                  auto task = tasks[k];
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

                if(hasObject and stGroup != nullptr) {
                  Robot* passive = nullptr;
                  for(auto r : passiveGroup->GetRobots()) {
                    if(r->GetMultiBody()->IsPassive()) {
                      passive = r;
                      break;
                    }
                  }
                  if(std::find(stGroup->GetRobots().begin(), stGroup->GetRobots().end(), passive) != stGroup->GetRobots().end())
                    continue;
                  std::cout << "vh: Possibly a geometric constraint" << std::endl;
                  if(m_debug)
                    std::cout << "\t     " << passive->GetLabel() << " (vertex) " << stGroup->GetLabel() << " (hyperarc) " << std::endl;
                  

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

                    m_motionConstraintSet.insert(std::make_pair(std::make_pair(v,h),passiveGCfg.GetRobotCfg(passive)));
                  }
                }
              }
              else if(isHyperarc1 and isHyperarc2) {
                if(m_debug) {
                  std::cout << "HH: wait if the scheduledLocalRepair can solve this" << std::endl;
                  std::cout << "\twith " << group1->GetLabel() << " (hyperarc) " << group2->GetLabel() << " (hyperarc) " << std::endl;
                }
                conflictType = "hh";
              }
              if(m_debug) {
                std::cout << "Motion constraint size: " << m_motionConstraintSet.size() << std::endl;
              }

              collidingTasks.insert(task2);

              constraints.push_back(std::make_pair(collidingTasks,t));              
              
              m_previousConstraints = constraints;
              
              return constraints;
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
      if(collision)
        break;
    }
  }

  m_previousConstraints = constraints;

  return constraints;
}

bool
ScheduledLocalRepair::
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
ScheduledLocalRepair::
ConvertToPlan(Plan* _plan) {
  double cost = 0;
  for(auto kv : m_solutionMap) {
    auto path = kv.second;
    cost = std::max(cost,double(m_endTimes[path]));
  }
  _plan->SetCost(cost);

  // if(!m_writeSolution)
  //   return;
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();

  MethodTimer mt(stats,this->GetNameAndLabel() + "::SaveSolution");

  // Collect all of the robots
  std::unordered_map<Robot*,std::vector<Cfg>> robotPaths;
  for(auto kv : m_solutionMap) {
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
  while(ordering.size() < m_solutionMap.size()) {

    // Find the set of tasks that are ready to be validated
    for(auto kv :m_solutionMap) {
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

  std::cout << "Total Path Length: " << finalTime << std::endl;
  MethodTimer mtWrietPaths(stats,this->GetNameAndLabel() + "::WritingPaths");

  // Add the cfgs to the paths
  for(size_t t = 0; t <= finalTime; t++) {
    std::unordered_set<Robot*> used;
    std::cout << t << "/" << finalTime << std::endl;

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


  std::cout << "Writing Paths" << std::endl;
  const size_t numCopies = 5;

  for(size_t i = 0; i < switches.size(); i++) {
    size_t t = switches[i] + numCopies*i;
    std::cout << "Switches: " << i << "/" << switches.size() << std::endl;
    for(auto& kv : robotPaths) {
      auto& path = kv.second;

      if(t >= path.size())
        continue;

      auto cfg = path[t];

      path.insert(path.begin()+t,numCopies,cfg);
    }
  }

  for(auto kv : robotPaths) {
    std::cout << "PATH FOR: " << kv.first->GetLabel() << std::endl;

    const std::string filename = this->GetMPProblem()->GetBaseFilename() 
                               + "::FinalPath::" + kv.first->GetLabel();

    std::ofstream ofs(filename);

    for(size_t i = 0; i < kv.second.size(); i++) {
      auto cfg = kv.second[i];
      // if(m_debug) {
      //   std::cout << "\t" << i << ": " << cfg.PrettyPrint() << std::endl;
      // }
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










void
ScheduledLocalRepair::
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
        timeTable[robot].insert({t1, {startTimes[t1], path1.size()}});
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


  // auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto mtg = mg->GetModeTransitionHypergraph();
  std::cout << "HYPERARC COST MAP: " << std::endl;
  const std::string filename = this->GetMPProblem()->GetBaseFilename() 
                               + "::HACOST";
  std::ofstream ofs(filename);
  for(auto kv : mtg.GetHyperarcMap()) {
    std::string label = "...";
    if(kv.second.property.action.first)
      label = kv.second.property.action.first->GetLabel();
    ofs << kv.first << " (" << label << "): " << kv.second.property.cost << "\n";
  }
  ofs.close();





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
ScheduledLocalRepair::
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
ScheduledLocalRepair::
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
ScheduledLocalRepair::
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
ScheduledLocalRepair::
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

      std::cout << "[" << vit->descriptor() << "] " 
                << ": "
                << (vit->property() != nullptr ? vit->property()->GetLabel() : "root")
                << std::endl;

      for(auto eit = vit->begin(); eit != vit->end(); eit++) {
        std::cout << "\tParent " << eit->target() << std::endl;;
      }
    }
  }
}

void
ScheduledLocalRepair::
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
ScheduledLocalRepair::
ComputeCriticalPaths(const Node& _node) {

  auto slack = ComputeScheduleSlack();

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
ScheduledLocalRepair::
ComputeScheduleSlack() {

  SSSPPathWeightFunction<ScheduleGraph> weight = [this](
      typename ScheduleGraph::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {
    
    auto source = this->m_scheduleGraph->GetVertex(_ei->source());

    if(!source)
      return 0.;

    return 1.;
  };

  auto sssp = DijkstraSSSP(m_scheduleGraph.get(),{0},weight);
  return sssp.distance;
}

/*------------------------------------------------------------*/
