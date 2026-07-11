#include "LazyModeQuery.h"

#include "MPLibrary/MapEvaluators/SIPPMethod.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"
#include "TMPLibrary/StateGraphs/ModeGraph.h"
#include "TMPLibrary/StateGraphs/LazyModeGraph.h"
#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"
#include "TMPLibrary/TaskEvaluators/ScheduledCBS.h"
#include "TMPLibrary/TaskEvaluators/ScheduledLocalRepair.h"
#include "TMPLibrary/TaskEvaluators/ObjectScheduler.h"

#include <algorithm>

/*----------------------- Construction -----------------------*/

LazyModeQuery::
LazyModeQuery() {
  this->SetName("LazyModeQuery");
}

LazyModeQuery::
LazyModeQuery(XMLNode& _node) : TMPStrategyMethod(_node) {
  this->SetName("LazyModeQuery");

  m_queryLabel = _node.Read("queryLabel",true,"",
        "Map Evaluator to use to query individual solutions.");

  m_queryStrategy = _node.Read("queryStrategy",true,"",
        "MPStrategy to use to query individual mode solution.");

  m_vcLabel = _node.Read("vcLabel",true,"",
        "Validity checker to use for multirobot collision checking.");

  // m_safeIntervalLabel = _node.Read("safeIntervalLabel",true,"",
  //       "Safe interval tool to use for compute safe intervals of roadmap.");

  // m_savePaths =_node.Read("savePaths",false,m_savePaths,
  //       "Flag to indicate if full paths should be output in files for reuse.");

  m_motionEvaluator = _node.Read("motionEvaluator",true,"",
              "Evaluator label for motion planning.");

  m_initialMotionEvaluator = _node.Read("initialMotionEvaluator",false,"",
              "Evaluator label for motion planning.");

  m_singleShot = _node.Read("singleShot",false,m_singleShot,
        "Flag to attempt only a single time to find a solution.");

  // m_extraEval = _node.Read("extraEval",false,"",
  //       "Temporary extra evaluator for experimental purposes.");

  m_ghLabel = _node.Read("ghLabel",true,"","Grounded Hypergraph Label.");

}

LazyModeQuery::
~LazyModeQuery() {}

/*------------------------ Interface -------------------------*/

/*--------------------- Helper Functions ---------------------*/

void
LazyModeQuery::
PlanTasks() {
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());

  std::cout << "Evaluating with TMP Strategy: " << this->GetNameAndLabel() << std::endl;

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  // MethodTimer mt1(stats,this->GetNameAndLabel() + "::PlanTasks");
  stats->SetStat(this->GetNameAndLabel() + "::PlanTasks",0);

  auto originalDecomp = plan->GetDecomposition();
  std::cout << "Initialize Task Evaluator: " << m_teLabel << std::endl;
  auto te = this->GetTaskEvaluator(m_teLabel);
  te->Initialize();



  m_upperBound = MAX_DBL;
  double lowerBound = 0;

  auto motionScheduler = dynamic_cast<ScheduledLocalRepair*>(this->GetTaskEvaluator(m_motionEvaluator).get());
  auto initialMotionScheduler = dynamic_cast<ObjectScheduler*>(this->GetTaskEvaluator(m_initialMotionEvaluator).get());


  std::unordered_map<size_t,std::set<size_t>> prevOGc{};
  std::unordered_map<size_t,std::set<size_t>> prevTsic{};


  while(true) {
    std::cout << "Arrived" << std::endl;
    mg->ImproveTaskSpace();
    // auto nmc = mg->GetNonMonotonicConstraints();
    auto nmc = mg->GetGeometricConstraints2();
    std::cout << "NMC size: " << nmc.size() << std::endl;
    // te->SetNonMonotonicConstraints(nmc);
    te->SetGeometricConstraints2(nmc);

    auto mtg = mg->GetModeTransitionHypergraph();

    std::set<size_t> prevIc{};
    std::unordered_map<size_t,std::set<size_t>> prevGc{};
    std::set<std::vector<size_t>> prevTc{};

    size_t outerLoopCount = 0;
    bool improveTaskSpace = false;
    while(true) {
      std::cout << "@@@@ " << outerLoopCount << std::endl;
      // std::cout << "prev etsc" << std::endl;
      // for(auto p : prevEtsc) {
      //   std::cout << p.first->GetLabel() << ", " << p.second->GetLabel() << std::endl; 
      // }
      std::cout << "Prev OGc size: " << prevOGc.size() << std::endl;
      std::cout << "Prev Gc size: " << prevGc.size() << std::endl;

      outerLoopCount += 1;
      lowerBound = 0;
      MethodTimer* taskQueryTimer = new MethodTimer(stats,this->GetNameAndLabel() + "::TaskQuery_" + std::to_string(outerLoopCount));
      FindTaskPlan(originalDecomp); // ModeQuery
      std::cout << "Task Hyperpath Exists" << std::endl;
      delete taskQueryTimer;

      mg->InitializeMotionHypergraph();

      initialMotionScheduler->Initialize();
      initialMotionScheduler->operator()(plan);
      auto objectGC = initialMotionScheduler->GetGeometricConstraints();
      if(objectGC != prevOGc) {
        std::cout << "Prev Gc size: " << prevOGc.size() << std::endl;
        std::cout << "Post Gc size: " << objectGC.size() << std::endl;

        te->SetGeometricConstraints(objectGC);
        prevOGc = objectGC;
        std::cout << "Replanning with GC" << std::endl;
        continue;
      }

      auto tsic = initialMotionScheduler->GetTaskSpaceImprovementCandidates();
      objectGC = initialMotionScheduler->GetGeometricConstraints();
      
      if(tsic != prevTsic) {
        // std::cout << "prev etsc" << std::endl;
        // for(auto p : prevEtsc) {
        //   std::cout << p.first->GetLabel() << ", " << p.second->GetLabel() << std::endl; 
        // }
        // std::cout << "post etsc" << std::endl;
        // for(auto p : etsc) {
        //   std::cout << p.first->GetLabel() << ", " << p.second->GetLabel() << std::endl; 
        // }
        std::cout << "Prev tsic size: " << prevTsic.size() << std::endl;
        std::cout << "Post tsic size: " << tsic.size() << std::endl;
        mg->SetTaskSpaceImprovementCandidates(tsic);
        te->SetGeometricConstraints(objectGC);
        prevTsic = tsic;
        prevOGc = objectGC;
        std::cout << "Replanning with NMC" << std::endl;
        improveTaskSpace = true;
        break;
      }

      // mg->ClearConstraints();
      bool planExist = false;
      size_t innerLoopCount = 0;
      while(true) {
        innerLoopCount += 1;
        MethodTimer* genMotionHypergraphTimer = new MethodTimer(stats,this->GetNameAndLabel() + "::GenerateMotionHypergraph_T" + std::to_string(outerLoopCount) + "_M" + std::to_string(innerLoopCount));
        mg->GenerateMotionHypergraph();
        delete genMotionHypergraphTimer;
        std::cout << "Finished generating grounded hypergraph" << std::endl;

        auto ic = mg->GetInteractionConstraints();
        if(ic != prevIc) {
          te->SetInteractionConstraints(ic);
          prevIc = ic;
          std::cout << "Setting Interaction constraints to task planner: " << ic << " " << prevIc << ". size of " << ic.size() << std::endl;
          std::cout << "Replanning Task Plan" << std::endl;
          break;
        }

        
        MethodTimer* motionQueryTimer = new MethodTimer(stats,this->GetNameAndLabel() + "::MotionQuery_T" + std::to_string(outerLoopCount) + "_M" + std::to_string(innerLoopCount));
        mg->PropagateMotions();
        
        auto gc = mg->GetGeometricConstraints();
        auto tc = mg->GetTaskOrderConstraints();

        std::cout << "gc: " << prevGc.size() << " --> " << gc.size() << std::endl;
        // for(auto pair : prevGc) {
        //   std::cout << pair.first->GetLabel() << ", " << pair.second->GetLabel() << " | ";
        // }
        // std::cout << std::endl;
        // std::cout << " --> " << std::endl;
        // for(auto pair : gc) {
        //   std::cout << pair.first->GetLabel() << ", " << pair.second->GetLabel() ;
        // }
        // std::cout << std::endl;
        std::cout << "    " << prevGc << " --> " << gc << std::endl;
        std::cout << "tc: " << prevTc.size() << " --> " << tc.size() << std::endl;
        std::cout << "    " << prevTc << " --> " << tc << std::endl;

        bool replan = false;
        if(gc != prevGc) {
          te->SetGeometricConstraints(gc);
          std::cout << "Setting Geometric constraints to task planner. size of " << gc.size() << std::endl;
          prevGc = gc;
          replan = true;
        }

        if(tc != prevTc) {
          te->SetTaskOrderConstraints(tc);
          std::cout << "Setting Task Order constraints to task planner. size of " << tc.size() << std::endl;
          prevTc = tc;
          replan = true;
        }

        if(replan) {
          std::cout << "Replanning Task Plan" << std::endl;
          delete motionQueryTimer;
          break;
        }

        lowerBound = FindMotionPlan(originalDecomp);
        delete motionQueryTimer;

        std::cout << "No Task Constraints Detected. Start Conflict Resolution" << std::endl;
        motionScheduler->Initialize();
        std::cout << "Scheduler Initialized" << std::endl;
        MethodTimer* conflictResolutionTimer = new MethodTimer(stats,this->GetNameAndLabel() + "::ConflictResolution_T" + std::to_string(outerLoopCount) + "_M" + std::to_string(innerLoopCount));
        if(motionScheduler->operator()(plan)) {
          if(plan->GetCost() < m_upperBound) {
            std::cout << "found solution with cost: " << lowerBound << std::endl;
            m_upperBound = plan->GetCost();
            motionScheduler->SetUpperBound(m_upperBound);
          }
        }
        delete conflictResolutionTimer;

        auto mc = motionScheduler->GetMotionConstraints();
        if(mc.size() > 0) {
          mg->SetMotionConstraints(mc);
          std::cout << "Setting Motion constraints to motion layer" << std::endl;
          continue;
        }

        std::cout << "MC SIZE: " << mc.size() << std::endl;
        if(mc.size() < 1 and
          m_upperBound < MAX_DBL) {
          planExist = true;
          break;
        }
      }
      
      if(planExist) {
        plan->SetDecomposition(originalDecomp);
        break;
      }
    }
    if(improveTaskSpace) {
      std::cout << "Loop back" << std::endl;
      continue;
    }

    stats->SetStat(this->GetNameAndLabel() + "::BestCost",m_upperBound);
    stats->SetStat("Success",m_upperBound < MAX_DBL ? 1:0);
    break;
  }
}
    
bool
LazyModeQuery::
FindTaskPlan(Decomposition* _decomp) {
  auto plan = this->GetPlan();

  plan->SetDecomposition(_decomp);

  auto te = this->GetTaskEvaluator(m_teLabel);
  std::cout << "Find Task Plan with: " << te->GetNameAndLabel() << std::endl;

  if(te->operator()()) {
    std::cout << "Found a solution!" << std::endl;
    // assert(10==1);
    return true;
  }
  else {
    return false;
  }
}


double
LazyModeQuery::
FindMotionPlan(Decomposition* _decomp, bool _useExtraEval) {
  auto plan = this->GetPlan();

  plan->SetDecomposition(_decomp);

  auto te2 = this->GetTaskEvaluator(m_teLabel2);
  te2->Initialize();

  std::cout << "Find motion plan with " << te2->GetNameAndLabel() << std::endl;

  if(te2->operator()()) {
    //return plan->GetCost();
    // Strange off by one error somewhere
    std::cout << "Return motion plan cost " << plan->GetCost() << std::endl;
    return plan->GetCost() - 1;
  }
  else {
    return MAX_DBL;
  }
}
