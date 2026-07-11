#include "ObjectScheduler.h"

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

ObjectScheduler::
ObjectScheduler() {
  this->SetName("ObjectScheduler");
}

ObjectScheduler::
ObjectScheduler(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("ObjectScheduler");

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

  m_GH = _node.Read("GH",false,"GH","Label for grounded hypergraph.");


}

ObjectScheduler::
~ObjectScheduler() {

}

/*------------------------ Overrides -------------------------*/
void
ObjectScheduler::
Initialize() {
  std::cout << "Initialize scheduler" << std::endl;
  m_startTimes.clear();
  m_endTimes.clear();
 
  m_scheduleAtomicDistances.clear();

  // m_geometricConstraintSet.clear();
  m_motionConstraintSet.clear();
  std::cout << "Initialize scheduler" << std::endl;
}


bool
ObjectScheduler::
Run(Plan* _plan) {
  std::cout << "Start Object Scheduler" << std::endl;
  if(!_plan)
    _plan = this->GetPlan();

  m_conflictCount.clear();

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  MethodTimer mt_ind(stats,this->GetNameAndLabel() + "::Run_ObjectScheduler");

  FindConflicts();
  return true;
}

std::unordered_map<size_t,std::set<size_t>>
ObjectScheduler::
GetGeometricConstraints() {
  return m_geometricConstraintSet;
}

std::unordered_map<size_t,std::set<size_t>>
ObjectScheduler::
GetTaskSpaceImprovementCandidates() {
  return m_taskSpaceImprovementCandidates;
}

std::set<std::pair<size_t,size_t>>
ObjectScheduler::
GetExtraTaskSpaceCandidates() {
  return m_extraTaskSpaceCandidateSet;
}
/*----------------------- CBS Functors -----------------------*/

void
ObjectScheduler::
FindConflicts() {
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  // auto sink = mg->GetSinkModeTransitionVID();
  auto mh = mg->GetModeTransitionHypergraph();
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  std::cout << "&&&&&&&&&&&&&&&&" << std::endl;
  auto vc = static_cast<CollisionDetectionValidityMethod<MPTraits<Cfg>>*>(
              this->GetMPLibrary()->GetValidityChecker(m_vcLabel));

  auto hyperpath = mg->GetRelevantMTHIDVector();
  auto groundedVIDMap = mg->GetGroundedVIDMap();
  std::set<VID> frontier{};
  std::cout << "Initial heads" << std::endl;
  for(auto h : mh.GetHyperarc(hyperpath[0]).head) {
    auto vidSet = groundedVIDMap[h];
    std::cout << mh.GetVertex(h).property->robotGroup->GetLabel() 
              << " " << h 
              << ": " << vidSet.size() << std::endl;
  }
  for(size_t i = 1 ; i < hyperpath.size() ; i++) {
    for(auto h : mh.GetHyperarc(hyperpath[i]).head) {
      auto vidSet = groundedVIDMap[h];
    }
  }

  // std::set<std::pair<size_t,size_t>> newGeometricConstraintSet;

  auto groundedHistory = mg->GetGroundedVertexHistory();
  std::set<VID> frontiers = groundedHistory[0].second.second;
  std::cout << "Grounded Vertex History" << std::endl;
  std::cout << "[0] {0}  --->>  " << frontiers << std::endl;
  for(size_t i = 1 ; i < groundedHistory.size() ; i++) {
    auto mhid = groundedHistory[i].first;
    auto pair = groundedHistory[i].second;
    auto tail = pair.first;
    auto head = pair.second;
    std::cout << "[" << mhid << "] " << tail << "  --->>  " << head << std::endl;
    std::cout << "\t" ;
    for(auto t : tail) {
      auto g = gh->GetVertex(t).first->GetVertex(gh->GetVertex(t).second).GetGroupRoadmap()->GetGroup();
      std::cout << g->GetLabel() << ", ";
    }
    std::cout << "  --->>  ";
    for(auto h : head) {
      if(h==1) {
        std::cout << "sink" << std::endl;
        break;
      }
      auto g = gh->GetVertex(h).first->GetVertex(gh->GetVertex(h).second).GetGroupRoadmap()->GetGroup();
      std::cout << g->GetLabel() << ", ";
    }
    std::cout << std::endl;
    std::cout << "frontiers: " << std::endl;
    for(auto f : frontiers) {
      auto ggg = gh->GetVertex(f).first->GetVertex(gh->GetVertex(f).second).GetGroupRoadmap()->GetGroup();
      std::cout << "\t" << ggg->GetLabel() << std::endl;
    }

    for(size_t t : tail) {
      frontiers.erase(t);
    }

    std::set<size_t> comp;
    for(size_t t : tail) { 
      comp.insert(t);
    }
    for(size_t h : head) {
      comp.insert(h);
    }
    std::set<std::pair<Robot*,Robot*>> used;
    for(size_t h : comp) {
      auto gcfg1 = gh->GetVertex(h).first->GetVertex(gh->GetVertex(h).second);
      auto group1 = gcfg1.GetGroupRoadmap()->GetGroup();
      gcfg1.ConfigureRobot();
      size_t mt = mg->GetTransitionModeOfGroundedVID(h);
      std::cout << "\t\t" << h << " (" << mt << "): " << group1->GetLabel() << " (" << gcfg1.PrettyPrint() << ")" << std::endl;
      for(size_t f : frontiers) {
        auto gcfg2 = gh->GetVertex(f).first->GetVertex(gh->GetVertex(f).second);
        auto group2 = gcfg2.GetGroupRoadmap()->GetGroup();
        gcfg2.ConfigureRobot();
        std::cout << "\t\t\tCC with " << group2->GetLabel() << ": " << gcfg2.PrettyPrint() << std::endl;

        // Check for collision
        bool collision = false;
        for(auto robot1 : group1->GetRobots()) {
          for(auto robot2 : group2->GetRobots()) {

            if(robot1 == robot2)
              continue;

            std::cout << "\t\t\t\t" << robot1->GetLabel() << " vs. " << robot2->GetLabel() << std::endl;

            auto mb1 = robot1->GetMultiBody();
            auto mb2 = robot2->GetMultiBody();
            robot1->SetVirtual(false);
            robot2->SetVirtual(false);

            CDInfo cdInfo;
            collision = collision or vc->IsMultiBodyCollision(cdInfo,
              mb1,mb2,this->GetNameAndLabel());
            if(collision && !(tail.size() == 1 && head.size() == 1)) {
              mt = mg->GetTransitionModeOfGroundedVID(f);
              std::cout << "\t\t\t\tcollides " << f << " (" << mt << "): " << group2->GetLabel() << " (" << gcfg2.PrettyPrint() << ")" << std::endl;
            }

            robot1->SetVirtual(true);
            robot2->SetVirtual(true);
          }
        }
        if(collision && !(tail.size() == 1 && head.size() == 1)) {
          Robot* passive1 = nullptr;
          Robot* passive2 = nullptr;
          for(auto robot1 : group1->GetRobots()) {
            if(robot1->GetMultiBody()->IsPassive()) {
              passive1 = robot1;
            }
          }
          for(auto robot2 : group2->GetRobots()) {
            if(robot2->GetMultiBody()->IsPassive()) {
              passive2 = robot2;
            }
          }
          if(passive1 and passive2) {
            if(!used.count(std::make_pair(passive1,passive2))) {
              used.insert(std::make_pair(passive1,passive2));
              size_t fMTVID = mg->GetTransitionModeOfGroundedVID(f);

              std::set<size_t> tailVIDs;
              std::set<size_t> headVIDs;
              for(auto ttt : tail)
                tailVIDs.insert(mg->GetTransitionModeOfGroundedVID(ttt));
              for(auto hhh : head)
                headVIDs.insert(mg->GetTransitionModeOfGroundedVID(hhh));
              size_t MTHID = MAX_UINT;
              for(auto kv : mh.GetHyperarcMap()) {
                if(kv.second.tail == tailVIDs and kv.second.head == headVIDs) {
                  MTHID = kv.first;
                  std::cout << "\t\t\t\tFound HID " << MTHID << ": " << kv.second.tail << " --> " << kv.second.head << std::endl;
                  break;
                }
              }
              // size_t activeMTVID = mg->GetTransitionModeOfGroundedVID(h);

              // size_t hMTVID = MAX_UINT;
              // size_t hMTVID = mg->GetTransitionModeOfGroundedVID(h);

              // while(true) {
              //   auto tails = mh.GetHyperarc(*mh.GetIncomingHyperarcs(activeMTVID).begin()).tail;
              //   std::cout << "res: " << *mh.GetIncomingHyperarcs(activeMTVID).begin() << " ... " << tails << std::endl;
              //   bool foundPassive = false;
              //   for(size_t t : tails) {
              //     std::cout << mh.GetVertex(t).property->robotGroup->GetLabel() << std::endl;
              //     if(mh.GetVertex(t).property->robotGroup->IsPassive()) {
              //       hMTVID = t;
              //       foundPassive = true;
              //       break;
              //     }
              //   }
              //   if(foundPassive)
              //     break;
              //   else {
              //     Robot* originalPassive;
              //     for(auto r : mh.GetVertex(activeMTVID).property->robotGroup->GetRobots()) {
              //       if(r->GetMultiBody()->IsPassive()) {
              //         originalPassive = r;
              //       }
              //     }
              //     for(auto t : tails) {
              //       auto robots = mh.GetVertex(t).property->robotGroup->GetRobots();
              //       if(std::find(robots.begin(), robots.end(), originalPassive) != robots.end()) {
              //         activeMTVID = t;
              //         break;
              //       }
              //     }
              //   }
              // }
              // std::cout << "Addddd " << h << " " << f << std::endl;
              std::cout << "\t\t\t\tAdd Constraint HID " << MTHID << " fVID " << fMTVID << std::endl;
              // newGeometricConstraintSet.insert(std::make_pair(hMTVID,fMTVID));
              m_geometricConstraintSet[MTHID].insert(fMTVID);
            }
          }
        }
      }
    }

    for(size_t h : head) {
      frontiers.insert(h);
    }
  }

  // std::vector<std::pair<size_t,size_t>> colliding;
  // colliding.reserve(std::min(m_prevGeometricConstraintSet.size(), newGeometricConstraintSet.size()));
  // std::set_intersection(m_prevGeometricConstraintSet.begin(), m_prevGeometricConstraintSet.end(),
  //                       newGeometricConstraintSet.begin(), newGeometricConstraintSet.end(),
  //                       std::back_inserter(colliding));
  // std::cout << "COlliding: " << colliding.size() << std::endl;
  // // 2) Remove colliding pairs (and their swapped versions) from sets
  // for (const auto& c : colliding) {
  //   std::pair<size_t,size_t> swapC = std::make_pair(c.second, c.first);

  //   // No need to check count() before erase
  //   m_geometricConstraintSet.erase(c);
  //   m_geometricConstraintSet.erase(swapC);

  //   newGeometricConstraintSet.erase(c);
  //   newGeometricConstraintSet.erase(swapC);

  //   m_extraTaskSpaceCandidateSet.insert(std::make_pair(c.second,c.first));
  //   std::cout << "Non-monotonic constraint: " << c.second << '\n';
  // }

  size_t source = mg->GetSourceModeTransitionVID();
  size_t sink = mg->GetSinkModeTransitionVID();
  std::set<RobotGroup*> used;
  std::unordered_map<RobotGroup*,std::set<size_t>> robotMap;
  for(auto kv : mh.GetVertexMap()) {
    if(kv.first == source or kv.first == sink)
      continue;
    auto g = mh.GetVertex(kv.first).property->robotGroup;
    if(g->IsActive())
      continue;
    used.insert(g);
    for(auto kv2 : mh.GetVertexMap()) {
      if(kv.first == source or kv.first == sink)
        continue;
      auto g2 = mh.GetVertex(kv2.first).property->robotGroup;
      if(g == g2) {
        robotMap[g2].insert(kv2.first);
      }
    }
  }

  std::cout << "Object MT VIDs" << std::endl;
  for(auto kv : robotMap) {
    std::cout << kv.first->GetLabel() << ": " << kv.second << std::endl;
  }

  std::cout << "Geometric Constraints" << std::endl;
  for(auto kv : m_geometricConstraintSet) {
    std::cout << "HID " << kv.first << ": " << kv.second << std::endl;
  }

  std::cout << "Identify Non Monotonic Constraints" << std::endl;
  for(auto kv : robotMap) {
    auto g = kv.first;
    auto set = kv.second;
    for(auto kv2 : m_geometricConstraintSet) {
      bool all = true;
      for(auto e : set) {
        if(!kv2.second.count(e)) {
          all = false;
        }
      }
      if(all) {
        size_t moveVID = MAX_UINT;
        size_t moveHID = kv2.first;
        auto ha = mh.GetHyperarc(moveHID);
        for(auto ttt : ha.tail) {
          if(mh.GetVertex(ttt).property->robotGroup->IsPassive()) {
            moveVID = ttt;
            break;
          }
        }
        for(auto hhh : ha.head) {
          if(mh.GetVertex(hhh).property->robotGroup->IsPassive()) {
            moveVID = hhh;
            break;
          }
        }
        m_taskSpaceImprovementCandidates[moveVID] = set;
        std::cout << g->GetLabel() << " needs to move out. " << set << " must move out for " << moveVID << std::endl;
      }
    }
  }

  // std::cout << "Prev Geometric constraints " << std::endl;
  // for (auto pair : m_prevGeometricConstraintSet) {
  //   std::cout << pair.first << ", " << pair.second << '\n';
  //   std::cout << mh.GetVertex(pair.first).property->robotGroup->GetLabel() << ", " << mh.GetVertex(pair.second).property->robotGroup->GetLabel() << '\n';
  // }

  // std::cout << "New Geometric constraints " << std::endl;
  // for (auto pair : newGeometricConstraintSet) {
  //   std::cout << pair.first << ", " << pair.second << '\n';
  //   std::cout << mh.GetVertex(pair.first).property->robotGroup->GetLabel() << ", " << mh.GetVertex(pair.second).property->robotGroup->GetLabel() << '\n';
  // }

  
  // // 3) Insert remaining new constraints and report
  // if (!newGeometricConstraintSet.empty()) {
  //   m_geometricConstraintSet.insert(newGeometricConstraintSet.begin(), newGeometricConstraintSet.end());
  //   std::cout << "new geometric constraints found\n";
  //   // for (auto pair : newGeometricConstraintSet) {
  //   //   std::cout << pair.first->GetLabel() << ", " << pair.seccond->GetLabel() << '\n';
  //   // }
  //   m_prevGeometricConstraintSet = m_geometricConstraintSet;
  // }
}




/*--------------------- Helper Functions ---------------------*/

/*------------------------------------------------------------*/
