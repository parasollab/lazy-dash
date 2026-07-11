#include "LazyModeGraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "ConfigurationSpace/GroupLocalPlan.h"

#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/InteractionStrategies/InteractionStrategyMethod.h"
#include "TMPLibrary/Solution/Plan.h"

#include "GroundedHypergraph.h"
#include "MPProblem/Robot/Kinematics/ur_kin.h"

#include <set>

/*------------------------------ Construction --------------------------------*/

LazyModeGraph::
LazyModeGraph() {
  this->SetName("LazyModeGraph");
}

LazyModeGraph::
LazyModeGraph(XMLNode& _node) : StateGraph(_node) {
  this->SetName("LazyModeGraph");

  m_unactuatedSM = _node.Read("unactuatedSM",true,"",
               "Sampler Method to use to generate unactuated cfgs.");

  m_querySM = _node.Read("querySM",true,"",
               "Sampler Method to use to generate query cfgs.");

  m_queryStrategy = _node.Read("queryStrategy",true,"",
                      "MPStrategy label to query roadaps.");

  m_queryStrategyStatic = _node.Read("queryStrategyStatic",false,m_queryStrategy,
                      "MPStrategy label to query roadaps without sampling and connecting new start/goals.");

  m_expansionStrategy = _node.Read("expansionStrategy",true,"",
                      "MPStrategy label to query roadaps.");
  m_numUnactuatedSamples = _node.Read("numUnactuatedSamples",false,0,0,1000,
                      "The number of samples to generate for each unactuated mode.");
  m_numInteractionSamples = _node.Read("numInteractionSamples",false,1,1,1000,
                      "The number of samples to generate for each transtion.");
  m_maxAttempts = _node.Read("maxAttempts",false,1,1,1000,
                      "The max number of attempts to generate a sample.");

  m_writeHypergraphs = _node.Read("writeHypergraphs",false,m_writeHypergraphs,
                      "Flag to write hypergraphs to output files.");

  m_GH = _node.Read("GH",false,"GH","Label for grounded hypergraph.");

  m_roadmap = _node.Read("roadmap",false,true,"");

  m_robotBaseMaxDistance = _node.Read("robotBaseMaxDistance",false,1.6,-1 * MAX_DBL, MAX_DBL,
            "Minumum z value for a handoff to occur.");
  
  m_resampleAttempts = _node.Read("resampleAttempts",true,1,1,1000,
                      "The number of samples to generate for each transtion.");

  m_robotRange = _node.Read("robotRange",false,2.0,0.0,5.0,
                      "The max number of attempts to generate a sample.");

  m_monotonic = _node.Read("monotonic", false, m_monotonic, "Show run-time debug info?");

}

LazyModeGraph::
~LazyModeGraph() {}

/*-------------------------------- Interface ---------------------------------*/

void
LazyModeGraph::
Initialize() {
  std::cout << "Initialize LazyModeGraph" << std::endl;
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  gh->Initialize();


  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Initialize");

  
  GenerateRepresentation();
}

void
LazyModeGraph::
GenerateMotionHypergraph() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GenerateMotionHypergraph");
  
  std::cout << "Generate motion hypergraph " << m_iterationCall << std::endl;
  // SampleTransitions();
  for(auto h : m_relevantMTHIDs) {
    if(m_interactionConstraintSet.count(h)) {
      std::cout << "Task Constraints exists in the relevant mt vids: " << m_interactionConstraintSet << " / " << m_relevantMTVIDs << std::endl;
      for(auto hid : m_interactionConstraintSet) {
        auto ha = m_modeTransitionHypergraph.GetHyperarc(hid);
        if(ha.property.action.first) {
          if(ha.property.action.first->GetLabel().find("Grasp") != std::string::npos){
            std::string error = "";
            error += hid + ": " + ha.property.action.first->GetLabel() + "\n";
            for(size_t vid : ha.tail) {
              error += m_modeTransitionHypergraph.GetVertex(vid).property->robotGroup->GetLabel() + " " ;
            }
            error += " --> " ;
            for(size_t vid : ha.head) {
              error += m_modeTransitionHypergraph.GetVertex(vid).property->robotGroup->GetLabel() + " " ;
            }
            error += hid + ": " + ha.property.action.first->GetLabel() + "\n";
            for(size_t vid : ha.tail) {
              error += std::to_string(vid) + " " ;
            }
            error += " --> " ;
            for(size_t vid : ha.head) {
              error += std::to_string(vid) + " " ;
            }
            throw RunTimeException(WHERE) << "Unable to solve interaction "
                                        << error
                                        << ".";
          }
        }
      }
      return;
    }
  }
  
  // if(m_iterationCall < 1)
  GenerateRoadmaps();

  m_iterationCall += 1;
}




void
LazyModeGraph::
InitializeMotionHypergraph() {
  if(m_iterationCall < 1)
    InitializeTransitions();
  
  std::cout << "Generate motion hypergraph " << m_iterationCall << std::endl;
  SampleTransitions();
  m_iterationCall += 1;
}

bool
LazyModeGraph::
PropagateMotions() {
  ConnectSource();
  auto goalGroundedVIDs = ConnectTransitions();
  if(goalGroundedVIDs.empty()) {
    std::cout << "Failed to generate motion hypergraph." << std::endl;
    return false;
  }
  ConnectSink(goalGroundedVIDs);
  m_iterationCall += 1;
  return true;
}





void 
LazyModeGraph::
ImproveTaskSpace() {
  
  std::cout << "Start task space modification" << std::endl;
  if (m_taskSpaceImprovementCandidates.empty())
    return;

  // ---------- Sample extra object poses (unactuated) ----------
  auto library = this->GetMPLibrary();
  auto unactuatedSampler = library->GetSampler(m_querySM);
  auto vc = static_cast<CollisionDetectionValidityMethod<MPTraits<Cfg>>*>(
      library->GetValidityChecker("pqp_solid"));

  // for(auto& robot : this->GetMPProblem()->GetRobots()) {
  //   if(robot.get() != this->GetPlan()->GetCoordinator()->GetRobot()) {
  //     if(robot->GetMultiBody()->IsPassive())
  //       robot->SetVirtual(false);
  //   }
  // }

  auto collidesWithPlaced = [&](Robot* A, const Cfg& cfgA) -> bool {
    CDInfo info;
    Cfg oldA(A);
    cfgA.ConfigureRobot();                 // set A to candidate

    for (auto kv : m_objectExtraPose) {
      auto B = kv.first;
      auto cfgB = kv.second;
      
      Cfg oldB(B);
      cfgB.ConfigureRobot();               // set B to its placed cfg

      const bool hit = vc->IsMultiBodyCollision(
          info, A->GetMultiBody(), B->GetMultiBody(), this->GetNameAndLabel());

      oldB.ConfigureRobot();               // restore B immediately
      if (hit) { oldA.ConfigureRobot(); return true; }
    }

    oldA.ConfigureRobot();                 // restore A
    return false;
  };

  // ---- attempt loop ----
  for (size_t attempt = 0; attempt < 1000; ++attempt) {
    std::unordered_map<Robot*, Cfg> proposedObjectCfg;  // this attempt's picks

    // bool computed = false;
    for (const auto& kv : m_taskSpaceImprovementCandidates) {
      // auto dependentVID = kv.first;
      auto improveVID = kv.second;

      // if(m_improveComputed.count(*improveVID.begin())) {
      //   std::cout << "Already Computed " << *improveVID.begin() << std::endl;
      //   computed = true;
      //   continue;
      // }
      // else {
      //   std::cout << "Computing " << *improveVID.begin() << std::endl;
      // }
      auto r = m_modeTransitionHypergraph.GetVertex(*improveVID.begin()).property->robotGroup->GetRobot(0);

      if (m_objectExtraPose.count(r)) 
        continue; // already has an extra pose

      auto task = std::unique_ptr<MPTask>(new MPTask(r));
      library->SetTask(task.get());

      // // Pick a terrain surface and boundary (keep your keying logic)
      // const auto& terrainsByCap =
      //     this->GetMPProblem()->GetEnvironment()->GetTerrains().at(
      //         r->GetCapability() + "_" + r->GetLabel().back());

      std::string label = r->GetLabel();              // e.g. "robot_12"
      auto pos = label.find_last_of('_');             // find last '_'
      std::string numberStr = label.substr(pos + 1);  // "12"

      const auto& terrainsByCap =
          this->GetMPProblem()->GetEnvironment()->GetTerrains().at(
              r->GetCapability() + "_" + numberStr);

              
      if (terrainsByCap.empty()) continue;
      const auto& surface = terrainsByCap[LRand() % terrainsByCap.size()];
      const auto& boundaries = surface.GetBoundaries();
      if (boundaries.empty()) continue;

      const Boundary* boundary = boundaries[LRand() % boundaries.size()].get();

      // Try to get a collision-free sample vs ONLY previously placed objects
      bool placed = false;
      for (size_t sTry = 0; sTry < 100 && !placed; ++sTry) {
        std::vector<Cfg> samples;
        unactuatedSampler->Sample(1, 100, boundary, std::back_inserter(samples));
        for (const auto& s : samples) {
          if (!collidesWithPlaced(r, s)) {
            proposedObjectCfg[r] = s;    // accept for this attempt
            placed = true;
            break;
          }
        }
      }
    }

    // Commit this attempt's accepted poses and stop (cumulative behavior)
    if (!proposedObjectCfg.empty()) {
      for (const auto& kv : proposedObjectCfg) {
        std::cout << kv.first->GetLabel() << ": " << kv.second.PrettyPrint() << std::endl;
        m_objectExtraPose[kv.first] = kv.second;
      }
      break;
    }
  }

  // ---------- Identify robot+object mode improvement candidates ----------
  std::set<std::vector<std::pair<HID, std::set<VID>>>> improvementSets;  // each element is a list of (releaseHID, dependentVID)
  std::vector<VID> dependentGoals;

  for (const auto& kv : m_taskSpaceImprovementCandidates) {
    std::cout << "------------------------------------------------- " << std::endl;
    auto dependentVID = kv.first;
    auto improveVIDs = kv.second;
    auto improveVID = kv.second;

    // if(m_improveComputed.count(*improveVID.begin())) {
    //   std::cout << "Already Computed " << *improveVID.begin() << std::endl;
    //   continue;
    // }
    // else {
    //   std::cout << "Computing " << *improveVID.begin() << std::endl;
    // }

    std::cout << dependentVID << " " << improveVIDs << std::endl;
    auto objectToMove = m_modeTransitionHypergraph.GetVertex(*improveVIDs.begin()).property->robotGroup->GetRobot(0);
    auto dependentRobot = m_modeTransitionHypergraph.GetVertex(dependentVID).property->robotGroup->GetRobot(0);

    std::vector<std::pair<VID, std::set<VID>>> improvePairs;  // (releaseHID, dependentVID)
    for (const auto& haKV : m_modeTransitionHypergraph.GetHyperarcMap()) {
      HID hid = haKV.first;
      const auto& ha = haKV.second;
      // Find release: tail must be a single active group; head must split into active+passive
      if (ha.tail.size() != 1 ||
          ha.head.size() != 2 ||
          m_modeTransitionHypergraph.GetVertex(*ha.tail.begin()).property->isDummy() ||
          m_modeTransitionHypergraph.GetVertex(*ha.tail.begin())
              .property->robotGroup->IsPassive()) {
        continue;
      }

      RobotGroup* headActiveGroup = nullptr;
      RobotGroup* headPassiveGroup = nullptr;
      for (VID hVid : ha.head) {
        auto g = m_modeTransitionHypergraph.GetVertex(hVid).property->robotGroup;
        if (g->IsPassive()) 
          headPassiveGroup = g; 
        else 
          headActiveGroup = g;
      }

      RobotGroup* tailGroup = m_modeTransitionHypergraph.GetVertex(*ha.tail.begin()).property->robotGroup;

      if (headPassiveGroup->GetRobot(0) != objectToMove) {
        continue;
      }
      auto interaction = dynamic_cast<Interaction*>(
          m_modeTransitionHypergraph.GetHyperarc(hid).property.action.first);

      auto activeGrm   = this->GetMPSolution()->GetGroupRoadmap(headActiveGroup);
      auto passiveGrm  = this->GetMPSolution()->GetGroupRoadmap(headPassiveGroup);
      auto compositeGrm= this->GetMPSolution()->GetGroupRoadmap(tailGroup);

      GroupCfgType activeGcfg(activeGrm);
      GroupCfgType passiveGcfg(passiveGrm);
      GroupCfgType compositeGcfg(compositeGrm);

      Cfg passiveCfg = m_objectExtraPose[headPassiveGroup->GetRobots()[0]];
      passiveGcfg.SetRobotCfg(headPassiveGroup->GetRobot(0), std::move(passiveCfg));


      bool canReach = false;
      for (size_t j = 0; j < m_maxAttempts; ++j) {
        auto graspCfg = CanReach(interaction, headActiveGroup, passiveGcfg);
        if (!graspCfg.GetRobot()) {
          for (auto& robot : this->GetMPProblem()->GetRobots()) {
            if (robot.get() != this->GetPlan()->GetCoordinator()->GetRobot())
              robot->SetVirtual(false);
          }
          continue;
        }
        std::cout << "releaseHID : " << hid << std::endl;
        std::cout << headActiveGroup->GetLabel()
                  << " can grasp extra obj "
                  << headPassiveGroup->GetLabel()
                  << " " << m_objectExtraPose[headPassiveGroup->GetRobots()[0]].PrettyPrint()
                  << " at " << graspCfg.PrettyPrint()
                  << std::endl;
        std::cout << "Dependent Robot: " << dependentRobot->GetLabel() << std::endl;
        std::cout << "Move Robot: " << objectToMove->GetLabel() << std::endl;
        std::cout << "Interaction Type: " << interaction->GetLabel()
                  << " (" << hid << ")" << std::endl;
        canReach = true;
        break;
      }

      if (!canReach) {
        continue;
      } 

      improvePairs.push_back({hid, improveVIDs});
    }

    improvementSets.insert(improvePairs);
    // m_improveComputed.insert(dependentVID);

    // // Simple extensions
    // dependentVID = MAX_UINT;

    // improvePairs.clear();  // (releaseHID, dependentVID)
    // for (const auto& haKV : m_modeTransitionHypergraph.GetHyperarcMap()) {
    //   HID hid = haKV.first;
    //   const auto& ha = haKV.second;
    //   // Find release: tail must be a single active group; head must split into active+passive
    //   if (ha.tail.size() != 1 ||
    //       ha.head.size() != 2 ||
    //       m_modeTransitionHypergraph.GetVertex(*ha.tail.begin()).property->isDummy() ||
    //       m_modeTransitionHypergraph.GetVertex(*ha.tail.begin())
    //           .property->robotGroup->IsPassive())
    //     continue;

    //   RobotGroup* headActiveGroup = nullptr;
    //   RobotGroup* headPassiveGroup = nullptr;
    //   for (VID hVid : ha.head) {
    //     auto g = m_modeTransitionHypergraph.GetVertex(hVid).property->robotGroup;
    //     if (g->IsPassive()) 
    //       headPassiveGroup = g; 
    //     else 
    //       headActiveGroup = g;
    //   }

    //   std::cout << "A" << std::endl;
    //   RobotGroup* tailGroup = m_modeTransitionHypergraph.GetVertex(*ha.tail.begin()).property->robotGroup;
    //   std::cout << "B" << std::endl;

    //   if (headPassiveGroup->GetRobot(0) != dependentRobot) {
    //     std::cout << "Pass " << hid << std::endl;
    //     continue;
    //   }

    //   std::cout << "C" << std::endl;
    //   auto interaction = dynamic_cast<Interaction*>(
    //       m_modeTransitionHypergraph.GetHyperarc(hid).property.action.first);
    //   std::cout << "D" << std::endl;

    //   auto activeGrm   = this->GetMPSolution()->GetGroupRoadmap(headActiveGroup);
    //   auto passiveGrm  = this->GetMPSolution()->GetGroupRoadmap(headPassiveGroup);
    //   auto compositeGrm= this->GetMPSolution()->GetGroupRoadmap(tailGroup);

    //   GroupCfgType activeGcfg(activeGrm);
    //   GroupCfgType passiveGcfg(passiveGrm);
    //   GroupCfgType compositeGcfg(compositeGrm);

    //   Cfg passiveCfg = m_objectExtraPose[headPassiveGroup->GetRobots()[0]];
    //   std::cout << passiveCfg << std::endl;
    //   passiveGcfg.SetRobotCfg(headPassiveGroup->GetRobot(0), std::move(passiveCfg));
    //   std::cout << "E" << passiveGcfg.PrettyPrint()<< std::endl;


    //   bool canReach = false;
    //   for (size_t j = 0; j < m_maxAttempts; ++j) {
    //     std::cout << "1" << std::endl;
    //     auto graspCfg = CanReach(interaction, headActiveGroup, passiveGcfg);
    //     std::cout << "2" << std::endl;
    //     if (!graspCfg.GetRobot()) {
    //       for (auto& robot : this->GetMPProblem()->GetRobots()) {
    //         if (robot.get() != this->GetPlan()->GetCoordinator()->GetRobot())
    //           robot->SetVirtual(false);
    //       }
    //       continue;
    //     }
    //     std::cout << "releaseHID : " << hid << std::endl;
    //     std::cout << headActiveGroup->GetLabel()
    //               << " can grasp extra obj "
    //               << headPassiveGroup->GetLabel()
    //               << " " << m_objectExtraPose[headPassiveGroup->GetRobots()[0]].PrettyPrint()
    //               << " at " << graspCfg.PrettyPrint()
    //               << std::endl;
    //     std::cout << "Interaction Type: " << interaction->GetLabel()
    //               << " (" << hid << ")" << std::endl;
    //     canReach = true;
    //     break;
    //   }

    //   if (!canReach) {
    //     std::cout << "Cannot reach" << std::endl;
    //     continue;
    //   }

    //   // NOTE: this fixes a syntactic issue while preserving intent (push a pair)
    //   improvePairs.push_back({hid, dependentVID});
    // }

    // improvementSets.insert(improvePairs);
  }

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  auto modeTransitionHypergraphLocal = m_modeTransitionHypergraph;

  std::cout << "\n\nFound all candids" << std::endl;
  // ---------- Apply improvements for each candidate set ----------
  for (const auto& improveList : improvementSets) {
    std::cout << "\nFound " << improveList.size() << " Candidates " << std::endl;
    for (size_t i = 0; i < improveList.size(); ++i) {
      const auto& pair = improveList[i];
      size_t hid = pair.first;
      auto restricted = pair.second;
      // size_t dependentGoalMTVID = pair.second;

      const auto& ha = modeTransitionHypergraphLocal.GetHyperarc(hid);
      VID tailMTVID = *ha.tail.begin();

      RobotGroup* robotObjectGroup =
          modeTransitionHypergraphLocal.GetVertex(tailMTVID).property->robotGroup;
      RobotGroup* activeGroup = nullptr;
      RobotGroup* passiveGroup = nullptr;
      size_t activeMTVID = MAX_UINT;
      size_t passiveMTVID = MAX_UINT;

      for (VID headVid : ha.head) {
        auto g = modeTransitionHypergraphLocal.GetVertex(headVid).property->robotGroup;
        if (g->IsActive()) {
          activeGroup = g;
          activeMTVID = headVid; 
        }
        else{
          passiveGroup = g;
          passiveMTVID = headVid;
        }
      }

      if(m_improveComputed.count(robotObjectGroup)) {
        std::cout << "Computed " << std::endl;
        continue;
      }
      m_improveComputed.insert(robotObjectGroup);

      std::cout << "============ Candid: "
                << robotObjectGroup->GetLabel() << std::endl;

      auto interaction =
          dynamic_cast<Interaction*>(modeTransitionHypergraphLocal.GetHyperarc(hid)
                                         .property.action.first);

      auto activeGrm    = this->GetMPSolution()->GetGroupRoadmap(activeGroup);
      auto passiveGrm   = this->GetMPSolution()->GetGroupRoadmap(passiveGroup);
      auto compositeGrm = this->GetMPSolution()->GetGroupRoadmap(robotObjectGroup);

      compositeGrm->SetAllFormationsInactive();
      auto formations = modeTransitionHypergraphLocal.GetVertex(tailMTVID).property->formations;
      for(auto f : formations) {
        compositeGrm->SetFormationActive(f);
      }


      GroupCfgType activeGcfg(activeGrm);
      GroupCfgType compositeGcfg(compositeGrm);
      GroupCfgType passiveGcfg(passiveGrm);

      Cfg passiveCfg = m_objectExtraPose[passiveGroup->GetRobots()[0]];
      passiveGcfg.SetRobotCfg(passiveGroup->GetRobot(0), std::move(passiveCfg));

      std::cout << "\nInteraction Type: " << interaction->GetLabel() << std::endl;

      bool canReach = false;
      Cfg graspCfg;
      for (size_t j = 0; j < m_maxAttempts; ++j) {
        graspCfg = CanReach(interaction, activeGroup, passiveGcfg);
        if (!graspCfg.GetRobot()) {
          for (auto& robot : this->GetMPProblem()->GetRobots()) {
            if (robot.get() != this->GetPlan()->GetCoordinator()->GetRobot())
              robot->SetVirtual(false);
          }
          continue;
        }
        std::cout << activeGroup->GetLabel()
                  << " can grasp extra obj "
                  << passiveGroup->GetLabel()
                  << " " << m_objectExtraPose[passiveGroup->GetRobots()[0]].PrettyPrint()
                  << " at " << graspCfg.PrettyPrint()
                  << std::endl;
        canReach = true;
        break;
      }
      if (!canReach) {
        std::cout << activeGroup->GetLabel()
                  << " cannot grasp extra obj "
                  << passiveGroup->GetLabel()
                  << " "
                  << m_objectExtraPose[passiveGroup->GetRobots()[0]].PrettyPrint()
                  << std::endl;
        continue;
      }

      // If reachable, add nodes and edges (same logic, clearer names)
      auto robotObjectMode = new Mode();
      robotObjectMode->robotGroup = robotObjectGroup;
      robotObjectMode->formations = m_modeTransitionHypergraph.GetVertex(tailMTVID).property->formations;
      for(auto& c : m_modeTransitionHypergraph.GetVertex(tailMTVID).property->constraints) {
        auto cclone = c->Clone();
        robotObjectMode->constraints.push_back(std::move(cclone));
      }
      VID robotObjectMTVID = m_modeTransitionHypergraph.AddVertex(robotObjectMode);

      LazyModeGraph::Transition tTailToComposite;
      tTailToComposite.cost = 0.1;
      m_modeTransitionHypergraph.AddHyperarc({robotObjectMTVID}, {tailMTVID}, tTailToComposite);

      std::set<VID> headMTVIDs;
      auto activeOnlyMode = new Mode();
      activeOnlyMode->robotGroup = activeGroup;
      activeOnlyMode->formations = m_modeTransitionHypergraph.GetVertex(activeMTVID).property->formations;
      for(auto& c : m_modeTransitionHypergraph.GetVertex(activeMTVID).property->constraints) {
        auto cclone = c->Clone();
        activeOnlyMode->constraints.push_back(std::move(cclone));
      }
      VID activeOnlyMTVID = m_modeTransitionHypergraph.AddVertex(activeOnlyMode);
      headMTVIDs.insert(activeOnlyMTVID);

      auto passiveOnlyMode = new Mode();
      passiveOnlyMode->robotGroup = passiveGroup;
      passiveOnlyMode->formations = m_modeTransitionHypergraph.GetVertex(passiveMTVID).property->formations;
      for(auto& c : m_modeTransitionHypergraph.GetVertex(passiveMTVID).property->constraints) {
        auto cclone = c->Clone();
        passiveOnlyMode->constraints.push_back(std::move(cclone));
      }
      VID passiveOnlyMTVID = m_modeTransitionHypergraph.AddVertex(passiveOnlyMode);
      headMTVIDs.insert(passiveOnlyMTVID);
      m_unactuatedMTVIDs.insert(passiveOnlyMTVID);

      // Configure cfgs and add to grounded hypergraph
      auto activeGraspCfgCopy = graspCfg;
      std::cout << passiveGroup->GetLabel() << std::endl;

      auto pcfg = m_objectExtraPose[passiveGroup->GetRobots()[0]];
      activeGcfg.SetRobotCfg(activeGroup->GetRobot(0), std::move(graspCfg));
      // compositeGcfg.SetRobotCfg(passiveGroup->GetRobot(0), std::move(pcfg));
      // compositeGcfg.SetRobotCfg(activeGroup->GetRobot(0), std::move(activeGraspCfgCopy));
      std::unordered_map<Robot*,std::unique_ptr<CSpaceConstraint>> constraintMap;
      constraintMap[activeGroup->GetRobot(0)] = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(activeGroup->GetRobot(0),activeGraspCfgCopy));
      compositeGcfg.GetRandomGroupCfg(constraintMap[activeGroup->GetRobot(0)]->GetBoundary());

      auto activeRMVID    = activeGrm->AddVertex(activeGcfg);
      auto passiveRMVID   = passiveGrm->AddVertex(passiveGcfg);
      auto compositeRMVID = compositeGrm->AddVertex(compositeGcfg);

      std::cout << "FFFOOOO 1" << std::endl;
      for(auto f : compositeGrm->GetFormations()) {
        std::cout << f << std::endl;
      }

      GroundedVertex activeGV    = std::make_pair(activeGrm, activeRMVID);
      GroundedVertex passiveGV   = std::make_pair(passiveGrm, passiveRMVID);
      GroundedVertex compositeGV = std::make_pair(compositeGrm, compositeRMVID);

      std::cout << "Adding Active robot " << activeGroup->GetLabel()
                << " gcfg " << activeGcfg.PrettyPrint()
                << ", grasping goal object " << passiveGroup->GetLabel()
                << ": rmVid " << activeRMVID << std::endl;

      std::cout << "Adding composite robot " << robotObjectGroup->GetLabel()
                << " gcfg " << compositeGcfg.PrettyPrint()
                << ", grasping goal object " << passiveGroup->GetLabel()
                << ": rmVid " << compositeRMVID << std::endl;

      auto activeGVID    = gh->AddVertex(activeGV);
      auto passiveGVID   = gh->AddVertex(passiveGV);
      auto compositeGVID = gh->AddVertex(compositeGV);

      m_modeTransitionGroundedVertices[activeOnlyMTVID].insert(activeGVID);
      m_modeTransitionGroundedVertices[passiveOnlyMTVID].insert(passiveGVID);
      m_modeTransitionGroundedVertices[robotObjectMTVID].insert(compositeGVID);
      m_modeTransitionGroundedVertices[tailMTVID].insert(compositeGVID);


      // Add Transitions
      auto robotBase = activeGroup->GetRobot(0)->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
      auto objectBase = passiveGroup->GetRobot(0)->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
      double distanceSquared = 0.0;
      for (size_t i = 0; i < 3; ++i) {
          double diff = robotBase[i] - objectBase[i];
          distanceSquared += diff * diff;
      }
      double distance = std::sqrt(distanceSquared);

      LazyModeGraph::Transition tCompositeToHeads;
      tCompositeToHeads.cost = distance*0.1000;
      tCompositeToHeads.action = std::make_pair(interaction, true);
      size_t newHID = m_modeTransitionHypergraph.AddHyperarc(headMTVIDs, {robotObjectMTVID}, tCompositeToHeads);
      m_modeTransitionHypergraph.GetHyperarc(newHID).property.action.first->SetDirectReverseAllowed(true);
      std::set<size_t> tempHead = {passiveGVID, activeGVID};
      std::set<size_t> tempTail = {compositeGVID};
      m_groundedTransitionMap[newHID].insert(std::make_pair(tempTail,tempHead));

      std::cout << tailMTVID << " -->> " << robotObjectMTVID << std::endl;
      std::cout << robotObjectMTVID << " -->> " << headMTVIDs << std::endl;

      for (VID ignVid : m_ignitionModeTransitionVIDs) {
        auto g = modeTransitionHypergraphLocal.GetVertex(ignVid).property->robotGroup;
        if (g->GetRobots().size() == 1 && g->IsActive() && g == activeGroup) {
          LazyModeGraph::Transition tIgnToActive;
          tIgnToActive.cost = 0.;
          size_t newHID = m_modeTransitionHypergraph.AddHyperarc({ignVid}, {activeOnlyMTVID}, tIgnToActive);
          std::cout << activeOnlyMTVID << " -->> " << ignVid << std::endl;
          m_modeTransitionGroundedVertices[ignVid].insert(activeGVID);
          tempHead = {activeGVID};
          tempTail = {activeGVID};
          m_groundedTransitionMap[newHID].insert(std::make_pair(tempTail,tempHead));
        }
      }

      for(size_t r : restricted) {
        if(!m_ignitionModeTransitionVIDs.count(r))
          m_geometricConstraintSet2[passiveOnlyMTVID].insert(r);
      }
      std::cout << " " << passiveOnlyMTVID << " must be visited for " << m_geometricConstraintSet2[passiveOnlyMTVID] << " to be appeared."<< std::endl;

      // ----- Secondary candidates using the first improvement’s state -----
      // for (size_t k = 0; k < improveList.size(); ++k) {
        // const auto& pair2 = improveList[k];
        // if (pair == pair2) {
        //   std::cout << "\tPass " << k << std::endl;
        //   continue;
        // }

        // HID hid2 = pair2.first;

        // auto ha2 = modeTransitionHypergraphLocal.GetHyperarc(hid2);
        // VID tailMTVID2 = *ha2.tail.begin();
      std::cout << "-------------- Find extra candidates --------------" << std::endl;
      for (VID iv : m_ignitionModeTransitionVIDs) {
        RobotGroup* activeGroup2 = nullptr;
        RobotGroup* passiveGroup2 = passiveGroup;
        RobotGroup* robotObjectGroup2 = nullptr;
        VID activeOnlyMTVID2 = MAX_UINT;
        VID passiveOnlyMTVID2 = passiveOnlyMTVID;
        VID robotObjectMTVID2 = MAX_UINT;

        auto g = modeTransitionHypergraphLocal.GetVertex(iv).property->robotGroup;
        if (g->IsPassive()) 
          continue;
        activeGroup2 = g;
        activeOnlyMTVID2 = iv;

        std::set<size_t> reverseHids{};
        for(auto kv : m_modeHypergraph.GetHyperarcMap()) {
          auto id = kv.first;
          auto ha = kv.second;
          std::set<RobotGroup*> gset;
          if(ha.tail.size() != 2 or ha.head.size() != 1) 
            continue;
          for(auto t : ha.tail) {
            auto gg = m_modeHypergraph.GetVertex(t).property->robotGroup;
            gset.insert(gg);
          }
          if(gset.count(activeGroup2) and gset.count(passiveGroup2)) {
            reverseHids.insert(id);
          }
        }
        
        std::cout << reverseHids << std::endl;

        for(size_t reverseHid : reverseHids) {
          auto ha = m_modeHypergraph.GetHyperarc(reverseHid);
          size_t tempRobotObjectMVID = *ha.head.begin();
          size_t robotObjectMTVID3 = m_vidConversionMap[tempRobotObjectMVID];

          robotObjectGroup2 = m_modeHypergraph.GetVertex(tempRobotObjectMVID).property->robotGroup;
            
          auto interaction2 = dynamic_cast<Interaction*>(m_modeHypergraph.GetHyperarc(reverseHid).property.first);

          auto activeGrm2    = this->GetMPSolution()->GetGroupRoadmap(activeGroup2);
          auto passiveGrm2   = this->GetMPSolution()->GetGroupRoadmap(passiveGroup2);
          auto compositeGrm2 = this->GetMPSolution()->GetGroupRoadmap(robotObjectGroup2);

          compositeGrm2->SetAllFormationsInactive();
          auto formations = m_modeHypergraph.GetVertex(tempRobotObjectMVID).property->formations;
          for(auto f : formations) {
            compositeGrm2->SetFormationActive(f);
          }

          GroupCfgType activeGcfg2(activeGrm2);
          GroupCfgType passiveGcfg2(passiveGrm2);
          GroupCfgType compositeGcfg2(compositeGrm2);
          
          Cfg passiveCfg2 = m_objectExtraPose[passiveGroup2->GetRobots()[0]];
          passiveGcfg2.SetRobotCfg(passiveGroup2->GetRobot(0), std::move(passiveCfg2));

          // std::cout << "Interaction Type: " << interaction2->GetLabel()
          //           << " (" << hid << ")" << std::endl;
          std::cout << "\nInteraction Type: " << interaction2->GetLabel() << std::endl;

          bool canReach2 = false;
          Cfg graspCfg2;
          for (size_t j = 0; j < m_maxAttempts; ++j) {
            graspCfg2 = CanReach(interaction2, activeGroup2, passiveGcfg2);
            if (!graspCfg2.GetRobot()) {
              for (auto& robot : this->GetMPProblem()->GetRobots()) {
                if (robot.get() != this->GetPlan()->GetCoordinator()->GetRobot())
                  robot->SetVirtual(false);
              }
              continue;
            }
            std::cout << activeGroup2->GetLabel()
                      << " can grasp extra obj "
                      << passiveGroup2->GetLabel()
                      << " " << m_objectExtraPose[passiveGroup2->GetRobots()[0]].PrettyPrint()
                      << " at " << graspCfg2.PrettyPrint()
                      << std::endl;
            canReach2 = true;
            break;
          }
          if (!canReach2) {
            std::cout << activeGroup2->GetLabel()
                      << " cannot grasp extra obj "
                      << passiveGroup2->GetLabel()
                      << " "
                      << m_objectExtraPose[passiveGroup2->GetRobots()[0]].PrettyPrint()
                      << std::endl;
            continue;
          }


          std::cout << "====== Candid 2: " << robotObjectGroup2->GetLabel() << std::endl;
          std::cout << "Active " << activeGroup2->GetLabel() << std::endl;
          std::cout << "Passive " << passiveGroup2->GetLabel() << std::endl;
          std::cout << "PassiveActive " << robotObjectGroup2->GetLabel() << std::endl;


          std::cout << "FFFOOOO 2" << std::endl;
          for(auto f : compositeGrm2->GetFormations()) {
            std::cout << f << std::endl;
          }

          auto robotObjectMode2 = new Mode();
          robotObjectMode2->robotGroup = robotObjectGroup;
          robotObjectMode2->formations = m_modeTransitionHypergraph.GetVertex(tailMTVID).property->formations;
          for(auto& c : m_modeTransitionHypergraph.GetVertex(tailMTVID).property->constraints) {
            auto cclone = c->Clone();
            robotObjectMode2->constraints.push_back(std::move(cclone));
          }
          robotObjectMTVID2 = m_modeTransitionHypergraph.AddVertex(robotObjectMode2);



          // Configure cfgs and add to grounded hypergraph (secondary)
          auto activeGraspCfgCopy2 = graspCfg2;
          auto pcfg2 = m_objectExtraPose[passiveGroup2->GetRobots()[0]];
          activeGcfg2.SetRobotCfg(activeGroup2->GetRobot(0), std::move(graspCfg2));
          // compositeGcfg2.SetRobotCfg(passiveGroup2->GetRobot(0), std::move(pcfg2));
          // compositeGcfg2.SetRobotCfg(activeGroup2->GetRobot(0), std::move(activeGraspCfgCopy2));
          std::unordered_map<Robot*,std::unique_ptr<CSpaceConstraint>> constraintMap2;
          constraintMap2[activeGroup2->GetRobot(0)] = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(activeGroup2->GetRobot(0),activeGraspCfgCopy2));
          compositeGcfg2.GetRandomGroupCfg(constraintMap2[activeGroup2->GetRobot(0)]->GetBoundary());

          auto activeRMVID2    = activeGrm2->AddVertex(activeGcfg2);
          auto passiveRMVID2   = passiveGrm2->AddVertex(passiveGcfg2);
          auto compositeRMVID2 = compositeGrm2->AddVertex(compositeGcfg2);

          GroundedVertex activeGV2    = std::make_pair(activeGrm2, activeRMVID2);
          GroundedVertex passiveGV2   = std::make_pair(passiveGrm2, passiveRMVID2);
          GroundedVertex compositeGV2 = std::make_pair(compositeGrm2, compositeRMVID2);

          std::cout << "Adding Active robot " << activeGroup2->GetLabel()
                    << " gcfg " << activeGcfg2.PrettyPrint()
                    << ", grasping goal object " << passiveGroup->GetLabel()
                    << ": rmVid " << activeRMVID2 << std::endl;

          std::cout << "Adding composite robot " << robotObjectGroup2->GetLabel()
                    << " gcfg " << compositeGcfg2.PrettyPrint()
                    << ", grasping goal object " << passiveGroup->GetLabel()
                    << ": rmVid " << compositeRMVID2 << std::endl;

          auto activeGVID2    = gh->AddVertex(activeGV2);
          auto passiveGVID2   = gh->AddVertex(passiveGV2);
          auto compositeGVID2 = gh->AddVertex(compositeGV2);

          m_modeTransitionGroundedVertices[activeOnlyMTVID2].insert(activeGVID2);
          m_modeTransitionGroundedVertices[passiveOnlyMTVID2].insert(passiveGVID2);
          m_modeTransitionGroundedVertices[robotObjectMTVID2].insert(compositeGVID2);
          m_modeTransitionGroundedVertices[robotObjectMTVID3].insert(compositeGVID2);



          LazyModeGraph::Transition tTailToComposite2;
          tTailToComposite2.cost = 0.1;
          size_t roNewHID =  m_modeTransitionHypergraph.AddHyperarc({robotObjectMTVID3}, {robotObjectMTVID2}, tTailToComposite2);

          std::cout << robotObjectMTVID2 << " -->> " << robotObjectMTVID3 << std::endl;

          tempHead = {compositeGVID2};
          tempTail = {compositeGVID2};
          m_groundedTransitionMap[roNewHID].insert(std::make_pair(tempTail,tempHead));

          

          // Add transitions (secondary)
          auto robotBase = activeGroup2->GetRobot(0)->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
          auto objectBase = passiveGroup2->GetRobot(0)->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
          double distanceSquared = 0.0;
          for (size_t i = 0; i < 3; ++i) {
              double diff = robotBase[i] - objectBase[i];
              distanceSquared += diff * diff;
          }
          double distance = std::sqrt(distanceSquared);


          LazyModeGraph::Transition t2;
          t2.cost = distance*0.1000;
          t2.action = std::make_pair(interaction2, false);
          size_t newHID = m_modeTransitionHypergraph.AddHyperarc({robotObjectMTVID2},
                                                {passiveOnlyMTVID2, activeOnlyMTVID2},
                                                t2);
          std::cout << passiveOnlyMTVID2 << " " << activeOnlyMTVID2 << " -->> " << robotObjectMTVID2 << std::endl;
          
          std::set<size_t> tempHead = {compositeGVID2};
          std::set<size_t> tempTail = {passiveGVID2, activeGVID2};
          m_groundedTransitionMap[newHID].insert(std::make_pair(tempTail,tempHead));
        }

        for(size_t r : restricted) {
          if(!m_ignitionModeTransitionVIDs.count(r))
            m_geometricConstraintSet2[passiveOnlyMTVID2].insert(r);
        }
        std::cout << " " << passiveOnlyMTVID2 << " must be visited for " << m_geometricConstraintSet2[passiveOnlyMTVID2] << " to be appeared."<< std::endl;
      }

      // // Update non-monotonic constraints
      // if(dependentGoalMTVID != MAX_UINT) {
      //   m_nonMonotonicConstraintSet[dependentGoalMTVID].insert(passiveOnlyMTVID);
      //   for (VID tv : m_terminationModeTransitionVIDs) {
      //     auto group = modeTransitionHypergraphLocal.GetVertex(tv).property->robotGroup;
      //     if (group == passiveGroup) {
      //       m_nonMonotonicConstraintSet[tv].insert(dependentGoalMTVID);
      //     }
      //   }
      // }
    }
    std::cout << "Done ==============" << std::endl;
  }

  // ---------- Grounded transition map maintenance ----------
  std::cout << "Before" << std::endl;
  for (const auto& kv : m_groundedTransitionMap)
    std::cout << kv.first << ": " << kv.second << std::endl;

  for (const auto& mtgv : m_modeTransitionGroundedVertices) {
    VID mtvid = mtgv.first;
    if (mtvid == m_sinkModeTransitionVID || mtvid == m_sourceModeTransitionVID)
      continue;

    std::cout << mtvid << " ("
              << m_modeTransitionHypergraph.GetVertex(mtvid).property->robotGroup->GetLabel()
              << ")" << ": " << std::endl;

    auto group = m_modeTransitionHypergraph.GetVertex(mtvid).property->robotGroup;

    if (group->GetRobots().size() == 1 &&
        group->GetRobot(0)->GetMultiBody()->IsPassive())
      continue;

    std::set<HID> mthid{};
    for (const auto& haKV : m_modeTransitionHypergraph.GetHyperarcMap()) {
      const auto& ha = haKV.second;
      if (!(ha.tail.size() == 1 && ha.head.size() == 1)) 
        continue;
      if (group == m_modeTransitionHypergraph.GetVertex(*ha.tail.begin())
                        .property->robotGroup) {
        mthid.insert(haKV.first);
      }
    }
    if (mthid.empty()) 
      continue;

    for (auto gvid : mtgv.second) {
      std::pair<std::set<VID>, std::set<VID>> both{ {gvid}, {gvid} };
      std::cout << "\tAdd " << mthid << ": " << both << std::endl;
      for(size_t hid : mthid)
        m_groundedTransitionMap[hid].insert(both);
    }
  }

  std::cout << "After" << std::endl;
  for (const auto& kv : m_groundedTransitionMap)
    std::cout << kv.first << ": " << kv.second << std::endl;

  // ---------- Debug prints ----------
  m_modeTransitionHypergraph.Print();
  std::cout << "Ignitions: " << m_ignitionModeTransitionVIDs << std::endl;
  std::cout << "Terminations: " << m_terminationModeTransitionVIDs << std::endl;
  std::cout << "VID : Label" << std::endl;
  for (const auto& vm : m_modeTransitionHypergraph.GetVertexMap()) {
    VID vid = vm.first;
    std::string label = vm.second.property->isDummy()
                        ? "..."
                        : vm.second.property->robotGroup->GetLabel();
    std::cout << vid << " : " << label << std::endl;
  }
}












void
LazyModeGraph::
GenerateRepresentation() {
  std::cout << "Start GenerateRepresentation" << std::endl;
  auto problem = this->GetMPProblem();
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GenerateRepresentation");

  // Initialize MPSolution
  auto c = this->GetPlan()->GetCoordinator();
  //this->GetMPSolution() = std::unique_ptr<MPSolution>(new MPSolution(c->GetRobot()));

  // Construct initial state from coordinator
  State start;

  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    auto formation = kv.second;

    // Add individual robots to MPSolution
    for(auto& r : group->GetRobots()) {
      this->GetMPSolution()->AddRobot(r);
    }

    // Create new group roadmaps in MPSolution
    this->GetMPSolution()->AddRobotGroup(group);
    auto grm = this->GetMPSolution()->GetGroupRoadmap(group);

    // Add the initial formation to the roadmap and set it active
    if(formation) {
      grm->AddFormation(formation);
      grm->SetFormationActive(formation);
    }

    // Create initial group cfgs
    auto gcfg = GroupCfgType(grm);

    // Add initial cfg to individual roadmaps
    for(auto& r : group->GetRobots()) {
      auto rm = this->GetMPSolution()->GetRoadmap(r);
      auto cfg = problem->GetInitialCfg(r);
      auto vid = rm->AddVertex(cfg);

      // Update group cfg
      gcfg.SetRobotCfg(r,vid);
    }

    // Add group cfg to group roadmap
    auto vid = grm->AddVertex(gcfg);

    // Add group and vertex to start state
    start[group] = std::make_pair(grm,vid);
  }
  m_start = start;

  auto initialModes = AddStartState(start);

  // Mode hypergraph
  GenerateModeHypergraph(initialModes);
  std::cout << "MODE HYPERGRAPH" << std::endl;
  for(auto kv : m_modeHypergraph.GetHyperarcMap()) {
    auto hid = kv.first;
    auto arc = kv.second;

    std::string actionLabel = "...";
    if(arc.property.first)
      actionLabel = arc.property.first->GetLabel();
    std::cout << hid 
              << "(" << actionLabel
              << ") : {Tail:[";

    for(auto iter = arc.tail.begin(); iter != arc.tail.end(); iter++) {
      std::cout << *iter;
      auto next = iter;
      next++;
      if(next != arc.tail.end()) {
        std::cout << ",";
      }
    }

    std::cout << "], Head:[";

    for(auto iter = arc.head.begin(); iter != arc.head.end(); iter++) {
      std::cout << *iter;
      auto next = iter;
      next++;
      if(next != arc.head.end()) {
        std::cout << ",";
      }
    }
    std::cout << "]}" << std::endl;
  }

  std::cout << "Mode : Label" << std::endl;
  for (auto kv : m_modeHypergraph.GetVertexMap()) {
    auto mode = kv.first;
    std::string label = "...";
    label = kv.second.property->robotGroup->GetLabel();

    std::cout << mode << " : " << label << std::endl;
  }

  // Scott - Generated Mode Hypergraph.
  // Grounded Hypergraph is yet generated.


  // Grounded hypergraph
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  GroundedVertex source = std::make_pair(nullptr,0);
  m_sourceGroundedVID = gh->AddVertex(source);
  GroundedVertex sink = std::make_pair(nullptr,MAX_UINT);
  m_sinkGroundedVID = gh->AddVertex(sink);

  SampleNonActuatedCfgs(start);

  GenerateModeTransitionHypergraph(start); 

  for(auto kv : m_modeTransitionHypergraph.GetHyperarcMap()) {
    auto hid = kv.first;
    auto arc = kv.second;

    std::string actionLabel = "...";
    if(arc.property.action.first)
      actionLabel = arc.property.action.first->GetLabel();
    std::cout << hid << " (c:" << (arc.property.cost) 
              << ", a: " << actionLabel
              << ") : {Tail:[";

    for(auto iter = arc.tail.begin(); iter != arc.tail.end(); iter++) {
      std::cout << *iter;
      auto next = iter;
      next++;
      if(next != arc.tail.end()) {
        std::cout << ",";
      }
    }

    std::cout << "], Head:[";

    for(auto iter = arc.head.begin(); iter != arc.head.end(); iter++) {
      std::cout << *iter;
      auto next = iter;
      next++;
      if(next != arc.head.end()) {
        std::cout << ",";
      }
    }
    std::cout << "]}" << std::endl;
  }

  std::cout << "END OF HYPERGRAPH" << std::endl;


  std::cout << "Mode : Label" << std::endl;
  for (auto kv : m_modeTransitionHypergraph.GetVertexMap()) {
    auto mode = kv.first;
    std::string label = "...";
    if(kv.first != m_sourceModeTransitionVID and kv.first != m_sinkModeTransitionVID) {
      label = kv.second.property->robotGroup->GetLabel();
    }
    std::cout << mode << " : " << label << std::endl;
  }

  // if(m_debug) {
  std::cout << "MODE HYPERGRAPH" << std::endl;
  m_modeHypergraph.Print();

  std::cout << "Mode : Label" << std::endl;
  for (auto kv : m_modeHypergraph.GetVertexMap()) {
    auto vid = kv.first;
    std::cout << vid << " : " << kv.second.property->robotGroup->GetLabel() << std::endl;
  }


  for(auto kv : m_modeHypergraph.GetHyperarcMap()) {
    std::cout << kv.first << ": " << kv.second.property.first->GetLabel() << std::endl;
  }

  if(m_writeHypergraphs) {
    std::string base = this->GetMPProblem()->GetBaseFilename();
    m_modeHypergraph.Print(base + "-mode-hypergraph.hyp");
    //m_groundedHypergraph.Print(base + "-grounded-hypergraph.hyp");
  }


}

/*-------------------------------- Accessors ---------------------------------*/

LazyModeGraph::ModeHypergraph&
LazyModeGraph::
GetModeHypergraph() {
  return m_modeHypergraph;
}

LazyModeGraph::GroundedHypergraphLocal&
LazyModeGraph::
GetGroundedHypergraphLocal() {
  return m_groundedHypergraph;
}

LazyModeGraph::GroupRoadmapType*
LazyModeGraph::
GetGroupRoadmap(RobotGroup* _group) {
  return this->GetMPSolution()->GetGroupRoadmap(_group);
}

/*MPSolution* 
LazyModeGraph::
GetMPSolution() {
  return this->GetMPSolution().get();
}*/
    
LazyModeGraph::VID 
LazyModeGraph::
GetModeOfGroundedVID(const VID& _vid) const {
  for(const auto& kv : m_modeGroundedVertices) {
    if(kv.second.count(_vid)){
      return kv.first; 
    }
  }
  return MAX_UINT;
}

LazyModeGraph::VID 
LazyModeGraph::
GetTransitionModeOfGroundedVID(const VID& _vid) const {
  for(const auto& kv : m_modeTransitionGroundedVertices) {
    if(kv.second.count(_vid)){
      return kv.first; 
    }
  }
  return MAX_UINT;
}


/*---------------------------- Helper Functions ------------------------------*/

std::vector<LazyModeGraph::VID>
LazyModeGraph::
AddStartState(const State& _start) {

  // Extract initial submodes
  std::vector<VID> newModes;
  for(auto kv : _start) {
    bool exist = false;
    for (const auto& mode : m_modes) {
      if (kv.first == mode->robotGroup)
        exist = true;
    }
    if (exist) {
      if(m_debug)
        std::cout << "mode already exist" << std::endl;
      continue;
    }

    std::unique_ptr<Mode> mode = std::unique_ptr<Mode>(new Mode);
    mode->robotGroup = kv.first;
    auto rm = kv.second.first;
    mode->formations = rm->GetActiveFormations();
    auto tasks = this->GetMPProblem()->GetTasks(mode->robotGroup);

    for(auto task : tasks) {

      for(auto individualTask : *(task.get())) {

        auto& constraints = individualTask.GetPathConstraints();

        for(auto& constraint : constraints) {

          auto c = constraint->Clone();

          mode->constraints.push_back(std::move(c));

          auto c2 = constraint->Clone();

          m_constraintMap[kv.first].push_back(std::move(c2));
        }
      }
    }

    m_modes.push_back(std::move(mode));

    auto vid = m_modeHypergraph.AddVertex(m_modes.back().get());
    newModes.push_back(vid);
  }

  return newModes;
}

void
LazyModeGraph::
GenerateModeTransitionHypergraph(const State& _start) {
                                  
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();                             
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GenerateModeTransitionHypergraph");
  auto problem = this->GetPlan()->GetCoordinator()->GetRobot()->GetMPProblem();

  auto lib = this->GetMPLibrary();
  auto qSM = lib->GetSampler(m_querySM);
  // auto prob = this->GetMPProblem();
  auto decomp = plan->GetDecomposition();
  lib->SetMPSolution(this->GetMPSolution());

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  
  std::unordered_map<RobotGroup*,VID> startRobotGroundedVIDs;
  SampleCfgs(_start,startRobotGroundedVIDs);
  m_startRobotGroundedVIDs = startRobotGroundedVIDs;

  // Copy vertices
  // std::map<std::set<RobotGroup*>,std::map<std::set<RobotGroup*>,std::pair<double,std::pair<Action*,bool>>>> hyperarcInfo;
  std::map<std::pair<std::set<RobotGroup*>,std::set<RobotGroup*>>,std::unordered_map<HID,double>> interactionCostInfo;
  std::set<Mode*> startModes;
  std::set<Mode*> goalModes;
  std::unordered_map<RobotGroup*,VID> initials;
  std::unordered_map<RobotGroup*,VID> ignitions;
  std::unordered_map<RobotGroup*,VID> terminations;
  for(const auto& mode : m_modes) {
    auto vid = m_modeHypergraph.GetVID(mode.get());
    auto newVid = m_modeTransitionHypergraph.AddVertex(mode.get());
    initials[mode->robotGroup] = vid;
    m_vidConversionMap.insert(std::make_pair(vid,newVid));
    if(mode->robotGroup->GetRobots().size()==1) {
      Mode* goalMode = new Mode();
      goalMode->robotGroup = mode->robotGroup;
      goalModes.insert(goalMode);
      ignitions[mode->robotGroup] = newVid;
      m_ignitionModeTransitionVIDs.insert(newVid);
    }
  }

  std::cout << "vid conversion map" << std::endl;
  for(auto vid : m_vidConversionMap) {
    std::cout << vid.first << " --> " << vid.second << std::endl;
  }

  // Add start and goal modes
  // std::unique_ptr<Mode> sourceMode = std::unique_ptr<Mode>(new Mode);
  Mode* sourceMode = new Mode();
  sourceMode->robotGroup = nullptr;
  sourceMode->setType(VertexType::Dummy);
  auto sourceTransitionModeVID = m_modeTransitionHypergraph.AddVertex(sourceMode);
  m_sourceModeTransitionVID = sourceTransitionModeVID;
  m_modeTransitionGroundedVertices[sourceTransitionModeVID].insert(m_sourceGroundedVID);

  Mode* sinkMode = new Mode();
  sinkMode->robotGroup = nullptr;
  sinkMode->setType(VertexType::Dummy);
  auto sinkTransitionModeVID = m_modeTransitionHypergraph.AddVertex(sinkMode);
  m_sinkModeTransitionVID = sinkTransitionModeVID;
  m_modeTransitionGroundedVertices[sinkTransitionModeVID].insert(m_sinkGroundedVID);

  std::cout << "source mode vid: " << sourceTransitionModeVID << std::endl;
  std::cout << "sink mode vid: " << sinkTransitionModeVID << std::endl;

  // Configure Start Modes
  std::unordered_map<RobotGroup*,GroupCfgType> startObjectGcfgs;
  for(auto kv : ignitions) {
    // auto group = kv.first;
    auto mVID = kv.second;
    auto startMode = m_modeTransitionHypergraph.GetVertex(mVID).property;
    std::cout << "Add start mode " << startMode->robotGroup->GetLabel() << std::endl;
    ignitions[startMode->robotGroup] = mVID;
    bool passive = true;
    for(auto robot : startMode->robotGroup->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive()) {
        passive = false;
        break;
      }
    }
    if(passive) {
      m_unactuatedMTVIDs.insert(mVID);
      for(auto kv : m_modeStartGroundedVIDs) {
        auto mvid = kv.first;
        auto group = m_modeHypergraph.GetVertex(mvid).property->robotGroup;
        if(startMode->robotGroup == group) {
          for(auto gvid : kv.second) {
            m_modeTransitionGroundedVertices[mVID].insert(gvid);
            auto iter = _start.find(group);
            if(iter != _start.end()) {
              auto state = _start.at(group);
              auto gcfg = state.first->GetVertex(state.second);
              startObjectGcfgs[group] = gcfg;
            }
          }
        }
      }
    }
    else {
      auto gvid = m_startRobotGroundedVIDs[startMode->robotGroup];
      m_modeTransitionGroundedVertices[mVID].insert(gvid);
    }
  }

















  // Configure Goal Modes
  std::unordered_map<RobotGroup*,std::set<GroupCfgType>> goalObjectGcfgs;
  for(auto goalMode : goalModes) {
    // auto originalModeVID = goalMode.first;
    std::cout << "Add goal mode " << goalMode->robotGroup->GetLabel() << std::endl;
    auto mVID = m_modeTransitionHypergraph.AddVertex(goalMode);
    terminations[goalMode->robotGroup] = mVID;
    bool passive = true;
    for(auto robot : goalMode->robotGroup->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive()) {
        passive = false;
        break;
      }
    }
    if(passive) {
      // m_unactuatedMTVIDs.insert(originalModeVID);
      m_unactuatedMTVIDs.insert(mVID);
      m_terminationModeTransitionVIDs.insert(mVID);
      for(auto kv : m_modeGoalGroundedVertices) {
        auto mvid = kv.first;
        auto group = m_modeHypergraph.GetVertex(mvid).property->robotGroup;
        if(goalMode->robotGroup == group) {
          for(auto gvid : kv.second) {
            m_modeTransitionGroundedVertices[mVID].insert(gvid);
            m_relevantGVIDs.insert(gvid);
          }
        }
      }
      std::set<RobotGroup*> used;
      std::cout << "GMT size: " << decomp->GetGroupMotionTasks().size() << std::endl;
      for(auto st : decomp->GetGroupMotionTasks()) {
        auto task = st->GetGroupMotionTask().get();
        if(!task)
          continue;
        
        auto group = task->GetRobotGroup();
        std::cout << group->GetLabel() << " : " << goalMode->robotGroup->GetLabel() << std::endl;
        if(group != goalMode->robotGroup)
          continue;
        // if(used.count(goalMode->robotGroup))
        //   continue;

        // used.insert(goalMode->robotGroup);
        
        // Sample goal cfg
        std::map<Robot*, const Boundary*> boundaryMap;
        lib->SetGroupTask(task);
        std::cout << "task size: " << task->Size() << std::endl;
        for(auto iter = task->begin(); iter != task->end(); iter++) {
          std::cout << "goal constraint size: " << iter->GetGoalConstraints().size() << std::endl;
          auto c = dynamic_cast<BoundaryConstraint*>(iter->GetGoalConstraints()[0].get());
          auto b = c->GetBoundary();
          boundaryMap[iter->GetRobot()] = b;
        }
        std::vector<GroupCfgType> samples;
        qSM->Sample(1,m_maxAttempts,boundaryMap,std::back_inserter(samples));
        
        if(samples.size() == 0)
          throw RunTimeException(WHERE) << "Unable to generate goal configuration for "
                                        << group->GetLabel()
                                        << ".";
        auto gcfg = samples[0];
        goalObjectGcfgs[group].insert(gcfg);
      }
    }
    else {
      auto gvid = m_startRobotGroundedVIDs[goalMode->robotGroup];
      m_modeTransitionGroundedVertices[mVID].insert(gvid);
    }
  }


  for(auto kv1 : ignitions) {
    auto group1 = kv1.first;
    auto mtvid1 = kv1.second;
    if(group1->GetRobot(0)->GetMultiBody()->IsPassive())
      continue;
    for(auto kv2 : terminations) {
      auto group2 = kv2.first;
      auto mtvid2 = kv2.second;
      if(group1 == group2) {
        LazyModeGraph::Transition transition;
        transition.cost = 0.;
        m_modeTransitionHypergraph.AddHyperarc({mtvid1},{mtvid2},transition);
      }
    }
  }



  // Get object's start and goal
  std::cout << "Identify hids for start and goal" << std::endl;
  std::set<RobotGroup*> foundStartInteraction;
  for(auto kv : m_modeHypergraph.GetHyperarcMap()) {
    auto tail = kv.second.tail;
    auto head = kv.second.head;

    RobotGroup* active = nullptr;
    RobotGroup* passive = nullptr;
    RobotGroup* activePassive = nullptr;
    VID activeVID = MAX_UINT;
    VID passiveVID = MAX_UINT;
    VID activePassiveVID = MAX_UINT;
    
    // TODO: need a better way to identify grasp interactions
    if(!(tail.size()==2 and head.size()==1)) 
      continue;

    for(size_t t : tail) {
      auto group = m_modeHypergraph.GetVertex(t).property->robotGroup;
      if(group->GetRobot(0)->GetMultiBody()->IsPassive()) {
        passive = group;
        passiveVID = ignitions[group];
      }
      else {
        active = group;
        activeVID = ignitions[group];
      }
    }

    auto newHead = m_vidConversionMap[*(head.begin())];
    activePassive = m_modeHypergraph.GetVertex(*(head.begin())).property->robotGroup;
    activePassiveVID = m_vidConversionMap[m_modeHypergraph.GetVertex(*(head.begin())).vid];
    
    auto interaction = dynamic_cast<Interaction*>(kv.second.property.first);
    auto activeGrm = this->GetMPSolution()->GetGroupRoadmap(active);
    auto compositeGrm = this->GetMPSolution()->GetGroupRoadmap(activePassive);
    GroupCfgType activeGcfg(activeGrm);
    GroupCfgType compositeGcfg(compositeGrm);

    // bool foundInteraction = false;
    for(size_t j = 0; j < m_maxAttempts; j++) {
      auto cfg = CanReach(interaction,active,startObjectGcfgs[passive]);
      if(!cfg.GetRobot()) {
        for(auto& robot : problem->GetRobots()) {
          if(robot.get() != plan->GetCoordinator()->GetRobot()) {
            robot->SetVirtual(false);
          }
        }
        std::cout << "[Start] " << active->GetLabel() << " can not grasp " << passive->GetLabel() << " " << startObjectGcfgs[passive].PrettyPrint() << std::endl;
        continue;
      }
      // foundInteraction = true;
      std::cout << "[Start] " << active->GetLabel() << " can grasp " << passive->GetLabel() << " " << startObjectGcfgs[passive].PrettyPrint() << std::endl;
      foundStartInteraction.insert(passive);

      // Add start cfg to grounded hypergraph
      auto activeCfgCopy = cfg;
      auto passiveCfg = startObjectGcfgs[passive].GetRobotCfg(passive->GetRobot(0));
      activeGcfg.SetRobotCfg(active->GetRobot(0),std::move(cfg));
      compositeGcfg.SetRobotCfg(passive->GetRobot(0),std::move(passiveCfg));
      compositeGcfg.SetRobotCfg(active->GetRobot(0),std::move(activeCfgCopy));
      auto activeRMVID = activeGrm->AddVertex(activeGcfg);
      auto compositeRMVID = compositeGrm->AddVertex(compositeGcfg);
      GroundedVertex activeGv = std::make_pair(activeGrm,activeRMVID);
      GroundedVertex compositeGv = std::make_pair(compositeGrm,compositeRMVID);
      std::cout << "Adding Active robot " << active->GetLabel() 
                << " gcfg " << activeGcfg.PrettyPrint() 
                << ", grasping start object " << passive->GetLabel()
                << ": rmVid " << activeRMVID << std::endl;
      std::cout << "Adding composite robot " << activePassive->GetLabel() 
                << " gcfg " << compositeGcfg.PrettyPrint() 
                << ", grasping start object " << passive->GetLabel()
                << ": rmVid " << compositeRMVID << std::endl;
      auto activeGVID = gh->AddVertex(activeGv);
      auto compositeGVID = gh->AddVertex(compositeGv);
      m_modeTransitionGroundedVertices[activeVID].insert(activeGVID);
      m_modeTransitionGroundedVertices[activePassiveVID].insert(compositeGVID);
      std::cout << "ADD Active Ignition Transition GVID: " << activeVID << " --> " << activeGVID << std::endl;
      std::cout << "ADD Active Ignition Transition GVID: " << activePassiveVID << " --> " << compositeGVID << std::endl;
      
      auto robotBase = active->GetRobot(0)->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
      auto objectBase = passive->GetRobot(0)->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
      double distanceSquared = 0.0;
      for (size_t i = 0; i < 3; ++i) {
          double diff = robotBase[i] - objectBase[i];
          distanceSquared += diff * diff;
      }
      double distance = std::sqrt(distanceSquared);


      LazyModeGraph::Transition transition;
      transition.cost = distance;
      transition.action = std::make_pair(kv.second.property.first,false);
      std::cout << "Add start hyperarc with cost: " << distance << std::endl;

      auto newHID = m_modeTransitionHypergraph.AddHyperarc({newHead},{activeVID,passiveVID},transition);
      std::cout << "Assign action " << transition.action.first->GetLabel() << " to " << newHID << std::endl;
      std::cout << active->GetLabel() << " grasping " << passive->GetLabel() << " " << startObjectGcfgs[passive].PrettyPrint() << std::endl;


      std::set<RobotGroup*> tailGroup = {active,passive};
      std::set<RobotGroup*> headGroup = {activePassive};
      interactionCostInfo[std::make_pair(tailGroup,headGroup)][newHID] = distance;
      break;
    }
  }




  std::set<RobotGroup*> foundGoalInteraction;
  for(auto kv : m_modeHypergraph.GetHyperarcMap()) {
    auto tail = kv.second.tail;
    auto head = kv.second.head;
    

    RobotGroup* active = nullptr;
    RobotGroup* passive = nullptr;
    RobotGroup* activePassive = nullptr;
    VID activeVID;
    VID passiveVID;
    VID activePassiveVID;
    
    // TODO: need a better way to identify grasp interactions
    if(!(tail.size()==1 and head.size()==2)) 
      continue;

    for(size_t h : head) {
      auto group = m_modeHypergraph.GetVertex(h).property->robotGroup;
      if(group->GetRobot(0)->GetMultiBody()->IsPassive()) {
        passive = group;
        passiveVID = terminations[group];
      }
      else {
        active = group;
        activeVID = terminations[group];
      }
    }

    auto newTail = m_vidConversionMap[*(tail.begin())];
    activePassive = m_modeHypergraph.GetVertex(*(tail.begin())).property->robotGroup;
    activePassiveVID = m_vidConversionMap[m_modeHypergraph.GetVertex(*(tail.begin())).vid];

    // auto hyperarc = kv.second;
    // auto head = hyperarc.head;
    // auto tail = hyperarc.tail;


    auto interaction = dynamic_cast<Interaction*>(kv.second.property.first);
    auto activeGrm = this->GetMPSolution()->GetGroupRoadmap(active);
    auto compositeGrm = this->GetMPSolution()->GetGroupRoadmap(activePassive);
    GroupCfgType activeGcfg(activeGrm);
    GroupCfgType compositeGcfg(compositeGrm);
    // bool foundInteraction = false;
    std::cout << "Interaction Type: " << interaction->GetLabel() << std::endl;
    for(auto objectGcfg : goalObjectGcfgs[passive]) {
      // if(foundInteraction)
      //   break;
      std::cout << "Check: " << active->GetLabel() << " + " << passive->GetLabel() << " " << objectGcfg.PrettyPrint() << std::endl;
      for(size_t j = 0; j < m_maxAttempts; j++) {
        auto cfg = CanReach(interaction,active,objectGcfg);
        if(!cfg.GetRobot()) {
          for(auto& robot : problem->GetRobots()) {
            if(robot.get() != plan->GetCoordinator()->GetRobot()) {
              robot->SetVirtual(false);
            }
          }
          std::cout << "[Goal] " << active->GetLabel() << " cannot grasp " << passive->GetLabel() << " " << objectGcfg.PrettyPrint() << std::endl;
          continue;
        }
        std::cout << "[Goal] " << active->GetLabel() << " can grasp " << passive->GetLabel() << " " << objectGcfg.PrettyPrint() << std::endl;
        foundGoalInteraction.insert(passive);

        // Add start cfg to grounded hypergraph
        auto activeCfgCopy = cfg;
        auto passiveCfg = startObjectGcfgs[passive].GetRobotCfg(passive->GetRobot(0));
        activeGcfg.SetRobotCfg(active->GetRobot(0),std::move(cfg));
        compositeGcfg.SetRobotCfg(passive->GetRobot(0),std::move(passiveCfg));
        compositeGcfg.SetRobotCfg(active->GetRobot(0),std::move(activeCfgCopy));
        auto activeRMVID = activeGrm->AddVertex(activeGcfg);
        auto compositeRMVID = compositeGrm->AddVertex(compositeGcfg);
        GroundedVertex activeGv = std::make_pair(activeGrm,activeRMVID);
        GroundedVertex compositeGv = std::make_pair(compositeGrm,compositeRMVID);
        std::cout << "Adding Active robot " << active->GetLabel() 
                  << " gcfg " << activeGcfg.PrettyPrint() 
                  << ", grasping goal object " << passive->GetLabel()
                  << ": rmVid " << activeRMVID << std::endl;
        std::cout << "Adding composite robot " << active->GetLabel() 
                  << " gcfg " << compositeGcfg.PrettyPrint() 
                  << ", grasping goal object " << passive->GetLabel()
                  << ": rmVid " << compositeRMVID << std::endl;
        auto activeGVID = gh->AddVertex(activeGv);
        auto compositeGVID = gh->AddVertex(compositeGv);
        m_modeTransitionGroundedVertices[activeVID].insert(activeGVID);
        m_modeTransitionGroundedVertices[activePassiveVID].insert(compositeGVID);
        std::cout << "ADD Active Termination Transition GVID: " << activeVID << " --> " << activeGVID << std::endl;
        std::cout << "ADD Active Ignition Transition GVID: " << activePassiveVID << " --> " << compositeGVID << std::endl;


        auto objectGoalGCfg = objectGcfg.GetRobotCfg(passive->GetRobot(0));
        std::vector<double> objectGoalPose = {objectGoalGCfg[0], objectGoalGCfg[1], objectGoalGCfg[2]};
        auto robotBase = active->GetRobot(0)->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
        double distanceSquared = 0.0;
        for (size_t i = 0; i < 3; ++i) {
            double diff = objectGoalPose[i] - robotBase[i];
            distanceSquared += diff * diff;
        }
        double distance = std::sqrt(distanceSquared);
        
        
        LazyModeGraph::Transition transition;
        transition.cost = distance;
        transition.action = std::make_pair(kv.second.property.first,true);
        auto newHID = m_modeTransitionHypergraph.AddHyperarc({activeVID,passiveVID},{newTail},transition);
        m_terminationModeTransitionVIDs.insert(passiveVID);

        std::cout << "Add start hyperarc with cost: " << distance << std::endl;
        std::cout << "Assign action " << transition.action.first->GetLabel() << " to " << newHID << std::endl;


        std::set<RobotGroup*> tailGroup = {activePassive};
        std::set<RobotGroup*> headGroup = {active,passive};
        interactionCostInfo[std::make_pair(tailGroup,headGroup)][newHID] = distance;
        break;
      }
    }
  }



  std::set<RobotGroup*> invalidStart;
  for(auto v : m_unactuatedModes) {
    auto passiveGroup = m_modeHypergraph.GetVertex(v).property->robotGroup;
    if(!foundStartInteraction.count(passiveGroup)) {
      invalidStart.insert(passiveGroup);
      continue;
    }
    std::cout << "Object " << passiveGroup->GetLabel() << " in start position is graspable!" << std::endl;
  }


  std::set<RobotGroup*> invalidGoal;
  for(auto v : m_unactuatedModes) {
    auto passiveGroup = m_modeHypergraph.GetVertex(v).property->robotGroup;
    if(!foundGoalInteraction.count(passiveGroup)) {
      invalidGoal.insert(passiveGroup);
      continue;
    }
    std::cout << "Object " << passiveGroup->GetLabel() << " in goal position is graspable!" << std::endl;
  }

  if(invalidStart.size() > 0 or invalidGoal.size() > 0) {
    std::cout << "Invalid start" << std::endl;
    for(auto group : invalidStart) {
      std::cout << "\t" << group->GetLabel() << std::endl;
    }
    std::cout << "Invalid goal" << std::endl;
    for(auto group : invalidGoal) {
      std::cout << "\t" << group->GetLabel() << std::endl;
    }
    throw RunTimeException(WHERE) << "At least one robot should be assigned to grasp " << std::endl;
  }






  
  std::cout << "Identify inter-robot hids" << std::endl;
  for(auto kv : m_modeHypergraph.GetHyperarcMap()) {
    // auto hid = kv.first;
    auto tail = kv.second.tail;
    auto head = kv.second.head;
    std::cout << tail << " -->> " << head << std::endl;

    RobotGroup* activePassive = nullptr;
    RobotGroup* activeIgnition = nullptr;
    RobotGroup* postActivePassive = nullptr;
    RobotGroup* postActive = nullptr;
    VID activePassiveVID = MAX_UINT;
    VID activeIgnitionVID = MAX_UINT;
    VID postActivePassiveVID = MAX_UINT;
    VID postActiveVID = MAX_UINT;
    
    if(tail.size() < 2 or head.size() < 2) 
      continue;
      
    
    for(size_t t : tail) {
      auto group = m_modeHypergraph.GetVertex(t).property->robotGroup;
      if(group->GetRobots().size()==1) {
        activeIgnition = group;
        activeIgnitionVID = ignitions[group];
      }
      else if(group->GetRobots().size()==2) {
        activePassive = group;
        activePassiveVID = t;
      }
    }      
    
    for(size_t h : head) {
      auto group = m_modeHypergraph.GetVertex(h).property->robotGroup;
      if(group->GetRobots().size()==1) {
        postActive = group;
        postActiveVID = terminations[group];
      }
      else if(group->GetRobots().size()==2) {
        postActivePassive = group;
        postActivePassiveVID = h;
      }
    }      


    auto tailActiveBase = activeIgnition->GetRobot(0)->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
    auto headActiveBase = postActive->GetRobot(0)->GetMultiBody()->GetBase()->GetWorldTransformation().translation();
    double distanceSquared = 0.0;
    for (size_t i = 0; i < 3; ++i) {
        double diff = tailActiveBase[i] - headActiveBase[i];
        distanceSquared += diff * diff;
    }
    double distance = std::sqrt(distanceSquared);

    if(distance > m_robotBaseMaxDistance)
      continue;

    LazyModeGraph::Transition transition;
    transition.cost = distance;
    transition.action = std::make_pair(kv.second.property.first,false);
    std::cout << "Add cost " << distance << " to interaction hyperarc " << kv.first << " cost: " << distance << std::endl;
    auto newHID = m_modeTransitionHypergraph.AddHyperarc({postActivePassiveVID,postActiveVID},{activePassiveVID,activeIgnitionVID},transition);

    std::set<RobotGroup*> tailGroup = {activeIgnition,activePassive};
    std::set<RobotGroup*> headGroup = {postActivePassive,postActive};
    interactionCostInfo[std::make_pair(tailGroup,headGroup)][newHID] = distance;
  }


  LazyModeGraph::Transition transition;
  transition.cost = 0.;
  m_ignitionMTHID = m_modeTransitionHypergraph.AddHyperarc(m_ignitionModeTransitionVIDs,{m_sourceModeTransitionVID},transition);
  std::cout << "Add hyperarc that connects the source to ignition robots and objects: mt hid " << m_ignitionMTHID << std::endl;
  transition.cost = 0.;
  m_terminationMTHID = m_modeTransitionHypergraph.AddHyperarc({m_sinkModeTransitionVID},m_terminationModeTransitionVIDs,transition);
  std::cout << "Add hyperarc that connects termination robots and objects to the sink: mt hid " << m_terminationMTHID << std::endl;
  
  // std::cout << "GVID Map " << std::endl;
  // for(auto kv : m_modeTransitionGroundedVertices) {
  //   if(kv.first == m_sinkModeTransitionVID or kv.first == m_sourceModeTransitionVID) 
  //     continue;
  //   std::cout << kv.first << "(" << m_modeTransitionHypergraph.GetVertex(kv.first).property->robotGroup->GetLabel() << ")" << ": " << std::endl;
  //   for(auto v : kv.second) {
  //     auto vertex = gh->GetVertex(v);
  //     auto gvid = vertex.second;
  //     auto grm = vertex.first;
  //     if(!grm) {
  //       std::cout << v << "..." << std::endl;
  //       continue;
  //     }
  //     std::cout << v << " " << grm->GetVertex(gvid).PrettyPrint() << std::endl;
  //   }
  //   std::cout << std::endl;
  // }
  // for(auto kv : m_modeTransitionHypergraph.GetHyperarcMap()) {
  //   if(kv.second.property.action.first)
  //     std::cout << kv.first << ": " << kv.second.property.action.first->GetLabel() << " " << kv.second.property.action.second << std::endl;
  // }

}


void
LazyModeGraph::
SampleCfgs(const State& _start, std::unordered_map<RobotGroup*,VID>& _startRobotVIDs) {

  auto plan = this->GetPlan();
  auto c = plan->GetCoordinator();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SampleCfgs");

  auto lib = this->GetMPLibrary();
  lib->SetMPSolution(this->GetMPSolution());

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());

  // Set all robots to virtual
  for(auto pair : c->GetInitialRobotGroups()) {
    auto group = pair.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(true);
    }
  }

  for(auto& kv : m_modeHypergraph.GetVertexMap()) {
    // Check if mode is actuated
    auto mode = kv.second.property;
    auto group = mode->robotGroup;
    bool active = false;
    for(auto robot : group->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive()) {
        active = true;
        break;
      }
    }
    if(!active or group->GetRobots().size()!=1) {
      continue;
    }

    auto grm = this->GetMPSolution()->GetGroupRoadmap(mode->robotGroup);

    // Check if mode is in the start state
    auto iter = _start.find(mode->robotGroup);

    // If mode is initial mode, add starting vertex
    if(iter != _start.end()) {
      // Add start cfg to grounded hypergraph
      auto state = _start.at(mode->robotGroup);
      auto gcfg = state.first->GetVertex(state.second);
      gcfg.SetGroupRoadmap(grm);
      auto vid = grm->AddVertex(gcfg);
      GroundedVertex gv = std::make_pair(grm,vid);
      // std::cout << "adding active robot " << mode->robotGroup->GetLabel() 
      //           << " gcfg " << gcfg.PrettyPrint() << ": rmVid " << vid << std::endl;
      auto groundedVID = gh->AddVertex(gv);
      // std::cout << "GVID: " << groundedVID << std::endl;

      m_modeGroundedVertices[kv.first].insert(groundedVID);
      m_startGroundedVIDs.insert(groundedVID);
      m_entryVertices.insert(groundedVID);
      _startRobotVIDs[group] = groundedVID;
    }
  }
  for(auto pair : c->GetInitialRobotGroups()) {
    auto group = pair.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(false);
    }
  }
}



bool
LazyModeGraph::
SampleHyperpath() {
  return false;
}

std::set<size_t>
LazyModeGraph::
GetGoalGroundedVIDs() {
  return m_goalGroundedVIDs;
}

void
LazyModeGraph::
ConnectSink(std::set<VID> goalGroundedVIDs) {
  m_goalGroundedVIDs = goalGroundedVIDs;
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  // std::cout << "goalGroundedVIDs: " ;
  // for(auto vid : goalGroundedVIDs) {
  //   std::cout << vid << " " ;
  // }
  std::cout << std::endl;
  if(gh->GetHID(goalGroundedVIDs,{m_sinkGroundedVID}) == MAX_UINT){
    GroundedHypergraph::Transition fromOrigin;
    fromOrigin.cost = -1;
    // std::cout << "Adding sink transition to grounded hypergraph" << std::endl;
    size_t hid = gh->AddTransition(goalGroundedVIDs,{m_sinkGroundedVID},fromOrigin);
    m_motionHistory.push_back(hid);
  }
}

void
LazyModeGraph::
ConnectSource() {
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  // std::cout << "m_startGroundedVIDs: " ;
  // for(auto vid : m_startGroundedVIDs) {
  //   std::cout << vid << " " ;
  // }
  // std::cout << std::endl;
  if(gh->GetHID({m_sourceGroundedVID},m_startGroundedVIDs) == MAX_UINT){
    GroundedHypergraph::Transition fromOrigin;
    // std::cout << "Adding start transition to grounded hypergraph" << std::endl;
    gh->AddTransition({m_sourceGroundedVID},m_startGroundedVIDs,fromOrigin);
  }
}


void
LazyModeGraph::
GenerateModeHypergraph(const std::vector<VID>& _initialModes) {

  if(m_complete)
    return;

  auto as = this->GetTMPLibrary()->GetActionSpace();

  auto newModes = _initialModes;

  // Keep track of already expanded mode/action combinations
  std::unordered_map<Action*,std::set<std::vector<VID>>> appliedActions;

  do {
    
    // Add new modes to the hyerpgraph
    for(auto vid : newModes) {
      m_modeGroundedVertices[vid] = std::unordered_set<VID>();
    }

    // Clear added modes
    newModes.clear();

    // Apply actions to discovered modes to make new modes
    for(auto actionLabel : as->GetActions()) {
      if(m_debug)
        std::cout << actionLabel << std::endl;
      auto action = actionLabel.second;
      ApplyAction(action,appliedActions[action],newModes);
      if(action->IsReversible()) {
        ApplyAction(action,appliedActions[action],newModes,false);
      }
    }

  } while(!newModes.empty());

  // TODO::Add other options, but current implementation builds entire task space hypergraph
  bool exhaustive = true;
  if(exhaustive)
    m_complete = true;
}

void
LazyModeGraph::
SampleNonActuatedCfgs(const State& _start) {

  auto plan = this->GetPlan();
  auto decomp = plan->GetDecomposition();
  auto lib = this->GetMPLibrary();
  auto uaSM = lib->GetSampler(m_unactuatedSM);
  auto qSM = lib->GetSampler(m_querySM);
  lib->SetMPSolution(this->GetMPSolution());
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());

  // Set all robots and objects to virtual
  auto c = this->GetPlan()->GetCoordinator();
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(true);
    }
  }
 
  for(auto& kv : m_modeHypergraph.GetVertexMap()) {
    // Check if mode is unactuated
    auto mode = kv.second.property;

    bool unactuated = true;
    for(auto robot : mode->robotGroup->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive())
        continue;
      unactuated = false;
      break;
    }

    if(!unactuated)
      continue;

    // TODO
    auto c = this->GetPlan()->GetCoordinator();
    for(auto& kv : c->GetInitialRobotGroups()) {
      auto group = kv.first;
      for(auto r : group->GetRobots()) {
        r->SetVirtual(true);
      }
    }
    if(m_monotonic) {
      for(auto robot : mode->robotGroup->GetRobots()) {
        robot->SetVirtual(false);
      }
    }

    m_unactuatedModes.insert(kv.first);
    std::cout << "Insert unactuated mode: " << kv.first << " " << kv.second.property->robotGroup->GetLabel() << std::endl;

    // Ensure robot group is in the mp solution
    this->GetMPSolution()->AddRobotGroup(mode->robotGroup);
    auto grm = this->GetMPSolution()->GetGroupRoadmap(mode->robotGroup);

    // Configure MPLibrary
    GroupTask gt(mode->robotGroup);
    lib->SetTask(nullptr);
    lib->SetGroupTask(&gt);

    // Check if mode is in the start state
    auto iter = _start.find(mode->robotGroup);
    if(m_debug)
      std::cout << "\n Start sampling " << std::endl;
    if(iter != _start.end()) {

      // Add start cfg to grounded hypergraph
      auto state = _start.at(mode->robotGroup);
      auto gcfg = state.first->GetVertex(state.second);
      if(m_debug)
        std::cout << gcfg.PrettyPrint() << std::endl;
      auto vid = grm->AddVertex(gcfg);
      GroundedVertex gv = std::make_pair(grm,vid);

      auto groundedVID = gh->AddVertex(gv);
      if(m_debug)
        std::cout << "Adding Start Local VID: " << vid << "| MVID: " << kv.first << "-->>" << " groundedVID: " << groundedVID << ": " << gcfg.PrettyPrint() << std::endl;
      m_modeGroundedVertices[kv.first].insert(groundedVID);
      m_modeStartGroundedVIDs[kv.first].insert(groundedVID);
      m_startGroundedVIDs.insert(groundedVID);

      m_entryVertices.insert(groundedVID);
    }

    // Check if mode is in the goal conditions
    for(auto st : decomp->GetGroupMotionTasks()) {
      auto task = st->GetGroupMotionTask().get();
      if(!task or task->GetRobotGroup() != mode->robotGroup)
        continue;

      // Sample goal cfg
      std::map<Robot*, const Boundary*> boundaryMap;
      for(auto iter = task->begin(); iter != task->end(); iter++) {
        auto c = dynamic_cast<BoundaryConstraint*>(iter->GetGoalConstraints()[0].get());
        auto b = c->GetBoundary();
        boundaryMap[iter->GetRobot()] = b;
      }

      std::vector<GroupCfgType> samples;
      qSM->Sample(1,m_maxAttempts,boundaryMap,std::back_inserter(samples));
      
      if(samples.size() == 0)
        throw RunTimeException(WHERE) << "Unable to generate goal configuration for "
                                      << mode->robotGroup->GetLabel()
                                      << "..";
      // Add goal cfg to grounded hypergraph
      //auto gcfg = samples[0].SetGroupRoadmap(grm);
      auto gcfg = samples[0];
      gcfg.SetGroupRoadmap(grm);
      auto vid = grm->AddVertex(gcfg);
      GroundedVertex gv = std::make_pair(grm,vid);

      auto groundedVID = gh->AddVertex(gv);
      if(m_debug)
        std::cout << "Adding Goal Local VID: " << vid << "| MVID: " << kv.first << "-->>" << " groundedVID: " << groundedVID << ": " << gcfg.PrettyPrint() << std::endl;
      m_modeGroundedVertices[kv.first].insert(groundedVID);
      m_modeGoalGroundedVertices[kv.first].insert(groundedVID);
      m_goalGroundedVIDs.insert(groundedVID);
      m_exitVertices.insert(groundedVID);
      m_goalVertexTaskMap[st] = groundedVID;
    }

    if(m_numUnactuatedSamples){
      std::map<Robot*, const Boundary*> boundaryMap;
      std::vector<GroupCfgType> samples;
      if(m_debug)
        std::cout << "sampling non actuated robot Group: " << mode->robotGroup->GetLabel() << std::endl; 

      for(auto robot : mode->robotGroup->GetRobots()) {
        const auto& surfaces = this->GetMPProblem()->GetEnvironment()->GetTerrains().at(robot->GetCapability());
        std::cout << "  surface size: " << surfaces.size() << std::endl;
        size_t index = LRand() % surfaces.size();
        std::cout << "  idx: " << index << std::endl;
        const auto& surface =  surfaces[index];
        
        const auto& boundaries = surface.GetBoundaries();
        std::cout << "  boundary size: " << boundaries.size() << std::endl;
        for (index = 0 ; index < boundaries.size() ; index++) {
          auto boundary = boundaries[index].get();
          boundaryMap[robot] = boundary;
          uaSM->Sample(m_numUnactuatedSamples,m_maxAttempts,boundaryMap,std::back_inserter(samples));
        }
      }

      std::cout << "  sample size: " << samples.size() << std::endl;
      for(auto sample : samples) { 
        // Add sample to grounded hypergraph
        auto gcfg = sample;
        if (m_debug) 
          std::cout << "unactuated sample: " << gcfg.PrettyPrint() << std::endl;
        gcfg.SetGroupRoadmap(grm);
        auto vid = grm->AddVertex(gcfg);
        GroundedVertex gv = std::make_pair(grm,vid);
  
        auto groundedVID = gh->AddVertex(gv);
        if(m_debug)
          std::cout << "Placement (MVID/GVID): (" << kv.first << " / " << groundedVID << ")" << std::endl;
        m_modeGroundedVertices[kv.first].insert(groundedVID);
        m_entryVertices.insert(groundedVID);
        m_exitVertices.insert(groundedVID);
      }
    }
  }

  
  // Set all robots and objects to virtual
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(false);
    }
  }
}

void
LazyModeGraph::
ConfigureGoalSets(const size_t& _sink, std::set<VID>& _goalVIDs) {
 
  // TODO::Find sets of valid goals to solve decomposition
  std::vector<std::set<VID>> goalSets;
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());

  // TODO::Compile sets of completed tasks that satisfy the decomposition
  // Currently assume one depth of OR values 
  std::set<SemanticTask*> seen;
  std::vector<std::set<SemanticTask*>> buckets;
  for(auto kv : m_goalVertexTaskMap) {
    auto st = kv.first;

    if(seen.count(st))
      continue;

    seen.insert(st);
    std::set<SemanticTask*> bucket = {st};

    auto parent = st->GetParent();
    auto type = parent->GetSubtaskRelation();
    
    if(type == SemanticTask::SubtaskRelation::XOR) {
      for(auto sibling : parent->GetSubtasks()) {
        bucket.insert(sibling);
        seen.insert(sibling);
      }
    }

    buckets.push_back(bucket);
  }
 
  auto taskSets = BuildTaskSets({},0,buckets);

  for(auto set : taskSets) {
    std::set<size_t> goalSet;
    for(auto st : set) {
      auto vid = m_goalVertexTaskMap[st];
      goalSet.insert(vid);
    }
    goalSets.push_back(goalSet);
  }

  // Add transition from each set to the sink
  for(auto set : goalSets) {
    GroundedHypergraph::Transition toGoal;
    if(gh->GetHID(set,{_sink}) == MAX_UINT) {
      gh->AddTransition(set,{_sink},toGoal,false,false);
    }
  } 
}

std::vector<std::set<SemanticTask*>>
LazyModeGraph::
BuildTaskSets(std::set<SemanticTask*> _taskSet, size_t _index, 
              const std::vector<std::set<SemanticTask*>>& _buckets) {
  
  if(_index == _buckets.size()) {
    if(m_debug) {
      std::cout << "Found Task Set:" << std::endl;
      for(auto st : _taskSet) {
        std::cout << "\t" << st->GetLabel();
      }
      std::cout << std::endl;
    }
    return {_taskSet};
  }

  auto bucket = _buckets[_index];

  std::vector<std::set<SemanticTask*>> newSets;

  for(auto st : bucket) {
    auto taskSet = _taskSet;
    taskSet.insert(st);
    auto sets = BuildTaskSets(taskSet,_index+1,_buckets);
    for(auto set : sets) {
      newSets.push_back(set);
    }
  }

  return newSets;
}

void
LazyModeGraph::
SetRelevantMTHIDVector(std::vector<HID> _relevantMTHIDs) {
  m_relevantMTHIDVector = _relevantMTHIDs;
}

std::vector<size_t>
LazyModeGraph::
GetRelevantMTHIDVector() {
  return m_relevantMTHIDVector;
}

void
LazyModeGraph::
SetRelevantMTVIDVector(std::vector<VID> _relevantMTVIDs) {
  m_relevantMTVIDVector = _relevantMTVIDs;
}

void
LazyModeGraph::
SetRelevantMTHIDs(std::set<HID> _relevantMTHIDs) {
  m_relevantMTHIDs = _relevantMTHIDs;
}

void
LazyModeGraph::
SetRelevantMTVIDs(std::set<VID> _relevantMTVIDs) {
  m_relevantMTVIDs = _relevantMTVIDs;
}

void
LazyModeGraph::
SetActiveTaskPlan(std::unordered_map<VID,std::set<std::pair<HID,HID>>> _activeTaskPlan) {
  m_activeTaskPlan = _activeTaskPlan;
}

std::unordered_map<size_t,std::set<std::pair<std::set<size_t>,std::set<size_t>>>>
LazyModeGraph::
GetGroundedTransitionMap() {
  return m_groundedTransitionMap;
}


void
LazyModeGraph::
InitializeTransitions() {

  std::set<VID> source = {GetSourceModeTransitionVID()};
  std::set<VID> sink = {GetSinkModeTransitionVID()};
  
  if(m_debug) {
    std::cout << "relevant MTHIDs: " << m_relevantMTHIDs << std::endl;
    std::cout << "relevant mt hids are: " ;
    
    for(auto h : m_relevantMTHIDs) {
      std::cout << h << " ";
    }
    std::cout << std::endl;
    std::cout << "mode hypergraph" << std::endl;
    m_modeTransitionHypergraph.Print();

    std::cout << "assigned actions: " << std::endl;
    for(auto kv : m_modeTransitionHypergraph.GetHyperarcMap()) {
      auto hid = kv.first;
      if(hid==m_sinkModeTransitionVID or hid==m_sourceModeTransitionVID)
        continue;
      auto ha = kv.second;
      std::cout << hid << ": " << ha.property.action.first << " " << ha.property.action.second << std::endl;
    }
  }
  

  std::pair<std::set<VID>,std::set<VID>> startPair;
  startPair.first = {0};
  for(auto kv : m_startRobotGroundedVIDs) {
    startPair.second.insert(kv.second);
  }
  for(auto gvid : m_startGroundedVIDs) {
    startPair.second.insert(gvid);
  }
  m_groundedTransitionMap[m_ignitionMTHID].insert(startPair);

  std::pair<std::set<VID>,std::set<VID>> goalPair;
  goalPair.second = {1};
  for(auto gvid : m_goalGroundedVIDs) {
    goalPair.first.insert(gvid);
  }
  m_groundedTransitionMap[m_terminationMTHID].insert(goalPair);

  // Set all robots and objects to virtual
  auto c = this->GetPlan()->GetCoordinator();
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(true);
    }
  }

  std::cout << "Add start robot & transitions" << std::endl;
  for(auto kv : m_modeTransitionHypergraph.GetVertexMap()) {
    // Add start robots to the starting transition
    if(kv.first == m_sourceModeTransitionVID) {
      auto mthid = *(m_modeTransitionHypergraph.GetOutgoingHyperarcs(kv.first).begin());
      auto head = m_modeTransitionHypergraph.GetHyperarc(mthid).head;
      for(auto h : head) {
        auto group = m_modeTransitionHypergraph.GetVertex(h).property->robotGroup;
        bool active = false;
        for(auto robot : group->GetRobots()) {
          if(!robot->GetMultiBody()->IsPassive()) {
            active = true;
            break;
          }
        }
        if(!active)
          continue;
        std::pair<std::set<VID>,std::set<VID>> pair;
        pair.first = {m_startRobotGroundedVIDs[group]};
        pair.second = {m_startRobotGroundedVIDs[group]};
        std::cout << mthid << ": " << m_startRobotGroundedVIDs[group] << std::endl;
        m_groundedTransitionMap[mthid].insert(pair);
      }
      continue;
    }
  }
}


void
LazyModeGraph::
SampleTransitions(std::set<HID> _targetHIDs) {
  std::cout << "Start sample transition" << std::endl;
  
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GenerateMotionHypergraph::SampleTransitions");

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  auto c = this->GetPlan()->GetCoordinator();

  // For each edge in the mode graph, generate n samples
  std::set<HID> used;
  int interactionCnt = 0;
  std::set<HID> failedInteractions;
  std::cout << "target hids: " << _targetHIDs << std::endl;
  std::cout << "relevant mt hids: " << m_relevantMTHIDs << std::endl;
  std::cout << "Setting robots virtual" << std::endl;
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(true);
      std::cout << "setting " << r->GetLabel() << " virtual " << std::endl;
    }
  }
  for(auto& kv : m_modeTransitionHypergraph.GetHyperarcMap()) {
    if(!_targetHIDs.empty() and !_targetHIDs.count(kv.first)) {
      continue;
    }

    auto hid = kv.first;
    auto& hyperarc = kv.second;


    if(m_computedInteraction.count(hid)) {
      std::cout << "Computed interaction: " << hid << std::endl;
      continue;
    }

    if(!m_relevantMTHIDs.count(hid)) {
      std::cout << "hid " << hid << " is not relevant to the current task plan" << std::endl;
      continue;
    }    
    if(used.count(hid)) {
      std::cout << "hid " << hid << " already computed" << std::endl;
      continue;
    }
    used.insert(hid);
    if(m_debug)
      std::cout << "Computing interaction of mt hid " << hid << ": " << hyperarc.tail << " --> " << hyperarc.head << std::endl;
    if(m_groundedInstanceTracker.find(kv.first) == m_groundedInstanceTracker.end()) {
      m_groundedInstanceTracker[kv.first] = 0;
    }

    std::cout << hyperarc.property.action.first << " " << hyperarc.property.action.second << std::endl;
    if(!hyperarc.property.action.first) {
      if(m_debug)
        std::cout << "action not assigned" << std::endl;
      continue;
    }
    else {
      if(m_debug)
        std::cout << "Assigned action: " << hyperarc.property.action.first->GetLabel() << std::endl;
    }
    
    // // Check if hyperarc is a reversed action, and only plan
    // // the forward actions as the reverse will also be saved
    // if(hyperarc.property.action.second) {
    //   std::cout << "Skip reversed action" << std::endl;
    //   continue;
    // }
    if(m_debug)
      std::cout << "=========================================================================" << std::endl;
    auto hyperarcTail = hyperarc.tail;
    auto hyperarcHead = hyperarc.head;
    if(hyperarc.property.action.second) {
      hyperarcTail = hyperarc.head;
      hyperarcHead = hyperarc.tail;
    }
    auto interaction = dynamic_cast<Interaction*>(hyperarc.property.action.first);

    State modeSet;
    std::unordered_map<RobotGroup*,Mode*> tailModeMap;
    std::unordered_map<RobotGroup*,Mode*> headModeMap;
    std::set<std::pair<size_t,RobotGroup*>> unactuatedModes;


    for(auto vid : hyperarcTail) {
      auto mode = m_modeTransitionHypergraph.GetVertex(vid).property;
      modeSet[mode->robotGroup] = std::make_pair(nullptr,MAX_UINT);
      tailModeMap[mode->robotGroup] = mode;
      if(m_debug)
        std::cout << "Tail MVID: " << vid << " / " << mode->robotGroup->GetLabel() << std::endl;
      if(m_unactuatedMTVIDs.count(vid)) {
        unactuatedModes.insert(std::make_pair(vid,mode->robotGroup));
      }
    }

    for(auto vid : hyperarcHead) {
      auto mode = m_modeTransitionHypergraph.GetVertex(vid).property;
      headModeMap[mode->robotGroup] = mode;
      if(m_debug)
        std::cout << "Head MVID: " << vid << " / " << mode->robotGroup->GetLabel() << std::endl;
    }

    auto label = interaction->GetInteractionStrategyLabel();
    auto is = this->GetInteractionStrategyMethod(label);

    // // Set robots involved as non virtual
    // for(auto kv : modeSet) {
    //   auto group = kv.first;
    //   for(auto r : group->GetRobots()) {
    //     r->SetVirtual(false);
    //   }
    // }

    bool foundInteraction = false;
    bool canReach = false;
    // If this hyperarc involves an unactuated mode, use the grounded vertices
    if(!unactuatedModes.empty()) {
      if(m_debug)
        std::cout << "unactuatedModes is not empty: " << std::endl;
      if(unactuatedModes.size() > 1)
        throw RunTimeException(WHERE) << "Multiple unactuated modes in an "
                      "interaction not currently supported.";

      auto unactuatedVID = unactuatedModes.begin()->first;
      auto unactuatedGroup = unactuatedModes.begin()->second;

      auto groundedVertices = m_modeTransitionGroundedVertices[unactuatedVID];
        
      if(m_debug) {
        std::cout << "Unactuated MVID: " << unactuatedVID << std::endl;
        std::cout << "with GVID: " ;
        for (auto iter = groundedVertices.begin(); iter !=groundedVertices.end(); iter++) {
          std::cout << *iter << " " ;
        }
        std::cout << " (size: " << groundedVertices.size() << ")" << std::endl;
      }

      //for(auto gv : m_modeGroundedVertices[unactuatedVID]) {
      for(auto iter = groundedVertices.begin(); iter != groundedVertices.end(); iter++) {
        if(m_debug)
          std::cout << "===>>> Unactuated GVID: " << *iter << " " << gh->GetVertex(*iter).first->GetVertex(gh->GetVertex(*iter).second).PrettyPrint() << std::endl;
        auto groundedVertex = gh->GetVertex(*iter);

        auto startSet = modeSet;
        startSet[unactuatedGroup] = groundedVertex;

        canReach = true;

        for(size_t j = 0; j < m_maxAttempts; j++) {
          // Make state copy and add grounded vertex to pass to IS.
          // Will get overwritten as goal state
          auto goalSet = startSet;
          if(!is->operator()(interaction,goalSet)) {
            if(m_debug)
              std::cout << "Failed to find interaction" << std::endl;
            continue;
          }

          foundInteraction = true;
          interactionCnt += 1;
          if(m_debug)
            std::cout << "Found an interaction" << std::endl;
          m_groundedInstanceTracker[kv.first] = m_groundedInstanceTracker[kv.first] + 1;
          // Save interaction paths
          SaveInteractionPaths(hid,interaction,modeSet,goalSet,tailModeMap,headModeMap);
          break;
        }
      }
    }
    // Otherwise, sample completely new grounded vertices
    else {
      std::cout << "Sampling new vertices" << std::endl;
      for(size_t i = 0; i < m_numInteractionSamples; i++) {
        for(size_t j = 0; j < m_maxAttempts; j++) {
          // Make state copy to pass by ref and get output state
          auto goalSet = modeSet;
          canReach = true;

          if(!is->operator()(interaction,goalSet)) {
            std::cout << "Failed to find interaction" << std::endl;
            continue;
          }

          foundInteraction = true;
          interactionCnt += 1;
          if(m_debug)
            std::cout << "Found an interaction " << std::endl;

          m_groundedInstanceTracker[kv.first] = m_groundedInstanceTracker[kv.first] + 1;

          // Save interaction paths
          SaveInteractionPaths(hid,interaction,modeSet,goalSet,tailModeMap,headModeMap); // interaction, start, goal
          break;
        }
      }
    }
    
    // Set robots involved as back to virtual
    for(auto kv : modeSet) {
      auto group = kv.first;
      for(auto r : group->GetRobots()) {
        r->SetVirtual(true);
      }
    }

    if(!foundInteraction and canReach) {
      failedInteractions.insert(hid);
      if(m_debug) {
        std::cout << "Failed to find interaction for " << interaction->GetLabel()
                  << " with starting mode";
        for(auto kv : modeSet) {
          auto group = kv.first;
          std::cout << " " << group->GetLabel();
        }
        std::cout << std::endl;
      }
    }

    if(foundInteraction)
      m_computedInteraction.insert(kv.first);
  }

  std::cout << "interactionCnt: " << interactionCnt << std::endl;
  // Set all robots and objects to back to non-virtual
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(false);
    }
  }



  std::cout << "Transitions GVIDs " << std::endl;
  for(auto kv : m_groundedTransitionMap) {
    std::cout << kv.first << ": " ;
    for(auto gvids : kv.second) {
      std::cout << gvids << ", " ;
    } 
    std::cout << std::endl;
  }
  
  for(auto kv : m_modeTransitionGroundedVertices) {
    if(kv.first == m_sinkModeTransitionVID or kv.first == m_sourceModeTransitionVID) 
      continue;
    std::cout << kv.first << "(" << m_modeTransitionHypergraph.GetVertex(kv.first).property->robotGroup->GetLabel() << ")" << ": " << std::endl;

    auto group = m_modeTransitionHypergraph.GetVertex(kv.first).property->robotGroup;
    if(group->GetRobots().size()>1)
      continue;
    
    if(group->GetRobots().size()==1 and group->GetRobot(0)->GetMultiBody()->IsPassive())
      continue;

    size_t mthid = MAX_UINT;
    for(auto kv2 : m_modeTransitionHypergraph.GetHyperarcMap()) {
      if(!(kv2.second.tail.size()==1 and kv2.second.head.size()==1))
        continue;
      if(group == m_modeTransitionHypergraph.GetVertex(*(kv2.second.tail.begin())).property->robotGroup)
        mthid = kv2.first;
    }

    if(mthid == MAX_UINT)
      continue;

    for(auto v : kv.second) {
      std::pair<std::set<VID>,std::set<VID>> pair;
      pair.first.insert(v);
      pair.second.insert(v);
      m_groundedTransitionMap[mthid].insert(pair);
    }
  }


  

  std::cout << "Failed Interactions: " << failedInteractions << std::endl;
  if(failedInteractions.size() > 0)  {
    for(auto hid : failedInteractions)
      m_interactionConstraintSet.insert(hid);
    // throw RunTimeException(WHERE) << "No possible motion for interactions " << failedInteractions ;
  }


  std::cout << "Transitions GVIDs " << std::endl;
  for(auto kv : m_groundedTransitionMap) {
    std::cout << kv.first << ": " ;
    for(auto gvids : kv.second) {
      std::cout << gvids << ", " ;
    } 
    std::cout << std::endl;
  }

  std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
}







void
LazyModeGraph::
GenerateRoadmaps() {
  // Transition for sink will be added after finding the possible connection between goal and origin (0)
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  std::cout << "Generate Roadmaps" << std::endl;
  std::cout << "Current gh: " << std::endl;
  gh->Print();

  auto plan = this->GetPlan();
  auto c = plan->GetCoordinator();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GenerateMotionHypergraph::GenerateRoadmaps");

  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
  lib->SetMPSolution(this->GetMPSolution());

  std::map<Robot*,Robot*> plannedRoadmapMap;

  // Set all robots to virtual
  for(auto pair : c->GetInitialRobotGroups()) {
    auto group = pair.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(true);
    }
  }

  if(m_debug)
    std::cout << "\nNow We Generate Roadmaps" << std::endl;
  std::cout << "Relevant vids are: " ;
  for(auto vid : m_relevantMTVIDs) {
    std::cout << vid << " " ;
  }
  std::cout << std::endl;
  // For each actuated mode in the mode hypergraph, run the expansion strategy
  std::set<RobotGroup*> used;
  int cnt = 0;
  int actualCnt = 0;
  for(auto kv : m_modeTransitionHypergraph.GetVertexMap()) {
    cnt++;
    auto vertex = kv.second;
    auto mode = vertex.property;
    if(kv.first == m_sourceModeTransitionVID or kv.first == m_sinkModeTransitionVID)
      continue;
    std::cout << "Generating roadmap for " << mode->robotGroup->GetLabel() << "(" << vertex.vid << ")" << std::endl; 

    if(!m_relevantMTVIDs.count(vertex.vid)) {
      std::cout << "It is not relevant vid. Skip generating roadmap" << std::endl;
      continue;
    }
    std::cout << "It is relevant vid. Generating roadmap" << std::endl;
    if(used.count(mode->robotGroup)) {
      std::cout << "Already used robot: " << mode->robotGroup->GetLabel() << std::endl;
      continue;
    }

    // if(m_computedRoadmap.count(mode->robotGroup)) {
    //   std::cout << "dddddddddddddddddddddd" << std::endl;
      
    //   continue;
    // }
    used.insert(mode->robotGroup);
    //bool actuated = false;
    Robot* actuated = nullptr;
    Robot* passive = nullptr;

    for(auto robot : mode->robotGroup->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive()) {
        actuated = robot;
      }
      else {
        passive = robot;
      }
    }

    if(!actuated){
      std::cout << "Not actuated. continue." << std::endl;
      continue;
    }

    // Check if this robot already has a planned object roadmap
    if(mode->robotGroup->Size() > 1 and plannedRoadmapMap.find(actuated) != plannedRoadmapMap.end()) {
      auto repObject = plannedRoadmapMap[actuated];
      std::vector<Robot*> oldGroup = {actuated,plannedRoadmapMap[actuated]};
      std::string label = oldGroup[0]->GetLabel() + "::" + oldGroup[1]->GetLabel();
      auto repGroup = this->GetMPProblem()->AddRobotGroup(oldGroup,label);
      auto sol = this->GetPlan()->GetMPSolution();
      auto repRm = sol->GetGroupRoadmap(repGroup);
      auto rm = sol->GetGroupRoadmap(mode->robotGroup);

      std::map<size_t,size_t> vidMap;
      for(auto vit = repRm->begin(); vit != repRm->end(); vit++) {
        auto vid = vit->descriptor();
        auto gcfg = vit->property();
        GroupCfgType newGcfg(rm);
        Cfg cfg(passive);
        cfg.SetData(gcfg.GetRobotCfg(repObject).GetData());
        newGcfg.SetRobotCfg(passive,std::move(cfg));
        cfg = gcfg.GetRobotCfg(actuated);
        newGcfg.SetRobotCfg(actuated,std::move(cfg));
        auto newVID = rm->AddVertex(newGcfg);
        vidMap[vid] = newVID;
      }

      for(auto vit = repRm->begin(); vit != repRm->end(); vit++) {
        for(auto eit = vit->begin(); eit != vit->end(); eit++) {
          auto source = vidMap[eit->source()];
          auto target = vidMap[eit->target()];
          auto edge = eit->property();
          GroupLocalPlanType newEdge(rm,edge.GetLPLabel(),edge.GetWeight());
          newEdge.SetTimeSteps(edge.GetTimeSteps());
          rm->AddEdge(source,target,newEdge);
        }
      }
      std::cout << "Already has a planned object roadmap for " << mode->robotGroup->GetLabel() << std::endl;
      continue;
    }

    actualCnt++;
    std::cout << "counter: " << actualCnt << "/" << cnt << std::endl;

    MethodTimer* mt3 = new MethodTimer(stats,this->GetNameAndLabel() + "::GenerateMotionHypergraph::GenerateRoadmaps_" + mode->robotGroup->GetLabel());

    // Initialize dummy task
    std::cout << mode->robotGroup->GetLabel() << std::endl;
    auto task = new GroupTask(mode->robotGroup);

    for(auto r : mode->robotGroup->GetRobots()) {
      auto t = MPTask(r);

      // Add start constraints
      auto startCfg = prob->GetInitialCfg(r);
      std::cout << "start Cfg of robot " << r->GetLabel() << ": " << startCfg.PrettyPrint() << std::endl;
      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
      t.SetStartConstraint(std::move(startConstraint));
      std::cout << kv.first << std::endl;
      std::cout << "mode constraints size: " << mode->constraints.size() << std::endl;

      // Add mode/path constraints
      for(const auto& c : mode->constraints) {
        std::cout << "constraint label: " << c->GetLabel() << std::endl;
        std::cout << "involved robot: " << c->GetRobot()->GetLabel() << std::endl;
        if(c->GetRobot() != r){
          continue;
        }
        std::cout << "Addig path constraint for " << c->GetRobot()->GetLabel() << std::endl;
        t.AddPathConstraint(std::move(c->Clone()));
      }
      task->AddTask(t);
    } 

    // Set active formation constraints
    auto formations = mode->formations;
    auto grm = this->GetMPSolution()->GetGroupRoadmap(mode->robotGroup);
    grm->SetAllFormationsInactive();
    for(auto f : formations) {
      grm->SetFormationActive(f);
    }
    std::cout << "grm pre size: " << grm->Size() << std::endl;

    // Set robots not virtual
    for(auto robot : grm->GetGroup()->GetRobots()) {
      robot->SetVirtual(false);
    }



    // Call the MPLibrary solve function to expand the roadmap
    lib->SetPreserveHooks(true);
    lib->Solve(prob,task,this->GetMPSolution(),m_expansionStrategy, LRand(), 
            "ExpandModeRoadmap");
    lib->SetPreserveHooks(false);
    std::cout << "grm post size: " << grm->Size() << std::endl;


    m_computedRoadmap.insert(mode->robotGroup);
    delete mt3;

    // Set robots not virtual
    for(auto robot : grm->GetGroup()->GetRobots()) {
      robot->SetVirtual(true);
    }

    delete task;

    if(actuated and passive) {
      plannedRoadmapMap[actuated] = passive;
    }

    // this->GetMPSolution()->GetGroupRoadmap(mode->robotGroup)->Write(this->GetMPProblem()->GetBaseFilename()+"::"+mode->robotGroup->GetLabel() + ".map", this->GetMPProblem()->GetEnvironment());
    // std::cout << "Saved Roadmap for " << mode->robotGroup->GetLabel() << std::endl;
  }

  // TODO::Collect set of modes for each robot-unique object type pairing

  // TODO::For each pairing of individual robot to object type, build an initial roadmap

  // TODO::Copy roadmap to all instances of individual robot-unique object type pairs

  // Set all robots back to non-virtual
  for(auto pair : c->GetInitialRobotGroups()) {
    auto group = pair.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(false);
    }
  }
}



std::set<size_t>
LazyModeGraph::
ConnectTransitions() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() + "::GenerateMotionHypergraph::ConnectTransitions_" + std::to_string(m_iterationCall));

  auto prob = this->GetMPProblem();
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());

  std::cout << "Print grounded vertex" << std::endl;
  for(auto& kv1 : m_modeTransitionHypergraph.GetVertexMap()) {
    if(kv1.first == m_sourceModeTransitionVID or kv1.first == m_sinkModeTransitionVID)
      continue;

    std::cout << kv1.first << " " << kv1.second.property->robotGroup->GetLabel() << ": " ;
    for(auto vid : m_modeTransitionGroundedVertices[kv1.first]) {
      std::cout << vid << " " ;
    }
    std::cout << std::endl;
  }

  std::cout << "Motion Constraints that need to be replan are" << std::endl;
  for(auto pair : m_motionConstraintSet) {
    std::cout << pair.first.first.id << " and " << pair.first.second.id << ": " << pair.second.PrettyPrint() << std::endl;
    if(!m_motionConstraintCount.count(pair)) {
      m_motionConstraintCount[pair] = 1;
    }
    else {
      m_motionConstraintCount[pair] += 1;
    }
  }


  for(auto kv : m_motionConstraintCount) {
    auto pair1 = kv.first;
    auto count1 = kv.second;
    if(count1 > 0) {
      // std::cout << pair1.first.first.id << ", " << pair1.first.second.id << std::endl;
      for(auto pair2 : m_motionConstraintSet) {
        // std::cout << pair2.first.first.id << ", " << pair2.first.second.id << std::endl;
        if((pair2.first.first == pair1.first.first and pair2.first.second == pair1.first.second) or 
            (pair2.first.second == pair1.first.first and pair2.first.first == pair1.first.second)) {
          size_t passiveMTVID = MAX_UINT;
          size_t passive2MTVID = MAX_UINT;
          if(pair1.first.first.vertex) {
            size_t passiveGVID = pair1.first.first.id;
            size_t activeGVID = *gh->GetHyperarc(pair1.first.second.id).tail.begin();
            passiveMTVID = GetTransitionModeOfGroundedVID(passiveGVID);
            size_t activeMTVID = GetTransitionModeOfGroundedVID(activeGVID);
            auto tails = m_modeTransitionHypergraph.GetHyperarc(*m_modeTransitionHypergraph.GetIncomingHyperarcs(activeMTVID).begin()).tail;
            for(size_t t : tails) {
              if(m_modeTransitionHypergraph.GetVertex(t).property->robotGroup->IsPassive())
                passive2MTVID = t;
            }
            std::cout << "passive: " << passiveMTVID << std::endl;
            std::cout << "active: " << passive2MTVID << std::endl;
          }
          else {
            size_t passiveGVID = pair1.first.second.id;
            size_t activeGVID = *gh->GetHyperarc(pair1.first.first.id).tail.begin();
            passiveMTVID = GetTransitionModeOfGroundedVID(passiveGVID);
            size_t activeMTVID = GetTransitionModeOfGroundedVID(activeGVID);
            auto tails = m_modeTransitionHypergraph.GetHyperarc(*m_modeTransitionHypergraph.GetIncomingHyperarcs(activeMTVID).begin()).tail;
            for(size_t t : tails) {
              if(m_modeTransitionHypergraph.GetVertex(t).property->robotGroup->IsPassive())
                passive2MTVID = t;
            }
            std::cout << "passive: " << passiveMTVID << std::endl;
            std::cout << "active: " << passive2MTVID << std::endl;
          }

          if(passiveMTVID != MAX_UINT and passive2MTVID != MAX_UINT) {
            m_geometricConstraintSet[passive2MTVID].insert(passiveMTVID);
          }
        }
      }
    }
  }

  if(m_geometricConstraintSet.size() > 0 and m_geometricConstraintSet.size() != m_prevGeometricConstraintSet.size()) {
    delete mt;
    std::cout << "Failure in connect transitions. Geometric constraints" << std::endl;
    m_prevGeometricConstraintSet = m_geometricConstraintSet;
    return {};
  }


  // std::unordered_map<size_t, std::set<Robot*>> motionConstraints;
  // RobotGroup* passiveGroup = nullptr;
  // for(auto pair1 : m_motionConstraintSet) {
  //   if(pair1.first.first.vertex) {
  //     passiveGroup = gh->GetVertex(pair1.first.first.id).first->GetGroup();
  //     auto hid = pair1.first.second.id;
  //     if(!motionConstraints.count(hid)) {
  //       motionConstraints[hid] = {passiveGroup->GetRobot(0)};
  //     }
  //     else {
  //       motionConstraints[hid].insert(passiveGroup->GetRobot(0));
  //     }
  //   }
  //   else {
  //     passiveGroup = gh->GetVertex(pair1.first.second.id).first->GetGroup();
  //     auto hid = pair1.first.first.id;
  //     if(!motionConstraints.count(hid)) {
  //       motionConstraints[hid] = {passiveGroup->GetRobot(0)};
  //     }
  //     else {
  //       motionConstraints[hid].insert(passiveGroup->GetRobot(0));
  //     }
  //   }
  // }


  // for(auto kv : motionConstraints) {
  //   std::cout << kv.first << " needs to avoid " ;
  //   for(auto robot : kv.second) {
  //     std::cout << robot->GetLabel() << " ";
  //   }
  //   std::cout << std::endl;
  // }










  //auto lib = this->GetMPLibrary();
  std::cout << "MOTION HYPERGRAPH" << std::endl;
  gh->Print();
  std::cout << "MOTION HYPERGRAPH WITH MODE" << std::endl;
  gh->PrintGraphWithModes();
  std::cout << "MOTION HYPERGRAPH MODE TYPE" << std::endl;
  gh->PrintModes();
  std::cout << "MOTION HYPERARC COSTS" << std::endl;

  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    robot->SetVirtual(true);
  }

  std::unordered_map<RobotGroup*,std::vector<size_t>> modes;

  for(auto kv : m_modeTransitionHypergraph.GetVertexMap()) {
    // std::cout << "Gather the same type of active robots" << std::endl;
    if(kv.first == m_sourceModeTransitionVID or kv.first == m_sinkModeTransitionVID)
      continue;
    if(m_unactuatedMTVIDs.count(kv.first))
      continue;
    if(!m_relevantMTVIDs.count(kv.first))
      continue;
    auto group = m_modeTransitionHypergraph.GetVertex(kv.first).property->robotGroup;
    modes[group].push_back(kv.first);
  }
















  std::set<VID> frontiers{0};
  std::unordered_map<RobotGroup*,VID> frontierGroups;

  std::pair<std::set<VID>,std::set<VID>> prevAssignment;
  prevAssignment = *(m_groundedTransitionMap[m_ignitionMTHID].begin());

  // std::vector<std::pair<std::set<VID>,std::set<VID>>> connectionCandidates;
  std::set<VID> goalGroundedVIDs;
  std::vector<VID> hyperarcSequence;
  std::cout << "Relevant MTHIDs " << m_relevantMTHIDVector << std::endl;
  for(size_t i = 0 ; i < m_relevantMTHIDVector.size()-1 ; ++i) {
    if(m_debug)
      std::cout << "==========================================================================================================" << std::endl;
    if(!(prevAssignment.first.size()==1 and prevAssignment.second.size()==1)) {
      hyperarcSequence.push_back(gh->GetHID(prevAssignment.first,prevAssignment.second));
      if(m_debug) {
        std::cout << "add sequence: " << gh->GetHID(prevAssignment.first, prevAssignment.second) << ": " << prevAssignment.first << " --> " << prevAssignment.second << std::endl;
      }
    }
    auto prev = m_relevantMTHIDVector[i];
    auto next = m_relevantMTHIDVector[i+1];
    if(m_debug) {
      std::cout << "prev mt hid: " << prev << std::endl;
      std::cout << "next mt hid: " << next << std::endl;
    }
    // Find the next assignment
    auto nextTail = m_modeTransitionHypergraph.GetHyperarc(next).tail;
    auto nextHead = m_modeTransitionHypergraph.GetHyperarc(next).head;


    if(m_debug) {
      std::cout << "next mt vids: " << nextTail << " --->>> " << nextHead << std::endl;
    }



    // Add heads to frontier
    if(m_debug)
      std::cout << "Current Frontier: " << frontiers << std::endl;
    auto prevGroundedTail = prevAssignment.first;
    auto prevGroundedHead = prevAssignment.second;
    for(size_t t : prevGroundedTail) {
      frontiers.erase(t);
      if(t != 0) {
        auto group = gh->GetVertex(t).first->GetGroup();
        auto it = frontierGroups.find(group);
        if (it != frontierGroups.end()) 
          frontierGroups.erase(it);
      }
    }
    for(size_t h : prevGroundedHead) {
      frontiers.insert(h);
      if(h != 1)
        frontierGroups[gh->GetVertex(h).first->GetGroup()] = h;
    }
    if(m_debug) {
      std::cout << "Applying the prev action " << prev << ": " << prevGroundedTail << " --->>> " << prevGroundedHead << std::endl;
      std::cout << "Frontier after applying the prev hid: " << frontiers << std::endl;
    }





    std::set<RobotGroup*> nextTailGroup;
    std::set<RobotGroup*> nextHeadGroup;
    for(auto t : nextTail) {
      if(t != m_sourceModeTransitionVID) {
        nextTailGroup.insert(m_modeTransitionHypergraph.GetVertex(t).property->robotGroup);
        if(m_debug) {
          std::cout << "Tail Group: " 
                    << m_modeTransitionHypergraph.GetVertex(t).property->robotGroup->GetLabel() << std::endl;
        }
      }
    }
    for(auto h : nextHead) {
      if(h != m_sinkModeTransitionVID) {
        nextHeadGroup.insert(m_modeTransitionHypergraph.GetVertex(h).property->robotGroup);
        if(m_debug) {
          std::cout << "Head Group: " 
                    << m_modeTransitionHypergraph.GetVertex(h).property->robotGroup->GetLabel() << std::endl;
        }
      }
    }



    // if(nextTail.size() == 1 and \
    //     nextHead.size() == 1 and \
    //     (*nextTailGroup.begin())->GetRobots().size() > 1 and \
    //     (*nextHeadGroup.begin())->GetRobots() == (*nextTailGroup.begin())->GetRobots())
    //   continue;


    auto nextAssignments = m_groundedTransitionMap[next];
    std::pair<std::set<VID>,std::set<VID>> nextAssignment;
    if(m_debug) 
      std::cout << "next possible assignments for mt hid " << next << ": " << nextAssignments << std::endl;
    // Compare the tail and head groups
    // Assume only two head tail combination exist. A->B or B->A
    for(auto na : nextAssignments) {
      auto nextGroundedTail = na.first;
      auto nextGroundedHead = na.second;
      if(nextGroundedTail == nextGroundedHead) {
        // std::cout << nextGroundedTail << "==" << nextGroundedHead << std::endl;
        if(frontiers.count(*(nextGroundedTail.begin()))) {
          nextAssignment = na;
          break;
        }
      }
      if(m_debug) 
        std::cout << "Checking: " << na << std::endl;
      std::set<RobotGroup*> nextTailGroundedGroup;
      std::set<RobotGroup*> nextHeadGroundedGroup;
      for(auto t : nextGroundedTail) {
        nextTailGroundedGroup.insert(gh->GetVertex(t).first->GetGroup());
        if(m_debug) {
          std::cout << "Tail Group: " 
                    << gh->GetVertex(t).first->GetGroup()->GetLabel() << std::endl;
        }
      }
      for(auto h : nextGroundedHead) {
        if(h != 1) {
          nextHeadGroundedGroup.insert(gh->GetVertex(h).first->GetGroup());
          if(m_debug) {
            std::cout << "Head Group: " 
                      << gh->GetVertex(h).first->GetGroup()->GetLabel() << std::endl;
          }
        }
      }
      if(nextTailGroundedGroup == nextTailGroup and nextHeadGroundedGroup == nextHeadGroup)
        nextAssignment = na;
    }


    if(nextTail.size() == 1 and nextHead.size() == 1) {
      if(m_debug)
        std::cout << "same robot transition " << std::endl;
      prevAssignment = nextAssignment;
      if(m_debug)
        std::cout << "prev assignment: " << prevAssignment << std::endl;
      // continue;
    }

    auto nextGroundedTail = nextAssignment.first;
    std::set<VID> exists;
    std::set<VID> notExists;
    std::unordered_map<RobotGroup*,VID> existsGroup;
    std::unordered_map<RobotGroup*,VID> notExistsGroup;
    bool passiveAllGoal = false ;
    if(next == m_terminationMTHID) {
      passiveAllGoal = true;
      for(auto f : frontiers) {
        auto group = gh->GetVertex(f).first->GetGroup();
        std::cout << group->GetLabel() << std::endl;
        bool passive = true;
        for(auto r : group->GetRobots()) {
          if(!r->GetMultiBody()->IsPassive()) {
            passive = false;
            break;
          }
        }
        if(!passive) 
          continue;
        if(!m_goalGroundedVIDs.count(f)) {
          passiveAllGoal = false;
          break;
        }
        std::cout << "Add " << std::endl;
        goalGroundedVIDs.insert(f);
      }
    }

    if(passiveAllGoal) {
      std::cout << "Found goal" << std::endl;
      break;
    }
    
    for(size_t gt : nextGroundedTail) {
      if(!frontiers.count(gt)) {
        notExists.insert(gt);
        notExistsGroup[gh->GetVertex(gt).first->GetGroup()] = gt;
        if(m_debug)
          std::cout << gt << " (" << gh->GetVertex(gt).first->GetGroup()->GetLabel() << ") not exists in the frontier" << std::endl;
      }
      else {
        exists.insert(gt);
        existsGroup[gh->GetVertex(gt).first->GetGroup()] = gt;
        if(m_debug)
          std::cout << gt << " (" << gh->GetVertex(gt).first->GetGroup()->GetLabel() << ") exists in the frontier" << std::endl;
      }
    }
    if(m_debug) {
      std::cout << "Tries to do: " << nextAssignment.first << " --->>> " << nextAssignment.second << std::endl;
      for(auto t : nextAssignment.first) {
        std::cout << gh->GetVertex(t).first->GetVertex(gh->GetVertex(t).second).PrettyPrint() << ", " ;
      }
      std::cout << " --->>> " ;
      for(auto h : nextAssignment.second) {
        std::cout << gh->GetVertex(h).first->GetVertex(gh->GetVertex(h).second).PrettyPrint() << ", " ;
      }
      std::cout << std::endl;
    }
    // Need Movement
    bool transitionExists = true;
    if(notExistsGroup.size() > 0) {
      if(m_debug) {
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << notExists << " not exists" << std::endl;
      }
      std::unordered_map<RobotGroup*,size_t> frontierNotExistsGrounedVids;
      std::unordered_map<RobotGroup*,size_t> nextPossibleConnectionGroundedVids;
      if(m_debug)
        std::cout << "Finding " << notExists << std::endl;

      transitionExists = true;
      for(auto kv : notExistsGroup) {
        auto group = kv.first;
        if(m_debug) {
          std::cout << "......." << std::endl;
          std::cout << "Finding movements for " << kv.second << "(" << group->GetLabel() << ")" << std::endl;
          std::cout << " L " << frontierGroups[group] << " --->>> " << kv.second << std::endl;
          std::cout << "Connecting " << gh->GetVertex(frontierGroups[group]).first->GetVertex(gh->GetVertex(frontierGroups[group]).second).PrettyPrint() << " --->> " 
                    << gh->GetVertex(kv.second).first->GetVertex(gh->GetVertex(kv.second).second).PrettyPrint() << std::endl; 
        }
        if(gh->GetHID({frontierGroups[group]},{kv.second}) == MAX_UINT) {
          std::cout << "Connecting " << gh->GetVertex(frontierGroups[group]).first->GetVertex(gh->GetVertex(frontierGroups[group]).second).PrettyPrint() << " --->> " 
                    << gh->GetVertex(kv.second).first->GetVertex(gh->GetVertex(kv.second).second).PrettyPrint() << std::endl; 
          transitionExists = gh->ConnectAllTransitions({frontierGroups[group], kv.second}, m_constraintMap[group], false);
        }

        // for(auto kv2 : motionConstraints) {
        //   if(kv2.first == gh->GetHID({frontierGroups[group]},{kv.second})) {
        //     std::cout << "Found conflict hid " << kv2.first << std::endl;
        //     auto haTaskSet = gh->GetHyperarc(kv2.first).property.taskSet;
        //     for(auto v : haTaskSet) {
        //       for(auto e : v) {
        //         auto group = e->GetRobotGroup();
        //         std::cout << "\tGroup " << group->GetLabel();
        //       }
        //     }
        //   }
        // }

        if(!transitionExists)
          break;
        
        if(m_debug)
          std::cout << "Hid found: " << gh->GetHID({frontierGroups[group]},{kv.second}) << ": " 
                    << frontierGroups[group] << " --->>> " << kv.second << std::endl;
      }


      // If transition not found, resample transition (handoff positions)
      if(transitionExists) {
        std::cout << "Transition exists " << i << "/" << m_relevantMTHIDVector.size() << std::endl;
        for(auto kv : notExistsGroup) {
          auto group = kv.first;
          std::cout << "add sequence: " << gh->GetHID({frontierGroups[group]},{kv.second}) << ": " << frontierGroups[group] << " --> " << kv.second << std::endl;
          hyperarcSequence.push_back(gh->GetHID({frontierGroups[group]},{kv.second}));
          // Apply movement A --> B
          // Remove A
          frontiers.erase(frontierGroups[group]);
          frontierGroups.erase(frontierGroups.find(group));
          // Insert B
          frontiers.insert(kv.second);
          frontierGroups[group] = kv.second;
          notExists.erase(kv.second);
        }
      }
      else {
        std::cout << "Transition not exists" << std::endl;
        for(size_t k = 0 ; k < m_resampleAttempts ; k++) {
          std::cout << "iteration: " << k << " ====================================" << std::endl;
          transitionExists = true;
          ResampleTransitions(next, nextAssignment);
          std::cout << "nextAssignment after resampling transitions: " << nextAssignment << std::endl;
          nextGroundedTail = nextAssignment.first;
          for(size_t gt : nextGroundedTail) {
            if(!frontiers.count(gt)) {
              notExists.insert(gt);
              notExistsGroup[gh->GetVertex(gt).first->GetGroup()] = gt;
            }
            else {
              exists.insert(gt);
              existsGroup[gh->GetVertex(gt).first->GetGroup()] = gt;
            }
          }

          for(auto kv : notExistsGroup) {
            auto group = kv.first;
            auto newNextFrontier = kv.second;
            std::cout << "Finding movements for " << newNextFrontier << "(" << group->GetLabel() << ")" << std::endl;
            std::cout << " L " << frontierGroups[group] << " --->>> " << newNextFrontier << std::endl;
            std::cout << "\tFrom: " << gh->GetVertex(frontierGroups[group]).first->GetVertex(gh->GetVertex(frontierGroups[group]).second).PrettyPrint() << std::endl;
            std::cout << "\tTo: " << gh->GetVertex(newNextFrontier).first->GetVertex(gh->GetVertex(newNextFrontier).second).PrettyPrint() << std::endl;
            
            if(gh->GetHID({frontierGroups[group]},{newNextFrontier}) == MAX_UINT) 
              transitionExists = gh->ConnectAllTransitions({frontierGroups[group], newNextFrontier}, m_constraintMap[group], false);
            
            if(!transitionExists) {
              std::cout << "Could not find a path connecting the transitions. Plan failed." << std::endl;
              std::cout << "Robot: " << group->GetLabel() << std::endl; 
              std::cout << "\tFrom: " << gh->GetVertex(frontierGroups[group]).first->GetVertex(gh->GetVertex(frontierGroups[group]).second).PrettyPrint() << std::endl;
              std::cout << "\tTo: " << gh->GetVertex(newNextFrontier).first->GetVertex(gh->GetVertex(newNextFrontier).second).PrettyPrint() << std::endl;
              break;
            }
            else {         
              std::cout << "Hid found: " << gh->GetHID({frontierGroups[group]},{newNextFrontier}) << ": '" 
                        << frontierGroups[group] << " --->>> " << newNextFrontier << std::endl;
            }
          }

          if(!transitionExists) {
            std::cout << "Could not find a path connecting the transitions. Plan failed. Resample." << std::endl; 
            continue;
          }

          std::cout << "Found a path for the new transitions" << std::endl;
          for(auto kv : notExistsGroup) {
            auto group = kv.first;
            auto newNextFrontier = kv.second;
            std::cout << "add sequence: " << gh->GetHID({frontierGroups[group]},{newNextFrontier}) << ": " << frontierGroups[group] << " --> " << newNextFrontier << std::endl;
            hyperarcSequence.push_back(gh->GetHID({frontierGroups[group]},{newNextFrontier}));
            // Apply movement A --> B
            // Remove A
            frontiers.erase(frontierGroups[group]);
            frontierGroups.erase(frontierGroups.find(group));
            // Insert B
            frontiers.insert(newNextFrontier);
            frontierGroups[group] = newNextFrontier;
            notExists.erase(newNextFrontier);
          }
          break;
        }
      }

      if(!transitionExists) {
        // m_replanSource = next;
        delete mt;
        std::cout << "Failure in connect transitions. Fail to find motions" << std::endl;
        // TODO: ADD Task Constraints
        std::cout << "Cannot perform head of mt hid " << prev << " to tail of mt hid " << next << std::endl;
        
        std::vector<size_t> taskOrder{prev,next};
        // for(size_t hid : m_relevantMTHIDVector) {
        //   taskOrder.insert(hid);
        //   if(hid == next)
        //     break;
        // }
        m_taskOrderConstraintSet.insert(taskOrder);

        return {};
      }

      if(m_debug) {
        std::cout << "Resulting frontiers: " << frontiers << std::endl;
        std::cout << "Resulting frontiers: {" ;
        for(auto v : frontiers) {
          std::cout << gh->GetVertex(v).first->GetGroup()->GetLabel() << ", " ;
        }
        std::cout << "}" << std::endl;
      }
    }
    else {
      std::cout << "All gvids exist" << std::endl;
      std::cout << "Resulting frontiers: " << frontiers << std::endl;
      std::cout << "Resulting frontiers: {" ;
      for(auto v : frontiers) {
        std::cout << gh->GetVertex(v).first->GetGroup()->GetLabel() << ", " ;
      }
      std::cout << "}" << std::endl;
    }


    if(m_debug) {
      std::cout << "Frontier Groups" << std::endl;
      for(auto kv : frontierGroups) {
        std::cout << kv.first->GetLabel() << ": " << kv.second << std::endl;
      }
    }

    prevAssignment = nextAssignment;
    if(m_debug)
      std::cout << "prev assignment  : " << prevAssignment << std::endl;
  }





  if(m_debug) {
    std::cout << "Grounded Path: " << hyperarcSequence << std::endl;
    for(auto hid : hyperarcSequence) {
      std::cout << hid << ": " << gh->GetHyperarc(hid).tail << " --> " << gh->GetHyperarc(hid).head << std::endl;
    }
  }
  
  m_motionHistory = hyperarcSequence;
  delete mt;
  // Strange behavior to avoid issues with roadmaps being improved after costs are saved
  if(m_queryStrategy != m_queryStrategyStatic) {
    auto stash = m_queryStrategy;
    m_queryStrategy = m_queryStrategyStatic;
    ConnectTransitions();
    m_queryStrategy = stash;
  }


  stats->SetStat(this->GetNameAndLabel() + "::MotionHypergraphVertexSize", gh->Size());
  stats->SetStat(this->GetNameAndLabel() + "::MotionHypergraphEgdeSize", gh->EdgeSize());
  stats->SetStat(this->GetNameAndLabel() + "::ModeHypergraphVertexSize", m_modeTransitionHypergraph.Size());
  stats->SetStat(this->GetNameAndLabel() + "::ModeHypergraphEdgeSize", m_modeTransitionHypergraph.EdgeSize());

  if(m_debug) {
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << std::endl;
  }
  
  return goalGroundedVIDs;
}







































































































std::vector<std::pair<size_t,std::pair<std::set<VID>,std::set<VID>>>>
LazyModeGraph::
GetGroundedVertexHistory() {
  auto prob = this->GetMPProblem();
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());

  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    robot->SetVirtual(true);
  }

  std::unordered_map<RobotGroup*,std::vector<size_t>> modes;

  for(auto kv : m_modeTransitionHypergraph.GetVertexMap()) {
    // std::cout << "Gather the same type of active robots" << std::endl;
    if(kv.first == m_sourceModeTransitionVID or kv.first == m_sinkModeTransitionVID)
      continue;
    if(m_unactuatedMTVIDs.count(kv.first))
      continue;
    if(!m_relevantMTVIDs.count(kv.first))
      continue;
    auto group = m_modeTransitionHypergraph.GetVertex(kv.first).property->robotGroup;
    modes[group].push_back(kv.first);
  }

  std::set<VID> frontiers{0};
  std::unordered_map<RobotGroup*,VID> frontierGroups;

  std::pair<std::set<VID>,std::set<VID>> prevAssignment;
  prevAssignment = *(m_groundedTransitionMap[m_ignitionMTHID].begin());

  // std::vector<std::pair<std::set<VID>,std::set<VID>>> connectionCandidates;
  std::set<VID> goalGroundedVIDs;
  std::vector<std::pair<size_t,std::pair<std::set<VID>,std::set<VID>>>> hyperarcSequence;
  size_t prev = MAX_UINT;
  size_t next = MAX_UINT;
  for(size_t i = 0 ; i < m_relevantMTHIDVector.size()-1 ; ++i) {
    if(!(prevAssignment.first.size()==1 and prevAssignment.second.size()==1)) {
      hyperarcSequence.push_back(std::make_pair(prev,std::make_pair(prevAssignment.first,prevAssignment.second)));
    }
    std::cout << prev << std::endl;
    prev = m_relevantMTHIDVector[i];
    next = m_relevantMTHIDVector[i+1];

    // Find the next assignment
    auto nextTail = m_modeTransitionHypergraph.GetHyperarc(next).tail;
    auto nextHead = m_modeTransitionHypergraph.GetHyperarc(next).head;
    
    
    auto prevGroundedTail = prevAssignment.first;
    auto prevGroundedHead = prevAssignment.second;
    for(size_t t : prevGroundedTail) {
      frontiers.erase(t);
      if(t != 0) {
        auto group = gh->GetVertex(t).first->GetGroup();
        auto it = frontierGroups.find(group);
        if (it != frontierGroups.end()) 
          frontierGroups.erase(it);
      }
    }
    for(size_t h : prevGroundedHead) {
      frontiers.insert(h);
      if(h != 1)
        frontierGroups[gh->GetVertex(h).first->GetGroup()] = h;
    }


    std::set<RobotGroup*> nextTailGroup;
    std::set<RobotGroup*> nextHeadGroup;
    for(auto t : nextTail) {
      if(t != m_sourceModeTransitionVID) {
        nextTailGroup.insert(m_modeTransitionHypergraph.GetVertex(t).property->robotGroup);
      }
    }
    for(auto h : nextHead) {
      if(h != m_sinkModeTransitionVID) {
        nextHeadGroup.insert(m_modeTransitionHypergraph.GetVertex(h).property->robotGroup);
      }
    }
    auto nextAssignments = m_groundedTransitionMap[next];
    std::cout << "SSIIIZE: " << nextAssignments.size() << std::endl;
    std::pair<std::set<VID>,std::set<VID>> nextAssignment;
    for(auto na : nextAssignments) {
      auto nextGroundedTail = na.first;
      auto nextGroundedHead = na.second;
      if(nextGroundedTail == nextGroundedHead) {
        if(frontiers.count(*(nextGroundedTail.begin()))) {
          nextAssignment = na;
          break;
        }
      }
      std::set<RobotGroup*> nextTailGroundedGroup;
      std::set<RobotGroup*> nextHeadGroundedGroup;
      for(auto t : nextGroundedTail) {
        nextTailGroundedGroup.insert(gh->GetVertex(t).first->GetGroup());
      }
      for(auto h : nextGroundedHead) {
        if(h != 1) {
          nextHeadGroundedGroup.insert(gh->GetVertex(h).first->GetGroup());
        }
      }
      if(nextTailGroundedGroup == nextTailGroup and nextHeadGroundedGroup == nextHeadGroup)
        nextAssignment = na;
    }


    if(nextTail.size() == 1 and nextHead.size() == 1) {
      prevAssignment = nextAssignment;
      continue;
    }

    auto nextGroundedTail = nextAssignment.first;
    std::set<VID> exists;
    std::set<VID> notExists;
    std::unordered_map<RobotGroup*,VID> existsGroup;
    std::unordered_map<RobotGroup*,VID> notExistsGroup;
    bool passiveAllGoal = false ;
    if(next == m_terminationMTHID) {
      passiveAllGoal = true;
      for(auto f : frontiers) {
        auto group = gh->GetVertex(f).first->GetGroup();
        std::cout << group->GetLabel() << std::endl;
        bool passive = true;
        for(auto r : group->GetRobots()) {
          if(!r->GetMultiBody()->IsPassive()) {
            passive = false;
            break;
          }
        }
        if(!passive) 
          continue;
        if(!m_goalGroundedVIDs.count(f)) {
          passiveAllGoal = false;
          break;
        }
        goalGroundedVIDs.insert(f);
      }
    }

    if(passiveAllGoal) {
      break;
    }
    
    for(size_t gt : nextGroundedTail) {
      if(!frontiers.count(gt)) {
        notExists.insert(gt);
        notExistsGroup[gh->GetVertex(gt).first->GetGroup()] = gt;
      }
      else {
        exists.insert(gt);
        existsGroup[gh->GetVertex(gt).first->GetGroup()] = gt;
      }
    }
    // Need Movement
    if(notExistsGroup.size() > 0) {
      std::unordered_map<RobotGroup*,size_t> frontierNotExistsGrounedVids;
      std::unordered_map<RobotGroup*,size_t> nextPossibleConnectionGroundedVids;

      for(auto kv : notExistsGroup) {
        auto group = kv.first;
        std::set<VID> tail{frontierGroups[group]};
        std::set<VID> head{kv.second};
        hyperarcSequence.push_back(std::make_pair(next,std::make_pair(tail,head)));

        frontiers.erase(frontierGroups[group]);
        frontierGroups.erase(frontierGroups.find(group));

        frontiers.insert(kv.second);
        frontierGroups[group] = kv.second;
        notExists.erase(kv.second);
      }
    }

    prevAssignment = nextAssignment;
  }











  std::vector<std::pair<size_t,std::pair<std::set<VID>,std::set<VID>>>> hyperarcSequence2;

  std::unordered_map<RobotGroup*,VID> frontierInfo;
  next = m_ignitionMTHID;
  auto ignha = m_modeTransitionHypergraph.GetHyperarc(next);
  for(size_t h : ignha.head) {
    RobotGroup* group = m_modeTransitionHypergraph.GetVertex(h).property->robotGroup;
    frontierInfo[group] = h;
  }
  hyperarcSequence2.push_back(std::make_pair(next,std::make_pair(ignha.tail,ignha.head)));

  std::cout << "Initiate" << std::endl;
  std::cout << next << " =========================" << std::endl;
  std::cout << "\t" << hyperarcSequence2.back() << std::endl;

  std::set<VID> goalGroundedVIDs2;
  for(size_t i = 1 ; i < m_relevantMTHIDVector.size()-1 ; ++i) {
    next = m_relevantMTHIDVector[i];
    std::cout << next << " =========================" << std::endl;

    // Find the next assignment
    auto nextTail = m_modeTransitionHypergraph.GetHyperarc(next).tail;
    auto nextHead = m_modeTransitionHypergraph.GetHyperarc(next).head;
    
    if(nextTail.size() == 1 and nextHead.size() == 1) 
      continue;

    std::set<RobotGroup*> nextTailGroup;
    std::set<RobotGroup*> nextHeadGroup;
    std::cout << "Tail Robots " << std::endl;
    for(auto t : nextTail) {
      auto g = m_modeTransitionHypergraph.GetVertex(t).property->robotGroup;
      std::cout << "\t" << t << ": " << g->GetLabel() << std::endl;
      nextTailGroup.insert(g);
    }
    std::cout << "Head Robots " << std::endl;
    for(auto h : nextHead) {
      auto g = m_modeTransitionHypergraph.GetVertex(h).property->robotGroup;
      std::cout << "\t" << h << ": " << g->GetLabel() << std::endl;
      nextHeadGroup.insert(g);
    }
    auto nextAssignments = m_groundedTransitionMap[next];
    std::cout << "Possible Assignments: " << nextAssignments << std::endl;
    
    bool foundNext = false;
    for(auto candid : nextAssignments) {
      std::cout << "\t-->> Check " << candid << std::endl;
      auto tail = candid.first;
      auto head = candid.second;
      std::set<RobotGroup*> candidTailGroup;
      std::set<RobotGroup*> candidHeadGroup;
      for(auto t : tail) {
        auto g = gh->GetVertex(t).first->GetGroup();
        std::cout << "\t" << t << ": " << g->GetLabel() << std::endl;
        candidTailGroup.insert(g);
      }
      for(auto h : head) {
        auto g = gh->GetVertex(h).first->GetGroup();
        std::cout << "\t" << h << ": " << g->GetLabel() << std::endl;
        candidHeadGroup.insert(g);
      }
      if(candidTailGroup == nextTailGroup and candidHeadGroup == nextHeadGroup) {
        hyperarcSequence2.push_back(std::make_pair(next,std::make_pair(tail,head)));
        foundNext = true;
        break;
      }
    }
    if(foundNext) {
      std::cout << "\tFound! " << hyperarcSequence2.back() << std::endl;
    }
    else {
      std::cout << "\tNot Found!" << std::endl;
    }
  }













  return hyperarcSequence;

}








































































void
LazyModeGraph::
ResampleTransitions(size_t _target, std::pair<std::set<VID>,std::set<VID>>& _nextAssignment) {
  std::cout << " === Resample Transitions of hid: " << _target << std::endl;
  // Might just add a new transitions ccandidate to create diverse options
  m_groundedTransitionMap[_target].clear();
  m_computedInteraction.erase(_target);
  SampleTransitions({_target});
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());

  auto nextTail = m_modeTransitionHypergraph.GetHyperarc(_target).tail;
  auto nextHead = m_modeTransitionHypergraph.GetHyperarc(_target).head;
  std::cout << "Resample Transitions: Done sampling of target: " << _target << std::endl;
  std::set<RobotGroup*> nextTailGroup;
  std::set<RobotGroup*> nextHeadGroup;
  for(auto t : nextTail) {
    if(t != m_sourceModeTransitionVID) {
      nextTailGroup.insert(m_modeTransitionHypergraph.GetVertex(t).property->robotGroup);
      if(m_debug) {
        std::cout << "Tail Group: " 
                  << m_modeTransitionHypergraph.GetVertex(t).property->robotGroup->GetLabel() << std::endl;
      }
    }
  }
  for(auto h : nextHead) {
    if(h != m_sinkModeTransitionVID) {
      nextHeadGroup.insert(m_modeTransitionHypergraph.GetVertex(h).property->robotGroup);
      if(m_debug) {
        std::cout << "Head Group: " 
                  << m_modeTransitionHypergraph.GetVertex(h).property->robotGroup->GetLabel() << std::endl;
      }
    }
  }


  auto nextAssignments = m_groundedTransitionMap[_target];
  std::pair<std::set<VID>,std::set<VID>> nextAssignment;
  if(m_debug) 
    std::cout << "next possible assignments: " << nextAssignments << std::endl;
  for(auto na : nextAssignments) {
    auto nextGroundedTail = na.first;
    auto nextGroundedHead = na.second;
    if(m_debug) 
      std::cout << "Checking: " << na << std::endl;
    std::set<RobotGroup*> nextTailGroundedGroup;
    std::set<RobotGroup*> nextHeadGroundedGroup;
    for(auto t : nextGroundedTail) {
      nextTailGroundedGroup.insert(gh->GetVertex(t).first->GetGroup());
      if(m_debug) {
        std::cout << "Tail Group: " 
                  << gh->GetVertex(t).first->GetGroup()->GetLabel() << std::endl;
      }
    }
    for(auto h : nextGroundedHead) {
      if(h != 1) {
        nextHeadGroundedGroup.insert(gh->GetVertex(h).first->GetGroup());
        if(m_debug) {
          std::cout << "Head Group: " 
                    << gh->GetVertex(h).first->GetGroup()->GetLabel() << std::endl;
        }
      }
    }
    if(nextTailGroundedGroup == nextTailGroup and nextHeadGroundedGroup == nextHeadGroup)
      nextAssignment = na;
  }

  std::cout << "NEXTAssignment: " << nextAssignment << std::endl;
  _nextAssignment= nextAssignment;
  std::cout << "NEXTAssignment: " << _nextAssignment << std::endl;

}

void
LazyModeGraph::
ApplyAction(Action* _action, std::set<std::vector<VID>>& _applied, std::vector<VID>& _newModes, bool _forward) {
  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Extract the formation and motion constraints
  std::vector<FormationCondition*> initialFormationConditions;
  std::vector<MotionCondition*> initialMotionConditions;
  auto initialStage = _forward ? _action->GetStages()[0] : _action->GetStages().back();
  for(auto label : _action->GetStageConditions(initialStage)) {
    auto c = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(c);
    if(f) {
      initialFormationConditions.push_back(f);
      continue;
    }
    auto m = dynamic_cast<MotionCondition*>(c);
    if(m)
      initialMotionConditions.push_back(m);
  }

  // Connect motion constraints
  std::unordered_map<std::string,Constraint*> motionConstraintMap;
  for(auto m : initialMotionConditions) {
    for(auto role : m->GetRoles()) {
      auto constraint = m->GetRoleConstraint(role);
      motionConstraintMap[role] = constraint;
    }
  }

  // Look at possible combinations of modes in the mode hyerpgraph 
  // that satisfy the formation constraint
  std::vector<std::vector<VID>> formationModes(initialFormationConditions.size());

  for(size_t i = 0; i < initialFormationConditions.size(); i++) {
    auto f = initialFormationConditions[i];

    for(const auto& kv : m_modeHypergraph.GetVertexMap()) {
      auto vid = kv.first;
      auto mode = kv.second.property;

      std::set<Robot*> used;
      for(auto type : f->GetTypes()) {
        for(auto robot : mode->robotGroup->GetRobots()) {
          // Make sure robot has not been accounted for
          if(used.count(robot))
            continue;

          // Reserve robot if it is a match
          if(robot->GetCapability() == type)
            used.insert(robot);
        }
      }

      // Check if the number of saved robots matches the required number
      if(f->GetTypes().size() != used.size() or used.size() != mode->robotGroup->Size()) 
        continue;

      // Check if mode meets formation requirements
      // Create state
      State state;
      state[mode->robotGroup] = std::make_pair(nullptr,MAX_UINT);
      std::unordered_map<std::string,Robot*> roleMap;
      f->AssignRoles(roleMap,state);
      bool satisfied = mode->formations.empty();
      for(auto formation : mode->formations) {
        if(f->DoesFormationMatch(roleMap,formation)) {
          satisfied = true;
          break;
        }
      }

      if(!satisfied)
        continue;

      // Check if mode meets motion constraints
      for(auto role : f->GetRoles()) {
        // Check if role has associated motion constraint
        auto iter1 = motionConstraintMap.find(role);
        if(iter1 == motionConstraintMap.end())
          continue;

        // Check if constraint is in the mode
        auto constraint = motionConstraintMap[role];
        auto b1 = constraint->GetBoundary();
        if(!b1)
          continue;

        for(const auto& c : mode->constraints) {
          // Check if boundaries are the same
          auto b2 = c->GetBoundary();
          if(b1->Type() == b2->Type()
            and b1->Name() == b2->Name()
            and b1->GetDimension() == b2->GetDimension()
            and b1->GetCenter() == b2->GetCenter()) {

            bool match = true;
            for(size_t i = 0; i < b1->GetDimension(); i++) {
              if(!(b1->GetRange(i) == b2->GetRange(i))) {
                match = false;
              }
            }
            if(match)
              continue;
          }

          satisfied = false;
          break;
        }
      }

      if(satisfied) {
        formationModes[i].push_back(vid);
      }
    }
  }

  // Convert potential individual mode assignments into mode sets
  std::vector<VID> partialSet;
  auto modeSets = CollectModeSets(formationModes,0,partialSet);

  // Apply the action and generate new modes for each valid combination
  // not already in _applied

  // Extract the final formation and motion constraints
  std::vector<FormationCondition*> finalFormationConditions;
  std::vector<MotionCondition*> finalMotionConditions;
  auto finalStage = _forward ? _action->GetStages().back() : _action->GetStages().front();

  for(auto label : _action->GetStageConditions(finalStage)) {
    auto c = as->GetCondition(label);
    auto f = dynamic_cast<FormationCondition*>(c);
    if(f)
      finalFormationConditions.push_back(f);

    auto m = dynamic_cast<MotionCondition*>(c);
    if(m)
      finalMotionConditions.push_back(m);
  }

  for(auto set : modeSets) {

    // Check that this combo has not been tried before
    if(_applied.count(set))
      continue;

    // Collect all available robots
    std::vector<Robot*> robots;
    for(auto vid : set) {
      auto mode = m_modeHypergraph.GetVertexType(vid);
      for(auto robot : mode->robotGroup->GetRobots()) {
        robots.push_back(robot);
      }
    }

    // Collect robot roles
    /*std::unordered_map<std::string,Robot*> roleMap;
    for(size_t i = 0; i < set.size(); i++) {
      auto vid = set[i];
      auto mode = m_modeHypergraph.GetVertexType(vid);
      auto formationCondition = initialFormationConditions[i];
      State state;
      state[mode->robotGroup] = std::make_pair(nullptr,MAX_UINT);
      formationCondition->AssignRoles(roleMap,state);
    }*/

    // Collect possible assignment of robots into groups
    std::vector<std::vector<std::vector<Robot*>>> possibleAssignments(finalFormationConditions.size());
    for(size_t i = 0; i < possibleAssignments.size(); i++) {
      auto fc = finalFormationConditions[i];
      auto types = fc->GetTypes();

      // Collect all robot matches for each type
      std::vector<std::vector<Robot*>> possibleTypeAssignments(types.size());
      for(size_t j = 0; j < types.size(); j++) {
        std::vector<Robot*> matches;

        for(auto robot : robots) {
          if(robot->GetCapability() == types[j]) {
            matches.push_back(robot);
          }
        }

        possibleTypeAssignments[j] = matches;
      }

      possibleAssignments[i] = possibleTypeAssignments;
    }

    // Build output mode sets
    std::vector<Mode*> partial;
    auto modeSetCombos = CollectModeSetCombinations(possibleAssignments,0,partial,{});

    // Add formation and path constraints
    for(auto combo : modeSetCombos) {
      for(size_t i = 0; i < combo.size(); i++) {
        auto formationCondition = finalFormationConditions[i];
        auto mode = combo[i];

        // Construct role map for mode set
        std::unordered_map<std::string,Robot*> roleMap;
        State state;
        state[mode->robotGroup] = std::make_pair(nullptr,MAX_UINT);
        formationCondition->AssignRoles(roleMap,state);

        // Create formation constraints from roleMap
        auto formation = formationCondition->GenerateFormation(roleMap);
        if(formation)
          mode->formations.insert(formation);

        // Grab path constraints from final stage
        for(auto motionCondition : finalMotionConditions) {
          auto roles = motionCondition->GetRoles();

          for(auto robot : mode->robotGroup->GetRobots()) {
            std::string role;

            for(auto kv : roleMap) {
              if(kv.second == robot) {
                role = kv.first;
                break;
              }
            }

            if(!roles.count(role))
              continue;
  
            auto constraints = motionCondition->GetConstraints(robot->GetCapability());
            for(auto c : constraints) {
              if(motionCondition->GetRole(c) != role)
                continue;

              auto constraint = c->Clone();
              constraint->SetRobot(robot);
              mode->constraints.push_back(std::move(constraint));
            }
          }
        }
      }
    }

    // Add new modes to the graph and connect them to the start modes
    std::set<VID> tail; 
    for(auto vid : set) {
      tail.insert(vid);
    }

    for(auto modeSet : modeSetCombos) {

      // TODO::Make sure this doesn't happen in the first place
      // Make sure there is not any overlap in head and tail
      bool overlap = false;

      std::set<VID> head;
      for(auto mode : modeSet) {
        //auto vid = m_modeHypergraph.AddVertex(mode);
        auto oldSize = m_modeHypergraph.Size();
        auto vid = AddMode(mode);

        if(tail.count(vid)) {
          overlap = true;
          break;
        }
        head.insert(vid);
        if(oldSize < m_modeHypergraph.Size())
          _newModes.push_back(vid);
      }

      if(overlap)
        continue;

      // Check if this exists already
      bool exists = false;
      for(auto iter1 = tail.begin(); iter1 != tail.end(); iter1++) {
        auto iter2 = iter1;
        iter2++;
        for(; iter2 != tail.end(); iter2++) {
          for(auto h1 : m_modeHypergraph.GetOutgoingHyperarcs(*iter1)) {
            for(auto h2 : m_modeHypergraph.GetOutgoingHyperarcs(*iter2)) {
              if(h1 != h2)
                continue;

              auto ha = m_modeHypergraph.GetHyperarc(h1);
              if(ha.property.first == _action) {
                exists = true;
                break;
              }
            }
            if(exists)
              break;
          }
          if(exists)
            break;
        }
        if(exists)
          break;
      }

      size_t newMHid;
      if(!exists) {
        newMHid = m_modeHypergraph.AddHyperarc(head,tail,std::make_pair(_action,!_forward));
        if(m_debug) {
          std::cout << "Adding Task Space Hyperarc from " << tail << " to " << head 
                    << " of action: " << _action->GetLabel() << std::endl;
        }
      }
      if(_action->IsReversible()) {
        // Check if this exists already
        bool exists = false;
        for(auto iter1 = head.begin(); iter1 != head.end(); iter1++) {
          auto iter2 = iter1;
          iter2++;
          for(; iter2 != head.end(); iter2++) {
            for(auto h1 : m_modeHypergraph.GetOutgoingHyperarcs(*iter1)) {
              for(auto h2 : m_modeHypergraph.GetOutgoingHyperarcs(*iter2)) {
                if(h1 != h2)
                  continue;
  
                auto ha = m_modeHypergraph.GetHyperarc(h1);
                if(ha.property.first == _action) {
                  exists = true;
                  break;
                }
              }
              if(exists)
                break;
            }
            if(exists)
              break;
          }
          if(exists)
            break;
        }
        auto newReverseMHid = m_modeHypergraph.AddHyperarc(tail,head,std::make_pair(_action,_forward));
        m_modeHIDReverseMap[newMHid] = newReverseMHid;
        m_modeHIDReverseMap[newReverseMHid] = newMHid;
        if(m_debug) {
          std::cout << "Adding Task Space Hyperarc from " << head << " to " << tail
                    << " of action: " << _action->GetLabel() << std::endl;
        }
      }
    }

    // Mark that this combination has already been tried.
    _applied.insert(set);
  }
}

std::vector<std::vector<LazyModeGraph::VID>>
LazyModeGraph::
CollectModeSets(const std::vector<std::vector<VID>>& _formationModes, size_t _index, 
               const std::vector<VID>& _partialSet) {

  // Check if we've covered all of our formations
  std::cout << "collect mode sets " << _formationModes.size() << " " << _index << " " << _partialSet.size() << std::endl;
  if(_index == _formationModes.size())
    return {_partialSet};

  // Intialize the return vector
  std::vector<std::vector<VID>> modeSets;
  

  // Add new vids (modes) to the partial set
  auto vids = _formationModes[_index];
  for(auto vid : vids) {
    std::cout << "check vid: " << vid << std::endl;

    // Make sure vid is not already included in the set
    auto iter = std::find(_partialSet.begin(), _partialSet.end(), vid);
    if(iter != _partialSet.end()) {
      std::cout << "continue " << std::endl;
      continue;
    }

    // Make sure this new mode does not intersect with the partial set
    auto mode = m_modeHypergraph.GetVertexType(vid);
    const auto& robots = mode->robotGroup->GetRobots();
    bool intersect = false;
    for(auto vid2 : _partialSet) {
      auto mode2 = m_modeHypergraph.GetVertexType(vid2);
      for(auto r1 : robots) {
        for(auto r2 : mode2->robotGroup->GetRobots()) {
          if(r1 == r2) {
            intersect = true;
            std::cout << "intersect true: " << r1->GetLabel() << " " << r2->GetLabel() << std::endl;
            break;
          }
        }
        if(intersect)
          break;
      }
      if(intersect)
        break;
    }
    if(intersect)
      continue;

    // Add vid ot copy of partial set
    std::vector<VID> set = _partialSet;
    set.push_back(vid);

    // Recursively add additional vids
    auto sets = CollectModeSets(_formationModes,_index+1,set);

    // Add newly discovered sets to the return vector
    for(auto set : sets) {
      modeSets.push_back(set);
    }
  }

  return modeSets;
}

std::vector<std::vector<LazyModeGraph::Mode*>>
LazyModeGraph::
CollectModeSetCombinations(const std::vector<std::vector<std::vector<Robot*>>>& _possibleAssignments,
                        size_t _index, std::vector<Mode*> _partial, 
                        const std::set<Robot*>& _used) {

  // Check if we've reached the end
  if(_index == _possibleAssignments.size())
    return {_partial};
  
  // Intialize the return vector
  std::vector<std::vector<Mode*>> combos;

  // Isolate current mode
  auto possibleModeAssignments = _possibleAssignments[_index];

  // Collect possible combinations for current mode
  std::vector<Robot*> emptyPartial;
  auto modes = CollectModeCombinations(possibleModeAssignments, 0, emptyPartial, _used);

  // Recursively build rest of mode sets
  for(auto mode : modes) {
    std::vector<Mode*> partial = _partial;
    partial.push_back(mode);

    std::set<Robot*> used = _used;
    for(auto robot : mode->robotGroup->GetRobots()) {
      used.insert(robot);
    }

    auto newCombos = CollectModeSetCombinations(_possibleAssignments,_index+1,partial,used);
    for(auto combo : newCombos) {
      combos.push_back(combo);
    }
  }

  return combos;
}

std::vector<LazyModeGraph::Mode*>
LazyModeGraph::
CollectModeCombinations(const std::vector<std::vector<Robot*>>& _possibleModeAssignments,
                        size_t _index, const std::vector<Robot*> _partial,
                        const std::set<Robot*>& _used) {

  if(_index == _possibleModeAssignments.size()) {
    // Convert vector into robot group
      std::string groupLabel = "";
      for(auto robot : _partial) {
        groupLabel += (robot->GetLabel() + "--");
      }

      // Add the group to the problem and solution
      auto problem = this->GetMPProblem();
      auto group = problem->AddRobotGroup(_partial,groupLabel);
      this->GetMPSolution()->AddRobotGroup(group);
      
      //TODO::Figure out how to construct other features of the mode
      Mode* mode = new Mode();
      mode->robotGroup = group;
      
      return {mode};
  }

  std::vector<Mode*> combos;

  auto typeOptions = _possibleModeAssignments[_index];
  for(auto robot : typeOptions) {
    if(_used.count(robot))
      continue;

    std::set<Robot*> used = _used;
    used.insert(robot);

    auto partial = _partial;
    partial.push_back(robot);

    auto newCombos = CollectModeCombinations(_possibleModeAssignments,_index+1,
                                             partial,used);
    
    for(auto combo : newCombos) {
      combos.push_back(combo);
    }
  }

  return combos;
}

void
LazyModeGraph::
SaveInteractionPaths(size_t _hid, Interaction* _interaction, State& _start, State& _end, 
                     std::unordered_map<RobotGroup*,Mode*> _startModeMap,
                     std::unordered_map<RobotGroup*,Mode*> _endModeMap) {
  
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SaveInteractionPaths");
  //auto problem = this->GetMPProblem();

  const auto& stages = _interaction->GetStages();
  State start = _start;
  State end = _end;


  GroundedHypergraph::Transition transition;

  // Collect set of individual paths
  for(size_t i = 1; i < stages.size(); i++) {
    auto paths = _interaction->GetToStagePaths(stages[i]);

    auto solution = _interaction->GetToStageSolution(stages[i]);

    double stageCost = 0;

    for(auto path : paths) {
      const auto& cfgs = path->Cfgs();
      auto robot = path->GetRobot();

      // Skip the first cfgs if this is not the first path
      bool isFirst = transition.explicitPaths[robot].size() == 0;
      for(size_t j = (isFirst) ? 0:1; j < cfgs.size(); j++) {
        transition.explicitPaths[robot].push_back(cfgs[j]);
      }

      // Update max cost at this stage
      stageCost = std::max(stageCost,double(path->TimeSteps()));
    }

    transition.cost += stageCost;

    // Collect stage tasks
    auto tasks = _interaction->GetToStageTasks(stages[i]);
    transition.taskSet.push_back(tasks);
    // Grab active formations from interaction solutions
    for(auto task : tasks) {
      auto grm = solution->GetGroupRoadmap(task->GetRobotGroup());
      auto formations = grm->GetActiveFormations();

      if(grm->GetGroup()->Size() > 1 and formations.empty())
        throw RunTimeException(WHERE) << "AT THE MOMENT, THIS SHOULD NEVER HAPPEN.";

      transition.taskFormations[task.get()] = formations;
    }
    

    // Copy the mp solution info into the local solution
    auto toStageSolution = _interaction->GetToStageSolution(stages[i]);
    for(auto task : tasks) {
      auto group = task->GetRobotGroup();
      std::vector<std::unordered_map<size_t,size_t>> vertexMaps;

      // Copy individual robot roadmaps
      for(auto robot : group->GetRobots()) {
        auto localRM = this->GetMPSolution()->GetRoadmap(robot);
        auto interRM = toStageSolution->GetRoadmap(robot);

        vertexMaps.push_back({});
        std::unordered_map<size_t,size_t>& vertexMap = vertexMaps.back();

        // Copy vertices
        for(auto vit = interRM->begin(); vit != interRM->end(); vit++) {
          auto oldVID = vit->descriptor();
          auto cfg = vit->property();
          auto newVID = localRM->AddVertex(cfg);
          vertexMap[oldVID] = newVID;
        }

        // Copy edges
        for(auto vit = interRM->begin(); vit != interRM->end(); vit++) {
          for(auto eit = vit->begin(); eit != vit->end(); eit++) {
            auto source = vertexMap[eit->source()];
            auto target = vertexMap[eit->target()];
            auto edge = eit->property();
            localRM->AddEdge(source,target,edge);
          }
        }
      }

      // Copy group roadmaps
      auto localGrm = this->GetMPSolution()->GetGroupRoadmap(group);
      auto interGrm = toStageSolution->GetGroupRoadmap(group);
    
      std::unordered_map<size_t,size_t> groupVertexMap;

      // Copy vertices
      for(auto vit = interGrm->begin(); vit != interGrm->end(); vit++) {
        auto oldVID = vit->descriptor();
        auto oldGcfg = vit->property();

        localGrm->SetAllFormationsInactive();
        for(auto f : oldGcfg.GetFormations()) {
          localGrm->AddFormation(f);
        }

        // Construct group cfg
        GroupCfgType newGcfg(localGrm);
        for(size_t i = 0; i < group->GetRobots().size(); i++) {
          auto oldVID = oldGcfg.GetVID(i);

          if(oldVID != MAX_UINT) {
            newGcfg.SetRobotCfg(i,vertexMaps[i][oldVID]);
          }
          else {
            auto cfg = oldGcfg.GetRobotCfg(i);
            newGcfg.SetRobotCfg(i,std::move(cfg));
          }
        }

        // Copy it over to local group roadmap
        auto newVID = localGrm->AddVertex(newGcfg);
        groupVertexMap[oldVID] = newVID;
      }

      // Copy edges
      for(auto vit = interGrm->begin(); vit != interGrm->end(); vit++) {
        for(auto eit = vit->begin(); eit != vit->end(); eit++) {
          auto source = groupVertexMap[eit->source()];
          auto target = groupVertexMap[eit->target()];
          auto oldEdge = eit->property();

          // Reconstruct edge in local group roadmap
          GroupLocalPlanType newEdge(localGrm);

          newEdge.SetLPLabel(oldEdge.GetLPLabel());

          auto& edgeDescriptors = oldEdge.GetEdgeDescriptors();
          for(size_t i = 0; i < group->GetRobots().size(); i++) {
            auto oldEd = edgeDescriptors[i];

            if(oldEd.source() != MAX_UINT and oldEd.target() != MAX_UINT) {
              auto source = vertexMaps[i][oldEd.source()];
              auto target = vertexMaps[i][oldEd.target()];
              GroupLocalPlanType::ED ed(source,target);
              newEdge.SetEdge(group->GetRobots()[i],ed);
            }
            else {
              auto edge = (*oldEdge.GetEdge(i));
              newEdge.SetEdge(i,std::move(edge));
            }
          }
        
          newEdge.SetWeight(oldEdge.GetWeight());
          newEdge.SetTimeSteps(oldEdge.GetTimeSteps());
          // Copy intermediates
          std::vector<GroupCfgType> intermediates;
          for(auto cfg : oldEdge.GetIntermediates()) {
            auto newCfg = cfg;
            newCfg.SetGroupRoadmap(localGrm);
            intermediates.push_back(newCfg);
          }
          newEdge.SetIntermediates(intermediates);

          localGrm->AddEdge(source,target,newEdge);
        }
      }
    }
  }

  std::unordered_map<Robot*,Cfg> startCfgs;
  std::unordered_map<Robot*,Cfg> endCfgs;

  // Grab start and end cfgs
  for(auto kv : transition.explicitPaths) {
    auto robot = kv.first;
    auto cfg = kv.second[0];
    startCfgs[robot] = cfg;
    cfg = kv.second.back();
    endCfgs[robot] = cfg;
  }

  // Update start state
  for(auto kv : start) {
    auto group = kv.first;
    auto grm = this->GetMPSolution()->GetGroupRoadmap(group);
    // Set mode formations
    grm->SetAllFormationsInactive();
    for(auto f : _startModeMap[group]->formations) {
      grm->SetFormationActive(f);
    }

    GroupCfgType gcfg(grm);
    for(auto robot : group->GetRobots()) {
      gcfg.SetRobotCfg(robot,std::move(startCfgs[robot]));
    }

    auto vid = grm->AddVertex(gcfg);

    start[group] = std::make_pair(grm,vid);

    grm->SetAllFormationsInactive();
  }

  // Update end state 
  for(auto kv : end) {
    auto group = kv.first;
    auto grm = this->GetMPSolution()->GetGroupRoadmap(group);
    // Set mode formations
    grm->SetAllFormationsInactive();
    for(auto f : _endModeMap[group]->formations) {
      grm->SetFormationActive(f);
    }

    GroupCfgType gcfg(grm);
    for(auto robot : group->GetRobots()) {
      gcfg.SetRobotCfg(robot,std::move(endCfgs[robot]));
    }

    auto vid = grm->AddVertex(gcfg);
    end[group] = std::make_pair(grm,vid);

    grm->SetAllFormationsInactive();
  }

  // Add the start state to the grounded vertices graph
  auto tail = AddStateToGroundedHypergraphLocal(start,_startModeMap);
  std::cout << "SaveInteraction.. " << std::endl;
  std::cout << "Tail: " ;
  for(auto v : tail) {
    m_exitVertices.insert(v);
    std::cout << v << " ";
  }
  std::cout << std::endl;

  // Add the end state to the grounded vertices graph
  auto head = AddStateToGroundedHypergraphLocal(end,_endModeMap);
  std::cout << "Head: " ;
  for(auto v : head) {
    m_entryVertices.insert(v);
    std::cout << v << " ";
  }
  std::cout << std::endl;

  // Make sure transition does not already exist in graph
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  std::cout << "found: " << gh->GetHID(tail,head) << std::endl;
  if(gh->GetHID(tail,head) == MAX_UINT) {
    // Save transition in hypergraph
    gh->AddTransition(tail,head,transition);
    std::cout << _hid << ": " << tail << " --->>> " << head << std::endl;
    std::cout << _hid << ": " ;
    for(auto t : tail) {
      std::cout << gh->GetVertex(t).first->GetVertex(gh->GetVertex(t).second).PrettyPrint() << ", ";
    }
    std::cout << " --->>> " ;
    for(auto h : head) {
      std::cout << gh->GetVertex(h).first->GetVertex(gh->GetVertex(h).second).PrettyPrint() << ", ";
    }
    std::cout << std::endl;

    m_groundedTransitionMap[_hid].insert(std::make_pair(tail,head));
    VID goalGVID = MAX_UINT;
    VID activeGVID = MAX_UINT;
    for(auto t : tail) {
      if(m_debug) {
        std::cout << "Goal GVID: " << m_goalGroundedVIDs << std::endl;
        std::cout << t << ": " << gh->GetVertex(t).first->GetVertex(gh->GetVertex(t).second).PrettyPrint() << std::endl;
      }
      if(m_goalGroundedVIDs.count(t)) {
        goalGVID = t;
      }
      else {
        activeGVID = t;
      }
    }
    std::cout << "goal gvid: " << goalGVID << ", active gvid: " << activeGVID << std::endl; 
    if(goalGVID != MAX_UINT and activeGVID != MAX_UINT) {
      RobotGroup* activeGroup = gh->GetVertex(activeGVID).first->GetGroup();
      std::cout << "active group " << activeGroup->GetLabel() << std::endl;
      for(auto kv : m_modeTransitionHypergraph.GetHyperarcMap()) {
        if(kv.second.head.size() == 1 and kv.second.tail.size() == 1 and 
            m_modeTransitionHypergraph.GetVertex(*(kv.second.head.begin())).property->robotGroup == activeGroup) {
          std::set<VID> activeGVIDTailSet = {activeGVID};
          std::set<VID> activeGVIDHeadSet = {activeGVID};
          std::cout << "insert " << activeGVIDTailSet << std::endl;
          m_groundedTransitionMap[kv.first].insert(std::make_pair(activeGVIDTailSet,activeGVIDHeadSet));
        }
      }
    }
    else {
      std::cout << "insert " << _hid << ": " << tail << " --> " << head << std::endl;
      m_groundedTransitionMap[_hid].insert(std::make_pair(tail,head));
    }

    for(auto t : tail)
      m_relevantGVIDs.insert(t);
    for(auto h : head)
      m_relevantGVIDs.insert(h);
    std::cout << m_modeTransitionHypergraph.GetHyperarc(_hid).property.action.first->GetLabel() << ": "
              << tail << " --- " << _hid << " --->>> " << head << std::endl;
  }
  else {
    std::cout << "insert " << _hid << ": " << tail << " --> " << head << std::endl;
    m_groundedTransitionMap[_hid].insert(std::make_pair(tail,head));
    m_groundedTransitionMap[_hid].insert(std::make_pair(head,tail));
  }

  if(_interaction->IsReversible()) {

    for(auto v : tail) {
      m_entryVertices.insert(v);
    }
    for(auto v : head) {
      m_exitVertices.insert(v);
    }

    GroundedHypergraph::Transition reverse;

    // Reverse explicit paths
    for(auto kv : transition.explicitPaths) {
      auto path = kv.second;
      std::reverse(path.begin(), path.end());
      reverse.explicitPaths[kv.first] = path;
    }

    // Reverse implicit paths
    for(auto kv : transition.implicitPaths) {
      auto path = std::make_pair(kv.second.second.second,kv.second.second.first);
      reverse.implicitPaths[kv.first] = std::make_pair(kv.second.first,path);
    }

    // Reverse tasks
    for(auto stage : transition.taskSet) {
      std::vector<std::shared_ptr<GroupTask>> newTasks;
      for(auto task : stage) {
        auto newTask = std::shared_ptr<GroupTask>(new GroupTask(task->GetRobotGroup()));
        for(auto iter = task->begin(); iter != task->end(); iter++) {
          MPTask t(iter->GetRobot());

          auto start = iter->GetGoalConstraints()[0]->Clone();
          auto goal  = iter->GetStartConstraint()->Clone();
          if(iter->GetPathConstraints().size() > 0) {
            auto path  = iter->GetPathConstraints()[0]->Clone();
            t.AddPathConstraint(std::move(path));
          }
          t.SetStartConstraint(std::move(start));
          t.AddGoalConstraint(std::move(goal));

          newTask->AddTask(t);
        }
        newTasks.push_back(newTask);
      }
      reverse.taskSet.push_back(newTasks);
    }
    std::reverse(reverse.taskSet.begin(),reverse.taskSet.end());
    reverse.cost = transition.cost;

    // Save reverse transition in hypergraph
    if(gh->GetHID(head,tail) == MAX_UINT) {
      // Save transition in hypergraph
      gh->AddTransition(head,tail,reverse);
      m_groundedTransitionMap[_hid].insert(std::make_pair(head,tail));
      for(auto t : tail) 
        m_relevantGVIDs.insert(t);
      for(auto h : head)
        m_relevantGVIDs.insert(h);
      std::cout << head << " --- " << _hid << " --->>> " << tail << std::endl;
    }
  }
}

LazyModeGraph::VID
LazyModeGraph::
AddMode(Mode* _mode) {

  std::cout << "start adding mode: " << _mode->robotGroup->GetLabel() << std::endl;
  // Check if mode already exists
  for(const auto& mode : m_modes) {
    // Check if robot groups are the same
    if(mode->robotGroup != _mode->robotGroup) {
      continue;
    }

    // Check if both have same number of formations and constraints
    auto formations1 = mode->formations;
    auto formations2 = _mode->formations;

    if(formations1.size() != formations2.size()) {
      continue;
    }

    const auto& constraints1 = mode->constraints;
    const auto& constraints2 = _mode->constraints;

    if(constraints1.size() != constraints2.size()) {
      continue;
    }

    // Check if the formations are the same
    bool fMatch = true;
    for(auto f1 : formations1) {

      bool match = false;

      for(auto f2 : formations2) {
        if(!f1 or !f2) {
          if(f1 == f2) {
            match = true;
            break;
          }
        }
        else if(*f1 == *f2) {
          match = true;
          break;
        }
      }

      if(!match) {
        fMatch = false;
        break;
      }
    }

    if(!fMatch) {
      continue;
    }

    // Check if constraints are the same
    bool cMatch = true;
    for(const auto& c1 : constraints1) {
      
      bool match = false;

      auto b1 = dynamic_cast<BoundaryConstraint*>(c1.get());
  
      for(const auto& c2 : constraints2) {
        auto b2 = dynamic_cast<BoundaryConstraint*>(c2.get());

        if(*b1 == *b2) {
          match = true;
          break;
        }
      }

      if(!match) {
        cMatch = false;
        break;
      }
    }

    if(!cMatch) {
      continue;
    }

    // Found existing copy of mode already saved
    return m_modeHypergraph.GetVID(mode.get());
  }

  auto mode = std::unique_ptr<Mode>(_mode);
  m_modes.push_back(std::move(mode));

  std::cout << "Adding a new mode: " << m_modes.back().get()->robotGroup->GetLabel() << " ( ";
  return m_modeHypergraph.AddVertex(m_modes.back().get());
}

std::set<LazyModeGraph::VID>
LazyModeGraph::
AddStateToGroundedHypergraphLocal(const State& _state, std::unordered_map<RobotGroup*,Mode*> _modeMap) {

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  std::set<VID> vids;

  for(const auto& kv : _state) {
    auto mode = _modeMap[kv.first];
    auto mvid = m_modeTransitionHypergraph.GetVID(mode);


    auto gvid = gh->AddVertex(kv.second);
    m_modeTransitionGroundedVertices[mvid].insert(gvid);

    // Check if grounded vertex violates mode
    auto gcfg = kv.second.first->GetVertex(kv.second.second);
    for(auto& c : mode->constraints) {
      auto robot = c->GetRobot();
      auto cfg = gcfg.GetRobotCfg(robot);
      if(!c->Satisfied(cfg)) {
        std::cout << "constraint conflict: " << c->GetLabel() << std::endl;
        std::cout << "Boundary is: [ " ;
        auto boundary = c->GetBoundary();
        for(size_t i=0 ; i<boundary->GetDimension() ; i++) {
          std::cout << boundary->GetRange(i).min << ":" << boundary->GetRange(i).max << " ; " ;
        }
        std::cout << " ] " << std::endl;

        throw RunTimeException(WHERE) << robot->GetLabel() 
                                      << " violated mode constraints at : " 
                                      << cfg
                                      << std::endl;
      }
    }

    vids.insert(gvid);
  }
  
  return vids;
}



Cfg
LazyModeGraph::
CanReach(Interaction* _interaction, const RobotGroup* _activeRobot, const GroupCfgType _gcfg) {
  // std::cout << _activeRobot->GetLabel() << " grasp " << _gcfg.GetRobot(0)->GetLabel() << ": " << _gcfg.PrettyPrint() << std::endl;

  auto plan = this->GetPlan();
  auto coord = plan->GetCoordinator();
  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Set all robots to virtual
  for(auto kv : coord->GetInitialRobotGroups()) {
    for(auto robot : kv.first->GetRobots()) {
      robot->SetVirtual(true);
    }
  } 
  auto stages = _interaction->GetStages();
  std::unordered_map<Robot*,Transformation> eeFrames;
  auto pregraspConditions = _interaction->GetStageConditions(stages[1]);

  for(auto condition : pregraspConditions) {
    auto f = dynamic_cast<FormationCondition*>(as->GetCondition(condition));
    if(!f)
      continue;

    for(auto role : f->GetRoles()) {
      const auto& roleInfo = f->GetRoleInfo(role); 
      if(roleInfo.leader)
        continue;
      Transformation transform = -roleInfo.transformation;

      // auto objectMb = _object->GetRobot(0)->GetMultiBody();
      // auto objectBaseFrame = objectMb->GetBase()->GetWorldTransformation();

      auto cfg = _gcfg.GetRobotCfg(_gcfg.GetRobot(0));
      cfg.ConfigureRobot();
      auto objectBase = cfg.GetRobot()->GetMultiBody()->GetBase();
      auto objectBaseFrame = objectBase->GetWorldTransformation();

      auto frame = objectBaseFrame * transform;
      // std::cout << "object: " << _gcfg.GetRobot(0)->GetLabel() << ": " << objectBaseFrame << std::endl;

      auto refRobot = _activeRobot->GetRobot(0);
      // Check that refRobot is active
      if(!refRobot or refRobot->GetMultiBody()->IsPassive())
        continue;

      auto refBase = refRobot->GetMultiBody()->GetBase();
      auto refBaseTransformation = refBase->GetWorldTransformation();
      // std::cout << "refRobot: " << refRobot->GetLabel() << ": " << refBaseTransformation << std::endl;

      // Extract first three elements (x, y, z)
      double dx = refBaseTransformation.translation()[0] - objectBaseFrame.translation()[0];
      double dy = refBaseTransformation.translation()[1] - objectBaseFrame.translation()[1];
      double dz = refBaseTransformation.translation()[2] - objectBaseFrame.translation()[2];

      // Compute Euclidean distance
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      // Return false if distance exceeds threshold
      if(distance > m_robotRange)
        return Cfg(nullptr);
      

      // Current have to rotate base of ur5e bc of weird urdf stuff, but it messes up this calculation
      refBaseTransformation = Transformation(refBaseTransformation.translation());

      //auto translation = (-refBaseTransformation).rotation() * frame.translation() + (-refBaseTransformation).translation();
      auto translation = frame.translation() + (-refBaseTransformation).translation();
      auto rotation = (-refBaseTransformation).rotation() * frame.rotation();

      //auto translation = (-frame).rotation() * refBaseTransformation.translation() + (-frame).translation();
      //auto rotation = (-frame).rotation() * refBaseTransformation.rotation();

      auto eeFrame = Transformation(translation,rotation);
      // std::cout << "EEFrame: " << eeFrame << std::endl;
      eeFrames[refRobot] = eeFrame;
    }


  }

  auto label = _interaction->GetInteractionStrategyLabel();
  auto is = this->GetInteractionStrategyMethod(label);
  // bool canReach = true;
  for(auto kv : eeFrames) {

    auto _robot = kv.first;
    auto _transform = kv.second;

    // if(m_debug) {
    //   std::cout << "Computing IK for a UR5e." << std::endl;
    // }

    //TODO::Convert transform to individual ur_kin format
  
    auto translation = _transform.translation();
    auto orientation = _transform.rotation().matrix();
  
    double* T = new double[16];
    double q_sols[8*6];
    // Point
    T[3] = translation[0];
    T[7] = translation[1];
    T[11] = translation[2];

    // Orientation matrix
    T[0] = orientation[0][0];
    T[1] = orientation[0][1];
    T[2] = orientation[0][2];

    T[4] = orientation[1][0];
    T[5] = orientation[1][1];
    T[6] = orientation[1][2];

    T[8] = orientation[2][0];
    T[9] = orientation[2][1];
    T[10] = orientation[2][2];

    T[12] = 0;
    T[13] = 0;
    T[14] = 0;
    T[15] = 1;


    // if(m_debug) {
    //   for(int i=0;i<4;i++) {
    //     for(int j=i*4;j<(i+1)*4;j++)
    //       printf("%1.3f ", T[j]);
    //     printf("\n");
    //   }
    // }

    auto num_sols = ur_kinematics::inverse(T, q_sols);

    // if(m_debug) {
    //   for(int i=0;i<num_sols;i++) 
    //     printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
    //         q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
    // }
    
    //TODO::Validity check cfg and if invalid try the next solution

    auto vc = this->GetMPLibrary()->GetValidityChecker("pqp_solid");

    for(int i = 0; i < num_sols; i++) {

      std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

      data[0] =  q_sols[i*6+2]/PI; 
      data[1] =  0;
      data[2] =  q_sols[i*6+1]/PI;
      data[3] =  q_sols[i*6+0]/PI;
      data[4] =  q_sols[i*6+3]/PI;
      data[5] =  q_sols[i*6+4]/PI;
      data[6] =  q_sols[i*6+5]/PI;

      //TODO::Find base joint index
      const size_t baseJointIndex = 3;
      data[baseJointIndex] += _robot->GetBaseRotation();

      for(size_t j = 0; j < data.size(); j++) {
        if(j == 1)
          continue;

        if(data[j] > 1) {
          data[j] = -2 + data[j];
        }
      }

      Cfg cfg(_robot);
      cfg.SetData(data);
      // std::cout << "Validating: " << cfg.PrettyPrint() << std::endl;
      if(vc->IsValid(cfg,this->GetNameAndLabel())) {
        for(auto kv : coord->GetInitialRobotGroups()) {
          for(auto robot : kv.first->GetRobots()) {
            robot->SetVirtual(false);
          }
        } 
       return cfg;
      }
    }
  }

  for(auto kv : coord->GetInitialRobotGroups()) {
    for(auto robot : kv.first->GetRobots()) {
      robot->SetVirtual(false);
    }
  } 
  return Cfg(nullptr);

}


double
LazyModeGraph::
ComputeEdgeCost(const VID _start, const VID _goal) {

  // Run a dijkstra search backwards through hypergraph as if it was a graph

  // Get graph representation grounded hypergraph
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  std::cout << "Grounded Hypergraph:" << std::endl;
  gh->Print();
  auto g = gh->GetReverseGraph();


  // Setup dijkstra functions
  SSSPTerminationCriterion<GroundedHypergraph::GH::GraphType> termination(
    [this](typename GroundedHypergraph::GH::GraphType::vertex_iterator& _vi,
           const SSSPOutput<typename GroundedHypergraph::GH::GraphType>& _sssp) {
    const auto& vertex = _vi->property();
    auto grm = vertex.first;
    if(!grm)
      return SSSPTermination::Continue;

    auto group = grm->GetGroup();

    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive())
        return SSSPTermination::Continue;
    }
    std::cout << "end branch" << std::endl;
    return SSSPTermination::EndBranch;
  });

  SSSPPathWeightFunction<GroundedHypergraph::GH::GraphType> weight(
    [this,g](typename GroundedHypergraph::GH::GraphType::adj_edge_iterator& _ei,
           const double _sourceDistance,
           const double _targetDistance) {

    auto target = _ei->target();
    auto grm = g->GetVertex(target).first;

    bool hasObject = false;

    if(grm) {
      auto group = grm->GetGroup();
      for(auto robot : group->GetRobots()) {
        if(robot->GetMultiBody()->IsPassive()) {
          hasObject = true;
          break;
        }
      }
    }

    if(!hasObject and grm)
      return std::numeric_limits<double>::infinity();

    auto groundedHA = _ei->property();
    double edgeWeight = groundedHA.cost;

    //TODO::Decide if this is what we want
    //edgeWeight = std::min(1.,edgeWeight);

    double newDistance = _sourceDistance + edgeWeight;
    return newDistance;
  });

  // Run dijkstra backwards from sink
  std::vector<size_t> starts = {_start};
  auto output = DijkstraSSSP(g,starts,weight,termination);
  if(m_debug) {
    std::cout << "cost map" << std::endl;
    for(auto kv : output.distance) { 
      std::cout << kv.first << ": " << kv.second << std::endl;
    }
  }

  return output.distance[_goal];
}


bool
LazyModeGraph::
ContainsSolution(std::set<VID>& _startVIDs) {

  // Run a dijkstra search backwards through hypergraph as if it was a graph

  // Get graph representation grounded hypergraph
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  auto g = gh->GetReverseGraph();

  // Setup dijkstra functions
  SSSPTerminationCriterion<GroundedHypergraph::GH::GraphType> termination(
    [this](typename GroundedHypergraph::GH::GraphType::vertex_iterator& _vi,
           const SSSPOutput<typename GroundedHypergraph::GH::GraphType>& _sssp) {
    const auto& vertex = _vi->property();
    auto grm = vertex.first;
    if(!grm)
      return SSSPTermination::Continue;

    auto group = grm->GetGroup();

    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive())
        return SSSPTermination::Continue;
    }
    std::cout << "end branch" << std::endl;
    return SSSPTermination::EndBranch;
  });

  SSSPPathWeightFunction<GroundedHypergraph::GH::GraphType> weight(
    [this,g](typename GroundedHypergraph::GH::GraphType::adj_edge_iterator& _ei,
           const double _sourceDistance,
           const double _targetDistance) {

    auto target = _ei->target();
    auto grm = g->GetVertex(target).first;

    bool hasObject = false;

    if(grm) {
      auto group = grm->GetGroup();
      for(auto robot : group->GetRobots()) {
        if(robot->GetMultiBody()->IsPassive()) {
          hasObject = true;
          break;
        }
      }
    }

    if(!hasObject and grm)
      return std::numeric_limits<double>::infinity();

    auto groundedHA = _ei->property();
    double edgeWeight = groundedHA.cost;

    //TODO::Decide if this is what we want
    //edgeWeight = std::min(1.,edgeWeight);

    double newDistance = _sourceDistance + edgeWeight;
    return newDistance;
  });

  // Run dijkstra backwards from sink
  std::vector<size_t> starts = {1};
  auto output = DijkstraSSSP(g,starts,weight,termination);

  //TODO::Change this to instead seach over robots/objects which are specified in the goal constraints
  // Ensure each object start vertex can reach the sink
  for(auto v : _startVIDs) {
    auto vertex = gh->GetVertex(v);
    auto group = vertex.first->GetGroup();
    std::cout << "\nChecking existance of goal for " << group->GetLabel() << std::endl;
    //bool passive = false;

    /*
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive()) {
        passive = true;
        break;
      }
    }
    */

    bool goalRobot = false;

    // Check if robot is specified in the goal constraints
    auto decomp = this->GetPlan()->GetDecomposition();

    // 
    for(auto st : decomp->GetGroupMotionTasks()) {
      auto task = st->GetGroupMotionTask();
      int numRobots = (*task).Size();
      // Count the number of robots composing the task
      // for(auto t : *task) {
      //   numRobots++;
      // }
      if(m_debug)
        std::cout << "===== Task: " << task->GetLabel() << " Numrobots: " << numRobots << std::endl; 
      
      // Check if the task composes in multiple robots
      if (numRobots > 1) {
        if(m_debug)
          std::cout << "multiple" << std::endl;
        for(auto t : *task) {
          if(m_debug)
            std::cout << "Task Robot Member: " << t.GetRobot()->GetLabel() << std::endl;
          for(auto robot: group->GetRobots()) {
            if(m_debug)
              std::cout << "Start Robot Member: " << robot->GetLabel() << std::endl;
            if(robot == t.GetRobot())
              numRobots--;
              if(m_debug)
                std::cout << numRobots << std::endl;
              if (numRobots==0){
                if(m_debug)
                  std::cout << "Found every robots. " << task->GetLabel() << " is a goal robot" << std::endl;
                goalRobot = true;
                break;
              }
          }
        }
      } else {
        if(m_debug)
          std::cout << "single" << std::endl;
        for(auto t : *task) {
          if(m_debug)
            std::cout << "Task Robot Member: " << t.GetRobot()->GetLabel() << std::endl;
          for(auto robot: group->GetRobots()) {
            if(m_debug)
              std::cout << "Start Robot Member: " << robot->GetLabel() << std::endl;
            if(robot == t.GetRobot()) {
              if(m_debug)
                std::cout << "Found every robots. " << robot->GetLabel() << " is a goal robot" << std::endl;
              goalRobot = true;
              break;
            }
          }
          if (goalRobot)
            break;
        }
      }

      if (goalRobot)
        break;
    }

    // for(auto st : decomp->GetGroupMotionTasks()) {
    //   auto task = st->GetGroupMotionTask();
    //   std::cout << "===== Task: " << task->GetLabel() << std::endl; 
    //   for(auto t : *task) {
    //     std::cout << "Task Robot Member: " << t.GetRobot()->GetLabel() << std::endl;

    //     std::cout << " Start Robot Group Member: " ;
    //     for(auto robot : group->GetRobots()) {
    //       std::cout << robot->GetLabel() << " " ;
    //     }
    //     std::cout << std::endl;


    //     for(auto robot : group->GetRobots()) {
    //       if(robot == t.GetRobot()) {
    //         std::cout << "found goal robot: " << robot->GetLabel() << std::endl;
    //         goalRobot = true;
    //         break;
    //       }
    //     }
    //     if(goalRobot)
    //       break;
    //   }
    //   if(goalRobot){
    //     std::cout << "Break" << std::endl;
    //     break;
    //   }
    // }

    if(!goalRobot) {
      continue;
    }

    //if(!passive)
    //  continue;

    auto iter = output.distance.find(v);
    if(iter == output.distance.end()) {
      // TODO::TEMP::Delete
      //gh->Print();
      std::cout << "START VID " << v << std::endl;
      //throw RunTimeException(WHERE) << "NO SOLUTION FOR " << group->GetLabel();
      std::cout << "NO SOLUTION FOR " << v << ": " << group->GetLabel() << std::endl << std::endl;
      return false;
    }
  }
  std::cout << "FOUND A SOLUTION" << std::endl;

  return true;
}






LazyModeGraph::ModeTransitionHypergraph&
LazyModeGraph::
GetModeTransitionHypergraph() {
  return m_modeTransitionHypergraph;
}

size_t
LazyModeGraph::
GetSourceModeTransitionVID() {
  return m_sourceModeTransitionVID;
}

size_t
LazyModeGraph::
GetSinkModeTransitionVID() {
  return m_sinkModeTransitionVID;
}

set<size_t>
LazyModeGraph::
GetIgnitionModeTransitionVIDs() {
  return m_ignitionModeTransitionVIDs;
}

set<size_t>
LazyModeGraph::
GetTerminationModeTransitionVIDs() {
  return m_terminationModeTransitionVIDs;
}

set<size_t>
LazyModeGraph::
GetActiveModeTransitionVIDs() {
  return m_activeModeTransitionVIDs;
}

set<size_t>
LazyModeGraph::
GetPassiveModeTransitionVIDs() {
  return m_passiveModeTransitionVIDs;
}

size_t
LazyModeGraph::
GetIgnitionMTHID() {
  return m_ignitionMTHID;
}

size_t
LazyModeGraph::
GetTerminationMTHID() {
  return m_terminationMTHID;
}

std::vector<size_t>
LazyModeGraph::
GetMotionHistory() {
  return m_motionHistory;
}

std::unordered_map<VID,std::set<VID>>
LazyModeGraph::
GetGroundedVIDMap() {
  return m_modeTransitionGroundedVertices;
}

// void 
// LazyModeGraph::
// SetModeExtendedHypergraph(ModeExtendedHypergraph _modeExtendedHypergraph) {
//   std::cout << "Set Mode Extended Hypergraph" << std::endl;
//   m_modeExtendedHypergraph = _modeExtendedHypergraph;
// }


void
LazyModeGraph::
SetExtraTaskSpaceCandidates(std::set<std::pair<size_t,size_t>> _nmc) {
  m_extraTaskSpaceCandidates = _nmc;
}


void
LazyModeGraph::
SetTaskSpaceImprovementCandidates(std::unordered_map<size_t,std::set<size_t>> _tsic) {
  m_taskSpaceImprovementCandidates = _tsic;
}

void
LazyModeGraph::
SetMotionConstraints(std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> _mcSet) {
  m_motionConstraintSet = _mcSet;
}

std::unordered_map<size_t,std::set<size_t>>
LazyModeGraph::
GetGeometricConstraints() {
  return m_geometricConstraintSet;
}

std::unordered_map<size_t,std::set<size_t>>
LazyModeGraph::
GetGeometricConstraints2() {
  return m_geometricConstraintSet2;
}

std::unordered_map<VID,std::set<VID>>
LazyModeGraph::
GetNonMonotonicConstraints() {
  return m_nonMonotonicConstraintSet;
}

std::set<std::vector<size_t>>
LazyModeGraph::
GetTaskOrderConstraints() {
  return m_taskOrderConstraintSet;
}

size_t
LazyModeGraph::
GetReplanSource() {
  return m_replanSource;
}

std::set<size_t>
LazyModeGraph::
GetInteractionConstraints() {
  return m_interactionConstraintSet;
}

std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>>
LazyModeGraph::
GetMotionConstraints() {
  return m_motionConstraintSet;
}

void
LazyModeGraph::
ClearConstraints() {
  m_motionConstraintSet.clear();
  m_geometricConstraintSet.clear();
  m_prevGeometricConstraintSet.clear();
}


/*----------------------------------------------------------------------------*/


istream&
operator>>(istream& _is, const LazyModeGraph::Mode* _mode) {
  return _is;
}

ostream&
operator<<(ostream& _os, const LazyModeGraph::Mode* _mode) {
  return _os;
}

// istream&
// operator>>(istream& _is, const LazyModeGraph::ReversibleAction _ra) {
//   return _is;
// }

// ostream&
// operator<<(ostream& _os, const LazyModeGraph::ReversibleAction _ra) {
//   return _os;
// }

//istream&
//operator>>(istream& _is, const LazyModeGraph::GroundedVertex _vertex) {
//  return _is;
//}

//ostream&
//operator<<(ostream& _os, const LazyModeGraph::GroundedVertex _vertex) {
//  return _os;
//}

istream&
operator>>(istream& _is, const LazyModeGraph::Transition _t) {
  return _is;
}

ostream&
operator<<(ostream& _os, const LazyModeGraph::Transition _t) {
  return _os;
}

istream&
operator>>(istream& _is, const LazyModeGraph::TransitionSwitch _t) {
  return _is;
}

ostream&
operator<<(ostream& _os, const LazyModeGraph::TransitionSwitch _t) {
  return _os;
}
