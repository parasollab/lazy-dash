#include "GreedyModeQuery.h"

#include "TMPLibrary/StateGraphs/GroundedHypergraph.h"
#include "TMPLibrary/StateGraphs/ModeGraph.h"

/*------------------------------ Construction ------------------------------*/

GreedyModeQuery::
GreedyModeQuery() {
  this->SetName("GreedyModeQuery");
}

GreedyModeQuery::
GreedyModeQuery(XMLNode& _node) : ModeQuery(_node) {
  this->SetName("GreedyModeQuery");
}

GreedyModeQuery::
~GreedyModeQuery() { }

/*------------------------- Task Evaluator Interface -----------------------*/

void
GreedyModeQuery::
Initialize() {
  std::cout << "Initialize GreedyModeQuery" << std::endl;
  ModeQuery::Initialize();
  m_historyGraph = std::unique_ptr<HistoryGraph>(new HistoryGraph());
  m_heuristicValues.clear();
}

/*------------------------- Task Evaluator Functions -----------------------*/

bool
GreedyModeQuery::
Run(Plan* _plan) {
  std::cout << "RUN GreedyModeQuery" << std::endl;
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  // MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  MethodTimer* mt_ind = new MethodTimer(stats,this->GetNameAndLabel() + "::Run_GreedyModeQuery" + std::to_string(m_counter));

  if(!_plan)
    _plan = this->GetPlan();

  // Prevent from losing data
  if(m_counter == 0) {
    Initialize();
  }
  this->ComputeHeuristicValues();

  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto mtg = mg->GetModeTransitionHypergraph();

  ModeExtendedVertex source;
  source.modeVID = mg->GetSourceModeTransitionVID();
  auto sourceVID = m_modeExtendedHypergraph.AddVertex(source);
  m_vertexMap[source.modeVID].insert(sourceVID);


  std::cout << "HI3" << std::endl;
  this->m_goalVID = DFS(sourceVID);
  std::cout << "HI4" << std::endl;
  
  if(this->m_goalVID == INVALID_VID) {
    m_counter++;
    delete mt_ind;
    return false;
  }

  std::cout << "printing task plan" << std::endl;
  ConvertToTaskPlan(_plan);

  stats->SetStat(this->GetNameAndLabel() + "::ModeExtendedHypergraphVertexSize", m_modeExtendedHypergraph.Size());
  stats->SetStat(this->GetNameAndLabel() + "::ModeExtendedHypergraphEgdeSize", m_modeExtendedHypergraph.EdgeSize());
  ClearData();

  m_counter++;
  delete mt_ind;
  return true;

}

/*----------------------------- Helper Functions ---------------------------*/

GreedyModeQuery::VID
GreedyModeQuery::
DFS(const VID _source) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  // MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() + "::Run_GreedyModeQuery::DFS" + std::to_string(m_counter));
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());

  std::cout << "RUN DFS " << m_counter << " " << m_replanSource <<std::endl;
  // Prevent from losing data
  std::vector<VID> priority_queue;
  

  if(m_interactionConstraintSet.size() > 0 or m_geometricConstraintSet.size() > 0 or m_taskOrderConstraintSet.size() > 0) {
    std::cout << "Clear previous data" << std::endl;
    ClearData();

    ModeExtendedVertex source;
    source.modeVID = mg->GetSourceModeTransitionVID();
    auto sourceVID = m_modeExtendedHypergraph.AddVertex(source);
    m_vertexMap[source.modeVID].insert(sourceVID);
  }
  std::cout << "!1" << std::endl;


  m_historyGraph = std::unique_ptr<HistoryGraph>(new HistoryGraph());
  ActionHistory root;
  auto rootVID = m_historyGraph->AddVertex(root);
  priority_queue = {rootVID};

  std::cout << "!2" << std::endl;
  
  while(!priority_queue.empty()) {

    VID v = priority_queue.back();
    // std::cout << "PQ pop out: " << v << std::endl;

    priority_queue.pop_back();

    std::cout << "!3" << std::endl;

    auto end = Termination(v);
    if(end != INVALID_VID) {
      // std::cout << "Termination: " << end << std::endl;
      delete mt;
      return end;
    }

    std::cout << "!4" << std::endl;
    auto frontier = Frontier(v);
    std::cout << "==== Done Computing Frontiers ====" << std::endl;
    // std::cout << "Frontiers are: " ;
    for(auto u : frontier) {
      // std::cout << u << " " ;
      priority_queue.push_back(u);
    }
  }

  delete mt;
  return INVALID_VID;
}


GreedyModeQuery::VID
GreedyModeQuery::
Termination(const VID _vid) {
  // std::cout << "Check Termination " << _vid << std::endl;
  
  auto history = m_historyGraph->GetVertex(_vid);
  // std::cout << history << std::endl;
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto mtg = mg->GetModeTransitionHypergraph();
  auto terminations = mg->GetTerminationModeTransitionVIDs();
  auto sink = mg->GetSinkModeTransitionVID();
  auto sinkHID = mg->GetTerminationMTHID();


  if(history.empty())
    return INVALID_VID;

  std::set<size_t> modeExtendedFrontiers{0};
  std::set<size_t> modeExtendedTerminationVIDs;
  // std::cout << "(" << _vid << ") History: " ;
  for(auto iter = history.begin() ; iter != history.end() ; ++iter) {
    size_t hid = *iter;
    auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
    // std::cout << hyperarc.property.modeHID << ", " ;
    for(auto t : hyperarc.tail) {
      modeExtendedFrontiers.erase(t);
    }
    for(auto h : hyperarc.head) {
      modeExtendedFrontiers.insert(h);
    }
  }
  // std::cout << std::endl;

  std::set<size_t> modeTransitionFrontiers;
  std::set<size_t> modeExtendedFrontiersPassive = {};
  for(auto f : modeExtendedFrontiers) {
    auto vertex = this->m_modeExtendedHypergraph.GetVertex(f);
    auto modeVID = vertex.property.modeVID;
    
    modeTransitionFrontiers.insert(modeVID);
    if(mtg.GetVertex(modeVID).property->robotGroup->IsPassive()) 
      modeExtendedFrontiersPassive.insert(f);
  }

  bool sinkConnect = true;
  int existNum = 0;
  for(auto t : terminations) {
    if(modeTransitionFrontiers.count(t)) {
      // std::cout << t << " exist in frontiers" << std::endl;
      existNum += 1;
      continue;
    }
    sinkConnect = false;
  } 
  std::cout << existNum << "/" << terminations.size() << " satisfied " << std::endl;
  if(sinkConnect) {
    std::cout << "SINK IS VALID" << std::endl;

    ModeExtendedVertex vertex;
    vertex.modeVID = sink;
    vertex.history = history;
    auto sinkMEVID = this->m_modeExtendedHypergraph.AddVertex(vertex);
    std::cout << "Created a sink me vid " << sinkMEVID << std::endl;

    TransitionSwitch transition;
    transition.modeHID = sinkHID;
    transition.cost = MAX_DBL;
    auto newHID = this->m_modeExtendedHypergraph.AddHyperarc({sinkMEVID},modeExtendedFrontiersPassive,transition,false);
    std::cout << "Created a sink me hid " << newHID << std::endl;

    VID sourceVID = 0; // This should be passed in or set somewhere instead of assumed to be 0
    m_mbt.vertexParentMap[sourceVID] = MAX_INT;

    history.insert(newHID);
    for(auto hid : history) {
      auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
      for(auto vid : hyperarc.head) {
        m_mbt.vertexParentMap[vid] = hid;
      }
      std::cout << hid << ": " << hyperarc.tail << " --> " << hyperarc.head << std::endl;

      auto vid = *(hyperarc.tail.begin());
      m_mbt.hyperarcParentMap[hid] = vid;
      m_prevHistoryGraphGoalVID = sinkMEVID;
    }

    m_mbt.weightMap[newHID] = 0.1;
    std::cout << "Found sink" << std::endl;

    return sinkMEVID;
  }

  return INVALID_VID;
}

std::vector<GreedyModeQuery::VID>
GreedyModeQuery::
Frontier(const VID _vid) {

  std::cout << "!" << std::endl;

  auto lmg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto mtg = lmg->GetModeTransitionHypergraph();
  auto goalMTVIDs = lmg->GetTerminationModeTransitionVIDs();
  auto source = lmg->GetSourceModeTransitionVID();
  auto sink = lmg->GetSinkModeTransitionVID();
  auto gc = GetGeometricConstraints();
  auto gc2 = GetGeometricConstraints2();
  auto nmc = GetNonMonotonicConstraints();
  auto tc = GetTaskOrderConstraints();
  auto ic = GetInteractionConstraints();

  auto history = m_historyGraph->GetVertex(_vid);

  
  // ////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::set<VID> frontierVIDs;
  std::set<VID> internalVIDs;
  std::vector<HID> mtVidHistory;
  for(auto hid : history) {
    auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
    mtVidHistory.push_back(hyperarc.property.modeHID);
    // std::cout << hyperarc.property.modeHID << std::endl;
    for(auto v : hyperarc.head) {
      if(internalVIDs.count(v))
        continue;

      frontierVIDs.insert(v);
    }

    for(auto v : hyperarc.tail) {
      if(frontierVIDs.erase(v))
        continue;

      internalVIDs.insert(v);
    }
  }

  if(history.empty()) {
    frontierVIDs.insert(0);
  }

  std::cout << "!" << std::endl;

  std::set<RobotGroup*> used;
  std::unordered_map<RobotGroup*,std::set<size_t>> robotMap;
  for(auto kv : mtg.GetVertexMap()) {
    if(kv.first == source or kv.first == sink)
      continue;
    auto g = mtg.GetVertex(kv.first).property->robotGroup;
    if(g->IsActive())
      continue;
    used.insert(g);
    for(auto kv2 : mtg.GetVertexMap()) {
      if(kv.first == source or kv.first == sink)
        continue;
      auto g2 = mtg.GetVertex(kv2.first).property->robotGroup;
      if(g == g2) {
        robotMap[g2].insert(kv2.first);
      }
    }
  }

  // ////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::set<size_t> restrictedHIDs;
  for(size_t hid : ic) {
    std::cout << "---!!---" << std::endl;
    restrictedHIDs.insert(hid);
  }
  std::cout << "!" << std::endl;

  // // ////////////////////////////////////////////////////////////////////////////////////////////////////////
  // if(tc.size() > 0 and history.size() > 1) {
  //   std::cout << "---!---" << std::endl;
  //   for(auto c : tc) {
  //     size_t prev = c[0];
  //     size_t next = c[1];
  //     // TODO: Remove this?
  //     auto it = std::find(mtVidHistory.begin(), mtVidHistory.end(), prev);
  //     if (it != mtVidHistory.end())
  //       restrictedHIDs.insert(next);
  //   }
  // }
  // std::cout << "!" << std::endl;

  // ////////////////////////////////////////////////////////////////////////////////////////////////////////
  // for(size_t vid : goalMTVIDs) {
  //   auto r = *(mtg.GetVertex(vid).property->robotGroup->GetRobots().begin());
  //   m_objectLevelMap[r] = 1;
  // }
  // for(auto c : nmc) {
  //   auto intermediateGoals = c.second;
  //   for(size_t ig : intermediateGoals) {
  //     auto r = mtg.GetVertex(ig).property->robotGroup->GetRobots()[0];
  //     m_objectLevelMap[r] = 2;
  //   }
  // }

  // if(nmc.size() > 0 and history.size() > 0) {
  //   std::cout << "MT history" << std::endl;
  //   for(size_t hid : mtVidHistory) {
  //     auto ha = mtg.GetHyperarc(hid);
  //     std::cout << ha.tail << " -->> " << ha.head << std::endl;
  //   }
  //   for(auto c : nmc) {
  //     size_t goal = c.first;
  //     auto intermediateGoals = c.second;
  //     std::cout << intermediateGoals << " must come before " << goal << std::endl;

  //     bool exist = false;
  //     for(auto hid : mtVidHistory) {
  //       auto head = mtg.GetHyperarc(hid).head;
  //       for(auto ig : intermediateGoals) {
  //         if(head.count(ig)) {
  //           std::cout << "intermediate goal " << ig << " exist" << std::endl;
  //           exist = true;
  //         }
  //       }
  //     }

  //     if(!exist) {
  //       auto incomings = mtg.GetIncomingHyperarcs(goal);
  //       for(auto i : incomings) {
  //         restrictedHIDs.insert(i);
  //         auto ha = mtg.GetHyperarc(i);
  //         std::cout << i << " is restricted: " << ha.tail << " -->> " << ha.head << std::endl;
  //       }
  //     }
  //   }
  // }
  // std::cout << "!" << std::endl;

  std::cout << "\nObject MT VIDs" << std::endl;
  for(auto kv : robotMap) {
    std::cout << kv.first->GetLabel() << ": " << kv.second << std::endl;
  }

  std::cout << "\nMT History: " << std::endl;
  std::set<size_t> frontierMTVIDs{};
  for(auto hid : mtVidHistory) {
    auto ts = mtg.GetHyperarc(hid).tail;
    auto hs = mtg.GetHyperarc(hid).head;
    for(auto t : ts) 
      frontierMTVIDs.erase(t);
    for(auto h : hs) 
      frontierMTVIDs.insert(h);
  }
  

  for(auto hid : mtVidHistory) {
    auto hyperarc = mtg.GetHyperarc(hid);
    
    std::cout << "[HID " << hid << "]: ";
    if(hyperarc.tail.size() == 1 and hyperarc.head.size() == 1) {
      std::cout << "Move: ";
    }
    else {
      std::cout << "Transition: ";
    } 
    for(auto vid : hyperarc.tail) {
      std::string label = " source ";
      if(vid != lmg->GetSourceModeTransitionVID())
        label = mtg.GetVertex(vid).property->robotGroup->GetLabel();
      std::cout << " (" << vid << ": " << label << ") ";
    }
    std::cout << " ----->>>> ";
    for(auto vid : hyperarc.head) {
      std::string label = " sink ";
      if(vid != lmg->GetSinkModeTransitionVID())
        label = mtg.GetVertex(vid).property->robotGroup->GetLabel();
      std::cout << " (" << vid << ": " << label << ") ";
    }
    std::cout << std::endl;
  }


  std::cout << "\nMT Frontiers: " << frontierMTVIDs << std::endl;

  // ////////////////////////////////////////////////////////////////////////////////////////////////////////

  for(size_t vid : goalMTVIDs) {
    auto r = *(mtg.GetVertex(vid).property->robotGroup->GetRobots().begin());
    m_objectLevelMap[r] = 1;
  }

  // if(gc2.size()) {
  //   size_t a = 90;
  //   std::set<size_t> b = {108};
  //   gc.insert(std::make_pair(a,b));

  //   a = 95;
  //   b = {96};
  //   gc.insert(std::make_pair(a,b));
  // }

  if(gc.size()) {
    for(auto kv : m_objectLevelMap) {
      m_objectLevelMap[kv.first] = 3;
    }

    std::cout << "\nGeo Const: " << std::endl;
    for(auto c : gc) {
      std::cout << c.first << ": " << c.second << std::endl;
      // std::cout << mtg.GetVertex(c.first).property->robotGroup->GetLabel() << ": " ;
      // for(auto a : c.second) {
      //   std::cout << mtg.GetVertex(a).property->robotGroup->GetLabel() << " " ;
      // }
      // std::cout << std::endl;
    }

    for(auto c : gc) {
      // bool clear = true;
      // for(auto e : c.second) {
      //   if(frontierMTVIDs.count(e)) {
      //     clear = false;
      //     break;
      //   }
      // }
      // if(!clear) {
      //   restrictedHIDs.insert(c.first);
      //   std::cout << c.second << " in the frontier." << std::endl;
      //   std::cout << c.first << " is restricted: " << mtg.GetHyperarc(c.first).tail << " -->> " << mtg.GetHyperarc(c.first).head << std::endl;
      // }
      
      for(size_t cVid : c.second) {
        size_t goal = MAX_UINT;
        for(auto kv : robotMap) {
          if(kv.second.count(cVid) and !lmg->GetIgnitionModeTransitionVIDs().count(cVid)) {
            goal = cVid;
            break;            
          }
        }
        if(goal == MAX_UINT)
          continue;

        size_t intermediateGoal = MAX_UINT;
        for(auto v : mtg.GetHyperarc(c.first).tail) {
          if(lmg->GetIgnitionModeTransitionVIDs().count(v))
            continue;
          for(auto kv : robotMap) {
            if(kv.second.count(v)) {
              std::cout << v << " must come before " << goal << std::endl;
              intermediateGoal = v;
            }
          }
        }
        for(auto v : mtg.GetHyperarc(c.first).head) {
          if(lmg->GetIgnitionModeTransitionVIDs().count(v))
            continue;
          for(auto kv : robotMap) {
            if(kv.second.count(v)) {
              std::cout << v << " must come before " << goal << std::endl;
              intermediateGoal = v;
            }
          }
        }

        if(intermediateGoal == MAX_UINT)
          continue;

        bool exist = false;
        for(auto hid : mtVidHistory) {
          auto head = mtg.GetHyperarc(hid).head;
          if(head.count(intermediateGoal)) {
            std::cout << "intermediate goal " << intermediateGoal << " exist" << std::endl;
            exist = true;
          }
        }

        if(!exist) {
          auto incomings = mtg.GetIncomingHyperarcs(goal);
          for(auto i : incomings) {
            restrictedHIDs.insert(i);
            auto ha = mtg.GetHyperarc(i);
            std::cout << i << " is restricted: " << ha.tail << " -->> " << ha.head << std::endl;
          }
        }
      }
    }
  }


  // ////////////////////////////////////////////////////////////////////////////////////////////////////////

  // for(size_t vid : goalMTVIDs) {
  //   auto r = *(mtg.GetVertex(vid).property->robotGroup->GetRobots().begin());
  //   m_objectLevelMap[r] = 1;
  // }
  // for(auto c : gc) {
  //   auto intermediateGoal = c.first;
  //   auto r = mtg.GetVertex(intermediateGoal).property->robotGroup->GetRobots()[0];
  //   std::cout << "gc robot: " << r->GetLabel() << std::endl;
  //   m_objectLevelMap[r] = 2;
  // }

  // for(size_t vid : goalMTVIDs) {
  //   auto r = *(mtg.GetVertex(vid).property->robotGroup->GetRobots().begin());
  //   m_objectLevelMap[r] = 1;
  // }
  for(auto c : gc2) {
    auto objectToMove = c.first;
    auto r = mtg.GetVertex(objectToMove).property->robotGroup->GetRobots()[0];
    m_objectLevelMap[r] = 3;
  }

  if(gc2.size() > 0 and history.size() > 0) {
    std::cout << "\nNon Monotonic Const: " << std::endl;
    for(size_t hid : mtVidHistory) {
      auto ha = mtg.GetHyperarc(hid);
    }
    for(auto c : gc2) {
      auto intermediateGoal = c.first;
      auto goal = c.second;
      std::cout << intermediateGoal << " must come before " << goal << std::endl;
      

      std::cout << mtg.GetVertex(c.first).property->robotGroup->GetLabel() << ": " ;
      for(auto a : c.second) {
        std::cout << mtg.GetVertex(a).property->robotGroup->GetLabel() << " " ;
      }
      std::cout << std::endl;

      bool exist = false;
      for(auto hid : mtVidHistory) {
        auto head = mtg.GetHyperarc(hid).head;
        if(head.count(intermediateGoal)) {
          std::cout << "intermediate goal " << intermediateGoal << " exist" << std::endl;
          exist = true;
        }
      }

      if(!exist) {
        auto incomings = mtg.GetIncomingHyperarcs(*goal.begin());
        for(auto i : incomings) {
          restrictedHIDs.insert(i);
          auto ha = mtg.GetHyperarc(i);
          std::cout << i << " is restricted: " << ha.tail << " -->> " << ha.head << std::endl;
        }
      }
    }
  }
  std::cout << "!" << std::endl;



  // if(gc.size() > 0 and history.size() > 0) {
  //   std::cout << "MT history" << std::endl;
  //   for(size_t hid : mtVidHistory) {
  //     auto ha = mtg.GetHyperarc(hid);
  //     std::cout << ha.tail << " -->> " << ha.head << std::endl;
  //   }
  //   for(auto c : gc) {
  //     size_t goal = *c.second.begin();
  //     auto intermediateGoal = c.first;
  //     std::cout << intermediateGoal << " must come before " << goal << std::endl;

  //     bool exist = false;
  //     for(auto hid : mtVidHistory) {
  //       auto head = mtg.GetHyperarc(hid).head;
  //       if(head.count(intermediateGoal)) {
  //         std::cout << "intermediate goal " << intermediateGoal << " exist" << std::endl;
  //         exist = true;
  //       }
  //     }

  //     if(!exist) {
  //       auto incomings = mtg.GetIncomingHyperarcs(goal);
  //       for(auto i : incomings) {
  //         restrictedHIDs.insert(i);
  //         auto ha = mtg.GetHyperarc(i);
  //         std::cout << i << " is restricted: " << ha.tail << " -->> " << ha.head << std::endl;
  //       }
  //     }
  //   }
  // }
  std::cout << "!" << std::endl;



  // bool satisfied = true;
  // std::set<Robot*> unSatisfiedPassiveRobots;
  // // Filter out restricted vid --> hid expansion
  // if(gc.size()) {
  //   // std::cout << "gc size: " << gc.size() << std::endl;
  //   for(auto c : gc) {
  //     satisfied = false;
  //     // std::cout << "-------------------" << std::endl;
  //     std::cout << c.first << " " << c.second << std::endl;
  //     Robot* first = mtg.GetVertex(c.first).property->robotGroup->GetRobot(0);
  //     Robot* second = mtg.GetVertex(*c.second.begin()).property->robotGroup->GetRobot(0);
  //     std::cout << first->GetLabel() << " must come before " << second->GetLabel() << std::endl;
  //     // Check if the first passive robot has reached the goal config.
  //     for(auto v : frontierVIDs) {
  //       size_t mtvid = this->m_modeExtendedHypergraph.GetVertex(v).property.modeVID;
        
  //       // std::cout << "Check frontier vid " << v << " (mtvid " << mtvid << ")" << std::endl;
  //       if(goalMTVIDs.count(mtvid)) {
  //         RobotGroup* goalGroup = mtg.GetVertex(mtvid).property->robotGroup;
  //         Robot* passive = nullptr;
  //         for(auto r : goalGroup->GetRobots()) {
  //           if(r->GetMultiBody()->IsPassive()) {
  //             passive = r;
  //             break;
  //           }
  //         }
  //         if(passive == nullptr)
  //           continue;

  //         // std::cout << passive->GetLabel() << " has reached the goal" << std::endl;
  //         if(passive == first) {
  //           // std::cout << passive->GetLabel() << " satisfied" << std::endl;
  //           satisfied = true;
  //           break;
  //         }
  //       }
  //     }
  //     if(!satisfied) {
  //       // std::cout << second->GetLabel() << " should not be expaneded." << std::endl;
  //       unSatisfiedPassiveRobots.insert(second);
  //     }
  //   }
  // }
  // std::cout << "!" << std::endl;


  auto filteredFrontierVIDs = frontierVIDs;
  // if(unSatisfiedPassiveRobots.size() > 0) {
  //   std::cout << "Need to satisfy the geometric constraints. Filter out frontiers." << std::endl;
  //   std::cout << "Current frontier VIDs: " << frontierVIDs << std::endl;
  //   for(auto f : frontierVIDs) {
  //     size_t mtvid = this->m_modeExtendedHypergraph.GetVertex(f).property.modeVID;
  //     if(source == mtvid or sink == mtvid)
  //       continue;
  //     RobotGroup* group = mtg.GetVertex(mtvid).property->robotGroup;
  //     Robot* passive = nullptr;
  //     for(auto r : group->GetRobots()) {
  //       if(r->GetMultiBody()->IsPassive()) {
  //         passive = r;
  //         break;
  //       }
  //     }
  //     if(passive == nullptr)
  //       continue;

  //     if(unSatisfiedPassiveRobots.count(passive)) {
  //       std::cout << "We cannot expand unsatisfied passive object " << passive->GetLabel() << std::endl;
  //       filteredFrontierVIDs.erase(f);
  //       std::cout << "Pop out the unsatisfied passive frontier " << f << std::endl;
  //     }
  //   }
  // }



  // std::set<VID> unsatisfiedMTVIDs;
  // if(nmc.size()) {
  //   std::cout << "Non Monotonic Constraints " << std::endl;
  //   for(auto hid : mtVidHistory) {
  //     auto hyperarc = mtg.GetHyperarc(hid);
  //     std::cout << hyperarc.tail << " -->> " << hyperarc.head << std::endl;
  //   }

  //   for(auto c : nmc) {
  //     size_t goal = c.first;
  //     auto intermediateGoals = c.second;
  //     std::cout << intermediateGoals << " must come before " << goal << std::endl;

  //     size_t existHID = MAX_UINT;
  //     for(auto hid : mtVidHistory) {
  //       auto hyperarc = mtg.GetHyperarc(hid);
  //       if(hyperarc.head.count(goal)) {
  //         std::cout << goal << " exists" << std::endl;
  //         existHID = hid;
  //         break;
  //       }
  //     }

  //     bool satisfied = false;
  //     if(existHID != MAX_UINT) {
  //       for(auto hid : mtVidHistory) {
  //         auto hyperarc = mtg.GetHyperarc(hid);
  //         for(auto h : hyperarc.head) {
  //           if(intermediateGoals.count(h)) {
  //             std::cout << h << " satisfied " << std::endl;
  //             satisfied = true;
  //             break;
  //           }
  //         }
  //         if(satisfied) {
  //           break;
  //         }
  //         if(hid == existHID)
  //           break;
  //       }
  //     }
      

  //     if(existHID !=MAX_UINT and !satisfied) {
  //       std::cout << intermediateGoals << " not exist before " << goal << std::endl;
  //       return {};
  //     }
  //   }
  // }

  // if(unsatisfiedMTVIDs.size() > 0) {
  //   for(auto f : frontierVIDs) {
  //     size_t mtvid = this->m_modeExtendedHypergraph.GetVertex(f).property.modeVID;
  //     if(source == mtvid or sink == mtvid)
  //       continue;
      
  //     if(unsatisfiedMTVIDs.count(mtvid)) {
  //       std::cout << "Pre frontier VIDs: " << frontierVIDs << std::endl;
  //       std::cout << "Pop out the unsatisfied passive frontier " << f << std::endl;
  //       filteredFrontierVIDs.erase(f);
  //       std::cout << "Post frontier VIDs: " << frontierVIDs << std::endl;
  //     }
  //   }
  // }


  // ////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>> transNeighbors;
  std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>> motionNeighbors;

  auto quantum = BuildQuantumFrontier(filteredFrontierVIDs,history,restrictedHIDs);

  for(auto v : quantum) {
    ExpandVertex(_vid,v,quantum,history,transNeighbors,motionNeighbors,restrictedHIDs);
  }

  std::vector<VID> neighbors;
  while(!motionNeighbors.empty()) {
    neighbors.push_back(motionNeighbors.top().second);
    motionNeighbors.pop();
  }

  while(!transNeighbors.empty()) {
    auto neighbor = transNeighbors.top();
    transNeighbors.pop();
    neighbors.push_back(neighbor.second);
  }

  

  return neighbors;
}

std::set<GreedyModeQuery::VID>
GreedyModeQuery::
BuildQuantumFrontier(std::set<VID> _frontier, ActionHistory _history, std::set<size_t> _restrictedHIDs) {
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto g = mg->GetModeTransitionHypergraph();
  auto sink = mg->GetSinkModeTransitionVID();
  
  // auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());

  // Add all possible motion transitions to the history (ignoring conflicts)
  std::set<VID> newVertices;
  for(auto v : _frontier) {
    auto fs = this->HyperpathForwardStar(v,&(this->m_modeExtendedHypergraph), _restrictedHIDs, true);
    for(auto hid : fs) {
      
      // Check if hyperarc is a motion transition
      auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);

      // Check that this isn't the head isn't the sink vertex
      if(hyperarc.head.size() == 1 and 
         this->m_modeExtendedHypergraph.GetVertexType(*(hyperarc.head.begin())).modeVID == sink)
        continue;

      // TODO::This is a pretty loose check - should go back to mode 
      //       graph (task space hypergraph) and check if they're the same
      auto modeTransitionHyperarc = g.GetHyperarc(hyperarc.property.modeHID);
      if(!(modeTransitionHyperarc.head.size() == 1 and modeTransitionHyperarc.tail.size() == 1)) {
        continue;
      }

      for(auto vid : hyperarc.head) {
        newVertices.insert(vid);
        this->HyperpathForwardStar(vid,&(this->m_modeExtendedHypergraph), _restrictedHIDs, true);
      }

      _history.insert(hid);
    }
  }

  // Compute the new frontier
  for(auto v : newVertices) {
    _frontier.insert(v);
  }

  return _frontier;
}

void
GreedyModeQuery::
ExpandVertex(const VID _source, const VID _vid, const std::set<VID> _frontier, 
             const ActionHistory _history, 
             std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>>& _transNeighbors,
             std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>>& _motionNeighbors,
             std::set<size_t> _interactionConstraints) {
  // Node: _source is a history vertex and _vid is a modeExtended vertex
  //       _frontier is the frontiers of the history contained in the _source vertex

  // auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto g = mg->GetModeTransitionHypergraph();
  // auto source = mg->GetSourceModeTransitionVID();
  auto sink = mg->GetSinkModeTransitionVID();
  auto goal = mg->GetTerminationModeTransitionVIDs();

  auto fs = this->HyperpathForwardStar(_vid,&(this->m_modeExtendedHypergraph),_interactionConstraints,true);
  for(auto hid : fs) {
    // std::cout << "** check " << hid << std::endl;
    
    // Check if hyperarc is a motion transition (and not that the head isn't the sink vertex)
    auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
    // if(m_interactionConstraintSet.count(hyperarc.property.modeHID)) {
    //   std::cout << "hid " << hyperarc.property.modeHID << " is restricted for expansion" << std::endl;
    //   continue;
    // }
    
    auto modeTransitionHyperarc = g.GetHyperarc(hyperarc.property.modeHID);
    // TODO::This is a pretty loose check - should go back to mode 
    //       graph (task space hypergraph) and check if they're the same
    if(modeTransitionHyperarc.head.size() == 1 and modeTransitionHyperarc.tail.size() == 1 and 
       this->m_modeExtendedHypergraph.GetVertex(*(hyperarc.head.begin())).property.modeVID != sink) {
      continue;
    }



    // Check if hyperarc is adjacent to history (all tail vertices along frontier)
    bool adjacent = true;
    for(auto vid : hyperarc.tail) {
      if(!_frontier.count(vid))
        adjacent = false;
    }
    if(!adjacent) {
      // std::cout << "b" << std::endl;
      continue;
    }

    // Create new history
    auto newHistory = _history;

    // Add last hyperarc
    newHistory.insert(hid);

    // Add any predecessor hyperarcs (computed in the quantum frontier)
    for(auto v : hyperarc.tail) {
      for(auto h : this->m_modeExtendedHypergraph.GetIncomingHyperarcs(v)) {
        newHistory.insert(h);
      }
    }
    // Check if hyperarc is compatible with full history
    if(!IsValidHistory(newHistory)) {
      std::cout << "Invalid history " << newHistory << std::endl;
      continue;
    }
    
    // ////////////////////////////////////////////////////////////////////
    auto terminations = mg->GetTerminationModeTransitionVIDs();
    double max_val = -std::numeric_limits<double>::infinity(); 
    for (const auto& kv : m_costToGoMap) {
        auto value = kv.second;
        if (value > max_val) {
            max_val = value;
        }
    }
    
    double motionHeuristic = 0;
    for(auto v : hyperarc.head) {
      auto gvid = this->m_modeExtendedHypergraph.GetVertexType(v).modeVID;
      motionHeuristic = std::max(motionHeuristic,m_costToGoMap[gvid]);
    }
    motionHeuristic /= (max_val);

    std::set<size_t> modeExtendedFrontiers{0};
    for(auto iter = newHistory.begin() ; iter != newHistory.end() ; ++iter) {
      size_t hid = *iter;
      auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
      for(auto t : hyperarc.tail) {
        modeExtendedFrontiers.erase(t);
      }
      for(auto h : hyperarc.head) {
        modeExtendedFrontiers.insert(h);
      }
    }

    std::set<size_t> modeTransitionFrontiers;
    for(auto f : modeExtendedFrontiers) {
      auto vertex = this->m_modeExtendedHypergraph.GetVertex(f);
      auto modeVID = vertex.property.modeVID;
      modeTransitionFrontiers.insert(modeVID);
    }

    double existNum = 0.;
    for(auto t : terminations) {
      if(modeTransitionFrontiers.count(t)) {
        existNum += 1.;
        continue;
      }
    } 
    double taskHeuristic = 1. - existNum / terminations.size();
    // taskHeuristic = 0.;
    motionHeuristic = 0.;
    double heuristic = motionHeuristic + taskHeuristic;




    double tailMotionHeuristic = 0;
    for(auto v : hyperarc.tail) {
      auto gvid = this->m_modeExtendedHypergraph.GetVertexType(v).modeVID;
      tailMotionHeuristic = std::max(tailMotionHeuristic,m_costToGoMap[gvid]);
    }
    tailMotionHeuristic /= (max_val);
    
    modeExtendedFrontiers = {0};
    for(auto iter = _history.begin() ; iter != _history.end() ; ++iter) {
      size_t hid = *iter;
      auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
      for(auto t : hyperarc.tail) {
        modeExtendedFrontiers.erase(t);
      }
      for(auto h : hyperarc.head) {
        modeExtendedFrontiers.insert(h);
      }
    }

    modeTransitionFrontiers.clear();
    for(auto f : modeExtendedFrontiers) {
      auto vertex = this->m_modeExtendedHypergraph.GetVertex(f);
      auto modeVID = vertex.property.modeVID;
      modeTransitionFrontiers.insert(modeVID);
    }

    existNum = 0.;
    for(auto t : terminations) {
      if(modeTransitionFrontiers.count(t)) {
        existNum += 1.;
        continue;
      }
    } 
    double tailTaskHeuristic = 1. - existNum / terminations.size();
    // tailTaskHeuristic = 0.;
    tailMotionHeuristic = 0.;
    double tailHeuristic = tailMotionHeuristic + tailTaskHeuristic;
    // ////////////////////////////////////////////////////////////////////


    
    // Check that we are not being dumb
    if(tailHeuristic < 0.1 and tailHeuristic < heuristic and _source > 1) {
      if(m_debug) {
        std::cout << "NOT BEING DUMB" << std::endl;
        std::cout << "Source: " << _source << std::endl;
        std::cout << newHistory << std::endl;
      }
      continue;
    }


    if(m_historyGraph->IsVertex(newHistory)) {
      // std::cout << "Vertex already exist" << std::endl;
      continue;
    }
    auto vid = m_historyGraph->AddVertex(newHistory);
    m_historyGraph->AddEdge(_source,vid,hid);

    if(hyperarc.head.size() == 1 and hyperarc.tail.size() == 1) {
      _motionNeighbors.emplace(heuristic,vid);
    }
    else {
      _transNeighbors.emplace(heuristic,vid);
    }

    if(m_debug) {
      std::cout << "Adding TE hid " << hyperarc.hid << ": (" << hyperarc.tail << ")->(" << hyperarc.head << ")" << std::endl;
      auto groundedHyperarc = g.GetHyperarc(hyperarc.property.modeHID);
      std::cout << "\tGrounded hid " << groundedHyperarc.hid << ": (" << groundedHyperarc.tail << ")->(" << groundedHyperarc.head << ")" << std::endl;
      std::cout << " With heuristic cost: " << heuristic << std::endl;
    }
  }
}

bool
GreedyModeQuery::
IsValidHistory(const ActionHistory& _history) {
  std::set<VID> outgoing;

  for(auto hid : _history) {
    auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
    for(auto vid : hyperarc.tail) {
      if(m_hyperarcConstraintTails[hid].count(vid)) {
        continue;
      }
      if(outgoing.count(vid)) {
        return false;
      }

      outgoing.insert(vid);
    }
  }

  return true;
}

void
GreedyModeQuery::
ClearData() {
  m_vertexMap.clear();
  m_hyperarcMap.clear();
  m_partiallyExtendedHyperarcs.clear();
  m_blockedHyperarcs.clear();
  m_blockingMap.clear();
  m_computedFS.clear();
  m_modeExtendedHypergraph = ModeExtendedHypergraph();
  m_historyGraph = std::unique_ptr<HistoryGraph>(new HistoryGraph());
  std::cout << "Cleared" << std::endl;
}

// Constraint Getter
std::unordered_map<size_t,std::set<size_t>>
GreedyModeQuery::
GetGeometricConstraints() {
  return m_geometricConstraintSet;
}

std::unordered_map<size_t,std::set<size_t>>
GreedyModeQuery::
GetGeometricConstraints2() {
  return m_geometricConstraintSet2;
}

std::unordered_map<VID,std::set<VID>>
GreedyModeQuery::
GetNonMonotonicConstraints() {
  return m_nonMonotonicConstraintSet;
}

std::set<std::vector<size_t>>
GreedyModeQuery::
GetTaskOrderConstraints() {
  return m_taskOrderConstraintSet;
}

std::set<size_t>
GreedyModeQuery::
GetInteractionConstraints() {
  return m_interactionConstraintSet;
}

// Constraint Setters
void
GreedyModeQuery::
SetGeometricConstraints(std::unordered_map<size_t,std::set<size_t>> _gcSet) {
  m_geometricConstraintSet = _gcSet;
}

// Constraint Setters
void
GreedyModeQuery::
SetGeometricConstraints2(std::unordered_map<size_t,std::set<size_t>> _gcSet) {
  m_geometricConstraintSet2 = _gcSet;
}

void
GreedyModeQuery::
SetNonMonotonicConstraints(std::unordered_map<VID,std::set<VID>> _nmcSet) {
  m_nonMonotonicConstraintSet = _nmcSet;
}

void
GreedyModeQuery::
SetTaskOrderConstraints(std::set<std::vector<size_t>> _tcSet) {
  m_taskOrderConstraintSet = _tcSet;
}
void
GreedyModeQuery::
SetInteractionConstraints(std::set<HID> _icSet) {
  m_interactionConstraintSet = _icSet;
}



void
GreedyModeQuery::
SetReplanSource(HID _rs) {
  m_replanSource = _rs;
}