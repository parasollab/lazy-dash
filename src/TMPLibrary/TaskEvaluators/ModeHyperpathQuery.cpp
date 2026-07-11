#include "ModeHyperpathQuery.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/StateGraphs/GroundedHypergraph.h"
#include "TMPLibrary/StateGraphs/LazyModeGraph.h"

/*------------------------------ Construction ------------------------------*/

ModeHyperpathQuery::
ModeHyperpathQuery() {
  this->SetName("ModeHyperpathQuery");
}

ModeHyperpathQuery::
ModeHyperpathQuery(XMLNode& _node) : SubmodeQuery(_node) {
  this->SetName("ModeHyperpathQuery");
}

ModeHyperpathQuery::
~ModeHyperpathQuery() { }

/*------------------------- Task Evaluator Interface -----------------------*/

void
ModeHyperpathQuery::
Initialize() {
  SubmodeQuery::Initialize();
  m_historyGraph = std::unique_ptr<HistoryGraph>(new HistoryGraph());
  m_heuristicValues.clear();
}

/*------------------------- Task Evaluator Functions -----------------------*/

bool
ModeHyperpathQuery::
Run(Plan* _plan) {
  std::cout << "RUN ModeHyperpathQuery" << std::endl;
  Initialize();
  //m_historyGraph = std::unique_ptr<HistoryGraph>(new HistoryGraph());

  // auto plan = this->GetPlan();
  // MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  // MethodTimer mt_ind(stats,this->GetNameAndLabel() + "::Run_ModeHyperpathQuery" + std::to_string(counter));
  counter++;

  if(!_plan)
    _plan = this->GetPlan();


  // this->m_goalVID = DFS(sourceVID);
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto motionHistory = mg->GetMotionHistory();

  // std::cout << "goal vid: " << this->m_goalVID << std::endl;


  // if(this->m_goalVID == INVALID_VID) {
  //   delete mt_ind;
  //   return false;
  // }

  bool conversion = ConvertToPlan(motionHistory);

  if(!conversion) {
    std::cout << "conversion failed" << std::endl;
    return false;
  }

  // if(this->m_writeHypergraph) {
  //   m_actionExtendedHypergraph.Print(this->GetMPProblem()->GetBaseFilename() +
  //               "-action-extended.hyp");
  // }

  return true;
}

/*----------------------------- Helper Functions ---------------------------*/




bool
ModeHyperpathQuery::
ConvertToPlan(std::vector<size_t> _path) {
  std::cout << "convert to plan " << std::endl;
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConvertToPlan");

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());

  // std::cout << "\n\n===== Resulting hyperarc sequence ====\n\n" << std::endl;
  // for(auto hid : _path) {
  //   auto ha = gh->GetHyperarc(hid);
  //   std::cout << hid << ": " << ha.tail << " -->> " << ha.head << std::endl;
  // }
  
  // for(auto hid : _path) {
  //   auto ha = gh->GetHyperarc(hid);
  //   std::cout << "\n" << hid << ": " << ha.tail << " -->> " << ha.head << std::endl;
  //   auto paths = ha.property.explicitPaths;
  //   for(auto kv : paths) {
  //     for(auto path : kv.second) {
  //       std::cout << "\t" << kv.first->GetLabel() << " | " << path.PrettyPrint() << std::endl;
  //     }
  //   }
  // }
  
  m_actionExtendedHypergraphLocal = ActionExtendedHypergraph();

  ActionExtendedVertex source;
  source.groundedVID = 0;
  auto sourceVID = m_actionExtendedHypergraphLocal.AddVertex(source);
  m_vertexMap[source.groundedVID].insert(sourceVID);
  std::set<VID> frontiers{0};


  std::vector<HPElem> path;

  HPElem vHPElem;
  vHPElem.first = true;
  vHPElem.second = 0;
  path.push_back(vHPElem);

  std::set<size_t> VIDs{0};
  std::set<size_t> used;

  std::cout << "path size: " << _path.size() << std::endl;
  std::cout << "path: " ;
  for(auto e : _path) { 
    std::cout << e << " " ;
  }
  std::cout << std::endl;
  for(size_t hid : _path) {
    if(used.count(hid))
      continue;
    used.insert(hid);

    if(m_debug) {
      std::cout << "grounded hid " << hid << std::endl;
    }

    auto tail = gh->GetHyperarc(hid).tail;
    auto head = gh->GetHyperarc(hid).head;
    
    if(m_debug) {
      std::cout << "\t" << tail << " --->> " << head << std::endl;
    }

    for(size_t v : tail) {
      if(!VIDs.count(v)) {
        std::cout << "Not exist! " << std::endl;
        return false;
      }
      VIDs.erase(v);
    } 
    for(size_t v : head) {
      VIDs.insert(v);
    }
    

    std::set<VID> tailVIDs;
    for(auto kv : m_actionExtendedHypergraphLocal.GetVertexMap()) {
      auto gvid = kv.second.property.groundedVID;
      for(size_t v : tail) {
        if(v==gvid)
          tailVIDs.insert(kv.first);
      }
      if(tailVIDs.size() == tail.size())
        break;
    }

    std::set<VID> headVIDs;
    for(size_t v : head) {    
      ActionExtendedVertex aev;
      aev.groundedVID = v;

      std::set<size_t> history = {};

      for(size_t t : tailVIDs) {
        if(t == 0 and tail.size() == 1) {
          if(m_debug)
            std::cout << "tail is the source " << std::endl;
          break;
        }
        auto incomings = m_actionExtendedHypergraphLocal.GetIncomingHyperarcs(t);
        // auto groundedVID = m_actionExtendedHypergraphLocal.GetVertexType(t).groundedVID;

        if(incomings.size() > 1) {
          m_actionExtendedHypergraphLocal.Print();
          throw RunTimeException(WHERE) << "should never have more than one incoming hyperarc.";
        }
        if(m_debug)
          std::cout << "ae tail is: " << m_actionExtendedHypergraphLocal.GetHyperarc(*(incomings.begin())).tail << std::endl; 
        for(size_t tt : m_actionExtendedHypergraphLocal.GetHyperarc(*(incomings.begin())).tail) {
          for(size_t e : m_actionExtendedHypergraphLocal.GetVertex(t).property.history) {
            history.insert(e);
          }
          history.insert(*(m_actionExtendedHypergraphLocal.GetOutgoingHyperarcs(tt).begin()));
        }
      }

      aev.history = history;
      if(m_debug)
        std::cout << "history is: " << history << std::endl;
      size_t newVID = m_actionExtendedHypergraphLocal.AddVertex(aev);
      if(m_debug)
        std::cout << "history of " << newVID << ": " << history << std::endl;

      headVIDs.insert(newVID);
    }




    size_t newHID = m_actionExtendedHypergraphLocal.AddHyperarc(headVIDs,tailVIDs,hid);
    HPElem hHPElem;
    hHPElem.first = false;
    hHPElem.second = newHID;
    path.push_back(hHPElem);


    for(size_t v : headVIDs) {    
      HPElem vHPElem;
      vHPElem.first = true;
      vHPElem.second = v;
      path.push_back(vHPElem);
    }

    if(m_debug)
      std::cout << "previous frontier: " << frontiers << std::endl;
    
    if(m_debug)
      std::cout << newHID << ": " << tailVIDs << " -->> " << headVIDs << std::endl;

    for(size_t t : tailVIDs)
      frontiers.erase(t);

    for(size_t h : headVIDs) 
      frontiers.insert(h);
    
    if(m_debug) {
      std::cout << "post frontier: " << frontiers << std::endl;
      std::cout << "------------------------------" << std::endl;
    }
  }


  if(m_debug) {
    std::cout << "done constructing path" << std::endl;

    std::cout << "Full Path" << std::endl;
    std::vector<size_t> vids;
    std::vector<size_t> hids;
    for(auto e : path) {
      if(e.first) {
        std::cout << "v";
        vids.push_back(e.second);
      }
      else {
        std::cout << "h";
        hids.push_back(e.second);
      }

      std::cout << e.second << ", ";
    }

    std::cout << std::endl;

    std::cout << "End points" << std::endl;
    for(auto vid : vids) {
      auto v = m_actionExtendedHypergraphLocal.GetVertexType(vid).groundedVID;
      auto vertex = gh->GetVertex(v);
      auto grm = vertex.first;
      if(!grm)
        continue;
      auto gvid = vertex.second;
      auto gcfg = grm->GetVertex(gvid);

      std::cout << vid << ": " << grm->GetGroup()->GetLabel() 
                << ": " << gcfg.PrettyPrint() << std::endl;
    }

    for(auto kv : m_actionExtendedHypergraphLocal.GetVertexMap()) {
      auto vid = kv.first;
      auto vertex = kv.second;
      std::cout << "hist. of " << vid << ": " ;
      for(auto v : vertex.property.history) {
        std::cout << v << ", " ;
      }
      std::cout << std::endl;
    }

    // std::cout << "Hyperarc costs" << std::endl;
    // for(auto hid : hids) {
      
    //   auto h = m_actionExtendedHypergraphLocal.GetHyperarcType(hid); 
    //   auto groundedHA = gh->GetTransition(h);
    //   auto hyperarcWeight = groundedHA.cost;

    //   std::cout << hid << ":" << h << " : " << hyperarcWeight << std::endl;
    // }
  }

  // Initialize a decomposition
  auto top = std::shared_ptr<SemanticTask>(new SemanticTask());
  Decomposition* decomp = new Decomposition(top);


  // Map of initial tasks for each group and a flag indicating if it has 
  // been used as a precedence constraint for any tasks yet.
  std::unordered_map<RobotGroup*,std::pair<bool,SemanticTask*>> initialTasks;
  // Create initial tasks for each robot group
  auto hyperarc = m_actionExtendedHypergraphLocal.GetHyperarc(0);
  for(auto vid : hyperarc.head) {
    auto aev = m_actionExtendedHypergraphLocal.GetVertexType(vid);
    auto vertex = gh->GetVertex(aev.groundedVID);
    auto grm = vertex.first;
    auto group = grm->GetGroup();
    auto gcfg = grm->GetVertex(vertex.second);
    auto task = std::shared_ptr<GroupTask>(new GroupTask(group));
    for(auto robot : group->GetRobots()) {
      auto cfg = gcfg.GetRobotCfg(robot);
      auto start = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,cfg));
      auto goal = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,cfg));
      MPTask t(robot);
      t.SetStartConstraint(std::move(start));
      t.AddGoalConstraint(std::move(goal));
      task->AddTask(t);
    }
    const std::string label = group->GetLabel()+ ":InitialPath"; 
    auto st = std::shared_ptr<SemanticTask>(new SemanticTask(label,top.get(),decomp,
        SemanticTask::SubtaskRelation::AND,false,true,task));
    decomp->AddTask(st);
    initialTasks[group] = std::make_pair(false,st.get());
    m_vertexTasks[st.get()] = aev.groundedVID;
  }

  // Save last set of tasks in each hyperarc
  std::unordered_map<size_t,std::vector<SemanticTask*>> hyperarcTaskMap;
  std::unordered_map<size_t,std::vector<SemanticTask*>> vertexTaskMap;
  m_taskSets.clear();


  // Convert each hyperarc into a task
  for(auto elem : path) {
    // Skip the vertices
    if(elem.first)
      continue;

    if(m_debug) {
      std::cout << "\n\n==== Creating semantic tasks for hyperarc: " << elem.second << std::endl;
    }

    // Convert hyperarc to semantic tasks

    // Get grounded hypergraph hyperarc
    auto aeh = m_actionExtendedHypergraphLocal.GetHyperarc(elem.second);
    auto hyperarc = gh->GetTransition(aeh.property);



    // std::cout << "Path" << std::endl;
    // for(auto kv : hyperarc.explicitPaths) {
    //   for(auto cfg : kv.second) {
    //     std::cout << kv.first->GetLabel() << " | " << cfg.PrettyPrint() << std::endl; 
    //   }
    // }

    // std::cout << "Hyperarc Task Map" << std::endl;
    // for(auto kv : hyperarcTaskMap) {
    //   for(auto e : kv.second) {
    //     std::cout << kv.first << ": " << e->GetLabel() << std::endl;
    //   }
    // }
    // std::cout << "\n" << std::endl;
    // Grab tasks of preceeding hyperarcs
    std::vector<SemanticTask*> previousStage;
    std::set<VID> frontiersTempTail;
    std::set<VID> frontiersTempHead;
    std::set<VID> history;
    std::vector<HID> fullHistory;
    // Check each of the head vertices incoming hyperarcs 
    // (should only be one per vertex)

    for(auto vid : aeh.tail) {
      auto incoming = m_actionExtendedHypergraphLocal.GetIncomingHyperarcs(vid);
      if(incoming.empty())
        continue;
      // auto groundedVID = m_actionExtendedHypergraphLocal.GetVertexType(vid).groundedVID;
      
      if(incoming.size() > 1) {
        m_actionExtendedHypergraphLocal.Print();
        throw RunTimeException(WHERE) << "should never have more than one incoming hyperarc.";
      }
      // std::cout << *incoming.begin() << std::endl;
      // std::cout << m_actionExtendedHypergraphLocal.GetHyperarc(*incoming.begin()).tail << " --> " << m_actionExtendedHypergraphLocal.GetHyperarc(*incoming.begin()).head << std::endl;
      history.insert(*incoming.begin());
      for(size_t h : m_actionExtendedHypergraphLocal.GetHyperarc(*incoming.begin()).tail) {
        frontiersTempTail.insert(h);
      }
      for(size_t h : m_actionExtendedHypergraphLocal.GetHyperarc(*incoming.begin()).head) {
        frontiersTempHead.insert(h);
      }
    }
    if(frontiersTempTail.empty()) {
      frontiersTempTail.insert(0);
    }

    auto maxHid = *(std::max_element(history.begin(),history.end()));
    auto minHid = *(std::min_element(history.begin(),history.end()));
    for(auto i = minHid ; i <= maxHid ; ++i) {
      fullHistory.push_back(i);
    }
    // std::cout << fullHistory << std::endl;
    
    for(size_t hid : fullHistory) {
      std::cout << hid << std::endl;
      auto ha = m_actionExtendedHypergraphLocal.GetHyperarc(hid);
      auto tail = ha.tail;
      auto head = ha.head;
      // bool allExists = true;
      std::cout << "frontier: " << frontiersTempTail << std::endl;
      std::cout << hid << ": " << tail << " --> " << head << std::endl;
      // for(size_t t : tail) {
      //   if(!frontiersTempTail.count(t))
      //     allExists = false;
      // }
      // if(!allExists)
      //   throw RunTimeException(WHERE) << "task history not valid";
      
      for(size_t t : tail) {
        frontiersTempTail.erase(t);
      }
      for(size_t h : head) {
        frontiersTempTail.insert(h);
      }
    }

    std::set<VID> finalFrontiers;
    for(size_t f : frontiersTempTail) {
      if(frontiersTempHead.count(f)) {
        finalFrontiers.insert(f);
      }
    }
    std::cout << "final frontiers: " << finalFrontiers << std::endl;

    for(auto vid : finalFrontiers) {
      auto tasks = vertexTaskMap[vid];
      for(auto task : tasks) {
        previousStage.push_back(task);
      }
    }


    // m_actionExtendedHypergraphLocal.Print();

 
    if(m_debug) {
      std::cout << "Previous stage tasks" << std::endl;
      for(auto task : previousStage) {
        std::cout << "\t" << task->GetLabel() << std::endl;
      }
    }


    std::cout << "\n--- Generate task for gvid " << aeh.property << std::endl;


    // Convert to set of sequentially dependent semantic tasks
    auto& taskSet = hyperarc.taskSet;
    size_t counter = 0;
    for(auto stage : taskSet) {

      if(stage.empty()) {
        counter++;
        continue;
      }
  
      std::vector<SemanticTask*> currentStage;
      for(auto groupTask : stage) {
        // Create semantic task
        const std::string label = "GHID." + std::to_string(aeh.property) + ":AEHID." 
                            + std::to_string(aeh.hid) + ":::::::::"
                            + groupTask->GetRobotGroup()->GetLabel() + ":"
                            + "stage-" + std::to_string(counter) + ":"
                            + groupTask->GetLabel();
        auto task = std::shared_ptr<SemanticTask>(new SemanticTask(label,top.get(),decomp,
                 SemanticTask::SubtaskRelation::AND,false,true,groupTask));
        decomp->AddTask(task);

        std::cout << "\tTask Label: " << label << std::endl;

        std::cout << "\n\tStart and Goal" << std::endl;
        for(auto it = groupTask.get()->begin() ; it != groupTask.get()->end() ; ++it) {
          auto individualTask = *it;
          if(individualTask.GetRobot()->GetLabel().find("gantry_1") != std::string::npos) {
            std::cout << "\t" << individualTask.GetRobot()->GetLabel() << ": " << std::endl;
            std::cout << "\t\tStart: " << individualTask.GetStartConstraint()->GetBoundary()->GetCenter() << std::endl;
            std::cout << "\t\tGoal: " ;
            for(const auto& constraint : individualTask.GetGoalConstraints())
              std::cout << constraint.get()->GetBoundary()->GetCenter() << std::endl;
          }
        }

        // If this is active
        if(groupTask->GetRobotGroup()->Size() > 1 or !groupTask->GetRobotGroup()->IsPassive()) {
          m_hyperarcTasks[task.get()] = aeh.property;
        }
        else {
          for(auto vid : aeh.head) {
            auto groundedVID = m_actionExtendedHypergraphLocal.GetVertexType(vid).groundedVID;
            auto vertex = gh->GetVertex(groundedVID);
            if(vertex.first->GetGroup()->Size() > 1)
              continue;
            if(!vertex.first->GetGroup()->IsPassive())
              continue;
            //m_vertexTasks[task.get()] = vid;
            m_vertexTasks[task.get()] = groundedVID;
          }
        }

        if(m_debug) {
          std::cout << "\nCreating task: " << task->GetLabel() << std::endl;
        }

        for(auto f : hyperarc.taskFormations[groupTask.get()]) {
          task->AddFormation(f);
        }

        currentStage.push_back(task.get());

        if(m_debug) {
          std::cout << "\tWith dependencies:" << std::endl;
        }

        // Add stage depedencies
        for(auto previous : previousStage) {
          task->AddDependency(previous,SemanticTask::DependencyType::Completion);
          if(m_debug) {
            std::cout << "\t\t" << previous->GetLabel() << std::endl;
          }
        }

        // Check if group has been given initial dependency
        auto& init = initialTasks[groupTask->GetRobotGroup()];
        if(init.first or !init.second)
          continue;

        // If not, assign the dependency
        task->AddDependency(init.second,SemanticTask::DependencyType::Completion);
        init.first = true;
        if(m_debug) {
          std::cout << "\tAdd initial dependency: " << init.second->GetLabel() << std::endl;
        }
      }

      // std::cout << "\t\nPaths:" << std::endl;
      // auto paths = gh->GetHyperarc(aeh.property).property.explicitPaths;
      // for(auto kv : paths) {
      //   for(auto cfg : kv.second) {
      //     std::cout << "\t" << kv.first->GetLabel() << " | " << cfg.PrettyPrint() << std::endl;
      //   }
      // }

      // Iterate stage forward
      previousStage = currentStage;
      counter++;
    }
    
    // Save last stage in hyperarc task map
    for(size_t vid : aeh.head) {
      size_t gvid = m_actionExtendedHypergraphLocal.GetVertexType(vid).groundedVID;
      if(gvid == 1) {
        vertexTaskMap.clear();
        break;
      }
      auto group1 = gh->GetVertex(gvid).first->GetGroup();
      for(auto task : previousStage) {
        auto group2 = task->GetGroupMotionTask()->GetRobotGroup();
        if(group1==group2)
          vertexTaskMap[vid].push_back(task);
      }
    }
    
    hyperarcTaskMap[aeh.hid] = previousStage;
  }
  
  plan->SetDecomposition(decomp);
  // plan->SetCost(m_mbt.weightMap.at(m_goalVID));
  plan->SetCost(0.);
  std::cout << "Return path " << std::endl;
  return true;

}
















ModeHyperpathQuery::VID
ModeHyperpathQuery::
DFS(const VID _source) {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  // MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() + "::Run_ModeHyperpathQuery::DFS" + std::to_string(counter));

  m_historyGraph = std::unique_ptr<HistoryGraph>(new HistoryGraph());

  ActionHistory root;
  auto rootVID = m_historyGraph->AddVertex(root);

  std::vector<VID> priority_queue = {rootVID};
  int cnt = 0;
  // auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());
  
  while(!priority_queue.empty()) {
    cnt++;
    VID v = priority_queue.back();
    priority_queue.pop_back();

    if(this->m_debug) {
      //this->m_actionExtendedHypergraph.Print();
      std::cout << std::endl << std::endl << "Transition History" << std::endl;
      auto history = m_historyGraph->GetVertex(v);
      for(auto hid : history) {
        std::cout << hid << std::endl;
      }
    }

    auto end = Termination(v);
    if(end != INVALID_VID) {
      delete mt;
      return end;
    }

    auto frontier = Frontier(v);
    for(auto u : frontier) {
      priority_queue.push_back(u);
    }

    // if(m_debug and frontier.empty() ) {
    if(m_debug) {
      std::cout << "Frontier of " << v << " is empty" << std::endl;

      auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());
      std::vector<size_t> path = {v};
      size_t current = v;
      while(current != 0) {
        current = *(m_historyGraph->GetPredecessors(current).begin());
        path.push_back(current);
      }

      std::reverse(path.begin(),path.end());
      
      for(size_t i = 1; i < path.size(); i++) {
        auto previous = m_historyGraph->GetVertex(path[i-1]);
        auto next = m_historyGraph->GetVertex(path[i]);

        for(auto hid : next) {
          if(previous.count(hid))
            continue;

          auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
          auto groundedHyperarc = gh->GetHyperarc(hyperarc.property);
          std::cout << "HID: " << hid << std::endl;
          if(groundedHyperarc.tail.size() == 1 and groundedHyperarc.head.size() == 1) {
            std::cout << "\tMove: ";
          }
          else {
            std::cout << "\tTransition: ";
          } 
          for(auto vid : groundedHyperarc.tail) {
            auto vertex = gh->GetVertex(vid);
            if(vertex.first)
              std::cout << vid << "(" << vertex.first->GetGroup()->GetLabel() << "), ";
          }
          std::cout << " ----->>>> ";
          for(auto vid : groundedHyperarc.head) {
            auto vertex = gh->GetVertex(vid);
            if(vertex.first)
              std::cout << vid << "(" << vertex.first->GetGroup()->GetLabel() << "), ";
          }
          std::cout << std::endl;
        }
      }


      std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
      for(size_t i = 1; i < path.size(); i++) {
        auto previous = m_historyGraph->GetVertex(path[i-1]);
        auto next = m_historyGraph->GetVertex(path[i]);

        for(auto hid : next) {
          if(previous.count(hid))
            continue;

          auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
          std::cout << "HID: " << hid << std::endl;
          if(hyperarc.tail.size() == 1 and hyperarc.head.size() == 1) {
            std::cout << "\tMove: ";
          }
          else {
            std::cout << "\tTransition: ";
          } 
          for(auto vid : hyperarc.tail) {
            std::cout << vid << ", ";
          }
          std::cout << " ----->>>> ";
          for(auto vid : hyperarc.head) {
            std::cout << vid << ", ";
          }
          std::cout << std::endl;
        }
      }

    }
  }

  delete mt;
  return INVALID_VID;
}


ModeHyperpathQuery::VID
ModeHyperpathQuery::
Termination(const VID _vid) {
  auto history = m_historyGraph->GetVertex(_vid);
  if(history.empty())
    return INVALID_VID;

  auto iter = history.end();
  iter--;
  for(;iter != history.begin(); iter--) {
    auto hid = *iter;
    auto hyperarc = m_actionExtendedHypergraph.GetHyperarc(hid);

    for(auto aid : hyperarc.head) {
      auto vertex = this->m_actionExtendedHypergraph.GetVertex(aid);
      auto gid = vertex.property.groundedVID;
      if(gid == 1) {

        // TODO::Put this in its own function
        // Write to the m_mbt to replicate having performed an actualy hyperpath query
        VID sourceVID = 0; // This should be passed in or set somewhere instead of assumed to be 0
        m_mbt.vertexParentMap[sourceVID] = MAX_INT;
        //auto history = this->m_actionExtendedHypergraph.GetVertexType(m_goalVID).history;
        for(auto hid : history) {
          auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
          for(auto vid : hyperarc.head) {
            m_mbt.vertexParentMap[vid] = hid;
          }

          auto vid = *(hyperarc.tail.begin());
          m_mbt.hyperarcParentMap[hid] = vid;
        }
        //TODO:: Compute the actual cost for this
        m_mbt.weightMap[aid] = 0.1;

        return aid;
      }
    }
  }

  return INVALID_VID;
}

std::vector<ModeHyperpathQuery::VID>
ModeHyperpathQuery::
Frontier(const VID _vid) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  // MethodTimer mt(stats,this->GetNameAndLabel() + "::Run");
  MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() + "::Run_ModeHyperpathQuery::DFS::Frontier");

  auto history = m_historyGraph->GetVertex(_vid);
  std::set<VID> frontierVIDs;
  std::set<VID> internalVIDs;
  for(auto hid : history) {
    auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
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
  std::cout << "frontier vids: " << frontierVIDs << std::endl;

  std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>> transNeighbors;
  std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>> motionNeighbors;

  auto quantum = BuildQuantumFrontier(frontierVIDs,history);

  std::cout << "quantum: " << quantum << std::endl;
  for(auto v : quantum) {
    ExpandVertex(_vid,v,quantum,history,transNeighbors,motionNeighbors);
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

  delete mt;
  return neighbors;
}

std::set<ModeHyperpathQuery::VID>
ModeHyperpathQuery::
BuildQuantumFrontier(std::set<VID> _frontier, ActionHistory _history) {
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());

  // Add all possible motion transitions to the history (ignoring conflicts)
  std::set<VID> newVertices;
  std::cout << "============= frontier: " << _frontier << std::endl;
  for(auto v : _frontier) {
    auto fs = this->HyperpathForwardStar(v,&(this->m_actionExtendedHypergraph));
    std::cout << " L " << v << ": " << fs << std::endl;
    for(auto hid : fs) {
      std::cout << "forward star hid of vid " << v << " is:  " << hid << std::endl;
      
      // Check if hyperarc is a motion transition
      auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);

      std::cout << "hid's head is: " << hyperarc.head << std::endl;
      // Check that this isn't the head isn't the sink vertex
      if(hyperarc.head.size() == 1 and 
         this->m_actionExtendedHypergraph.GetVertexType(*(hyperarc.head.begin())).groundedVID == 1) {
        std::cout << "head vid is 1 " << std::endl;
        continue;
      }

      // TODO::This is a pretty loose check - should go back to mode 
      //       graph (task space hypergraph) and check if they're the same
      auto groundedHyperarc = gh->GetHyperarc(hyperarc.property);
      // Avoid the frontier that only do move transition to find actual progress 
      std::cout << "GHArc: " << groundedHyperarc.tail << " --->>> " << groundedHyperarc.head << std::endl;
      if(!(groundedHyperarc.head.size() == 1 and groundedHyperarc.tail.size() == 1)) {
        std::cout << "head and tail sizes are not 1 " << std::endl;
        continue;
      }

      std::cout << "Adding progress: " << std::endl;
      std::cout << "GHArc: " << groundedHyperarc.tail << " --->>> " << groundedHyperarc.head << std::endl;
      for(auto vid : hyperarc.head) {
        std::cout << "add new vtc: " << vid << std::endl;
        newVertices.insert(vid);
        this->HyperpathForwardStar(vid,&(this->m_actionExtendedHypergraph));
      }

      _history.insert(hid);
    }
  }
  std::cout << "new vertices: " << newVertices << std::endl;
  // Compute the new frontier
  for(auto v : newVertices) {
    _frontier.insert(v);
  }
  return _frontier;
}

void
ModeHyperpathQuery::
ExpandVertex(const VID _source, const VID _vid, const std::set<VID> _frontier, 
             const ActionHistory _history, 
             std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>>& _transNeighbors,
             std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>>& _motionNeighbors) {
  std::cout << ">>>>>> Expand Vertex " << std::endl;
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());

  auto fs = this->HyperpathForwardStar(_vid,&(this->m_actionExtendedHypergraph));
  std::cout << "fs: " << fs << std::endl;
  for(auto hid : fs) {
    std::cout << " L " << hid << std::endl;

    // Check if hyperarc is a motion transition (and not that the head isn't the sink vertex)
    auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
    auto groundedHyperarc = gh->GetHyperarc(hyperarc.property);
    // TODO::This is a pretty loose check - should go back to mode 
    //       graph (task space hypergraph) and check if they're the same
    std::cout << "GHArc: " << groundedHyperarc.tail << " --->>> " << groundedHyperarc.head << std::endl;
    // if(groundedHyperarc.head.size() == 1 and groundedHyperarc.tail.size() == 1 and 
    //    this->m_actionExtendedHypergraph.GetVertexType(*(hyperarc.head.begin())).groundedVID != 1) {
    //   std::cout << "111" << std::endl;
    //   // Already computed possible motion transitions in the quantum frontier
    //   continue;
    // }

    // Check if hyperarc is adjacent to history (all tail vertices along frontier)
    bool adjacent = true;
    for(auto vid : hyperarc.tail) {
      if(!_frontier.count(vid))
        adjacent = false;
    }
    if(!adjacent) {
      std::cout << "not adjacent" << std::endl;
      continue;
    }

    // Create new history
    auto newHistory = _history;
    // Add last hyperarc
    newHistory.insert(hid);

    // Add any predecessor hyperarcs (computed in the quantum frontier)
    for(auto v : hyperarc.tail) {
      for(auto h : this->m_actionExtendedHypergraph.GetIncomingHyperarcs(v)) {
        newHistory.insert(h);
      }
    }

    // Check if hyperarc is compatible with full history
    if(!IsValidHistory(newHistory)) {
      //if(newHistory.empty() and hid != 0)
      std::cout << "Not a valid history" << std::endl;
      continue;
    }

    // TODO::Find better way to compute this (with caching)
    double heuristic = 0;
    //for(auto vid : hyperarc.tail) {
    for(auto v : hyperarc.head) {
      //heuristic = std::max(heuristic,HyperpathHeuristic(vid));
      auto gvid = this->m_actionExtendedHypergraph.GetVertexType(v).groundedVID;
      heuristic = std::max(heuristic,m_costToGoMap[gvid]);
      //break;
    }

    double tailHeuristic = 0;
    for(auto v : hyperarc.tail) {
      //heuristic = std::max(heuristic,HyperpathHeuristic(vid));
      auto gvid = this->m_actionExtendedHypergraph.GetVertexType(v).groundedVID;
      tailHeuristic = std::max(tailHeuristic,m_costToGoMap[gvid]);
      //break;
    }

    // Check that we are not being dumb
    if(tailHeuristic < 0.1 and tailHeuristic < heuristic and _source > 1) {
      if(m_debug) {
        std::cout << "NOT BEING DUMB" << std::endl;
        std::cout << "Source: " << _source << std::endl;
        std::cout << newHistory << std::endl;
      }
      continue;
    }

    // If it is, add the extension to the graph
    if(m_historyGraph->IsVertex(newHistory)) {
      std::cout << "IIII " << std::endl;
      continue;
    }

    auto vid = m_historyGraph->AddVertex(newHistory);
    
    std::cout << "hid: " << hid << std::endl;
    std::cout << " L----->>> " << newHistory << std::endl;
    std::cout << " L----->>> " << _source << " -->>> " << vid << std::endl; 
    m_historyGraph->AddEdge(_source,vid,hid);

    m_heuristicValues[vid] = heuristic;
    // TODO::This is a pretty loose check - should go back to mode 
    //       graph (task space hypergraph) and check if they're the same
    if(hyperarc.head.size() == 1 and hyperarc.tail.size() == 1) {
      _motionNeighbors.emplace(heuristic,vid);
    }
    else {
      _transNeighbors.emplace(heuristic,vid);
    }

    if(m_debug) {
      auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_ghLabel).get());
      std::cout << "Adding TE hid " << hyperarc.hid << ": (" << hyperarc.tail << ")->(" << hyperarc.head << ")" << std::endl;
      auto groundedHyperarc = gh->GetHyperarc(hyperarc.property);
      std::cout << "\tGrounded hid " << groundedHyperarc.hid << ": (" << groundedHyperarc.tail << ")->(" << groundedHyperarc.head << ")" << std::endl;
      std::cout << " With heuristic cost: " << heuristic << std::endl;
    }
  }
}

bool
ModeHyperpathQuery::
IsValidHistory(const ActionHistory& _history) {
  std::set<VID> outgoing;

  for(auto hid : _history) {
    auto hyperarc = this->m_actionExtendedHypergraph.GetHyperarc(hid);
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



