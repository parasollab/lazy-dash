#include "ModeQuery.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/StateGraphs/ModeGraph.h"

#include "Utilities/SSSP.h"

#include <thread> // Required for sleep_for
#include <chrono> // Required for duration


/*------------------------------ Construction ------------------------------*/

ModeQuery::
ModeQuery() {
  this->SetName("ModeQuery");
}

ModeQuery::
ModeQuery(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("ModeQuery");

  m_reverseActions = _node.Read("reverseActions",false,m_reverseActions,
        "Flag to allow immediate reversal of actions in plan.");

  m_writeHypergraph = _node.Read("writeHypergraph",false,m_writeHypergraph,
                      "Flag to write hypergraphs to output files.");

  m_mgLabel = _node.Read("mgLabel",true,"","Mode Graph Label");

  // m_ghLabel = _node.Read("ghLabel",true,"","Grounded Hypergraph Label.");

  // m_mhLabel = _node.Read("ghLabel",true,"","Grounded Hypergraph Label.");

  // m_monotonic = _node.Read("monotonic",false,true,
  //               "Flag for monotonic property. Assume objects are regraspable if monotonic is false.");
}

ModeQuery::
~ModeQuery() {}

/*------------------------- Task Evlauator Interface -----------------------*/

void
ModeQuery::
Initialize() {
  std::cout << "Initialize ModeQuery" << std::endl;
  m_previousSolutions.clear();
  m_vertexMap.clear();
  m_hyperarcMap.clear();
  m_partiallyExtendedHyperarcs.clear();
  m_blockedHyperarcs.clear();
  m_blockingMap.clear();
  m_computedFS.clear();
  m_modeExtendedHypergraph = ModeExtendedHypergraph();
}

void
ModeQuery::
AddSchedulingConstraint(SemanticTask* _task, SemanticTask* _constraint) {
  std::cout << "AddSchedulingConstraint::" << std::endl;
  SchedulingConstraint t;
  SchedulingConstraint c;

  // Check if the _task is a vertex or a hyperarc
  auto vi = m_vertexTasks.find(_task);
  if(vi != m_vertexTasks.end()) {
    t.vertex = true;
    t.id = vi->second;;
  }

  auto hi = m_hyperarcTasks.find(_task);
  if(hi != m_hyperarcTasks.end()) {
    t.id = hi->second;
  }
  else if(!t.vertex) {

    std::cout << "Task Vertices" << std::endl;
    for(auto kv : m_vertexTasks) {
      std::cout << kv.second << " -> " << kv.first->GetLabel() << std::endl;
    }
    std::cout << "Task Hyperarcs" << std::endl;
    for(auto kv : m_hyperarcTasks) {
      std::cout << kv.second << " -> " << kv.first->GetLabel() << std::endl;
    }

    std::cout << this->GetNameAndLabel() << std::endl;
    throw RunTimeException(WHERE) << _task->GetLabel() 
                                  << " does not have a corresponding vertex or hyperarc."
                                  << std::endl;
  }

  // Check if the _constraint is a vertex or a hyperarc
  vi = m_vertexTasks.find(_constraint);
  if(vi != m_vertexTasks.end()) {
    c.vertex = true;
    c.id = vi->second;
  }

  hi = m_hyperarcTasks.find(_constraint);
  if(hi != m_hyperarcTasks.end()) {
    c.id = hi->second;
  }
  else if(!c.vertex) {

    std::cout << "Task Vertices" << std::endl;
    for(auto kv : m_vertexTasks) {
      std::cout << kv.second << " -> " << kv.first->GetLabel() << std::endl;
    }
    std::cout << "Task Hyperarcs" << std::endl;
    for(auto kv : m_hyperarcTasks) {
      std::cout << kv.second << " -> " << kv.first->GetLabel() << std::endl;
    }

    std::cout << this->GetNameAndLabel() << std::endl;
    throw RunTimeException(WHERE) << _task->GetLabel() 
                                  << " does not have a corresponding vertex or hyperarc."
                                  << std::endl;
  }

  // Save constraint
  if(m_constraintMap[t].count(c))
    throw RunTimeException(WHERE) << "Adding already existing scheduling constraint.";
  
  m_constraintMap[t].insert(c);

  // Have to reset everything at the moment, probably a better way to do this,
  // but don't have a good way to track the rippling effect right now on a cached
  // search
  //Initialize();
  m_computedFS.clear();
  m_blockedHyperarcs.clear();
  m_blockingMap.clear();
  m_pq = std::priority_queue<SSSHPElement,std::vector<SSSHPElement>,std::greater<SSSHPElement>>();
}

/*----------------------------- Helper Functions ---------------------------*/

bool
ModeQuery::
Run(Plan* _plan) {

  //Initialize();

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer* mt_ind = new MethodTimer(stats,this->GetNameAndLabel() + "::Run_ModeQuery" + std::to_string(m_counter));
  // m_counter++;

  if(!_plan)
    _plan = this->GetPlan();

  // Initialize action extended hyperpath from mode hyperpath
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());

  ModeExtendedVertex source;
  source.modeVID = mg->GetSourceModeTransitionVID();
  auto sourceVID = m_modeExtendedHypergraph.AddVertex(source);
  m_vertexMap[source.modeVID].insert(sourceVID);

  std::cout << "HI3" << std::endl;
  HyperpathQuery();
  std::cout << "HI4" << std::endl;
  // assert(10==100);

  if(m_goalVID == MAX_UINT) {
    std::cout << "Valid history empty" << std::endl; 
    delete mt_ind;
    return false;
  }

  std::cout << "printing task plan" << std::endl;
  ConvertToTaskPlan(_plan);

  stats->SetStat(this->GetNameAndLabel() + "::ModeExtendedHypergraphVertexSize", m_modeExtendedHypergraph.Size());
  stats->SetStat(this->GetNameAndLabel() + "::ModeExtendedHypergraphEgdeSize", m_modeExtendedHypergraph.EdgeSize());
  // ClearData();

  std::cout << "Valid history exists" << std::endl; 
  delete mt_ind;
  return true;
}

// ModeQuery::ModeExtendedHypergraph&
// ModeQuery::
// GetModeExtendedHypergraph() {
//   return m_modeExtendedHypergraph;
// }



ModeQuery::ActionHistory
ModeQuery::
CombineHistories(size_t _vid, const std::set<size_t>& _pgh, const ActionHistory& _history, size_t hid, bool _greedy) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  // if(m_debug)
  //   std::cout << "# Combine history " << std::endl;
  MethodTimer mt(stats,this->GetNameAndLabel() + "::CombineHistories");
  
  auto composite = _history;
  auto foo = m_modeExtendedHypergraph.GetVertexType(_vid).history;
  auto parentHIDs = m_modeExtendedHypergraph.GetIncomingHyperarcs(_vid);
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  // auto& mh = mg->GetModeTransitionHypergraph();
  auto t = mg->GetTerminationModeTransitionVIDs();
  // auto sink = mg->GetSinkModeTransitionVID();
  // auto i = mg->GetIgnitionModeTransitionVIDs();

  if(_greedy) {
    auto mvid = m_modeExtendedHypergraph.GetVertex(_vid).property.modeVID;
    if(parentHIDs.size() > 1 and t.count(mvid)){
      throw RunTimeException(WHERE) << "Unexpected number of incoming hyperarcs: " << _vid << "(mt vid: " << mvid <<  ")";
    }

    for(auto h : foo) {
      composite.insert(h);
    }

    for(size_t hid : parentHIDs) {
      composite.insert(hid);
    }
  }
  else {
    for(auto h : foo) {
      composite.insert(h);
    }

    if(!parentHIDs.empty())
      composite.insert(*parentHIDs.begin());

    // Check that there are no conflicts

    std::set<size_t> incoming;
    std::set<size_t> outgoing;

    outgoing.insert(_vid);
    for(auto v : _pgh) {
      outgoing.insert(v);
    }

    for(auto hid : composite) {
      auto hyperarc = m_modeExtendedHypergraph.GetHyperarc(hid);
      // Make sure tail has not been used as outgoing yet
      for(auto vid : hyperarc.tail) {
        if(m_hyperarcConstraintTails[hid].count(vid)) {
          continue;
        }
        if(outgoing.count(vid)) {
          return {};
        }
        outgoing.insert(vid);
      }

      // Make sure head has not been used as incoming yet
      for(auto vid : hyperarc.head) {
        if(incoming.count(vid))
          return {};
        incoming.insert(vid);
      }
    }
  }

  return composite;
}


void
ModeQuery::
ConvertToTaskPlan(Plan* _plan) {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::ConvertToPlan");

  // auto mh = dynamic_cast<LazyModeGraph::ModeTransitionHypergraph*>(this->GetStateGraph(m_mhLabel).get());
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto g = mg->GetModeTransitionHypergraph();
  //auto& mh = mg->GetGroundedHypergraph();
  auto src = mg->GetSourceModeTransitionVID();
  auto sink = mg->GetSinkModeTransitionVID();
    
  auto last = m_goalVID;
 
  HPElem source = std::make_pair(true,0);
  std::set<HPElem> parents = {source};

  std::cout << "last: " << m_goalVID << std::endl;
  auto path = ConstructPath(last,parents,m_mbt);
  std::cout << "last2: " << m_goalVID << std::endl;

  path = AddDanglingNodes(path,parents);
  path = OrderPath(path);

  std::vector<HID> relevantMTHIDVector;
  std::vector<VID> relevantMTVIDVector;
  std::set<VID> relevantMTHIDs;
  std::set<VID> relevantMTVIDs;
  std::vector<size_t> hids;
  std::vector<size_t> vids;
  std::unordered_map<VID,std::set<std::pair<HID,HID>>> activeTaskPlan;
  for(auto e : path) {
    if(e.first) {
      vids.push_back(e.second);
    }
    else {
      hids.push_back(e.second);
    }
  }

  for(auto hid : hids) {
    auto h = m_modeExtendedHypergraph.GetHyperarcType(hid); 
    auto modeHID = h.modeHID;
    relevantMTHIDs.insert(modeHID);
    relevantMTHIDVector.push_back(modeHID);
  }
  for(auto vid : vids) {
    auto v = m_modeExtendedHypergraph.GetVertexType(vid); 
    auto modeVID = v.modeVID;
    relevantMTVIDs.insert(modeVID);
    relevantMTVIDVector.push_back(modeVID);
  }


  for(auto v : vids) {
    auto vertex = m_modeExtendedHypergraph.GetVertex(v).property;
    if(vertex.modeVID == src or vertex.modeVID == sink) 
      continue;
    auto group = vertex.robotGroup;
    bool active = false;
    for(auto robot : group->GetRobots()) {
      if(!robot->GetMultiBody()->IsPassive()) {
        active = true;
        break;
      }
    }
    if(!active)
      continue;

    auto incomings = m_modeExtendedHypergraph.GetIncomingHyperarcs(v);
    auto outgoings = m_modeExtendedHypergraph.GetOutgoingHyperarcs(v);
    std::pair<HID,HID> pair{MAX_UINT,MAX_UINT};
    for(auto incoming : incomings) {
      auto it = std::find(hids.begin(), hids.end(), incoming); 
      if(it != hids.end()) {
        pair.first = m_modeExtendedHypergraph.GetHyperarc(incoming).property.modeHID;
      }
    }
    for(auto outgoing : outgoings) {
      auto it = std::find(hids.begin(), hids.end(), outgoing);
      if(it != hids.end()) {
        pair.second = m_modeExtendedHypergraph.GetHyperarc(outgoing).property.modeHID;
      }
    }
    if(pair.first != MAX_UINT and pair.second != MAX_UINT) {
      auto activeMTVID = m_modeExtendedHypergraph.GetVertex(v).property.modeVID;
      activeTaskPlan[activeMTVID].insert(pair);
      // throw RunTimeException(WHERE) << "Cannot construt task plan";
    }
  }

  // mg->SetModeExtendedHypergraph(m_modeExtendedHypergraph);
  mg->SetRelevantMTHIDs(relevantMTHIDs);
  mg->SetRelevantMTVIDs(relevantMTVIDs);
  mg->SetRelevantMTHIDVector(relevantMTHIDVector);
  mg->SetRelevantMTVIDVector(relevantMTVIDVector);
  mg->SetActiveTaskPlan(activeTaskPlan);

  std::cout << "Full Mode Plan" << std::endl;
  std::cout << "Source: " << mg->GetSourceModeTransitionVID() << std::endl;
  std::cout << "Ignitions: " << mg->GetIgnitionModeTransitionVIDs() << std::endl;
  std::cout << "Terminations: " << mg->GetTerminationModeTransitionVIDs() << std::endl;
  std::cout << "Sink: " << mg->GetSinkModeTransitionVID() << std::endl;
  // std::vector<size_t> vids;
  // std::vector<size_t> hids;
  vids.clear();
  hids.clear();

  const std::string filename = this->GetMPProblem()->GetBaseFilename() 
                               + "::FullTaskPlan_" + std::to_string(m_counter);
  std::ofstream ofs(filename);

  for(auto e : path) {
    if(e.first) {
      // std::cout << "v";
      vids.push_back(e.second);
    }
    else {
      hids.push_back(e.second);
      auto hid = e.second;
      auto ha = m_modeExtendedHypergraph.GetHyperarc(hid);
      auto tail = ha.tail;
      auto head = ha.head;
      auto cost = g.GetHyperarc(ha.property.modeHID).property.cost;
      auto action = g.GetHyperarc(ha.property.modeHID).property.action;
      std::string actionLabel = "...";
      std::string log;
      if(action.first)
        actionLabel = action.first->GetLabel();
      log += "[" + std::to_string(hid) + "] " + "(" + std::to_string(ha.property.modeHID) + ", " + std::to_string(cost) + ", " + actionLabel + "): {Tail:[" ;
      std::cout << "[" << hid << "] " << "(" << ha.property.modeHID << ", " << cost 
                << ", " << actionLabel << "): {Tail:[" ;
      for(auto t : tail) {
        auto mvid = m_modeExtendedHypergraph.GetVertex(t).property.modeVID;
        std::string label = "...";
        if(src != mvid and sink != mvid) 
          label = g.GetVertex(mvid).property->robotGroup->GetLabel();
        log += std::to_string(t) + "(" + std::to_string(mvid) + " " + label + ")" + "," ;
        std::cout << t << "(" << mvid << " " << label << ")" << "," ;  
      }
      log += "], Head:[" ;
      std::cout << "], Head:[" ;
      for(auto h : head) {
        auto mvid = m_modeExtendedHypergraph.GetVertex(h).property.modeVID;
        std::string label = "...";
        if(src != mvid and sink != mvid) 
          label = g.GetVertex(mvid).property->robotGroup->GetLabel();
        log += std::to_string(h) + "(" + std::to_string(mvid) + " " + label + ")" + "," ;  
        std::cout << h << "(" << mvid << " " << label << ")" << "," ;  
      }
      log += "]}";
      std::cout << "]}" << std::endl;
      ofs << log << "\n";
    }
  }
  std::cout << std::endl;
  ofs.close();

  // std::cout << "Hyperarc costs" << std::endl;
  // for(auto hid : hids) {
  //   auto h = m_modeExtendedHypergraph.GetHyperarcType(hid); 
  //   auto modeHID = g.GetHyperarcType(h.modeHID);
  //   auto hyperarcWeight = modeHID.cost;

  //   std::cout << hid << ":" << modeHID << " : " << hyperarcWeight << std::endl;
  // }


  
}


std::vector<ModeQuery::HPElem>
ModeQuery::
ConstructPath(size_t _sink, std::set<HPElem>& _parents, const MBTOutput& _mbt) {
  std::vector<HPElem> path;

  HPElem current;
  current.first = true;
  current.second = _sink;

  path.push_back(current);
  for(auto kv : _mbt.vertexParentMap) {
    std::cout << kv.first << ": " << kv.second << std::endl;
  }
  std::cout << "--------------------------" << std::endl;
  while(!_parents.count(current)) {
    std::cout << current.first << ": " << current.second << std::endl;
    _parents.insert(current);
    if(current.first) {
      current.second = _mbt.vertexParentMap.at(current.second);
    }
    else {
      current.second = _mbt.hyperarcParentMap.at(current.second);
    }

    current.first = !current.first;

    path.push_back(current);
  }
    
  _parents.insert(current);
  std::reverse(path.begin(), path.end());

  std::cout << "222" << std::endl;
  path = AddBranches(path, _parents, _mbt);

  return path;
}

std::vector<ModeQuery::HPElem>
ModeQuery::
AddBranches(std::vector<HPElem> _path, std::set<HPElem>& _parents, const MBTOutput& _mbt) {

  std::vector<HPElem> finalPath = _path;

  size_t offset = 0;
  for(size_t i = 0; i < _path.size(); i++) {
    auto elem = _path[i];

    if(elem.first) 
      continue;

    auto arc = m_modeExtendedHypergraph.GetHyperarc(elem.second);

    // Check if tail set is accounted for.
    for(auto vid : arc.tail) {

      HPElem e;
      e.first = true;
      e.second = vid;

      if(!_parents.count(e)) {
        // If tail vertex not in path, compute its branch back to parents.
        auto branch = ConstructPath(vid,_parents,_mbt);

        // Add branch to final path.
        auto iter = finalPath.begin();
        auto branchStart = branch.begin();
        branchStart++;
        finalPath.insert(iter+(i+offset),branchStart,branch.end());

        // Update offset
        offset += (branch.size()-1);
      }
    }
  }

  return finalPath;
}

std::vector<ModeQuery::HPElem>
ModeQuery::
AddDanglingNodes(std::vector<HPElem> _path, std::set<HPElem>& _parents) {

  std::vector<HPElem> finalPath = _path;

  size_t offset = 0;

  for(size_t i = 0; i < _path.size(); i++) {
    auto elem = _path[i];
    if(elem.first) 
      continue;

    const auto arc = m_modeExtendedHypergraph.GetHyperarc(elem.second);
    
    for(auto vid : arc.head) {
      HPElem e;
      e.first = true;
      e.second = vid;

      if(_parents.count(e))
        continue;

      _parents.insert(e);

      auto iter = finalPath.begin();
      finalPath.insert(iter+(i+offset+1),e);
      
      offset++;
    }
  }

  return finalPath; 
}

std::vector<ModeQuery::HPElem>
ModeQuery::
OrderPath(std::vector<HPElem> _path) {
  std::set<HPElem> used;

  std::vector<HPElem> ordered = {_path.front()};
  used.insert(_path.front());

  while(ordered.size() != _path.size()) {
    for(auto elem : _path) {
      // Skip vertices
      if(elem.first)
        continue;

      if(used.count(elem))
        continue;

      // Check if entire tail set is in the ordered path
      auto hyperarc = m_modeExtendedHypergraph.GetHyperarc(elem.second);
      bool ready = true;
      for(auto vid : hyperarc.tail) {
        if(!used.count(std::make_pair(true,vid))) {
          ready = false;
          break;
        }
      }

      // Skip if the entire tail set is not in the ordered path
      if(!ready)
        continue;

      ordered.push_back(elem);
      used.insert(elem);

      for(auto vid : hyperarc.head) {
        HPElem ve = std::make_pair(true,vid);
        ordered.push_back(ve);
        used.insert(ve);
      }
    }
  }

  return ordered; 
}

void
ModeQuery::
ComputeHeuristicValues() {
  std::cout << "Compute Heuristic values" << std::endl;
  // Run a dijkstra search backwards through hypergraph as if it was a graph

  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& mh = mg->GetModeTransitionHypergraph();
  auto m = mh.GetReverseGraph();
  auto sink = mg->GetSinkModeTransitionVID();

  // for(auto h : mh.GetHyperarcMap()) {
  //   auto cost = h.second.property.cost;
  //   std::cout << h.first << ": " << cost << std::endl;
  // }
  // for(auto v : mh.GetVertexMap()) {
  //   std::cout << v.first << ": " << v.second.property->robotGroup << std::endl;
  // }

  // Setup dijkstra functions
  SSSPTerminationCriterion<LazyModeGraph::ModeTransitionHypergraph::GraphType> termination(
    [this](typename LazyModeGraph::ModeTransitionHypergraph::GraphType::vertex_iterator& _vi,
           const SSSPOutput<typename LazyModeGraph::ModeTransitionHypergraph::GraphType>& _sssp) {
    const auto& vertex = _vi->property();
    if(vertex->isDummy()) {
      return SSSPTermination::Continue;
    }

    auto group = vertex->robotGroup;
    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive()) {
        return SSSPTermination::Continue;
      }
    }

    return SSSPTermination::EndBranch;
  });

  SSSPPathWeightFunction<LazyModeGraph::ModeTransitionHypergraph::GraphType> weight(
    [this,m](typename LazyModeGraph::ModeTransitionHypergraph::GraphType::adj_edge_iterator& _ei,
           const double _sourceDistance,
           const double _targetDistance) {

    // auto source = _ei->source();
    auto target = _ei->target();

    auto vertex = m->GetVertex(target);
    bool hasObject = false;

    auto modeHa = _ei->property();
    double edgeWeight = modeHa.cost;
    if(!vertex->isDummy()){
      auto group = vertex->robotGroup;
      for(auto robot : group->GetRobots()) {
        if(robot->GetMultiBody()->IsPassive()) {
          hasObject = true;
          break;
        }
      }
    }

    if(!vertex->isDummy() and !hasObject)
      return std::numeric_limits<double>::infinity();

    
    //TODO::Decide if this is what we want
    //edgeWeight = std::min(1.,edgeWeight);

    double newDistance = _sourceDistance + edgeWeight;
    return newDistance;
  });
  


  // Run dijkstra backwards from sink
  std::vector<size_t> starts = {sink};
  std::cout << "Start Expending from sink: " << starts << std::endl;
  auto output = DijkstraSSSP(m,starts,weight,termination);

  // Save output distances as heuristic values
  m_costToGoMap = output.distance;

  std::cout << "Cost 2 Go Values From " << starts << std::endl;
  for(auto kv : m_costToGoMap) {
    m_maxDistance = std::max(kv.second,m_maxDistance);
    std::cout << kv.first << " : " << kv.second << std::endl;
  }
  std::cout << "Finished computing heuristic values" << std::endl;
}



/*---------------------------- Hyperpath Functions -------------------------*/

void
ModeQuery::
HyperpathQuery() {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer* mt_ind = new MethodTimer(stats,this->GetNameAndLabel() + "::HyperpathQuery" + std::to_string(m_counter));

  // Compute heuristic values
  ComputeHeuristicValues();

  // TODO::Change to check start vertex for each goal specified robot/object
  // if(m_costToGoMap.at(0) == 0)
  //   throw RunTimeException(WHERE) << "Start not connected to goal.";

  // Define hyperpath query functors
  SSSHPTerminationCriterion termination(
    [this](size_t& _vid, const MBTOutput& _mbt) {
      return this->HyperpathTermination(_vid,_mbt);
    }
  );

  SSSHPPathWeightFunction<ModeExtendedVertex,TransitionSwitch> weight(
    [this](const typename ModeExtendedHypergraph::Hyperarc& _hyperarc,
           const std::unordered_map<size_t,double> _weightMap,
           const size_t _target) {
      return this->HyperpathPathWeightFunction(_hyperarc,_weightMap,_target);
    }
  );

  SSSHPForwardStarWithConstraints<ModeExtendedVertex,TransitionSwitch> forwardStar(
    [this](const size_t& _vid, ModeExtendedHypergraph* _h, std::set<size_t> _restrictedHIDs) {
      return this->HyperpathForwardStar(_vid,_h,_restrictedHIDs);
    }
  );

  SSSHPHeuristic<ModeExtendedVertex,TransitionSwitch> heuristic(
    [this](const size_t& _target) {
      return this->HyperpathHeuristic(_target);
    }
  );

  std::set<VID> restrictedHIDs;
  if(m_pq.empty()) {
    std::cout << "m_pq empty" << std::endl;
    m_goalVID = MAX_UINT;
    auto output = SBTDijkstraWithConstraints(&m_modeExtendedHypergraph,0,weight,termination,forwardStar,heuristic,restrictedHIDs);
    m_mbt = output.first;
    m_pq = output.second;
  }
  else {
    std::cout << "m_pq not empty" << std::endl;
    m_mbt.weightMap[m_goalVID] = MAX_DBL;
    m_mbt.vertexParentMap.erase(m_goalVID);

    m_goalVID = MAX_UINT;
    SBTDijkstra(&m_modeExtendedHypergraph,m_mbt,m_pq,weight,termination,forwardStar,heuristic,restrictedHIDs);
  }
  if(m_goalVID != MAX_UINT) 
    m_goalVID = ConnectSink(); // Update goal vid

  std::cout << "DONE MODE QUERY WITH GOAL TEMODE: " << m_goalVID << std::endl;

  delete mt_ind;
  m_previousSolutions.insert(m_goalVID);
}

size_t
ModeQuery::
ConnectSink() {
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto goalModeVID = mg->GetSinkModeTransitionVID();
  auto sinkMTHID = mg->GetTerminationMTHID();
  // auto modeVID = m_modeExtendedHypergraph.GetVertexType(_vid).modeVID;
  auto history = m_modeExtendedHypergraph.GetVertexType(m_goalVID).history;
  auto& mh = mg->GetModeTransitionHypergraph();
  std::set<size_t> modeExtendedFrontiers{0};
  for(auto iter = history.begin() ; iter != history.end() ; ++iter) {
    size_t hid = *iter;
    auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
    for(auto t : hyperarc.tail) {
      modeExtendedFrontiers.erase(t);
    }
    for(auto h : hyperarc.head) {
      modeExtendedFrontiers.insert(h);
    }
  }

  std::set<size_t> modeExtendedFrontiersPassive = {};
  for(auto f : modeExtendedFrontiers) {
    auto vertex = this->m_modeExtendedHypergraph.GetVertex(f);
    auto modeVID = vertex.property.modeVID;
    
    if(mh.GetVertex(modeVID).property->robotGroup->IsPassive()) 
      modeExtendedFrontiersPassive.insert(f);
  }



  ModeExtendedVertex vertex;
  vertex.modeVID = goalModeVID;
  vertex.history = history;
  size_t sinkMEVID = this->m_modeExtendedHypergraph.AddVertex(vertex);
  std::cout << "Created a sink me vid " << sinkMEVID << std::endl;

  TransitionSwitch transition;
  transition.modeHID = sinkMTHID;
  transition.cost = MAX_DBL;
  size_t newHID = this->m_modeExtendedHypergraph.AddHyperarc({sinkMEVID},modeExtendedFrontiersPassive,transition,false);
  std::cout << "Created a sink me hid " << newHID << std::endl;

  VID sourceVID = 0; 
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

  return sinkMEVID;
}

void
ModeQuery::
ClearData() {
  // m_vertexMap.clear();
  // m_hyperarcMap.clear();
  // m_partiallyExtendedHyperarcs.clear();
  // m_blockedHyperarcs.clear();
  // m_blockingMap.clear();
  // m_computedFS.clear();
  // m_modeExtendedHypergraph = ModeExtendedHypergraph();
  // m_historyGraph = std::unique_ptr<HistoryGraph>(new HistoryGraph());
  std::cout << "Cleared" << std::endl;
}

SSSHPTermination
ModeQuery::
HyperpathTermination(const size_t& _vid, const MBTOutput& _mbt) {
  std::cout << "Check Termination" << std::endl;


  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());

  auto terminations = mg->GetTerminationModeTransitionVIDs();
  auto history = m_modeExtendedHypergraph.GetVertexType(_vid).history;

  // auto goalModeVID = mg->GetSinkModeTransitionVID();
  // auto modeVID = m_modeExtendedHypergraph.GetVertexType(_vid).modeVID;
  // if(goalModeVID == modeVID) {
  //   m_goalVID = _vid;
  //   std::cout << "Found SINK" << std::endl;
  //   return SSSHPTermination::EndSearch;
  // }

  auto& mh = mg->GetModeTransitionHypergraph();
  if(!history.empty()) {
    bool sinkConnect = false;
    std::set<size_t> modeExtendedFrontiers{0};
    for(auto iter = history.begin() ; iter != history.end() ; ++iter) {
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
    std::set<size_t> modeExtendedFrontiersPassive = {};
    for(auto f : modeExtendedFrontiers) {
      auto vertex = this->m_modeExtendedHypergraph.GetVertex(f);
      auto modeVID = vertex.property.modeVID;
      
      modeTransitionFrontiers.insert(modeVID);
      if(mh.GetVertex(modeVID).property->robotGroup->IsPassive()) 
        modeExtendedFrontiersPassive.insert(f);
    }

    size_t existNum = 0;
    for(auto t : terminations) {
      if(modeTransitionFrontiers.count(t)) {
        // std::cout << t << " exist in frontiers" << std::endl;
        existNum += 1;
        continue;
      }
    } 
    std::cout << existNum << "/" << terminations.size() << " satisfied " << std::endl;
    if(existNum == terminations.size())
      sinkConnect = true;
    
    if(sinkConnect) {
      std::cout << "Found Sink" << std::endl;
      m_goalVID = _vid;

      // ModeExtendedVertex vertex;
      // vertex.modeVID = goalModeVID;
      // vertex.history = history;
      // size_t sinkMEVID = this->m_modeExtendedHypergraph.AddVertex(vertex);
      // m_goalVID = sinkMEVID;
      // std::cout << "Created a sink me vid " << sinkMEVID << std::endl;

      // TransitionSwitch transition;
      // transition.modeHID = sinkMTHID;
      // transition.cost = MAX_DBL;
      // size_t newHID = this->m_modeExtendedHypergraph.AddHyperarc({sinkMEVID},modeExtendedFrontiersPassive,transition,false);
      // std::cout << "Created a sink me hid " << newHID << std::endl;

      // VID sourceVID = 0; 
      // m_mbt.vertexParentMap[sourceVID] = MAX_INT;

      // history.insert(newHID);
      // for(auto hid : history) {
      //   auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
      //   for(auto vid : hyperarc.head) {
      //     m_mbt.vertexParentMap[vid] = hid;
      //   }
      //   std::cout << hid << ": " << hyperarc.tail << " --> " << hyperarc.head << std::endl;

      //   auto vid = *(hyperarc.tail.begin());
      //   m_mbt.hyperarcParentMap[hid] = vid;
      //   m_prevHistoryGraphGoalVID = sinkMEVID;
      // }

      // m_mbt.weightMap[newHID] = 0.1;
      std::cout << "Found sink" << std::endl;
      return SSSHPTermination::EndSearch;
    }
  }

  if(m_previousSolutions.count(_vid))
    return SSSHPTermination::EndBranch;

  // Assuming grounded goal is vid 1
  // auto modeVID = m_modeExtendedHypergraph.GetVertexType(_vid).modeVID;
  // std::cout << "Termination Check. Sink: " << goalModeVID << ", TEVID: " << _vid << ", MVID: " << modeVID << std::endl;
  // if(modeVID == goalModeVID) {
  //   m_goalVID = _vid;
  //   std::cout << "Solution Exists. Terminate Search with goal te vid: " << m_goalVID << std::endl;
  //   assert(1==2);
  //   return SSSHPTermination::EndSearch;
  // }

  std::cout << "Solution Not Exists. Continue Search." << std::endl;
  return SSSHPTermination::Continue;
}

double
ModeQuery::
HyperpathPathWeightFunction(
          const typename ModeExtendedHypergraph::Hyperarc& _hyperarc,
          const std::unordered_map<size_t,double> _weightMap,
          const size_t _target) {

  double hyperarcWeight = 0;

  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& mh = mg->GetModeTransitionHypergraph();
  auto modeHA = mh.GetHyperarcType(_hyperarc.property.modeHID);
  
  hyperarcWeight = modeHA.cost;

  double tailWeight = 0.;

  for(auto vid : _hyperarc.tail) {
    auto cost = _weightMap.at(vid);
    tailWeight = std::max(tailWeight,cost);
    // tailWeight += cost;
  }

  double max_val = -std::numeric_limits<double>::infinity(); 
  for (const auto& kv : m_costToGoMap) {
    auto value = kv.second;
      if (value > max_val) {
          max_val = value;
      }
  }

  return (hyperarcWeight + tailWeight)/(2.*max_val);
}

std::set<size_t>
ModeQuery::
HyperpathForwardStar(const size_t& _vid, ModeExtendedHypergraph* _h, const std::set<size_t>& _interactionConstraints, bool _greedy) {
  if(m_debug)
    std::cout << "######## HyperpathForwardStar of TE VID " << _vid << std::endl;
  if(_vid > 1000000000)
     throw RunTimeException(WHERE) << "TE VID size exceeds threshold";

  if(m_computedFS.count(_vid))
    return _h->GetOutgoingHyperarcs(_vid);

  m_computedFS.insert(_vid);

  // auto mg = dynamic_cast<ModeGraph*>(this->GetStateGraph(m_mgLabel).get());
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& mh = mg->GetModeTransitionHypergraph();
  auto source = mg->GetSourceModeTransitionVID();
  auto sink = mg->GetSinkModeTransitionVID();

  auto aev = _h->GetVertexType(_vid);
  auto modeVID = aev.modeVID;







  // Check if vid's parent was in the same mode
  // Used for inter mode blocks
  // size_t blockedMode = MAX_UINT;

  // // Extra for blocking immediate reverse actions
  // std::cout << "+======= FINDING VERTICES TO BLOCK ========+" << std::endl;
  std::set<RobotGroup*> blockedVertices; // In mode query, this is a set of mode vertices
  // std::set<size_t> blockedTEVertices;
  // std::set<size_t> permanantBlockedVertices;

  bool initialStage = false;

  auto incoming  = _h->GetIncomingHyperarcs(_vid);
  if(incoming.size() > 0) {
    auto hid = *(incoming.begin());
    auto hyperarc = _h->GetHyperarc(hid);
    auto tail = hyperarc.tail;

    for(auto parent : tail) {
      auto parentVertex = _h->GetVertexType(parent);
      // auto mvid = parentVertex.modeVID;
      if(parentVertex.modeVID==source) {
        initialStage = true;
        continue;
      }
      auto group = parentVertex.robotGroup; 
      blockedVertices.insert(group);
      // blockedTEVertices.insert(parent);
    }
    if(m_debug){
      std::cout << "Blocked robots: ";
      for(auto v : blockedVertices){
        std::cout << v->GetLabel() << " " ;
      }
      std::cout << std::endl;
    }
  }

 
  std::unordered_map<Robot*,size_t> graspCountMap;
  auto history = aev.history;
  history.insert(*(_h->GetIncomingHyperarcs(_vid).begin()));

  if(m_debug)
    std::cout << "Check grasp history " << history << std::endl;

  for(auto hid : history) {
    if(hid == 0) 
      continue;

    auto hyperarc = _h->GetHyperarc(hid);
    Robot* passive = nullptr;
    Robot* active = nullptr;
    // Sort out handoff
    if(hyperarc.tail.size() == hyperarc.head.size())
      continue;
    if(hyperarc.tail.size() != 2)
      continue;
      
    // Start computing only for the grasp action
    for(auto v : hyperarc.tail) {
      auto robotGroup = _h->GetVertexType(v).robotGroup->GetRobots();
      if(robotGroup.size()==1) {
        if(robotGroup[0]->GetMultiBody()->IsPassive()) {
          passive = robotGroup[0];
        }
        else {
          active = robotGroup[0];
        }
      }
    }
    if(active and passive) {
      if(m_debug)
        std::cout << active->GetLabel() << " grasps " << passive->GetLabel() << std::endl;
      graspCountMap[passive] += 1;
    }
  }

  if(m_debug) {
    std::cout << "=============== HISTORY ===============" << std::endl;
    for(auto hid : history) {
      if(hid == 0) 
        continue;

      auto hyperarc = _h->GetHyperarc(hid);
      
      std::cout << "[HID " << _h->GetHyperarc(hid).property.modeHID << "]: ";
      if(hyperarc.tail.size() == 1 and hyperarc.head.size() == 1) {
        std::cout << "Move: ";
      }
      else {
        std::cout << "Transition: ";
      } 
      for(auto vid : hyperarc.tail) {
        auto group = _h->GetVertexType(vid).robotGroup;
        std::cout << " (" << _h->GetVertexType(vid).modeVID << ": " << group->GetLabel() << ") ";
      }
      std::cout << " ----->>>> ";
      for(auto vid : hyperarc.head) {
        auto group = _h->GetVertexType(vid).robotGroup;
        std::cout << " (" << _h->GetVertexType(vid).modeVID << ": " << group->GetLabel() << ") ";
      }
      std::cout << std::endl;
    }
  }


  std::set<size_t> fullyGroundedHyperarcs;

  // Build partially extended hyperarcs
  // Grab forward star in extended hypergraph
  std::string label = "...";
  if(modeVID != source and modeVID != sink)
    label = mh.GetVertex(modeVID).property->robotGroup->GetLabel();
  if(m_debug)
    std::cout << "**** Building Partially Extended Hypergraph of MT VID: " 
              << modeVID << " " << label << " ****" << std::endl;


  for(auto hid : mh.GetOutgoingHyperarcs(modeVID)) {
    if(_interactionConstraints.count(hid))
      continue;

    auto head = mh.GetHyperarc(hid).head;
    auto tail = mh.GetHyperarc(hid).tail;
    
    if(m_debug) {
      std::cout << "===>>> Current Outgoing MT HID: " << hid << std::endl;

      std::cout << "Tail MT VID: ";
      for (auto v: tail) {
        std::cout << v << " " ;
      }
      std::cout << "--> Head MT VID: ";
      for (auto v: head) {
        std::cout << v << " " ;
      }
      std::cout << std::endl;
    }

    if(head.size() == 1 and *head.begin() == sink) {
      // std::cout << "Skip sink related hid for memmory efficiency" << hid << std::endl;
      continue;
    }

    auto terminationModeTransitionVIDs = mg->GetTerminationModeTransitionVIDs();

    bool possibleRelease = true;
    if(head.size() > 1 and tail.size() == 1) {
      std::cout << "Release possible. Release check: " << tail << " --> " << head << std::endl;
      
      for(auto h : head) {
        auto group = mh.GetVertex(h).property->robotGroup;
        if(group->Size() > 1) {
          possibleRelease = false;
          break;
        }
      }

      if(*(tail.begin())==source)
        possibleRelease = false;

      if(possibleRelease) {
        std::cout << "Possible release" << std::endl;
        Robot* active = nullptr;
        Robot* passive = nullptr;
        VID activeVID;
        VID passiveVID;
        for(auto h: head) {
          auto robot = mh.GetVertex(h).property->robotGroup->GetRobot(0);
          if(robot->GetMultiBody()->IsPassive()) {
            passive = robot;
            passiveVID = h;
          }
          else {
            active = robot;
            activeVID = h;
          }
        }

        if(active and passive) {
          std::cout << active->GetLabel() << "(" << activeVID <<  ") tries to release " << passive->GetLabel() << "(" << passiveVID << ")" << std::endl;
          std::cout << terminationModeTransitionVIDs << " / " << passiveVID << std::endl;
          if(terminationModeTransitionVIDs.count(passiveVID)) {
            std::cout << "Releasing " << passive->GetLabel() << " to the goal state" << std::endl; 
          }
          else {
            std::cout << "== Check grasps" << std::endl;
            for(auto kv : graspCountMap) {
              std::cout << kv.first->GetLabel() << ": " << kv.second << std::endl;
            }
            std::cout << "== Check levels" << std::endl;
            for(auto kv : m_objectLevelMap) {
              std::cout << kv.first->GetLabel() << ": " << kv.second << std::endl;
            }
            if(graspCountMap[passive] == m_objectLevelMap[passive]) {
              std::cout << "Cannot release. This object should be directly released to the goal state." << std::endl;
              std::cout << "This action is not connected to the goal state." << std::endl;
              std::cout << "Do something else (handoff or goal)" << std::endl;
              std::cout << "Moving on to the next possible outgoing hyperarc" << std::endl;
              continue;
            }
          }



          if(m_debug) {
            std::cout << "Object Grasp Count Map #: " << graspCountMap.size() << std::endl;
            for(auto kv : graspCountMap) {
              std::cout << kv.first->GetLabel() << ": " << kv.second << std::endl;
            }
          }
          
          if(graspCountMap[passive] > 0) {
            if(terminationModeTransitionVIDs.count(passiveVID)) {
              std::cout << "Releasing " << passive->GetLabel() << " to the goal state" << std::endl; 
            }
            else {
              if(graspCountMap[passive] == m_objectLevelMap[passive]) {
                std::cout << "Cannot release. This object should be directly released to the goal state." << std::endl;
                std::cout << "This action is not connected to the goal state." << std::endl;
                std::cout << "Do something else (handoff or goal)" << std::endl;
                std::cout << "Moving on to the next possible outgoing hyperarc" << std::endl;
                continue;
              }
            }
          }
        }
      }
    }


    if(blockedVertices.size() == head.size() and !m_reverseActions and !initialStage) {
      
      bool match = true;
      if(m_debug)
        std::cout << "Check Block Vertices: " << std::endl;
      for(auto group : blockedVertices) {
        bool exist = false;
        for(auto vid : head) {
          if(m_debug)
            std::cout << "head vid: " << vid << std::endl;
          auto headRobot = mh.GetVertex(vid).property->robotGroup;
          if(group == headRobot) {
          if(m_debug)
              std::cout << vid << " " << group->GetLabel() << " exists in blocked vertices" << std::endl;
            exist = true;
            break;
          }
        }
        if(exist) {
          continue;
        }
        if(m_debug)
          std::cout << "Valid" << std::endl;
        match = false;
        break;
      }


      VID passiveVID = MAX_INT;
      for(auto h: head) {
        if(mg->GetSinkModeTransitionVID() == h) {
          break;
        }
        auto group = mh.GetVertex(h).property->robotGroup;
        if(group->Size() > 1) 
          break;
        if(group->GetRobot(0)->GetMultiBody()->IsPassive()) {
          passiveVID = h;
        }
      }

      bool directReverseAllowed = false;
      if(mh.GetHyperarc(hid).property.action.first != nullptr && mh.GetHyperarc(hid).property.action.first->IsDirectReverseAllowed()) {
        directReverseAllowed = true;
      }
      if(!directReverseAllowed && (match && !terminationModeTransitionVIDs.count(passiveVID))) {
        if(m_debug)
          std::cout << "Invalid: Direct reverse. continue." << std::endl;
        continue;
      }
    }
    
    // std::unordered_map<size_t,std::vector<PartiallyExtendedHyperarc>> m_partiallyExtendedHyperarcs;
    if(m_partiallyExtendedHyperarcs[hid].empty())
      m_partiallyExtendedHyperarcs[hid].push_back(PartiallyExtendedHyperarc());

    // New partially extended hyperarcs
    std::vector<std::pair<std::set<size_t>,ActionHistory>> newPartiallyExtendedHyperarcs;

    size_t initialSize = m_partiallyExtendedHyperarcs[hid].size();

    for(size_t i = 0 ; i < initialSize ; ++i) {
    // for(auto& pair : m_partiallyExtendedHyperarcs[hid]) {
      auto& pair = m_partiallyExtendedHyperarcs[hid][i];
      
      auto pgh = pair.first;
      auto history = pair.second;

      // Check if the corresponding extended vertex is already included
      bool unique = true;
      for(auto v : pgh) {
        if(v == _vid) {
          unique = false;
          break;
        }
      }

      for(auto v : pgh) {
        size_t vid = m_modeExtendedHypergraph.GetVertex(v).property.modeVID;
        auto group1 = mh.GetVertex(vid).property->robotGroup;
        auto group2 = mh.GetVertex(m_modeExtendedHypergraph.GetVertex(_vid).property.modeVID).property->robotGroup;
        if(group1 == group2) {
          unique = false;
          break;
        }
      }

      if(!unique) { 
        std::cout << "Not unique" << std::endl;
        continue;
      }
      auto newHistory = CombineHistories(_vid,pgh,history,hid,_greedy);
      if(newHistory.empty() and _vid != 0){
        std::cout << "Invalid history" << std::endl;
        continue;
      }  

      auto newPGH = pgh; // Tails
      newPGH.insert(_vid);

      if(newPGH.size() != mh.GetHyperarc(hid).tail.size()) {
        for(auto v : newPGH) {
          size_t vid = m_modeExtendedHypergraph.GetVertex(v).property.modeVID;
          auto group1 = mh.GetVertex(vid).property->robotGroup;
          auto group2 = mh.GetVertex(m_modeExtendedHypergraph.GetVertex(_vid).property.modeVID).property->robotGroup;
          std::cout << group1->GetLabel() << std::endl;
          std::cout << group2->GetLabel() << std::endl;
        }
        m_partiallyExtendedHyperarcs[hid].emplace_back(std::make_pair(newPGH,newHistory));
        // newPartiallyExtendedHyperarcs.push_back(std::make_pair(newPGH,newHistory));
        std::cout << "Size not match" << std::endl;
        continue;
      }

      // Add fully extended hyperarc to the action extended hypergraph
      CheckSchedulingConstraints(hid,newPGH,newHistory,fullyGroundedHyperarcs);
    }

    for(auto pgh : newPartiallyExtendedHyperarcs) {
      m_partiallyExtendedHyperarcs[hid].push_back(pgh);
    }
  }


  // std::cout << "returning fullyGroundedHyperarcs" << std::endl;
  // std::cout << "Sizes" << std::endl;
  // size_t total = 0;
  // for(auto kv : mh.GetHyperarcMap()) {
  //   auto hid = kv.first;
  //   // std::cout << "MT HID " << hid << ": " << m_partiallyExtendedHyperarcs[hid].size() << std::endl;
  //   total += m_partiallyExtendedHyperarcs[hid].size();
  // }
  // std::cout << "Total " << total << std::endl;
  // std::cout << "**** DONE. Fully grounded hyperarcs: " <<  fullyGroundedHyperarcs.size()  
  //           << " among the partially grounded hyperarcs num " << total << std::endl;


  // for(auto hid : fullyGroundedHyperarcs) {
  //   auto mthid = m_modeExtendedHypergraph.GetHyperarc(hid).property.modeHID;
  //   auto head = mh.GetHyperarc(mthid).head;
  //   for(auto h : head) {
  //     if(h == mg->GetSinkModeTransitionVID()) {
  //       std::cout << "OMG!!! Found SINK!" << std::endl;
  //     }
  //   }
  // }

  
  return fullyGroundedHyperarcs;
}

double 
ModeQuery::
HyperpathHeuristic(const size_t& _target) {
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_mgLabel).get());
  // auto& mh = mg->GetModeTransitionHypergraph();

  // auto t = mg->GetTerminationModeTransitionVIDs();
  // auto source = mg->GetSourceModeTransitionVID();
  // auto sink = mg->GetSinkModeTransitionVID();
  // auto goalModeVID = mg->GetSinkModeTransitionVID();

  auto terminations = mg->GetTerminationModeTransitionVIDs();
  auto history = m_modeExtendedHypergraph.GetVertexType(_target).history;

  double taskHeuristic = 0.;

  if(!history.empty()) {
    std::set<size_t> modeExtendedFrontiers{0};
    for(auto iter = history.begin() ; iter != history.end() ; ++iter) {
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
    taskHeuristic = 1. - existNum / terminations.size();
    std::cout << existNum << "/" << terminations.size() << std::endl;
  }

  auto aev = m_modeExtendedHypergraph.GetVertexType(_target);
  auto vid = aev.modeVID;

  double max_val = -std::numeric_limits<double>::infinity(); 
  for (const auto& kv : m_costToGoMap) {
      auto value = kv.second;
      if (value > max_val) {
          max_val = value;
      }
  }
  auto motionHeuristic = m_costToGoMap[vid] / (max_val);
  std::cout << motionHeuristic << " : " << taskHeuristic << std::endl;

  return motionHeuristic + taskHeuristic;
}

std::set<size_t>
ModeQuery::
GetFrontier(ActionHistory _history) {

  // std::set<size_t> frontier;

  // for(auto hid : _history) {
  //   auto hyperarc = m_modeExtendedHypergraph.GetHyperarc(hid);
  //   for(auto vid : hyperarc.head) {
  //     frontier.insert(vid);
  //   }
  //   for(auto vid : hyperarc.tail) {
  //     frontier.erase(vid);
  //   }
  // }

  // return frontier;

  std::set<VID> frontierVIDs;
  std::set<VID> internalVIDs;

  for(auto hid : _history) {
    auto hyperarc = this->m_modeExtendedHypergraph.GetHyperarc(hid);
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

  return frontierVIDs;
}

std::set<size_t>
ModeQuery::
GetExtendedFrontier(ActionHistory _history) {

  auto frontiers = GetFrontier(_history);
  std::set<size_t> extendedFrontiers;
  for(auto vid : frontiers)
    extendedFrontiers.insert(m_modeExtendedHypergraph.GetVertexType(vid).modeVID);

  return extendedFrontiers;
}

void
ModeQuery::
CheckSchedulingConstraints(size_t _hid, std::set<size_t> _tail, ActionHistory _history, std::set<size_t>& _fullyExtendedHyperarcs) {
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& mh = mg->GetModeTransitionHypergraph();

  if(m_debug){
    std::cout << "# CheckSchedulingCostraints for hid " << _hid << std::endl;
  }
  
  // Collect last positions and movements of all bodies/robots
  auto frontier = GetFrontier(_history);
  auto extendedFrontier = GetExtendedFrontier(_history);
  std::set<size_t> mostRecentHyperarcs;
  for(auto vid : frontier) {
    auto incoming = m_modeExtendedHypergraph.GetIncomingHyperarcs(vid);
    for(auto h : incoming) {
      // Check that all head vertices are on the frontier
      auto ha = m_modeExtendedHypergraph.GetHyperarc(h);
      bool recent = true;
      for(auto v : ha.head) {
        if(!frontier.count(v)) {
          recent = false;
          break;
        }
      }

      if(recent) {
        //mostRecentHyperarcs.insert(m_modeExtendedHypergraph.GetHyperarcType(h));
        mostRecentHyperarcs.insert(ha.property.modeHID);
      }
    }
  }
  std::map<size_t,std::set<size_t>> allVertices;
  for(auto hid : _history) {
    auto hyperarc = m_modeExtendedHypergraph.GetHyperarc(hid);
    for(auto vid : hyperarc.head) {
      auto mtvid = m_modeExtendedHypergraph.GetVertexType(vid).modeVID;
      allVertices[mtvid].insert(vid);
    }
  }

  std::set<size_t> groundedTail = _tail;
  std::set<size_t> constraintTail;
  std::set<size_t> blockingVertices;

  // Check hyperarc scheduling constraints
  SchedulingConstraint hc;
  hc.vertex = false;
  hc.id = _hid;
  for(auto dep : m_constraintMap[hc]) {
    // Check if any conflicting vertices are currently on the frontier
    if(dep.vertex and extendedFrontier.count(dep.id)) {
      std::cout << "FOUND VERTEX CONFLICT" << std::endl;

      // TODO::Look for other vertices which alleviate this constraint
      blockingVertices.insert(dep.id);
    }
    // Check if vertex is further back in this history
    else if(dep.vertex) {
      auto iter = allVertices.find(dep.id);
      if(iter != allVertices.end()) {
        for(auto v : iter->second) {
          // Ensure that no robot remains at the constraint vertex before taking this edge
          //TODO::Add a head vertex from outgoing hyperarc of v rahter than v
          for(auto h : this->m_modeExtendedHypergraph.GetOutgoingHyperarcs(v)) {
            if(_history.count(h)) {
              auto head = this->m_modeExtendedHypergraph.GetHyperarc(h).head;
              _tail.insert(*(head.begin()));
              constraintTail.insert(*(head.begin()));
            }
          }
        }
      }
    }
    else if(!dep.vertex and mostRecentHyperarcs.count(dep.id)) {
      std::cout << "FOUND HYPERARC CONFLICT" << std::endl;

      // Add dep head vertices to the _tail set to force completion
      for(auto v : m_modeExtendedHypergraph.GetHyperarc(dep.id).head) {
        _tail.insert(v);
        constraintTail.insert(v);
      }
    }
  }

  // Check vertex scheduling constraints
  SchedulingConstraint vc;
  vc.vertex = true;
  auto transitionHyperarc = mh.GetHyperarc(_hid);
  for(auto vid : transitionHyperarc.head) {
    vc.id = vid;
    for(auto dep : m_constraintMap[vc]) {
      // Check if any conflicting vertices are currently on the frontier
      if(dep.vertex and extendedFrontier.count(dep.id)) {
        std::cout << "FOUND VERTEX CONFLICT" << std::endl;

        // TODO::Look for other vertices which alleviate this constraint
        blockingVertices.insert(dep.id);
      }
      // Check if vertex is further back in this history
      else if(dep.vertex) {
        auto iter = allVertices.find(dep.id);
        if(iter != allVertices.end()) {
          for(auto v : iter->second) {
            // Ensure that no robot remains at the constaint vertex before taking this edge
            _tail.insert(v);
            constraintTail.insert(v);
          }
        }
      }
      else if(!dep.vertex and mostRecentHyperarcs.count(dep.id)) {
        // Add dep head vertices to the _tail set to force completion
        for(auto v : m_modeExtendedHypergraph.GetHyperarc(dep.id).head) {
          _tail.insert(v);
          constraintTail.insert(v);
        }
      }
    }
  }

  // Check if this hyperarc is blocked
  if(!blockingVertices.empty()) {
    if(m_debug){
      std::cout << "Blocking Vertices" << std::endl;
      for (auto vid : blockingVertices) {
        std::cout << vid ;
      }
      std::cout << std::endl;
    }
    PartiallyScheduledHyperarc psh;
    psh.modeHID = _hid;
    psh.pgh = std::make_pair(_tail,_history);
    psh.constraintTail = constraintTail;
    psh.blockingVertices = blockingVertices;

    size_t index = m_blockedHyperarcs.size();
    m_blockedHyperarcs.push_back(psh);

    for(auto vid : blockingVertices) {
      m_blockingMap[vid].insert(index);
    }

    return;
  }

  // Construct head set
  std::set<size_t> head;

  for(auto v : transitionHyperarc.head) {
    ModeExtendedVertex vertex;
    vertex.modeVID = v;
    if(m_debug)
      std::cout << "Adding mode extended vertex vid: " << v << std::endl;

    vertex.history = _history;
    vertex.robotGroup = mh.GetVertex(v).property->robotGroup;
    auto newVID = m_modeExtendedHypergraph.AddVertex(vertex);
    head.insert(newVID);
  }
  if(m_debug){
    std::cout << "ModeQuery::CheckSchedulingSconstraints" << std::endl;
    std::cout << "Tail TEVID: " ;
    for (auto v : _tail) {
      auto vid = m_modeExtendedHypergraph.GetVertex(v).property.modeVID;
      auto source = mg->GetSourceModeTransitionVID();
      auto sink = mg->GetSinkModeTransitionVID();
      std::string label = "...";
      if(source != vid and sink != vid) {
        label = mh.GetVertex(vid).property->robotGroup->GetLabel();
      }
      std::cout << v << " (" << vid << " " << label << ") " ;
    }
    std::cout << std::endl;

    std::cout << "Head TEVID: " ;
    for (auto v : head) {
      auto vid = m_modeExtendedHypergraph.GetVertex(v).property.modeVID;
      auto source = mg->GetSourceModeTransitionVID();
      auto sink = mg->GetSinkModeTransitionVID();
      std::string label = "...";
      if(source != vid and sink != vid) {
        label = mh.GetVertex(vid).property->robotGroup->GetLabel();
      }
      std::cout << v << " (" << vid << " " << label << ") " ;
    }
    std::cout << std::endl;
  }

  auto newHID = AddHyperarc(_tail,head,_hid,_fullyExtendedHyperarcs);
  if(!constraintTail.empty()) {
    m_hyperarcConstraintTails[newHID] = constraintTail;
  }
}

size_t
ModeQuery::
AddHyperarc(std::set<size_t> _tail, std::set<size_t> _head, size_t _modeHID, std::set<size_t>& _fullyExtendedHyperarcs) {
  auto mg = dynamic_cast<LazyModeGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto& mh = mg->GetModeTransitionHypergraph();

  TransitionSwitch transition;
  transition.modeHID = _modeHID;
  transition.cost = mh.GetHyperarcType(_modeHID).cost;

  // std::cout << "Adding hyperarc" << std::endl;
  auto newHID = m_modeExtendedHypergraph.AddHyperarc(_head,_tail,transition,false);
  _fullyExtendedHyperarcs.insert(newHID);
  m_hyperarcMap[_modeHID].insert(newHID);

  if(m_debug){
    std::cout << "ModeQuery::AddHyperarc" << std::endl;
    std::cout << "Tail TEVID: ";
    auto source = mg->GetSourceModeTransitionVID();
    auto sink = mg->GetSinkModeTransitionVID();
    for (auto v : _tail) {
      auto vid = m_modeExtendedHypergraph.GetVertex(v).property.modeVID;
      std::string label = "...";
      if(source != vid and sink != vid) {
        label = mh.GetVertex(vid).property->robotGroup->GetLabel();
      }
      std::cout << v << " (" << vid << " " << label << ") " ;
    }
    std::cout << std::endl;
    std::cout << "Head TEVID: ";
    for (auto v : _head) {
      auto vid = m_modeExtendedHypergraph.GetVertex(v).property.modeVID;
      std::string label = "...";
      if(source != vid and sink != vid) {
        label = mh.GetVertex(vid).property->robotGroup->GetLabel();
      }
      std::cout << v << " (" << vid << " " << label << ") " ;
    }
    std::cout << std::endl;
    std::cout << "modeHID: " << _modeHID << std::endl;
    std::cout << "newHid: " << newHID << std::endl;
  }

  // auto sink = mg->GetSinkModeTransitionVID();
  // for (auto v : _head) {
  //   auto vid = m_modeExtendedHypergraph.GetVertex(v).property.modeVID;
  //   // if(vid==sink) 
  //   //   std::cout << "You found the sink!" << std::endl;
  // }

  // Check if this has unblocked any other hyperarcs
  for(auto vid : _tail) {
    for(auto index : m_blockingMap[vid]) {
      if(m_debug)
        std::cout << "Blocked: " << index ;
      auto bh = m_blockedHyperarcs[index];

      // Remove this vertex as blocking
      bh.blockingVertices.erase(vid);

      // Add this hyperarc to the history
      bh.pgh.second.insert(_modeHID);

      // Add this vertex to the constraint vertices and tail
      bh.constraintTail.insert(vid);
      bh.pgh.first.insert(vid);

      // Check if this has unblocked the hyperarc
      if(!bh.blockingVertices.empty()) {
        // If not, add new partially blocked hyperarc to the set
        size_t index = m_blockedHyperarcs.size();
        m_blockedHyperarcs.push_back(bh);

        for(auto vid : bh.blockingVertices) {
          m_blockingMap[vid].insert(index);
        }
      }
      continue;

      // Otherwise, add unblocked hyperarc to the hypergraph
      std::set<size_t> newHead;
      auto gha = mh.GetHyperarc(bh.modeHID);
      for(auto v : gha.head) {
        ModeExtendedVertex vertex;
        vertex.modeVID = v;
        vertex.history = bh.pgh.second;
        vertex.robotGroup = mh.GetVertex(v).property->robotGroup;

        auto newVID = m_modeExtendedHypergraph.AddVertex(vertex);
        newHead.insert(newVID);
      }
      if(m_debug){
        std::cout << "NEW HEAD" << std::endl;
        for (auto v : newHead) {
          std::cout << v ;
        }
        std::cout << std::endl;
        
        // Recusrsively add hyperarc and check for additional unblocked hyperarcs
        std::cout << "Recur" << std::endl;
      }
      auto newHID2 = AddHyperarc(bh.pgh.first,newHead,bh.modeHID, _fullyExtendedHyperarcs);
      m_hyperarcConstraintTails[newHID2] = bh.constraintTail;
    }
  }
  
  if(m_debug)
    std::cout << "Done Adding. Cost: " << transition.cost << std::endl;
  return newHID;
}

/*--------------------------------------------------------------------------*/
istream&
operator>>(istream& _is, const ModeQuery::ModeExtendedVertex& _vertex) {
  return _is;
}

ostream&
operator<<(ostream& _os, const ModeQuery::ModeExtendedVertex& _vertex) {
  return _os;
}

ostream& 
operator<<(std::ostream& os, const ModeQuery::ActionHistory& info) {
  return os;
}