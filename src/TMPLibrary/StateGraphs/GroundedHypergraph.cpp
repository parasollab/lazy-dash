#include "GroundedHypergraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/Solution/Plan.h"

/*------------------------------ Construction --------------------------------*/

GroundedHypergraph::
GroundedHypergraph() {
  this->SetName("GroundedHypergraph");
}

GroundedHypergraph::
GroundedHypergraph(XMLNode& _node) : StateGraph(_node) {
  this->SetName("GroundedHypergraph");

  m_queryStrategy = _node.Read("queryStrategy",true,"",
                      "MPStrategy label to query roadaps.");

  m_queryStaticStrategy = _node.Read("queryStaticStrategy",false,m_queryStrategy,
                      "Strategy method to compute paths over static roadmaps.");
}

GroundedHypergraph::
~GroundedHypergraph() {}

/*-------------------------------- Interface ---------------------------------*/

void
GroundedHypergraph::
Initialize() {
  m_hypergraph = std::unique_ptr<GH>(new GH());
}

void
GroundedHypergraph::
SetStartSet(const std::set<VID>& _startSet) {

}

bool
GroundedHypergraph::
ConnectTransition(const VID& _tail, const VID& _head, const PathConstraints& _pathConstraints,
                  bool _bidirectional, std::set<Robot*> _avoid) {
  
  // std::cout << "\nConnect Transitions in Grounded Hypergraph: " << _tail << " --> " << _head << std::endl;
  // auto plan = this->GetPlan();
  // auto stats = plan->GetStatClass();

  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    robot->SetVirtual(true);
  }

  // auto plan = this->GetPlan();
  // auto stats = plan->GetStatClass();

  auto vertex1 = m_hypergraph->GetVertex(_tail);
  auto vertex2 = m_hypergraph->GetVertex(_head);

  // MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() + "::ConnectTransition::" + std::to_string(vertex1.vid) + "+" + std::to_string(vertex2.vid));

  // MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() 
  //                     + "::ConnectTransitions::ConnectTransition_" 
  //                     + std::to_string(m_count));

  // Create start constraint from vertex
  auto startGcfg = vertex1.property.first->GetVertex(vertex1.property.second);
  auto grm = startGcfg.GetGroupRoadmap();
  auto group = grm->GetGroup();
  if(m_debug) {
    std::cout << "group name: " << group->GetLabel() << std::endl;
    std::cout << "=== Start === " << std::endl;
  }
  std::vector<CSpaceConstraint> startConstraints;

  for(auto robot : group->GetRobots()) {
    CSpaceConstraint startConstraint(robot,startGcfg.GetRobotCfg(robot));
    startConstraints.push_back(startConstraint);
    if(m_debug) {
      std::cout << "\trobot name: " << robot->GetLabel() << std::endl;
      std::cout << "\tstart cfg: " << startConstraint.GetBoundary()->GetCenter() << std::endl;
    }
  }

  // Create goal constraint from vertex 2
  if(m_debug)
    std::cout << "=== Goal === " << std::endl;
  auto goalGcfg = vertex2.property.first->GetVertex(vertex2.property.second);
  std::vector<CSpaceConstraint> goalConstraints;

  for(auto robot : group->GetRobots()) {
    CSpaceConstraint goalConstraint(robot,goalGcfg.GetRobotCfg(robot));
    goalConstraints.push_back(goalConstraint);
    if(m_debug) {
      std::cout << "\trobot name: " << robot->GetLabel() << std::endl;
      std::cout << "\tgoal cfg: " << goalConstraint.GetBoundary()->GetCenter() << std::endl;
    }
  }

  // Create group task
  auto groupTask = std::shared_ptr<GroupTask>(new GroupTask(group));

  if(m_debug)
    std::cout << "\nCreate individual robot task" << std::endl;
  for(size_t i = 0; i < goalConstraints.size(); i++) {
    // Create individual robot task

    const auto& startConstraint = startConstraints[i];
    const auto& goalConstraint = goalConstraints[i];

    auto robot = startConstraint.GetRobot();
    if(m_debug) {
      std::cout << "robot: " << robot->GetLabel() << std::endl;
      std::cout << " - start constraint: " << startConstraint << std::endl;
      std::cout << " - goal constraint: " << goalConstraint << std::endl;
    }
    if(robot != goalConstraint.GetRobot())
      throw RunTimeException(WHERE) << "Mismatching robots.";

    MPTask task(robot);
    task.SetStartConstraint(std::move(startConstraint.Clone()));
    task.AddGoalConstraint(std::move(goalConstraint.Clone()));
    if(m_debug) 
      std::cout << "size of path constraints: " << _pathConstraints.size() << std::endl;
    for(const auto& c : _pathConstraints) {
      if(m_debug) 
        std::cout << "pathConstraints: " << *(c->GetBoundary()) << std::endl;
      if(c->GetRobot() == robot) {
        if(m_debug) 
          std::cout << "adding path constraint to robot " << robot->GetLabel() << std::endl;
        task.AddPathConstraint(std::move(c->Clone()));
      }
    }

    groupTask->AddTask(task);
    std::cout << std::endl;
  }
  std::cout << std::endl;
  // Set active formation constraints
  //auto formations = mode->formations;
  //auto grm = this->GetMPSolution()->GetGroupRoadmap(group);
  //grm->SetAllFormationsInactive();
  //for(auto f : formations) {
  //  grm->SetFormationActive(f);
  //}
  auto formations = startGcfg.GetFormations();
  grm->SetAllFormationsInactive();
  for(auto f : formations) {
    if(m_debug) 
      std::cout << "formation: " << f << std::endl;
    grm->SetFormationActive(f);
  }

  // Set robots not virtual
  for(auto robot : grm->GetGroup()->GetRobots()) {
    robot->SetVirtual(false);
  }

  for(const auto& robot : prob->GetRobots()) {
    if(_avoid.count(robot.get())) {
      robot->SetVirtual(false);
    }
  }

  if(m_debug) {
    auto cfg1 = vertex1.property.first->GetVertex(vertex1.property.second);
    auto cfg2 = vertex2.property.first->GetVertex(vertex2.property.second);
    std::cout << "Querying path for " << cfg1.GetGroupRoadmap()->GetGroup()->GetLabel() << std::endl;
    std::cout << "\tFrom: " << cfg1.PrettyPrint() << std::endl;
    std::cout << "\tTo: " << cfg2.PrettyPrint() << std::endl;
  }

  // lib->SetTask(nullptr);
  // lib->SetGroupTask(groupTask.get());

  // Query path for task
  lib->SetPreserveHooks(true);
  lib->Solve(prob,groupTask.get(),this->GetMPSolution(),m_queryStrategy, LRand(), 
      "Query transition path");
  lib->SetPreserveHooks(false);
  std::cout << "Checkpoint" << std::endl;

  grm->SetAllFormationsInactive();

  // Return all robots back to non-virtual
  //for(auto robot : grm->GetGroup()->GetRobots()) {
  //  robot->SetVirtual(true);
  //}
  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    if(this->GetPlan()->GetCoordinator()->GetRobot() == robot.get())
      continue;
    robot->SetVirtual(false);
  }

  // Extract cost of path from solution
  auto path = this->GetMPSolution()->GetGroupPath(groupTask->GetRobotGroup());

  if(m_debug and !path->Empty()) {
    std::cout << "Path for transition: " << _tail << " --> " << _head << std::endl;
    std::cout << "Path timesteps: " << path->TimeSteps  () << std::endl;
    std::cout << "Path length: " << path->Cfgs().size() << std::endl;
    for(const auto& cfg : path->Cfgs()) {
      std::cout << "\t" << cfg.PrettyPrint() << std::endl;
    }
    std::cout << std::endl << std::endl;

    std::cout << "Full Path length: " << path->FullCfgs(lib).size() << std::endl;
    for(const auto& cfg : path->FullCfgs(lib)) {
      std::cout << "\t" << cfg.PrettyPrint() << std::endl;
    }
    std::cout << std::endl;
  }

  if(path->Empty()) {
    if(m_debug) {
      std::cout << "Failed to find a path for: " 
        << _tail 
        << " --> " 
        << _head
        << std::endl;
    }
    //ConnectTransition(_tail, _head, _pathConstraints,_bidirectional);
    // delete mt;

    return false;
  }

  Transition transition;
  transition.taskSet.push_back({groupTask});
  transition.cost = path->TimeSteps();
  //transition.taskFormations[groupTask.get()] = formations;

  // Add arc to hypergraph
  AddTransition({_tail},{_head},transition,true);

  if(_bidirectional) {
    AddTransition({_head},{_tail},transition,true);
  }
  
  // delete mt;

  return true;
}

bool
GroundedHypergraph::
ConnectAllTransitions(const std::vector<VID>& _vertices, const PathConstraints& _pathConstraints,
                      const bool& _bidirectional) {
  std::cout << "ConnectAllTransitions GVIDs: " << _vertices << std::endl;
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() + "::ConnectTransitions");
  auto prob = this->GetMPProblem();
  bool res = false;

  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    robot->SetVirtual(true);
  }

  // For each actuated mode in the mode hypergraph, attempt to connect grounded transition samples
  int cnt = 0;
  for(size_t i = 0; i < _vertices.size(); i++) {
    for(size_t j = (_bidirectional) ? 0 : i; j < _vertices.size(); j++) {
      cnt++;
      auto vid1 = _vertices[i];
      auto vid2 = _vertices[j];
      // std::cout << ">> iteration " << cnt << ":" << vid1 << " + " << vid2 << std::endl; 

      // Make sure vertices are unique
      if(vid1 == vid2)
        continue;
      std::cout << "vid1: " << vid1 << " ... " << "vid2: " << vid2 << std::endl;
      // Set robots non-virtual
      auto vertex = GetVertex(vid1);
      auto group = vertex.first->GetGroup();
      for(auto robot : group->GetRobots()) {
        robot->SetVirtual(false);
      }
      res = ConnectTransition(vid1,vid2,_pathConstraints,_bidirectional);

      m_count += 1;
      // Set robots back to virtual
      for(auto robot : group->GetRobots()) {
        robot->SetVirtual(true);
      }
    }
  }

  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    if(this->GetPlan()->GetCoordinator()->GetRobot() == robot.get())
      continue;
    robot->SetVirtual(false);
  }

  delete mt;
  // Strange behavior to avoid issues with roadmaps being improved after costs are saved
  if(m_queryStrategy != m_queryStaticStrategy) {
    std::cout << "m_queryStrategy is different from m_queryStrategyStatic (Grounded Hypergraph)" << std::endl;
    auto stash = m_queryStrategy;
    m_queryStrategy = m_queryStaticStrategy;
    ConnectAllTransitions(_vertices,_pathConstraints,_bidirectional);
    m_queryStrategy = stash;
  }

  return res;
}

bool
GroundedHypergraph::
ContainsSolution() {
  return false;
}

/*---------------------------- Vertex Accessors ------------------------------*/

GroundedHypergraph::VID
GroundedHypergraph::
AddVertex(const Vertex& _vertex) {
  return m_hypergraph->AddVertex(_vertex);
}

GroundedHypergraph::Vertex
GroundedHypergraph::
GetVertex(const VID& _vid) {
  return m_hypergraph->GetVertex(_vid).property;
}

/*--------------------------- Hyperarc Accessors ------------------------------*/

GroundedHypergraph::HID
GroundedHypergraph::
AddTransition(const std::set<VID>& _tail, const std::set<VID>& _head,
          const Transition& _transition, const bool& _override,
          const bool& _checkExists) {
  // TODO::Flip head/tail order in Hypergraph class
  return m_hypergraph->AddHyperarc(_head,_tail,_transition,_override,_checkExists);
}

GroundedHypergraph::GH::Hyperarc
GroundedHypergraph::
GetHyperarc(const HID& _hid) {
  return m_hypergraph->GetHyperarc(_hid);
}

GroundedHypergraph::HID
GroundedHypergraph::
GetHID(const std::set<VID>& _tail, const std::set<VID>& _head) {
  return m_hypergraph->GetHID(_head,_tail);
}

GroundedHypergraph::Transition
GroundedHypergraph::
GetTransition(const HID& _hid) {
  return m_hypergraph->GetHyperarc(_hid).property;
}

GroundedHypergraph::Transition
GroundedHypergraph::
GetTransition(const std::set<VID>& _tail, const std::set<VID>& _head) {
  // TODO::Flip head/tail order in Hypergraph class
  auto hid = m_hypergraph->GetHID(_head,_tail);
  return GetTransition(hid);
}

const std::set<GroundedHypergraph::HID>
GroundedHypergraph::
GetOutgoingHyperarcs(const VID& _vid) {
  return m_hypergraph->GetOutgoingHyperarcs(_vid);
}

const std::unordered_map<size_t,GroundedHypergraph::GH::Hyperarc>&
GroundedHypergraph::
GetHyperarcMap() {
  return m_hypergraph->GetHyperarcMap();
}



/*------------------------- Miscellaneous Accessors --------------------------*/

GroundedHypergraph::GH::GraphType*
GroundedHypergraph::
GetReverseGraph() {
  return m_hypergraph->GetReverseGraph();
}

void
GroundedHypergraph::
Print() {
  m_hypergraph->Print();
}

void
GroundedHypergraph::
PrintModes() {
  m_hypergraph->PrintModes();
}

void
GroundedHypergraph::
PrintGraphWithModes() {
  m_hypergraph->PrintGraphWithModes();
}

size_t
GroundedHypergraph::
Size() {
  return m_hypergraph->Size();
}

size_t
GroundedHypergraph::
EdgeSize() {
  return m_hypergraph->EdgeSize();
}

// void
// Hypergraph<VertexType,HyperarcType>::
// PrintModes() const {
//   int vid = 0;
//   while (true) {
//     try {
//       std::cout << this->GetVertex(vid) << vid << ": " << this->m_hypergraph->GetVertex(vid).property.first->GetGroup()->GetLabel() << std::endl;
//       vid++;
//     }
//     catch (...) {
//       break;
//     }
//   }
// }

/*---------------------------- Helper Functions ------------------------------*/


/*----------------------------------------------------------------------------*/

istream&
operator>>(istream& _is, const GroundedHypergraph::Vertex _vertex) {
  return _is;
}

ostream&
operator<<(ostream& _os, const GroundedHypergraph::Vertex _vertex) {
  return _os;
}

istream&
operator>>(istream& _is, const GroundedHypergraph::Transition _t) {
  return _is;
}

ostream&
operator<<(ostream& _os, const GroundedHypergraph::Transition _t) {
  return _os;
}
