#include "ModeGraph.h"

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

#include <set>

/*------------------------------ Construction --------------------------------*/

ModeGraph::
ModeGraph() {
  this->SetName("ModeGraph");
}

ModeGraph::
ModeGraph(XMLNode& _node) : StateGraph(_node) {
  this->SetName("ModeGraph");

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
}

ModeGraph::
~ModeGraph() {}

/*-------------------------------- Interface ---------------------------------*/

void
ModeGraph::
Initialize() {
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  gh->Initialize();


  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::Initialize");

  
  GenerateRepresentation();

}


void
ModeGraph::
GenerateRepresentation() {
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

  auto initialModes = AddStartState(start);

  // Mode hypergraph
  GenerateModeHypergraph(initialModes);
  // Scott - Generated Mode Hypergraph.
  // Grounded Hypergraph is yet generated. 



  // Grounded hypergraph
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  GroundedVertex origin = std::make_pair(nullptr,0);
  auto originVID = gh->AddVertex(origin);
  GroundedVertex goal = std::make_pair(nullptr,MAX_UINT);
  auto goalVID = gh->AddVertex(goal);

  std::set<VID> startVIDs;
  std::set<VID> goalVIDs;

  std::cout << "MODE HYPERGRAPH" << std::endl;
  m_modeHypergraph.Print();

  std::cout << "Mode : Label" << std::endl;
  auto vm = m_modeHypergraph.GetVertexMap();
  for (auto kv : vm) {
    auto mode = kv.first;
    auto label = kv.second.property->robotGroup->GetLabel();
    std::cout << mode << " : " << label << std::endl;
  }


  std::cout << "HERE0" << std::endl;
  do {

    std::cout << "HERE1" << std::endl;
    SampleNonActuatedCfgs(start,startVIDs,goalVIDs);
    std::cout << "HERE2" << std::endl;
    SampleTransitions();
    // SampleTransitions();

    std::cout << "HERE3" << std::endl;
    GenerateRoadmaps(start,startVIDs,goalVIDs);
    std::cout << "HERE4" << std::endl;
    ConnectTransitions(); 

    GroundedHypergraph::Transition fromOrigin;
    fromOrigin.cost = -1;
    
    std::cout << "HERE5" << std::endl;
    //if(m_groundedHypergraph.GetHID(startVIDs,{originVID}) == MAX_UINT)
    std::cout << gh->GetHID({originVID},startVIDs) << std::endl;
    if(gh->GetHID({originVID},startVIDs) == MAX_UINT){
      std::cout << "Adding start transition" << std::endl;
      gh->AddTransition({originVID},startVIDs,fromOrigin);
    }
    std::cout << gh->GetHID({originVID},startVIDs) << std::endl;

    //Transition toGoal;
  
    std::cout << "HERE6" << std::endl;
    if(!m_allGoals) {
      std::cout << "Configuring Goal Sets" << std::endl;
      ConfigureGoalSets(goalVID, goalVIDs);
      m_allGoals = true; //TODO::Find a better way to recongnize this rather than assuming it's true
    }
    std::cout << "HERE7" << std::endl;

  } while(!ContainsSolution(startVIDs));

  // if(m_debug) {
  std::cout << "MODE HYPERGRAPH" << std::endl;
  m_modeHypergraph.Print();
  // std::cout << "GROUNDED HYPERGRAPH" << std::endl;
  // gh->Print();
  std::cout << "GROUNDED HYPERGRAPH WITH MODE" << std::endl;
  gh->PrintGraphWithModes();
  std::cout << "GROUNDED HYPERGRAPH MODE TYPE" << std::endl;
  gh->PrintModes();
  std::cout << "GROUNDED HYPERARC COSTS" << std::endl;
  // for(auto kv : m_groundedHypergraph.GetHyperarcMap()) {
  //   auto hid = kv.first;
  //   auto hyperarc = kv.second;
  //   std::cout << hid << " : " << hyperarc.property.cost << std::endl;
  // }

  if(m_writeHypergraphs) {
    std::string base = this->GetMPProblem()->GetBaseFilename();

    m_modeHypergraph.Print(base + "-mode-hypergraph.hyp");
    //m_groundedHypergraph.Print(base + "-grounded-hypergraph.hyp");
  }
}

/*-------------------------------- Accessors ---------------------------------*/

ModeGraph::ModeHypergraph&
ModeGraph::
GetModeHypergraph() {
  return m_modeHypergraph;
}

ModeGraph::GroundedHypergraphLocal&
ModeGraph::
GetGroundedHypergraphLocal() {
  return m_groundedHypergraph;
}

ModeGraph::GroupRoadmapType*
ModeGraph::
GetGroupRoadmap(RobotGroup* _group) {
  return this->GetMPSolution()->GetGroupRoadmap(_group);
}

/*MPSolution* 
ModeGraph::
GetMPSolution() {
  return this->GetMPSolution().get();
}*/
    
ModeGraph::VID 
ModeGraph::
GetModeOfGroundedVID(const VID& _vid) const {
  for(const auto& kv : m_modeGroundedVertices) {
    if(kv.second.count(_vid)){
      return kv.first; 
    }
  }
  return MAX_UINT;
}


/*---------------------------- Helper Functions ------------------------------*/

std::vector<ModeGraph::VID>
ModeGraph::
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
ModeGraph::
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
      std::cout << "vid: " << vid << std::endl;
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
ModeGraph::
SampleNonActuatedCfgs(const State& _start, std::set<VID>& _startVIDs, std::set<VID>& _goalVIDs) {

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

    for(auto robot : mode->robotGroup->GetRobots()) {
      robot->SetVirtual(false);
    }

    m_unactuatedModes.insert(kv.first);

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
      _startVIDs.insert(groundedVID);
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
                                      << ".";
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
      _goalVIDs.insert(groundedVID);
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
ModeGraph::
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
ModeGraph::
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
ModeGraph::
SampleTransitions() {

  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::SampleTransitions");
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());

  // Set all robots and objects to virtual
  auto c = this->GetPlan()->GetCoordinator();
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(true);
    }
  }

  if(m_debug){
    std::cout << "ModeGraph::SampleTransition. Sampling Transitions" << std::endl;
  }

  // For each edge in the mode graph, generate n samples
  for(auto& kv : m_modeHypergraph.GetHyperarcMap()) {

    auto& hyperarc = kv.second;
    if(m_groundedInstanceTracker.find(kv.first) == m_groundedInstanceTracker.end()) {
      m_groundedInstanceTracker[kv.first] = 0;
    }
    
    // Check if hyperarc is a reversed action, and only plan
    // the forward actions as the reverse will also be saved
    if(hyperarc.property.second) {
      continue;
    }

    auto interaction = dynamic_cast<Interaction*>(hyperarc.property.first);

    State modeSet;
    std::unordered_map<RobotGroup*,Mode*> tailModeMap;
    std::unordered_map<RobotGroup*,Mode*> headModeMap;
    std::set<std::pair<size_t,RobotGroup*>> unactuatedModes;


    for(auto vid : hyperarc.tail) {
      auto mode = m_modeHypergraph.GetVertex(vid).property;
      modeSet[mode->robotGroup] = std::make_pair(nullptr,MAX_UINT);
      tailModeMap[mode->robotGroup] = mode;
      if(m_debug)
        std::cout << "Tail MVID: " << vid << " / " << mode->robotGroup->GetLabel() << std::endl;
      if(m_unactuatedModes.count(vid)) {
        if(m_debug)
          std::cout << "  adding unactuated tail mode: " << vid << std::endl;
        unactuatedModes.insert(std::make_pair(vid,mode->robotGroup));
      }
    }

    for(auto vid : hyperarc.head) {
      auto mode = m_modeHypergraph.GetVertex(vid).property;
      headModeMap[mode->robotGroup] = mode;
    }

    auto label = interaction->GetInteractionStrategyLabel();
    auto is = this->GetInteractionStrategyMethod(label);

    // Set robots involved as non virtual
    for(auto kv : modeSet) {
      auto group = kv.first;
      for(auto r : group->GetRobots()) {
        r->SetVirtual(false);
      }
    }

    bool foundInteraction = false;
    bool canReach = false;
    // If this hyperarc involves an unactuated mode, use the grounded vertices
    if(!unactuatedModes.empty()) {
      std::cout << "unactuatedModes is not empty" << std::endl;
      if(unactuatedModes.size() > 1)
        throw RunTimeException(WHERE) << "Multiple unactuated modes in an "
                      "interaction not currently supported.";

      auto unactuatedVID = unactuatedModes.begin()->first;
      auto unactuatedGroup = unactuatedModes.begin()->second;

      auto groundedVertices = m_modeGroundedVertices[unactuatedVID];
        
      if(m_debug) {
        std::cout << "  Unactuated MVID: " << unactuatedVID << std::endl;
        std::cout << "  groundedVertices: " ;
        for (auto iter = groundedVertices.begin(); iter !=groundedVertices.end(); iter++) {
          std::cout << *iter << " " ;
        }
        std::cout << " (size of: " << groundedVertices.size() << ")" << std::endl;
      }

      //for(auto gv : m_modeGroundedVertices[unactuatedVID]) {
      for(auto iter = groundedVertices.begin(); iter != groundedVertices.end(); iter++) {
        // Get grounded vertex
        //auto groundedVertex = m_groundedHypergraph.GetVertexType(*iter);
        if(m_debug)
          std::cout << "    Unactuated GVID: " << *iter << std::endl;
        auto groundedVertex = gh->GetVertex(*iter);

        auto startSet = modeSet;
        startSet[unactuatedGroup] = groundedVertex;

        // Check if robot can even reach pose
        //if(!CanReach(startSet))
        //  continue;

        canReach = true;

        for(size_t j = 0; j < m_maxAttempts; j++) {
          // Make state copy and add grounded vertex to pass to IS.
          // Will get overwritten as goal state
          auto goalSet = startSet;
          if(!is->operator()(interaction,goalSet)) {
            if(m_debug)
              std::cout << "    Failed to find intercation: " << *iter << std::endl;
            continue;
          }

          foundInteraction = true;
          if(m_debug)
            std::cout << "    Found an interaction: GVID " << *iter << std::endl;
          m_groundedInstanceTracker[kv.first] = m_groundedInstanceTracker[kv.first] + 1;
          // Save interaction paths
          SaveInteractionPaths(interaction,modeSet,goalSet,tailModeMap,headModeMap);
          break;
        }
      }
    }
    // Otherwise, sample completely new grounded vertices
    else {
      if(m_debug)
        std::cout << "Sampling new vertices" << std::endl;
      for(size_t i = 0; i < m_numInteractionSamples; i++) {
        for(size_t j = 0; j < m_maxAttempts; j++) {
          // Make state copy to pass by ref and get output state
          auto goalSet = modeSet;

          // Check if robots can even reach to perform interaction
          // if(!CanReach(goalSet)) {
          //   std::cout << "cannot reach" << std::endl;            
          //   continue;
          // }

          canReach = true;

          if(!is->operator()(interaction,goalSet)) {
            std::cout << "    Failed to find intercation: " << std::endl;
            continue;
          }

          foundInteraction = true;
          if(m_debug)
            std::cout << "    Found an interaction: GVID " << std::endl;

          m_groundedInstanceTracker[kv.first] = m_groundedInstanceTracker[kv.first] + 1;

          // Save interaction paths
          SaveInteractionPaths(interaction,modeSet,goalSet,tailModeMap,headModeMap); // interaction, start, goal
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

    if(m_debug and !foundInteraction and canReach) {
      std::cout << "Failed to find interaction for " << interaction->GetLabel()
                << " with starting mode";
      for(auto kv : modeSet) {
        auto group = kv.first;
        std::cout << " " << group->GetLabel();
      }
      std::cout << std::endl;
    }
  }

  // Set all robots and objects to back to non-virtual
  for(auto& kv : c->GetInitialRobotGroups()) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {
      r->SetVirtual(false);
    }
  }

  if(m_debug) {
    std::cout << "Grounding instance count of each mode hyperarc." << std::endl;
    for(auto kv : m_groundedInstanceTracker) {
      auto hid = kv.first;
      auto count = kv.second;
      std::cout << hid << " : " << count << std::endl;
    }
  }
}

void
ModeGraph::
GenerateRoadmaps(const State& _start, std::set<VID>& _startVIDs, std::set<VID>& _goalVIDs) {

  auto plan = this->GetPlan();
  auto c = plan->GetCoordinator();
  auto stats = plan->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel() + "::GenerateRoadmaps");

  auto decomp = plan->GetDecomposition();
  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
  auto qSM = lib->GetSampler(m_querySM);
  lib->SetMPSolution(this->GetMPSolution());

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());

  std::map<Robot*,Robot*> plannedRoadmapMap;

  // Set all robots to virtual
  for(auto pair : c->GetInitialRobotGroups()) {
    auto group = pair.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(true);
    }
  }
  if(m_debug)
    std::cout << "ModeGraph::GenerateRoadmaps Actuated Roadmap" << std::endl;
  for(auto& kv : m_modeHypergraph.GetVertexMap()) {
    // Check if mode is actuated
    auto mode = kv.second.property;
    if(m_unactuatedModes.count(kv.first)){
      continue;
    }

    if(m_debug)
      std::cout << "Checking the start and goal\n" << kv.first << std::endl;
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
      if(m_debug){
        std::cout << "Adding START vertex to the grounded hypergraph" << std::endl;
        std::cout << "Local VID: " << vid << std::endl;
      }
      auto groundedVID = gh->AddVertex(gv);
      if(m_debug)
        std::cout << "GVID: " << groundedVID << std::endl;

      m_modeGroundedVertices[kv.first].insert(groundedVID);
      _startVIDs.insert(groundedVID);
      m_entryVertices.insert(groundedVID);
    }

    // Check if mode is in the goal conditions
    for(auto st : decomp->GetGroupMotionTasks()) {
      auto task = st->GetGroupMotionTask().get();
      if(!task or task->GetRobotGroup() != mode->robotGroup)
        continue;

      // Sample goal cfg
      lib->SetGroupTask(task);
      std::map<Robot*, const Boundary*> boundaryMap;
      for(auto iter = task->begin(); iter != task->end(); iter++) {
        auto c = dynamic_cast<BoundaryConstraint*>(iter->GetGoalConstraints()[0].get());
        auto b = c->GetBoundary();
        boundaryMap[iter->GetRobot()] = b;
      }

      std::vector<GroupCfgType> samples;
      qSM->Sample(1,m_maxAttempts,boundaryMap,std::back_inserter(samples));
      
      if(samples.size() == 0)
        throw RunTimeException(WHERE) << "Unable to generate goal configuration for"
                                      << mode->robotGroup->GetLabel()
                                      << ".";
      // Add goal cfg to grounded hypergraph
      //auto gcfg = samples[0].SetGroupRoadmap(grm);
      auto gcfg = samples[0];
      gcfg.SetGroupRoadmap(grm);
      auto vid = grm->AddVertex(gcfg);
      auto gv = std::make_pair(grm,vid);

      //auto groundedVID = m_groundedHypergraph.AddVertex(gv);
      if(m_debug){
        std::cout << "Adding GOAL vertex to the grounded hypergraph" << std::endl;
        std::cout << "Local VID: " << vid << std::endl;
      }
      auto groundedVID = gh->AddVertex(gv);
      if(m_debug)
        std::cout << "GVID: " << groundedVID << std::endl;
      
      m_modeGroundedVertices[kv.first].insert(groundedVID);
      _goalVIDs.insert(groundedVID);
      m_exitVertices.insert(groundedVID);
    }
  }
  if(m_debug)
    std::cout << "************" << std::endl;
  /*for(const auto& kv : _start) {
    auto group = kv.first;
    auto grm = kv.second.first;
    auto vid = kv.second.second;
    auto gcfg = grm->GetVertex(vid);

    auto newGrm = this->GetMPSolution()->GetGroupRoadmap(group);
    auto newGcfg = gcfg.SetGroupRoadmap(newGrm);
    newGrm->AddVertex(newGcfg);
  }*/

  /*
  auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();
  */
  if(m_debug)
    std::cout << "Now We Generate the Roadmap" << std::endl;
  // For each actuated mode in the mode hypergraph, run the expansion strategy
  for(auto kv : m_modeHypergraph.GetVertexMap()) {
    std::cout << "--1--" << std::endl;
    auto vertex = kv.second;
    auto mode = vertex.property;

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
      continue;
    }
    std::cout << "--2--" << std::endl;
      

    // Initialize dummy task
    auto task = new GroupTask(mode->robotGroup);

    for(auto r : mode->robotGroup->GetRobots()) {
      auto t = MPTask(r);

      // Add start constraints
      auto startCfg = prob->GetInitialCfg(r);
      auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(r,startCfg));
      t.SetStartConstraint(std::move(startConstraint));

      // Add mode/path constraints
      for(const auto& c : mode->constraints) {
        if(c->GetRobot() != r){
          continue;
        }
        t.AddPathConstraint(std::move(c->Clone()));
      }
      task->AddTask(t);
    } 
    std::cout << "--3--" << std::endl;

    // Set active formation constraints
    auto formations = mode->formations;
    auto grm = this->GetMPSolution()->GetGroupRoadmap(mode->robotGroup);
    grm->SetAllFormationsInactive();
    for(auto f : formations) {
      grm->SetFormationActive(f);
    }
    std::cout << "--4--" << std::endl;

    // Set robots not virtual
    for(auto robot : grm->GetGroup()->GetRobots()) {
      robot->SetVirtual(false);
    }
    std::cout << "--5--" << std::endl;

    // Call the MPLibrary solve function to expand the roadmap
    lib->SetPreserveHooks(true);
    lib->Solve(prob,task,this->GetMPSolution(),m_expansionStrategy, LRand(), 
            "ExpandModeRoadmap");
    lib->SetPreserveHooks(false);

    // Set robots not virtual
    for(auto robot : grm->GetGroup()->GetRobots()) {
      robot->SetVirtual(true);
    }
    std::cout << "--6--" << std::endl;

    delete task;

    if(actuated and passive) {
      std::cout << "actuated and passive: " << mode->robotGroup->GetLabel() <<  std::endl;
      plannedRoadmapMap[actuated] = passive;
    }
    std::cout << "--7--" << std::endl;

    // this->GetMPSolution()->GetGroupRoadmap(mode->robotGroup)->Write(this->GetMPProblem()->GetBaseFilename()+"::"+mode->robotGroup->GetLabel() + ".map", this->GetMPProblem()->GetEnvironment());
    // std::cout << "Saved Roadmap of " << mode->robotGroup->GetLabel() << std::endl;
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

void
ModeGraph::
ConnectTransitions() {
  auto plan = this->GetPlan();
  auto stats = plan->GetStatClass();
  MethodTimer* mt = new MethodTimer(stats,this->GetNameAndLabel() + "::ConnectTransitions");

  std::cout << "Print grounded vertex" << std::endl;
  for(auto& kv1 : m_modeHypergraph.GetVertexMap()) {
    std::cout << kv1.first << ": " << std::endl;
    for(auto vid : m_modeGroundedVertices[kv1.first]) {
      std::cout << vid << " " ;
    }
    std::cout << std::endl;
  }

  //auto lib = this->GetMPLibrary();
  auto prob = this->GetMPProblem();

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());

  // Set robots virtual
  for(const auto& robot : prob->GetRobots()) {
    robot->SetVirtual(true);
  }

  // For each actuated mode in the mode hypergraph, attempt to connect grounded transition samples
  for(auto kv1 : m_modeHypergraph.GetVertexMap()) {
    
    std::cout << "Connecting transitions of mode: " << kv1.first << std::endl;

    if(m_unactuatedModes.count(kv1.first))
      continue;

    auto mode = m_modeHypergraph.GetVertexType(kv1.first);
    std::vector<size_t> temp;
    for(auto vid : m_modeGroundedVertices[kv1.first]) {
      temp.push_back(vid);
    }
    gh->ConnectAllTransitions(temp,mode->constraints);

  }

  delete mt;
  // Strange behavior to avoid issues with roadmaps being improved after costs are saved
  if(m_queryStrategy != m_queryStrategyStatic) {
    auto stash = m_queryStrategy;
    m_queryStrategy = m_queryStrategyStatic;
    ConnectTransitions();
    m_queryStrategy = stash;
  }

}

void
ModeGraph::
ApplyAction(Action* _action, std::set<std::vector<VID>>& _applied, std::vector<VID>& _newModes, bool _forward) {
  auto as = this->GetTMPLibrary()->GetActionSpace();

  std::cout << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+" << std::endl;
  std::cout << "   Action: " << _action->GetLabel() << std::endl;
  std::cout << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+" << std::endl;

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
      std::cout << "saving motion Constraints: " << role << std::endl;
      auto constraint = m->GetRoleConstraint(role);
      motionConstraintMap[role] = constraint;
    }
  }
  std::cout << "motion constraint size: " << initialMotionConditions.size() << std::endl;
  std::cout << "formation constraint size: " << initialFormationConditions.size() << std::endl;

  // Look at possible combinations of modes in the mode hyerpgraph 
  // that satisfy the formation constraint
  
  std::vector<std::vector<VID>> formationModes(initialFormationConditions.size());

  std::cout << "===========================================" << std::endl;
  for(size_t i = 0; i < initialFormationConditions.size(); i++) {
    auto f = initialFormationConditions[i];
    std::cout << "--------------" << std::endl;
    std::cout << "initial conditions for the formation constraints are: " << std::endl;
    int cnt = 1;
    for(auto type : f->GetTypes()) {
      std::cout << cnt << ". " << type << std::endl;
      cnt++;
    }

    for(const auto& kv : m_modeHypergraph.GetVertexMap()) {
      auto vid = kv.first;
      auto mode = kv.second.property;
      std::cout << "(+) vid is : " << vid << " ( " << mode->robotGroup->GetLabel() << std::endl;
      std::set<Robot*> used;
      for(auto type : f->GetTypes()) {
        std::cout << "constraint type: " << type << " ?= " << "mode capability: " << mode->robotGroup->GetLabel() << std::endl;;
        for(auto robot : mode->robotGroup->GetRobots()) {
          // Make sure robot has not been accounted for
          if(used.count(robot)){
            std::cout << "already inserted " << std::endl;
            continue;
          }

          // Reserve robot if it is a match
          if(robot->GetCapability() == type) {
            std::cout << robot->GetCapability() << " == " << type << std::endl;
            used.insert(robot);
            std::cout << "insert robot " << robot->GetLabel() << std::endl;
          }
        }
      }

      // Check if the number of saved robots matches the required number
      if(f->GetTypes().size() != used.size() or used.size() != mode->robotGroup->Size()) {
        std::cout << "size not matched: " << f->GetTypes().size() << " " << used.size() << " " << mode->robotGroup->Size() << std::endl;
        continue;
      }
      else {
        std::cout << "size matched!: " << f->GetTypes().size() << " " << used.size() << " " << mode->robotGroup->Size() << std::endl;
      }


      // Check if mode meets formation requirements
      // Create state
      State state;
      state[mode->robotGroup] = std::make_pair(nullptr,MAX_UINT);
      std::unordered_map<std::string,Robot*> roleMap;
      std::cout << "Assign Roles 1" << std::endl;
      f->AssignRoles(roleMap,state);
      std::cout << "Assigned Roles are: " << std::endl;
      for(auto kv : roleMap) {
        std::cout << kv.first << "--> " << kv.second->GetLabel() << std::endl;
      }
      bool satisfied = mode->formations.empty();
      for(auto formation : mode->formations) {
        if(f->DoesFormationMatch(roleMap,formation)) {
          std::cout << "formation matched. satisfied" << std::endl;
          satisfied = true;
          break;
        }
      }

      if(!satisfied) {
        std::cout << "Formation not match" << std::endl;
        continue;
      }

      // Check if mode meets motion constraints
      for(auto role : f->GetRoles()) {
        // Check if role has associated motion constraint
        auto iter1 = motionConstraintMap.find(role);
        if(iter1 == motionConstraintMap.end()){
          std::cout << "motion constraint not exist for role: " << role << std::endl;
          continue;
        }

        // Check if constraint is in the mode
        auto constraint = motionConstraintMap[role];
        auto b1 = constraint->GetBoundary();
        if(!b1) {
          std::cout << "Bounday for motion constraint not exists" << std::endl;
          continue;
        }

        for(const auto& c : mode->constraints) {
          // Check if boundaries are the same
          auto b2 = c->GetBoundary();

          std::cout << b1->Type() << " " << b2->Type() << std::endl;
          std::cout << b1->Name() << " " << b2->Name() << std::endl;
          std::cout << b1->GetDimension() << " " << b2->GetDimension() << std::endl;
          std::cout << b1->GetCenter() << " " << b2->GetCenter() << std::endl;

          if(b1->Type() == b2->Type()
            and b1->Name() == b2->Name()
            and b1->GetDimension() == b2->GetDimension()
            and b1->GetCenter() == b2->GetCenter()) {

            bool match = true;
            for(size_t i = 0; i < b1->GetDimension(); i++) {
              if(!(b1->GetRange(i) == b2->GetRange(i))) {
                match = false;
                std::cout << "match false" << std::endl;
              }
            }
            if(match)
              continue;
          }

          std::cout << "not satisfied" << std::endl;
          satisfied = false;
          break;
        }
      }

      if(satisfied) {
        std::cout << "satisfied! formation modes elements: " << std::endl;
        formationModes[i].push_back(vid);
        for (auto j : formationModes[i]) {
          std::cout << j << " ";
        }
        std::cout << std::endl;
      }
    }
  }

  // Convert potential individual mode assignments into mode sets
  std::vector<VID> partialSet;
  std::cout << "\nformationModes size: " << formationModes.size() << std::endl;
  auto modeSets = CollectModeSets(formationModes,0,partialSet);
  std::cout << "modeSets size: " << modeSets.size() << std::endl;
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
        std::cout << "modesets size: " << modeSets.size() << std::endl;
        std::cout << "types: " << types[j] << std::endl;

        for(auto robot : robots) {
          if(robot->GetCapability() == types[j]) {
            std::cout << "match. insert." << std::endl;
            matches.push_back(robot);
          }
        }

        possibleTypeAssignments[j] = matches;
      }

      possibleAssignments[i] = possibleTypeAssignments;
    }

    // Build output mode sets
    std::vector<Mode*> partial;
    for(auto i : possibleAssignments) {
      std::cout << "-------------------------" << std::endl;
      for(auto j : i) {
        std::cout << "    L-------------------------" << std::endl;
        for(auto k : j) {
          std::cout << "      " << k->GetLabel() << std::endl;
        }
      }
    }
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
        if(formation) {
          std::cout << "Add formation." << std::endl;
          mode->formations.insert(formation);
        }

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
        auto oldSize = m_modeHypergraph.Size();
        auto vid = AddMode(mode);
        if(m_debug)
          std::cout << "returned vid: " << vid << std::endl;
        std::cout << "check overlap" << std::endl;
        for(auto vid : tail) {
          std::cout << vid << std::endl;
        }
        if(tail.count(vid)) {
          std::cout << "overlaps true... break" << std::endl;
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
              std::cout << h1 << " " << h2 << std::endl;
              if(h1 != h2)
                continue;

              auto ha = m_modeHypergraph.GetHyperarc(h1);
              std::cout << "action compare: " << ha.property.first << " " << _action << std::endl;
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


      if(!exists) {
        m_modeHypergraph.AddHyperarc(head,tail,std::make_pair(_action,!_forward),true);
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
        m_modeHypergraph.AddHyperarc(tail,head,std::make_pair(_action,_forward),true);
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

std::vector<std::vector<ModeGraph::VID>>
ModeGraph::
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

std::vector<std::vector<ModeGraph::Mode*>>
ModeGraph::
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

std::vector<ModeGraph::Mode*>
ModeGraph::
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
ModeGraph::
SaveInteractionPaths(Interaction* _interaction, State& _start, State& _end, 
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

    // Collect individual robot paths
    /*std::unordered_map<Robot*,size_t> startVIDs;
    std::unordered_map<Robot*,size_t> endVIDs;
    std::unordered_map<Robot*,double> individualWeights;
    */
    for(auto path : paths) {
      const auto& cfgs = path->Cfgs();
      auto robot = path->GetRobot();
      /*auto rm = this->GetMPSolution()->GetRoadmap(robot);

      auto startVID = grm->AddVertex(cfgs.front());
      auto goalVID = grm->AddVertex(cfgs.back());

      startVIDs[robot] = startVID;      
      endVIDs[robot] = goalVID;
      individualWeights[robot] = path->TimeSteps();
      */

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
    
    /*// Add an edge in the underlying group roadmaps for these paths
    for(auto task : tasks) {
      auto group = task->GetRobotGroup();
      auto grm = this->GetMPSolution()->GetGroupRoadmap(group);
      if(!grm) {
        this->GetMPSolution()->AddRobotGroup(group); 
        grm = this->GetMPSolution()->GetGroupRoadmap(group);
      }

      GroupCfgType start(grm);
      GroupCfgType end(grm);
      double weight = 0;
      for(auto robot : group->GetRobots()) {
        start.SetRobotCfg(robot,std::move(startCfgs[robot]));
        end.SetRobotCfg(robot,std::move(endCfgs[robot]));
        weight = std::max(weight,individualWeights[robot]);
      }

      auto startVID = grm->AddVertex(start);
      auto goalVID = grm->AddVertex(end);

      GroupLocalPlan<Cfg> edge(grm);
      edge.SetWeight(weight);
      grm->AddEdge(startVID,goalVID,edge);
    }*/

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
  std::cout << "SaveInteraction.. Tail: " ;
  for(auto v : tail) {
    m_exitVertices.insert(v);
    std::cout << v << " ";
  }
  std::cout << std::endl;

  // Add the end state to the grounded vertices graph
  auto head = AddStateToGroundedHypergraphLocal(end,_endModeMap);
  std::cout << "SaveInteraction.. Head: " ;
  for(auto v : head) {
    m_entryVertices.insert(v);
    std::cout << v << " ";
  }
  std::cout << std::endl;

  // Make sure transition does not already exist in graph
  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  if(gh->GetHID(tail,head) == MAX_UINT) {
    // Save transition in hypergraph
    gh->AddTransition(tail,head,transition);
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
          auto start = iter->GetGoalConstraints()[0]->Clone();
          auto goal  = iter->GetStartConstraint()->Clone();
          MPTask t(iter->GetRobot());
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
    }
  }
}

ModeGraph::VID
ModeGraph::
AddMode(Mode* _mode) {

  std::cout << "start adding mode: " << _mode->robotGroup->GetLabel() << std::endl;
  // Check if mode already exists
  for(const auto& mode : m_modes) {
    std::cout << "~~ Comparing mode [" << m_modeHypergraph.GetVID(mode.get()) << "] with a new mode" << std::endl;
    // Check if robot groups are the same
    std::cout << mode->robotGroup->GetLabel() << " =? " << _mode->robotGroup->GetLabel() << std::endl;
    if(mode->robotGroup != _mode->robotGroup) {
      std::cout << "robots are different" << std::endl;
      continue;
    }

    // Check if both have same number of formations and constraints
    auto formations1 = mode->formations;
    auto formations2 = _mode->formations;

    if(formations1.size() != formations2.size()) {
      std::cout << "formation " << formations1.size() << " != " << formations2.size() << std::endl;
      continue;
    }

    const auto& constraints1 = mode->constraints;
    const auto& constraints2 = _mode->constraints;

    if(constraints1.size() != constraints2.size()) {
      std::cout << "constraints " << constraints1.size() << " != " << constraints2.size() << std::endl;
      continue;
    }

    // Check if the formations are the same
    bool fMatch = true;
    for(auto f1 : formations1) {

      bool match = false;

      for(auto f2 : formations2) {
        if(!f1 or !f2) {
          if(f1 == f2) {
            std::cout << "f1 and f2 are the same" << std::endl;
            match = true;
            break;
          }
        }
        else if(*f1 == *f2) {
          std::cout << "*f1 and *f2 are the same" << std::endl;
          match = true;
          break;
        }
      }

      if(!match) {
        std::cout << "formation not matched" << std::endl;
        fMatch = false;
        break;
      }
    }

    if(!fMatch) {
      std::cout << "formation not matched" << std::endl;
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
          std::cout << "b1 and b2 are the same" << std::endl;
          match = true;
          break;
        }
      }

      if(!match) {
        std::cout << "constraints not matched" << std::endl;
        cMatch = false;
        break;
      }
    }

    if(!cMatch) {
      std::cout << "constraints not matched" << std::endl;
      continue;
    }

    // Found existing copy of mode already saved
    std::cout << "Already exist in the mode hypergraph: [" << m_modeHypergraph.GetVID(mode.get()) << "] " << mode.get()->robotGroup->GetLabel() << std::endl;
    return m_modeHypergraph.GetVID(mode.get());
  }

  auto mode = std::unique_ptr<Mode>(_mode);
  m_modes.push_back(std::move(mode));

  std::cout << "Adding a new mode: " << m_modes.back().get()->robotGroup->GetLabel() << " ( ";
  return m_modeHypergraph.AddVertex(m_modes.back().get());
}

std::set<ModeGraph::VID>
ModeGraph::
AddStateToGroundedHypergraphLocal(const State& _state, std::unordered_map<RobotGroup*,Mode*> _modeMap) {

  auto gh = dynamic_cast<GroundedHypergraph*>(this->GetStateGraph(m_GH).get());
  std::set<VID> vids;

  for(const auto& kv : _state) {
    auto mode = _modeMap[kv.first];
    auto mvid = m_modeHypergraph.GetVID(mode);


    auto gvid = gh->AddVertex(kv.second);
    m_modeGroundedVertices[mvid].insert(gvid);

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

bool
ModeGraph::
CanReach(const State& _state) {
  
  // Construct bounding spheres
  std::vector<std::pair<Vector3d,double>> spheres;

  for(const auto& kv : _state) {
    auto group = kv.first;
    auto rm = kv.second.first;
    auto vid = kv.second.second;
    
    if(rm and vid != MAX_UINT) {
      auto gcfg = rm->GetVertex(vid);
      gcfg.ConfigureRobot();
    }

    for(auto robot : group->GetRobots()) {
      auto mb = robot->GetMultiBody();

      Vector3d center = mb->GetBase()->GetWorldTransformation().translation();
      double radius = rm ? 0 : std::numeric_limits<double>::infinity();

      if(!mb->IsPassive()) {
        // TODO::Compute proper radius - for now, we know UR5e is roughly 1 meter
        radius = .8;
      }

      spheres.emplace_back(center,radius);
    }
  }

  // Check if all spheres intersect
  for(size_t i = 0; i < spheres.size()-1; i++) {
    const auto& sphere1 = spheres[i];
    const auto& center1 = sphere1.first;
    const auto& radius1 = sphere1.second;
    
    for(size_t j = i+1; j < spheres.size(); j++) {
      const auto& sphere2 = spheres[j];
      const auto& center2 = sphere2.first;
      const auto& radius2 = sphere2.second;

      // Check if both spheres correspond to passive objects
      if(radius1 == 0 and radius2 == 0)
        continue;

      double distance = 0;
      for(size_t k = 0; k < 3; k++) {
        distance += std::pow((center1[k]-center2[k]),2);
      }
      distance = sqrt(distance);

      if(distance > radius1 + radius2)
        return false;
    }
  }
  
  return true;
}

bool
ModeGraph::
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

/*----------------------------------------------------------------------------*/

istream&
operator>>(istream& _is, const ModeGraph::Mode* _mode) {
  return _is;
}

ostream&
operator<<(ostream& _os, const ModeGraph::Mode* _mode) {
  return _os;
}

istream&
operator>>(istream& _is, const ModeGraph::ReversibleAction _ra) {
  return _is;
}

ostream&
operator<<(ostream& _os, const ModeGraph::ReversibleAction _ra) {
  return _os;
}

//istream&
//operator>>(istream& _is, const ModeGraph::GroundedVertex _vertex) {
//  return _is;
//}

//ostream&
//operator<<(ostream& _os, const ModeGraph::GroundedVertex _vertex) {
//  return _os;
//}

istream&
operator>>(istream& _is, const ModeGraph::Transition _t) {
  return _is;
}

ostream&
operator<<(ostream& _os, const ModeGraph::Transition _t) {
  return _os;
}
