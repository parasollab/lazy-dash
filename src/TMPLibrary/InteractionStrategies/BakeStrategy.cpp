#include "BakeStrategy.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/ActionSpace/ActionSpace.h"
#include "TMPLibrary/ActionSpace/Interaction.h"
#include "TMPLibrary/ActionSpace/FormationCondition.h"
#include "TMPLibrary/ActionSpace/MotionCondition.h"
#include "TMPLibrary/Solution/Plan.h"
#include <cmath>

#ifdef PPL_USE_URDF
  #include "MPProblem/Robot/Kinematics/ur_kin.h"
  #include "MPProblem/Robot/Kinematics/gantry_kin.h"
#endif
/*----------------------- Construction -----------------------*/

BakeStrategy::
BakeStrategy() {
  this->SetName("BakeStrategy");
}

BakeStrategy::
BakeStrategy(XMLNode& _node) : GraspStrategy(_node) {
  this->SetName("BakeStrategy");

  m_objectSMLabel = _node.Read("objectSMLabel",true, "",
                    "SamplerMethod used to create object pose samples.");
  m_objectVCLabel = _node.Read("objectVCLabel", true, "",
                    "ValidityChecker used to check object samples.");
  m_maxAttempts = _node.Read("maxAttempts", false, 100, 1, MAX_INT,
                  "Max number of attempts to get a valid pose sample.");
  m_physicalDemo = _node.Read("physicalDemo",false,m_physicalDemo,
          "Flag to add extra constraints for clean physical demo.");
}

BakeStrategy::
~BakeStrategy() {}

/*------------------------ Interface -------------------------*/

bool
BakeStrategy::
operator()(Interaction* _interaction, State& _start) {

  auto problem = this->GetPlan()->GetCoordinator()->GetRobot()->GetMPProblem();
  auto lib = this->GetMPLibrary();
  auto plan = this->GetPlan();
  auto coord = plan->GetCoordinator();
  auto as = this->GetTMPLibrary()->GetActionSpace();

  // Set all robots to virtual
  for(auto kv : coord->GetInitialRobotGroups()) {
    for(auto robot : kv.first->GetRobots()) {
      robot->SetVirtual(true);
    }
  } 

  // Set all involved robots back to non virtual
  for(auto kv : _start) {
    auto group = kv.first;
    for(auto robot : group->GetRobots()) {
      robot->SetVirtual(false);
    }
  }

  _interaction->Initialize();

  // Get initial conditions
  auto stages = _interaction->GetStages();

  if(m_debug) {
    std::cout << "\nPlanning interaction: " << _interaction->GetLabel()
              << " with stages:" << std::endl;
    for(auto s : stages) {
      std::cout << "\t" << s << std::endl;
    }
    std::cout << " Initial State: " <<std::endl;
    for(auto kv : _start) {
      auto group = kv.first;
      auto pair = kv.second;
      auto rm = pair.first;
      auto vid = pair.second;

      std::cout << group->GetLabel() << " : ";
      if(!rm)
        std::cout << " null" << std::endl;
      else 
        std::cout << rm->GetVertex(vid).PrettyPrint() << std::endl;
    }
  } 

  Robot* obj;
  std::map<Robot*, Cfg> initialObjectCfg;
  for(auto kv : _start) {
    auto group = kv.first;
    auto pair = kv.second;
    auto rm = pair.first;
    auto vid = pair.second;
    for(auto robot : group->GetRobots()){
      if(robot->GetMultiBody()->IsPassive()){
        obj = robot;
        initialObjectCfg[robot] = rm->GetVertex(vid).GetRobotCfg(robot);
        std::cout << "Initial object pose: " << initialObjectCfg[robot].PrettyPrint() << std::endl;
        break;
      }
    }
  }

  // Check that there are at least 4 stages
  if(stages.size() < 3)
    throw RunTimeException(WHERE) << "Grasp sampling assumes at least 3 stages:"
                                     "\n\t0:Groups"
                                     "\n\t1:PreGrasp stages"
                                     "\n\t2:Grasp"
                                     "\n\t(Optional): PostGrasp stages"
                                  << std::endl;

  auto initialConditions = _interaction->GetStageConditions(stages[0]);


  // Assign roles
  AssignRoles(_start,initialConditions);

  std::string allRobotsLabel;
  std::vector<Robot*> allRobots;

  // Grab object robot models
  std::vector<Robot*> objects;
  std::string allObjectsLabel;

  // Construct initial groups
  std::unordered_map<Robot*,RobotGroup*> initialGroups;

  auto originalStart = _start;

  for(auto kv : _start) {
    auto group = kv.first;
    for(auto r : group->GetRobots()) {

      initialGroups[r] = group;

      // Save object robots
      if(r->GetMultiBody()->IsPassive()) {
        // Add obj to set of all objects
        objects.push_back(r);
        allObjectsLabel += "::"+r->GetLabel();
      }
      else  {
        // Add manip to set of all robots
        allRobots.push_back(r);
        allRobotsLabel += "::"+r->GetLabel();
      }
    }
  }

  auto objGroup = problem->AddRobotGroup(objects,allObjectsLabel);
  auto group = problem->AddRobotGroup(allRobots,allRobotsLabel);


  //TODO::Figure out where grasp stage is
  size_t graspStage = stages.size() - 3;
  for(size_t i = 1; i < graspStage; i++) {

    // Add composite groups to to grasp solution
    auto toStageSolution = _interaction->GetToStageSolution(stages[i+1]);
    toStageSolution->AddRobotGroup(objGroup);
    auto objGrm = toStageSolution->GetGroupRoadmap(objGroup);

    toStageSolution->AddRobotGroup(group);
    lib->SetMPSolution(toStageSolution);
    auto grm = toStageSolution->GetGroupRoadmap(group);

    // Get object placements
    double x = 0;
    double y = 0;
    double z = 0;

    for(auto robot : group->GetRobots()) {
      if(robot->GetMultiBody()->IsPassive())
        continue;
      std::cout << "active robot: " << robot << std::endl;
      auto transform = robot->GetMultiBody()->GetBase()->GetWorldTransformation();
      auto translation = transform.translation();
      x += translation[0];
      y += translation[1];
      z += translation[2];
    }

    Cfg objectCfg(obj);
    std::map<Robot*,Cfg> objectPoses;

    objectCfg[0] = x;
    objectCfg[1] = y;
    objectCfg[2] = z+.1;
    objectCfg[3] = initialObjectCfg[obj][3]; // This value differentiate the object's state.
    objectCfg[4] = 0;
    objectCfg[5] = 0;

    objectPoses[obj] = objectCfg;
    objectCfg.ConfigureRobot();

    if(m_debug) {
      std::cout << "Initial object position:  " << obj->GetLabel() << " " 
                << objectCfg.PrettyPrint() 
                << std::endl;
    }

    if(fabs(objectPoses[obj][0]-initialObjectCfg[obj][0]) > 0.03 
          or fabs(objectPoses[obj][1]-initialObjectCfg[obj][1]) > 0.03 
            or fabs(objectPoses[obj][2]-initialObjectCfg[obj][2]) > 0.03) {
              std::cout << "OUT OF BOUNDS" << std::endl;
              return false;
            }

    // Robot* ws;
    // for(auto robot : group->GetRobots()){
    //   std::cout << "saving ws as: " << robot->GetLabel() << std::endl;
    //   ws = robot;
    // }

    // std::map<Robot*,Cfg> objectPoses;
    // for(auto object : objects) {  
    //   Cfg objectPose(object);

    //   // Check if object placement is given
    //   objectPose = SampleObjectPose(object,ws,_interaction);
    //   std::cout << "dd object position:  " 
    //             << objectPose.PrettyPrint() 
    //             << std::endl;

    //   objectPoses[object] = objectPose;
    //   objectPose.ConfigureRobot();
    // }


    // Sample pregrasp joint angles
    auto eeFrames = ComputeEEFrames(_interaction,objectPoses,i);
    std::cout << "A" << std::endl;
    std::unordered_map<Robot*,Cfg> pregraspCfgs;
    for(auto kv : eeFrames) {
      auto cfg = ComputeManipulatorCfg(kv.first,kv.second);
      if(!cfg.GetRobot()) {
        if(m_debug) {
          std::cout << "Failed to find a valid grasp pose for " << kv.first->GetLabel();
        }
        m_roleMap.clear();
        // Set all robots to virtual
        for(auto kv : coord->GetInitialRobotGroups()) {
          for(auto robot : kv.first->GetRobots()) {
            robot->SetVirtual(false);
          }
        } 
      	return false;
      }

      // SetEEDOF(_interaction,cfg,stages[i]);
      pregraspCfgs[kv.first] = cfg;

      if (m_debug) {
        std::cout << "pregrasp: " << cfg << std::endl;
        std::cout << "at stage: " << stages[i] << std::endl;
      }
      
    }
    // Sample grasp joint angles
    std::cout << "B" << std::endl;
    auto nextStageEEFrames = ComputeEEFrames(_interaction,objectPoses,i+1);
    std::unordered_map<Robot*,Cfg> graspCfgs;
    for(auto kv : nextStageEEFrames) {
      auto cfg = ComputeManipulatorCfg(kv.first,kv.second);
      if(!cfg.GetRobot()) {
        // Set all uninvolved robots back to non virtual
        for(auto& robot : problem->GetRobots()) {
          if(robot.get() != plan->GetCoordinator()->GetRobot()) {
            robot->SetVirtual(false);
          }
        }
        return false;
      }
      // SetEEDOF(_interaction,cfg,stages[i+1]);

      graspCfgs[kv.first] = cfg;
      if (m_debug) {
        std::cout << "grasp: " << cfg << std::endl;
      }
    }

    // Setup group cfgs
    GroupCfgType objGcfg(objGrm);
    for(auto kv : objectPoses) {
      objGcfg.SetRobotCfg(kv.first,std::move(kv.second));
    }

    GroupCfgType pregrasp(grm);
    for(auto kv : pregraspCfgs) {
      pregrasp.SetRobotCfg(kv.first,std::move(kv.second));
    }

    GroupCfgType grasp(grm);
    for(auto kv : graspCfgs) {
      grasp.SetRobotCfg(kv.first,std::move(kv.second));
    }

    // Get start constraints
    auto objVID = objGrm->AddVertex(objGcfg);
    auto preGraspVID = grm->AddVertex(pregrasp);
    State preGraspState;
    preGraspState[objGroup] = std::make_pair(objGrm,objVID);
    preGraspState[group] = std::make_pair(grm,preGraspVID);
    auto preGraspConstraints = GenerateConstraints(preGraspState);

    // Get goal constraints
    auto graspVID = grm->AddVertex(grasp);
    State graspState;
    graspState[objGroup] = std::make_pair(objGrm,objVID);
    graspState[group] = std::make_pair(grm,graspVID);
    auto graspConstraints = GenerateConstraints(graspState);

    // Set active formations for grasp planning problem
    auto startConditions = _interaction->GetStageConditions(stages[0]);
    SetActiveFormations(startConditions,toStageSolution);

    // Create grasp planning tasks
    auto graspTasks = GenerateTasks(startConditions,
        preGraspConstraints,
        graspConstraints);

    _interaction->SetToStageTasks(stages[i+1],graspTasks);

    // Configure objects as static robots
    std::set<Robot*> staticRobots;  
    for(auto obj : objects) {
      staticRobots.insert(obj);
    }

    ConfigureStaticRobots(staticRobots,preGraspState);

    auto toGraspPaths = PlanMotions(graspTasks,toStageSolution,
        "PlanInteraction::"+_interaction->GetLabel()+"::To"+stages[i+1],
        staticRobots,preGraspState);

    ResetStaticRobots(staticRobots);

    // Check if valid solution was found
    if(toGraspPaths.empty()) {
      m_roleMap.clear();
      // Set all uninvolved robots back to non virtual
      for(auto& robot : problem->GetRobots()) {
        if(robot.get() != plan->GetCoordinator()->GetRobot()) {
          robot->SetVirtual(false);
        }
      }
      return false;
    }

    // Save plan information
    _interaction->SetToStagePaths(stages[i+1],toGraspPaths);

    _start = InterimState(_interaction,stages[i+1],stages[i+2],toGraspPaths);
  }



  std::cout << " >>>> Post Grasp <<<< " << std::endl;



  // Compute to post grasp path
  
  // Configure to stage solution
  auto toStageSolution = _interaction->GetToStageSolution(stages[graspStage+1]);
 
  // Configure start constraints from previous stage path end
  auto startConstraints = GenerateConstraints(_start); 

  // Get object placements
  std::map<Robot*,Cfg> objectPoses;
  for(auto object : objects) {  
    Cfg objectPose(object);

    // Check if object placement is given
    auto initGroup = initialGroups[object];
    auto given = originalStart[initGroup];
    auto gcfg = given.first->GetVertex(given.second);
    objectPose = gcfg.GetRobotCfg(object);
    std::cout << "Post stage object pose is: " << objectPose.PrettyPrint() << std::endl;

    objectPoses[object] = objectPose;
    objectPose.ConfigureRobot();
  }

  std::cout << "C" << std::endl;

  // Compute goal pose for robot based of transform and initial object pose
  auto nextStageEEFrames = ComputeEEFrames(_interaction,objectPoses,graspStage+1);
  std::unordered_map<Robot*,std::unique_ptr<CSpaceConstraint>> constraintMap;
  // Cfg activeCfg;
  // Cfg passiveCfg;
  for(auto kv : nextStageEEFrames) {
    auto robot = kv.first;
    auto cfg = ComputeManipulatorCfg(robot,kv.second);
    if(!cfg.GetRobot()) {
      if(m_debug) {
        std::cout << "Failed to find a valid grasp pose for " << kv.first->GetLabel();
      }
      m_roleMap.clear();
      // Set all uninvolved robots back to non virtual
      for(auto& robot : problem->GetRobots()) {
        if(robot.get() != plan->GetCoordinator()->GetRobot()) {
          robot->SetVirtual(false);
        }
      }
      return false;
    }
    // activeCfg = cfg;
    // SetEEDOF(_interaction,cfg,stages[graspStage]);

    constraintMap[robot] = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,cfg));
  }
  // passiveCfg = objectPoses[obj];

  State goal;
  for(auto kv : _start) {
    auto group = kv.first;
    std::cout << "Setting up the final goal: " << group->GetLabel() << std::endl;
    auto rm = kv.second.first;
    rm->SetAllFormationsInactive();
    for(auto f : rm->GetVertex(kv.second.second).GetFormations()) {
      rm->AddFormation(f);
    }

    // // Identify active robot
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

    GroupCfgType gcfg(rm);
    if(active) {
      std::cout << "active robot: " << active->GetLabel() << std::endl;
      gcfg.GetRandomGroupCfg(constraintMap[active]->GetBoundary());
      std::cout << gcfg.PrettyPrint() << std::endl;
      auto vid = rm->AddVertex(gcfg);
      goal[group] = std::make_pair(rm,vid);
    }
    else if(passive) {
      std::cout << "passive robot: " << passive->GetLabel() << std::endl;
      goal[group] = _start[group];
    }

    // // gcfg.GetRandomGroupCfg(constraintMap[active]->GetBoundary());
    // gcfg.SetRobotCfg(active,std::move(activeCfg));
    // gcfg.SetRobotCfg(passive,std::move(passiveCfg));
    // std::cout << gcfg.PrettyPrint() << std::endl;
    // auto vid = rm->AddVertex(gcfg);
    // goal[group] = std::make_pair(rm,vid);

    // goal[group] = _start[group];
  }


  // Convert group cfg to goal constraints
  auto goalConstraints = GenerateConstraints(goal); 

  // Create tasks from constraints
  auto startConditions = _interaction->GetStageConditions(stages[graspStage]);
  auto tasks = GenerateTasks(startConditions,startConstraints,goalConstraints);
  _interaction->SetToStageTasks(stages[graspStage+1],tasks);

  /*
  // Set all uninvolved robots to virtual
  for(auto& robot : problem->GetRobots()) {
    robot->SetVirtual(true);
  }
  for(auto& kv : _start) {
    for(auto robot : kv.first->GetRobots()) {
      robot->SetVirtual(false);
    }
  }
  */

  // Grab all static robots from conditions and pass to PlanMotions
  std::set<Robot*> staticLiftRobots;
  for(auto c : startConditions) {
    auto f = dynamic_cast<FormationCondition*>(as->GetCondition(c));
    if(!f or !f->IsStatic())
      continue;

    for(auto role : f->GetRoles()) {
      auto r = m_roleMap[role];
      staticLiftRobots.insert(r);
    }
  }

  // Plan path
  auto paths = PlanMotions(tasks,toStageSolution,
      "PlanInteraction::"+_interaction->GetLabel()+"::To"+stages[graspStage+1],
      staticLiftRobots,_start);

  // size_t delay = _interaction->GetDelay(stages[graspStage+1]);
  // if(delay > 0) {
  //   for(auto path : paths) {
  //     auto pair = path->VIDsWaiting();
  //     auto vids = pair.first;
  //     auto wait = pair.second;
  //     if(wait.empty()) {
  //       auto last = vids.back();
  //       std::vector<size_t> add(delay,last);
  //       *path += add;
  //       path->SetTimeSteps(path->TimeSteps() + delay);
  //     }
  //     else {
  //       wait = std::vector<size_t>(vids.size(),0);
  //       wait[wait.size()-1] += delay;
  //       path->SetWaitTimes(wait);
  //     }
  //   }
  // }
  size_t delay = _interaction->GetDelay(stages[graspStage+1]);
  if(delay > 0) {
    std::cout << "wait time: " << delay << std::endl;
    for(auto path : paths) {
      auto pair = path->VIDsWaiting();
      auto vids = pair.first;
      auto wait = pair.second;
      if(wait.empty()) {
        wait = std::vector<size_t>(vids.size(),0); // 
      }
      wait[wait.size()-1] += delay;
      path->SetWaitTimes(wait);
    }
  }


  // Set all uninvolved robots back to non virtual
  for(auto& robot : problem->GetRobots()) {
    if(robot.get() != plan->GetCoordinator()->GetRobot()) {
      robot->SetVirtual(false);
    }
  }

  // Check if valid solution was found
  if(paths.empty()) {
    m_roleMap.clear();
    return false;
  }

  // Save plan information
  _interaction->SetToStagePaths(stages[graspStage+1],paths);

  _start = InterimState(_interaction,stages[graspStage+2],stages[graspStage+1],paths);


  m_roleMap.clear();
  m_objectPoseTasks.clear();

  std::cout << "PATH FOUND FOR TASK " << _interaction->GetLabel() 
            << " WITH OBJECT POSE: " << initialObjectCfg[obj].PrettyPrint() << std::endl;
  return true;
}

/*--------------------- Helper Functions ---------------------*/

void
BakeStrategy::
AssignRoles(const State& _state, const std::vector<std::string>& _conditions) {

  // If no start state, filter out motion conditions
  bool validStart = true;
  for(auto kv : _state) {
    if(!kv.second.first or kv.second.second == MAX_UINT) {
      validStart = false;
      break;
    }
  }

  std::vector<std::string> filteredConditions;
  if(!validStart) {
    for(auto condition : _conditions) {
      auto c = this->GetTMPLibrary()->GetActionSpace()->GetCondition(condition);
      auto m = dynamic_cast<MotionCondition*>(c);
      if(!m) 
        filteredConditions.push_back(condition);
    }
    // Assign roles from filtered conditions
    InteractionStrategyMethod::AssignRoles(_state,filteredConditions);
  }
  else {
    InteractionStrategyMethod::AssignRoles(_state,_conditions);
  }
}


// Cfg
// BakeStrategy::
// SampleObjectPose(Robot* _object, Robot* _refRobot, Interaction* _interaction) {
//   std::cout << "SampleObjectPose: " << _object->GetLabel() << std::endl;
//   auto library = this->GetMPLibrary();
//   auto problem = _object->GetMPProblem();
//   auto env = problem->GetEnvironment();
//   std::string capability = _object->GetCapability() + "-" + _refRobot->GetCapability();
//   std::cout << " capability: " << capability << std::endl;
//   const auto& surfaces = env->GetTerrains().at(capability);
  
 
//   // Sample a stable object pose
//   for(size_t i = 0; i < m_maxAttempts; i++) {

//     // Create task for object pose
//     auto task = std::unique_ptr<MPTask>(new MPTask(_object));
//     m_objectPoseTasks.push_back(std::move(task));
//     library->SetTask(m_objectPoseTasks.back().get());

//     // Grab a random surface boundary
//     size_t index = LRand() % surfaces.size();
//     const auto& surface = surfaces[index];
    
//     const auto& boundaries = surface.GetBoundaries();
//     index = LRand() % boundaries.size();
//     auto boundary = boundaries[index].get();

//     // Sample within the surface boundary
//     std::vector<Cfg> samples;
//     auto sm = this->GetMPLibrary()->GetSampler(m_objectSMLabel);
//     sm->Sample(1,1,boundary,std::back_inserter(samples));

//     if(samples.empty())
//       continue;

//     auto cfg = samples[0];

//     // Check if sample is valid
//     auto vc = library->GetValidityChecker(m_objectVCLabel);
//     if(vc->IsValid(cfg,this->GetNameAndLabel() + "::" + _interaction->GetLabel()))
//       return cfg;
//   }

//   return Cfg(nullptr);
// }

// Cfg
// BakeStrategy::
// SampleObjectPoseSimple(Robot* _object, Robot* _refRobot, Interaction* _interaction) {
//   std::cout << "SampleObjectPose: " << _object->GetLabel() << std::endl;
//   auto library = this->GetMPLibrary();
//   auto problem = _object->GetMPProblem();
//   auto env = problem->GetEnvironment();
//   std::string capability = _object->GetCapability() + "-" + _refRobot->GetCapability();
//   std::cout << " capability: " << capability << std::endl;
//   const auto& surfaces = env->GetTerrains().at(capability);
  
 
//   // Sample a stable object pose
//   for(size_t i = 0; i < m_maxAttempts; i++) {

//     // Grab a random surface boundary
//     size_t index = LRand() % surfaces.size();
//     const auto& surface = surfaces[index];
    
//     const auto& boundaries = surface.GetBoundaries();
//     index = LRand() % boundaries.size();
//     auto boundary = boundaries[index].get();

//     // Sample within the surface boundary
//     std::vector<Cfg> samples;
//     auto sm = this->GetMPLibrary()->GetSampler(m_objectSMLabel);
//     sm->Sample(1,1,boundary,std::back_inserter(samples));

//     if(samples.empty())
//       continue;

//     auto cfg = samples[0];

//     // Check if sample is valid
//     auto vc = library->GetValidityChecker(m_objectVCLabel);
//     if(vc->IsValid(cfg,this->GetNameAndLabel() + "::" + _interaction->GetLabel()))
//       return cfg;
//   }

//   return Cfg(nullptr);
// }


// Cfg
// BakeStrategy::
// SampleObjectPose(Robot* _object, Interaction* _interaction) {
//   std::cout << "SampleObjectPose: " << _object->GetLabel() << std::endl;
//   auto library = this->GetMPLibrary();
//   auto problem = _object->GetMPProblem();
//   auto env = problem->GetEnvironment();
//   const auto& surfaces = env->GetTerrains().at(_object->GetCapability());
//   std::cout << " capability: " << _object->GetCapability() << std::endl;
 
//   // Sample a stable object pose
//   for(size_t i = 0; i < m_maxAttempts; i++) {

//     // Create task for object pose
//     auto task = std::unique_ptr<MPTask>(new MPTask(_object));
//     m_objectPoseTasks.push_back(std::move(task));
//     library->SetTask(m_objectPoseTasks.back().get());

//     // Grab a random surface boundary
//     size_t index = LRand() % surfaces.size();
//     const auto& surface = surfaces[index];
    
//     const auto& boundaries = surface.GetBoundaries();
//     index = LRand() % boundaries.size();
//     auto boundary = boundaries[index].get();

//     // Sample within the surface boundary
//     std::vector<Cfg> samples;
//     auto sm = this->GetMPLibrary()->GetSampler(m_objectSMLabel);
//     sm->Sample(1,1,boundary,std::back_inserter(samples));

//     if(samples.empty())
//       continue;

//     auto cfg = samples[0];

//     // Check if sample is valid
//     auto vc = library->GetValidityChecker(m_objectVCLabel);
//     if(vc->IsValid(cfg,this->GetNameAndLabel() + "::" + _interaction->GetLabel()))
//       return cfg;
//   }

//   return Cfg(nullptr);
// }

// BakeStrategy::GroupCfgType
// BakeStrategy::
// SampleObjectPoses(RobotGroup* _group, Interaction* _interaction) {

//   // Identify leader
//   Robot* leader = nullptr;
//   auto as = this->GetTMPLibrary()->GetActionSpace();
//   auto stages = _interaction->GetStages();
//   auto pregraspConditions = _interaction->GetStageConditions(stages[0]);
//   for(auto condition : pregraspConditions) {
//     auto f = dynamic_cast<FormationCondition*>(as->GetCondition(condition));
//     if(!f)
//       continue;

//     for(auto label : f->GetRoles()) {
//       auto info = f->GetRoleInfo(label);
//       if(info.referenceRole != "")
//         continue;

//       leader = m_roleMap[label];
//       break;
//     }

//     if(leader)
//       break;
//   }

//   // Sample pose
//   std::map<Robot*,const Boundary*> boundaryMap;
//   auto envBoundary = this->GetMPProblem()->GetEnvironment()->GetBoundary();
//   for(auto r : _group->GetRobots()) {
//     if(r == leader)
//       continue;

//     boundaryMap[r] = envBoundary;
//   }

//   auto sm = this->GetMPLibrary()->GetSampler(m_smLabel);
//   for(size_t i = 0; i < m_maxAttempts; i++) {
//     // Sample pose for leader
//     auto base = SampleObjectPose(leader,_interaction);

//     // If base has no robot, then we have exceeded max attempts for finding the base location
//     if(!base.GetRobot())
//       return GroupCfgType();

//     // Update boundary map
//     auto leaderBoundary = new CSpaceBoundingBox(base.DOF());
//     leaderBoundary->ShrinkToPoint(base);
//     boundaryMap[leader] = leaderBoundary;

//     // Sample group cfg for boundary
//     std::vector<GroupCfgType> samples;
//     sm->Sample(1,1,boundaryMap,std::back_inserter(samples));

//     delete leaderBoundary;

//     // Check if it is valid and either return or continue
//     if(samples.empty())
//       continue;

//     return samples[0];
//   }

//   return GroupCfgType();
// }

// Transformation
// BakeStrategy::
// ComputeEEWorldFrame(const Cfg& _objectPose, const Transformation& _transform) {

//   _objectPose.ConfigureRobot();

//   auto base = _objectPose.GetRobot()->GetMultiBody()->GetBase();
//   auto baseFrame = base->GetWorldTransformation();

//   //auto translation = (-_transform).rotation() * baseFrame.translation() + _transform.translation();
//   //auto rotation = (-_transform).rotation() * baseFrame.rotation();
//   //return Transformation(translation,rotation);

//   return baseFrame * _transform;
// }

// std::unordered_map<Robot*,Transformation>
// BakeStrategy::
// ComputeEEFrames(Interaction* _interaction, std::map<Robot*,Cfg>& objectPoses, size_t _stage) {

//   auto as = this->GetTMPLibrary()->GetActionSpace();
//   auto stages = _interaction->GetStages();

//   std::unordered_map<Robot*,Transformation> eeFrames;

//   auto pregraspConditions = _interaction->GetStageConditions(stages[_stage]);
//   for(auto condition : pregraspConditions) {
//     auto f = dynamic_cast<FormationCondition*>(as->GetCondition(condition));
//     if(!f)
//       continue;

//     for(auto role : f->GetRoles()) {
//       auto robot = m_roleMap[role];
//       if(!robot->GetMultiBody()->IsPassive())
//         continue;

//       const auto& roleInfo = f->GetRoleInfo(role); 
//       Transformation transform = roleInfo.transformation;
//       Cfg cfg = objectPoses[robot];
//       cfg.ConfigureRobot();

//       auto frame = ComputeEEWorldFrame(cfg, -transform);
      
//       auto refRobot = m_roleMap[roleInfo.referenceRole];
//       // Check that refRobot is active
//       if(!refRobot or refRobot->GetMultiBody()->IsPassive())
//         continue;

//       auto refBase = refRobot->GetMultiBody()->GetBase();
//       auto refBaseTransformation = refBase->GetWorldTransformation();

//       // Current have to rotate base of ur5e bc of weird urdf stuff, but it messes up this calculation
//       if(m_doctorBaseOrientation)
//         refBaseTransformation = Transformation(refBaseTransformation.translation());

//       //auto translation = (-refBaseTransformation).rotation() * frame.translation() + (-refBaseTransformation).translation();
//       auto translation = frame.translation() + (-refBaseTransformation).translation();
//       auto rotation = (-refBaseTransformation).rotation() * frame.rotation();

//       std::cout << "WS Frame: " << frame.translation() << std::endl;

//       //auto translation = (-frame).rotation() * refBaseTransformation.translation() + (-frame).translation();
//       //auto rotation = (-frame).rotation() * refBaseTransformation.rotation();

//       auto eeFrame = Transformation(translation,rotation);
//       eeFrames[refRobot] = eeFrame;
//     }
//   }

//   return eeFrames;
// }


// Cfg
// BakeStrategy::
// ComputeManipulatorCfg(Robot* _robot, std::map<Robot*,Cfg>& objectPoses, Transformation& _transform) {
//   #ifdef PPL_USE_URDF

//   /*
//   auto coord = this->GetPlan()->GetCoordinator();
//   for(auto kv : coord->GetInitialRobotGroups()) {
//     for(auto robot : kv.first->GetRobots()) {
//       robot->SetVirtual(true);
//     }
//   } 

//   _robot->SetVirtual(false);
//   */
//   std::cout << "num dof: " << _robot->GetMultiBody()->DOF() << std::endl;
//   // std::vector<double> robotXYZ;
//   // std::vector<double> refXYZ;
//   // auto transform = _robot->GetMultiBody()->GetBase()->GetWorldTransformation();
//   // auto translation = transform.translation();
//   // double y = translation[1];

  
//   // for(auto kv : objectPoses) {
//   //   Cfg refCfg = kv.second;
//   //   for(size_t i = 0 ; i < 3 ; i++){
//   //     refXYZ.push_back(refCfg[i]);
//   //   }
//   //   for(size_t i = 0 ; i < 3 ; i++){
//   //     robotXYZ.push_back(translation[i]);
//   //   }

//   //   // std::cout << "ref object pos: " << std::endl;
//   //   // for(auto x : refXYZ){
//   //   //   std::cout << x << " " ;
//   //   // }
//   //   // std::cout << std::endl;

//   //   // std::cout << "robot pos: " << std::endl;
//   //   // for(auto x : robotXYZ){
//   //   //   std::cout << x << " " ;
//   //   // }
//   //   // std::cout << std::endl;

//   //   // double distance = 0.0;
//   //   // for (size_t i = 0; i < refXYZ.size(); ++i) {
//   //   //     double diff = refXYZ[i] - robotXYZ[i];
//   //   //     distance += diff * diff;
//   //   // }

//   //   // distance = std::sqrt(distance);
//   //   // std::cout << "distance from object: " << distance << std::endl;
//   //   if(refCfg[0] > 0 or fabs(y-refCfg[1])){
//   //     std::cout << " object out of sight. " << std::endl;
//   //     return Cfg(nullptr);
//   //   }
//   // }
  
//   if(_robot->GetLabel().find("ws") != std::string::npos) {
//     std::cout << "cfg for workstation." << std::endl;
//     auto translation = _transform.translation();
//     auto orientation = _transform.rotation().matrix();

//     std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

//     std::cout << "translation elements: ";
//     for (const auto& element : translation) {
//         std::cout << element << " ";
//     }
//     std::cout << std::endl;


//     data[0] =  atan2(orientation[2][1],orientation[2][2])/PI;

//     std::cout << "data elements: ";
//     for (const auto& element : data) {
//         std::cout << element << " ";
//     }
//     std::cout << std::endl;

//     for(size_t j = 0; j < data.size(); j++) {
//       if(j == 1)
//         continue;

//       if(data[j] > 1) {
//         data[j] = -2 + data[j];
//       }
//     }

  

//     Cfg cfg(_robot);
//     cfg.SetData(data);

//     auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

//     if(vc->IsValid(cfg,this->GetNameAndLabel())) {
//       std::cout << "returning valid cfg: " << cfg.PrettyPrint() << std::endl;
//       return cfg;
//     }
//     else {
//       std::cout << "returning INVALID cfg: " << cfg.PrettyPrint() << std::endl;
//       return Cfg(nullptr);
//     }

//   }
//   // else if(m_robotType=="ur5e"){
//   else if(_robot->GetLabel().find("ur5e") != std::string::npos) {
//     if(m_debug) {
//       std::cout << "Computing IK for a UR5e." << std::endl;
//     }

//     //TODO::Convert transform to individual ur_kin format
  
//     auto translation = _transform.translation();
//     auto orientation = _transform.rotation().matrix();
  
//     double* T = new double[16];
//     double q_sols[8*6];
//     // Point
//     T[3] = translation[0];
//     T[7] = translation[1];
//     T[11] = translation[2];

//     // Orientation matrix
//     T[0] = orientation[0][0];
//     T[1] = orientation[0][1];
//     T[2] = orientation[0][2];

//     T[4] = orientation[1][0];
//     T[5] = orientation[1][1];
//     T[6] = orientation[1][2];

//     T[8] = orientation[2][0];
//     T[9] = orientation[2][1];
//     T[10] = orientation[2][2];

//     T[12] = 0;
//     T[13] = 0;
//     T[14] = 0;
//     T[15] = 1;


//     // if(m_debug) {
//     //   for(int i=0;i<4;i++) {
//     //     for(int j=i*4;j<(i+1)*4;j++)
//     //       printf("%1.3f ", T[j]);
//     //     printf("\n");
//     //   }
//     // }

//     auto num_sols = ur_kinematics::inverse(T, q_sols);

//     // if(m_debug) {
//     //   for(int i=0;i<num_sols;i++) 
//     //     printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
//     //         q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
//     // }
    
//     //TODO::Validity check cfg and if invalid try the next solution

//     auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

//     for(int i = 0; i < num_sols; i++) {
//       std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

//       data[0] =  q_sols[i*6+2]/PI; 
//       data[1] =  0;
//       data[2] =  q_sols[i*6+1]/PI;
//       data[3] =  q_sols[i*6+0]/PI;
//       data[4] =  q_sols[i*6+3]/PI;
//       data[5] =  q_sols[i*6+4]/PI;
//       data[6] =  q_sols[i*6+5]/PI;

//       //TODO::Find base joint index
//       const size_t baseJointIndex = 3;
//       data[baseJointIndex] += _robot->GetBaseRotation();

//       for(size_t j = 0; j < data.size(); j++) {
//         if(j == 1)
//           continue;

//         if(data[j] > 1) {
//           data[j] = -2 + data[j];
//         }
//       }

//       Cfg cfg(_robot);
//       cfg.SetData(data);

//       if(vc->IsValid(cfg,this->GetNameAndLabel())) {

//         /*
//         for(auto kv : coord->GetInitialRobotGroups()) {
//           for(auto robot : kv.first->GetRobots()) {
//             robot->SetVirtual(false);
//           }
//         } 
//         */
//         std::cout << cfg << " valid" << std::endl;
//         return cfg;
//       }
//       else 
//         std::cout << cfg << " invalid." << std::endl;
//     }

//     /*
//     for(auto kv : coord->GetInitialRobotGroups()) {
//       for(auto robot : kv.first->GetRobots()) {
//         robot->SetVirtual(false);
//       }
//     } 
//     */

//     //if(num_sols == 0)
//     return Cfg(nullptr);

    
//   }
//   // else if(m_robotType=="gantry"){
//   else if(_robot->GetLabel().find("gantry") != std::string::npos) {
//     auto translation = _transform.translation();
//     auto orientation = _transform.rotation().matrix();
  
//     double* T = new double[16];
//     double q_sols[4];
//     // Point
//     T[3] = translation[0];
//     T[7] = translation[1];
//     T[11] = translation[2];

//     // Orientation matrix
//     T[0] = orientation[0][0];
//     T[1] = orientation[0][1];
//     T[2] = orientation[0][2];

//     T[4] = orientation[1][0];
//     T[5] = orientation[1][1];
//     T[6] = orientation[1][2];

//     T[8] = orientation[2][0];
//     T[9] = orientation[2][1];
//     T[10] = orientation[2][2];

//     T[12] = 0;
//     T[13] = 0;
//     T[14] = 0;
//     T[15] = 1;


//     // if(m_debug) {
//     //   for(int i=0;i<4;i++) {
//     //     for(int j=i*4;j<(i+1)*4;j++)
//     //       printf("%1.3f ", T[j]);
//     //     printf("\n");
//     //   }
//     // }

//     gantry_kinematics::inverse(T, q_sols);

//     auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

//     std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

//     data[0] =  0; 
//     data[1] =  q_sols[0]/PI;
//     data[2] =  q_sols[1];
//     data[3] =  q_sols[2];
//     data[4] =  q_sols[3];


//     Cfg cfg(_robot);
//     cfg.SetData(data);

//     if(vc->IsValid(cfg,this->GetNameAndLabel())) {
//       if(m_debug)
//         std::cout << cfg << " valid" << std::endl;
//       return cfg;
//     }
//     else 
//       std::cout << cfg << " invalid." << std::endl;

//     return Cfg(nullptr);
//   }
//   else{
//     throw RunTimeException(WHERE) << "Unsupported Robot Name";
//     return Cfg(nullptr);
//   }

//   #else

//   throw RunTimeException(WHERE) << "IK not supported without ROS integration.";
//   return Cfg(_robot);

//   #endif

// }


// Cfg
// BakeStrategy::
// ComputeManipulatorCfg(Robot* _robot, Transformation& _transform) {
//   #ifdef PPL_USE_URDF

//   /*
//   auto coord = this->GetPlan()->GetCoordinator();
//   for(auto kv : coord->GetInitialRobotGroups()) {
//     for(auto robot : kv.first->GetRobots()) {
//       robot->SetVirtual(true);
//     }
//   } 

//   _robot->SetVirtual(false);
//   */
//  std::cout << "num dof: " << _robot->GetMultiBody()->DOF() << std::endl;
//   if(_robot->GetMultiBody()->DOF()==5){
//     std::cout << "cfg for workstation." << std::endl;
//     auto translation = _transform.translation();
//     auto orientation = _transform.rotation().matrix();

//     std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

//     std::cout << "translation elements: ";
//     for (const auto& element : translation) {
//         std::cout << element << " ";
//     }
//     std::cout << std::endl;


//     data[0] =  0;
//     data[1] =  atan2(orientation[2][1],orientation[2][2])/PI;
//     data[2] =  0.;
//     data[3] =  0.;
//     data[4] =  0.;

//     std::cout << "data elements: ";
//     for (const auto& element : data) {
//         std::cout << element << " ";
//     }
//     std::cout << std::endl;

//     for(size_t j = 0; j < data.size(); j++) {
//       if(j == 1)
//         continue;

//       if(data[j] > 1) {
//         data[j] = -2 + data[j];
//       }
//     }

  

//     Cfg cfg(_robot);
//     cfg.SetData(data);



//     auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

//     if(vc->IsValid(cfg,this->GetNameAndLabel())) {
//       std::cout << "returning valid cfg: " << cfg.PrettyPrint() << std::endl;
//       return cfg;
//     }
//     else {
//       std::cout << "returning INVALID cfg: " << cfg.PrettyPrint() << std::endl;
//       return Cfg(nullptr);
//     }

//   }
//   else if(_robot->GetLabel().find("ur5e") != std::string::npos) {
//   // else if(m_robotType=="ur5e"){
//     if(m_debug) {
//       std::cout << "Computing IK for a UR5e." << std::endl;
//     }

//     //TODO::Convert transform to individual ur_kin format
  
//     auto translation = _transform.translation();
//     auto orientation = _transform.rotation().matrix();
  
//     double* T = new double[16];
//     double q_sols[8*6];
//     // Point
//     T[3] = translation[0];
//     T[7] = translation[1];
//     T[11] = translation[2];

//     // Orientation matrix
//     T[0] = orientation[0][0];
//     T[1] = orientation[0][1];
//     T[2] = orientation[0][2];

//     T[4] = orientation[1][0];
//     T[5] = orientation[1][1];
//     T[6] = orientation[1][2];

//     T[8] = orientation[2][0];
//     T[9] = orientation[2][1];
//     T[10] = orientation[2][2];

//     T[12] = 0;
//     T[13] = 0;
//     T[14] = 0;
//     T[15] = 1;


//     // if(m_debug) {
//     //   for(int i=0;i<4;i++) {
//     //     for(int j=i*4;j<(i+1)*4;j++)
//     //       printf("%1.3f ", T[j]);
//     //     printf("\n");
//     //   }
//     // }

//     auto num_sols = ur_kinematics::inverse(T, q_sols);

//     // if(m_debug) {
//     //   for(int i=0;i<num_sols;i++) 
//     //     printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
//     //         q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
//     // }
    
//     //TODO::Validity check cfg and if invalid try the next solution

//     auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

//     for(int i = 0; i < num_sols; i++) {
//       std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

//       data[0] =  q_sols[i*6+2]/PI; 
//       data[1] =  0;
//       data[2] =  q_sols[i*6+1]/PI;
//       data[3] =  q_sols[i*6+0]/PI;
//       data[4] =  q_sols[i*6+3]/PI;
//       data[5] =  q_sols[i*6+4]/PI;
//       data[6] =  q_sols[i*6+5]/PI;

//       //TODO::Find base joint index
//       const size_t baseJointIndex = 3;
//       data[baseJointIndex] += _robot->GetBaseRotation();

//       for(size_t j = 0; j < data.size(); j++) {
//         if(j == 1)
//           continue;

//         if(data[j] > 1) {
//           data[j] = -2 + data[j];
//         }
//       }

//       Cfg cfg(_robot);
//       cfg.SetData(data);

//       if(vc->IsValid(cfg,this->GetNameAndLabel())) {

//         /*
//         for(auto kv : coord->GetInitialRobotGroups()) {
//           for(auto robot : kv.first->GetRobots()) {
//             robot->SetVirtual(false);
//           }
//         } 
//         */
//         std::cout << cfg << " valid" << std::endl;
//         return cfg;
//       }
//       else 
//         std::cout << cfg << " invalid." << std::endl;
//     }

//     /*
//     for(auto kv : coord->GetInitialRobotGroups()) {
//       for(auto robot : kv.first->GetRobots()) {
//         robot->SetVirtual(false);
//       }
//     } 
//     */

//     //if(num_sols == 0)
//     return Cfg(nullptr);

    
//   }
//   else if(_robot->GetLabel().find("gantry") != std::string::npos) {
//   // else if(m_robotType=="gantry"){
//     auto translation = _transform.translation();
//     auto orientation = _transform.rotation().matrix();
  
//     double* T = new double[16];
//     double q_sols[4];
//     // Point
//     T[3] = translation[0];
//     T[7] = translation[1];
//     T[11] = translation[2];

//     // Orientation matrix
//     T[0] = orientation[0][0];
//     T[1] = orientation[0][1];
//     T[2] = orientation[0][2];

//     T[4] = orientation[1][0];
//     T[5] = orientation[1][1];
//     T[6] = orientation[1][2];

//     T[8] = orientation[2][0];
//     T[9] = orientation[2][1];
//     T[10] = orientation[2][2];

//     T[12] = 0;
//     T[13] = 0;
//     T[14] = 0;
//     T[15] = 1;


//     // if(m_debug) {
//     //   for(int i=0;i<4;i++) {
//     //     for(int j=i*4;j<(i+1)*4;j++)
//     //       printf("%1.3f ", T[j]);
//     //     printf("\n");
//     //   }
//     // }

//     gantry_kinematics::inverse(T, q_sols);

//     auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

//     std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

//     data[0] =  0; 
//     data[1] =  q_sols[0]/PI;
//     data[2] =  q_sols[1];
//     data[3] =  q_sols[2];
//     data[4] =  q_sols[3];


//     Cfg cfg(_robot);
//     cfg.SetData(data);

//     if(vc->IsValid(cfg,this->GetNameAndLabel())) {
//       if(m_debug)
//         std::cout << cfg << " valid" << std::endl;
//       return cfg;
//     }
//     else 
//       std::cout << cfg << " invalid." << std::endl;

//     return Cfg(nullptr);
//   }
//   else{
//     throw RunTimeException(WHERE) << "Unsupported Robot Name";
//     return Cfg(nullptr);
//   }

//   #else

//   throw RunTimeException(WHERE) << "IK not supported without ROS integration.";
//   return Cfg(_robot);

//   #endif

// }

// void
// BakeStrategy::
// SetEEDOF(Interaction* _interaction, Cfg& _cfg, const std::string& _stage) {
//   Constraint* constraint = nullptr;
//   auto as = this->GetTMPLibrary()->GetActionSpace();
//   std::cout << "  setting ee " << _cfg.PrettyPrint() << std::endl;
//   // Grap role
//   std::string role;
//   for(auto kv : m_roleMap) {
//     if(kv.second == _cfg.GetRobot()) {
//       role = kv.first;
//       break;
//     }
//   }

//   for(auto condition : _interaction->GetStageConditions(_stage)) {
//     auto c = as->GetCondition(condition);
//     auto m = dynamic_cast<MotionCondition*>(c);
//     if(!m)
//       continue;
//     constraint = m->GetRoleConstraint(role);
//     if(constraint){
//       std::cout << "    constraint exists." << std::endl;
//       break;
//     }
//   }

//   auto b = dynamic_cast<BoundaryConstraint*>(constraint);
//   auto boundary = b->GetBoundary();

//   if(robot->GetLabel().find("ur5e") != std::string::npos) {
//   // if(m_robotType=="ur5e"){
//     std::cout << "  this robot is ur5e" << std::endl;
//     for(size_t i = 0; i < _cfg.DOF(); i++) {
      
//       auto range = boundary->GetRange(i);

//       // Check if dof is ee 
//       // TODO::temp assume ee dof is 1
//       if(i == 1) {
//         // Sample ee dof within boundary
//         auto ee = range.Sample();
//         _cfg[i] = ee;
//       }
//       else {
//         // Ensure that cfg satisfies boundary
//         if(!range.Contains(_cfg[i]))
//           throw RunTimeException(WHERE) << "Sampled EE position does not satisfy condition constraint.";
//       }
//     }
//   }
//   else if(robot->GetLabel().find("gantry") != std::string::npos) {
//   // else if(m_robotType=="gantry"){
//     std::cout << "  this robot is gantry" << std::endl;
//     for(size_t i = 0; i < _cfg.DOF(); i++) {
      
//       auto range = boundary->GetRange(i);

//       // Check if dof is ee 
//       // TODO::temp assume ee dof is 1
//       if(i == 0) {
//         // Sample ee dof within boundary
//         auto ee = range.Sample();
//         _cfg[i] = ee;
//       }
//       else {
//         // Ensure that cfg satisfies boundary
//         if(!range.Contains(_cfg[i])){
//           if (m_debug) {
//             std::cout << "interaction: " << _interaction->GetLabel() << std::endl;
//             std::cout << "index: " << i << std::endl;
//             std::cout << "range: " << range << std::endl;
//             std::cout << "cfg: " << _cfg[i] << std::endl;
//           }
//           throw RunTimeException(WHERE) << "Sampled EE position does not satisfy condition constraint.";
//         }
//       }
//     }
//   }
//   std::cout << "  Final ee " << _cfg.PrettyPrint() << std::endl;

// }




// Cfg
// BakeStrategy::
// ComputeManipulatorCfg(Robot* _robot, Transformation& _transform) {

//   #ifdef PPL_USE_URDF
//   //TODO::Convert transform to individual ur_kin format
 
//   auto translation = _transform.translation();
//   auto orientation = _transform.rotation().matrix();
 
//   double* T = new double[16];
//   double q_sols[4];
//   // Point
//   T[3] = translation[0];
//   T[7] = translation[1];
//   T[11] = translation[2];

//   // Orientation matrix
//   T[0] = orientation[0][0];
//   T[1] = orientation[0][1];
//   T[2] = orientation[0][2];

//   T[4] = orientation[1][0];
//   T[5] = orientation[1][1];
//   T[6] = orientation[1][2];

//   T[8] = orientation[2][0];
//   T[9] = orientation[2][1];
//   T[10] = orientation[2][2];

//   T[12] = 0;
//   T[13] = 0;
//   T[14] = 0;
//   T[15] = 1;


//   if(m_debug) {
//     for(int i=0;i<4;i++) {
//       for(int j=i*4;j<(i+1)*4;j++)
//         printf("%1.3f ", T[j]);
//       printf("\n");
//     }
//   }

//   gantry_kinematics::inverse(T, q_sols);

//   auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

//   std::vector<double> data(_robot->GetMultiBody()->DOF()); //Should be 7

//   data[0] =  0; 
//   data[1] =  q_sols[0]/PI;
//   data[2] =  q_sols[1];
//   data[3] =  q_sols[2];
//   data[4] =  q_sols[3];


//   Cfg cfg(_robot);
//   cfg.SetData(data);

//   if(vc->IsValid(cfg,this->GetNameAndLabel())) {
//     if(m_debug)
//       std::cout << cfg << " valid" << std::endl;
//     return cfg;
//   }
//   else 
//     std::cout << cfg << " invalid." << std::endl;

//   return Cfg(nullptr);

//   #else

//   throw RunTimeException(WHERE) << "IK not supported without ROS integration.";
//   return Cfg(_robot);

//   #endif

// }

// void
// BakeStrategy::
// SetEEDOF(Interaction* _interaction, Cfg& _cfg, const std::string& _stage) {
  
//   Constraint* constraint = nullptr;
//   auto as = this->GetTMPLibrary()->GetActionSpace();

//   // Grap role
//   std::string role;
//   for(auto kv : m_roleMap) {
//     if(kv.second == _cfg.GetRobot()) {
//       role = kv.first;
//       break;
//     }
//   }

//   for(auto condition : _interaction->GetStageConditions(_stage)) {
//     auto c = as->GetCondition(condition);
//     auto m = dynamic_cast<MotionCondition*>(c);
//     if(!m)
//       continue;
//     constraint = m->GetRoleConstraint(role);
//     if(constraint)
//       break;
//   }

//   auto b = dynamic_cast<BoundaryConstraint*>(constraint);
//   auto boundary = b->GetBoundary();

//   for(size_t i = 0; i < _cfg.DOF(); i++) {
    
//     auto range = boundary->GetRange(i);

//     // Check if dof is ee 
//     // TODO::temp assume ee dof is 1
//     if(i == 0) {
//       // Sample ee dof within boundary
//       auto ee = range.Sample();
//       _cfg[i] = ee;
//     }
//     else {
//       // Ensure that cfg satisfies boundary
//       if(!range.Contains(_cfg[i])){
//         if (m_debug) {
//           std::cout << "interaction: " << _interaction->GetLabel() << std::endl;
//           std::cout << "index: " << i << std::endl;
//           std::cout << "range: " << range << std::endl;
//           std::cout << "cfg: " << _cfg[i] << std::endl;
//         }
//         throw RunTimeException(WHERE) << "Sampled EE position does not satisfy condition constraint.";
//       }
//     }
//   }
// }
