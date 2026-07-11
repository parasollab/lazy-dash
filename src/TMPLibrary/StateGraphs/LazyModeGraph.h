#ifndef PPL_LAZY_MODE_GRAPH_H_
#define PPL_LAZY_MODE_GRAPH_H_

#include "StateGraph.h"

#include "ConfigurationSpace/Formation.h"
#include "ConfigurationSpace/GroupRoadmap.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/ActionSpace/Action.h"

#include "Traits/CfgTraits.h"

#include "Utilities/Hypergraph.h"
#include "TMPLibrary/TaskEvaluators/SubmodeQuery.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>

class Interaction;

class LazyModeGraph : public StateGraph {

  public:
    
    ///@name Local Types 
    ///@{
    enum class VertexType {
      Normal,
      Dummy
    };

    struct Mode {
      VertexType type = VertexType::Normal;
      RobotGroup*                              robotGroup;
      std::unordered_set<Formation*>           formations;
      std::vector<std::unique_ptr<Constraint>> constraints;

      void setType(VertexType newType) {
          type = newType;
      }

      bool isDummy() const {
        return type == VertexType::Dummy;
      }
    };


    typedef std::pair<Action*,bool>                              ReversibleAction;
    typedef Hypergraph<Mode*,ReversibleAction>                   ModeHypergraph;
    typedef size_t                                               VID;
    typedef size_t                                               HID;
    typedef TMPBaseObject::GroupCfgType                          GroupCfgType;
    typedef TMPBaseObject::GroupLocalPlanType                    GroupLocalPlanType;
    typedef TMPBaseObject::GroupRoadmapType                      GroupRoadmapType;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>>          GroupPathType;
    typedef PathType<MPTraits<Cfg,DefaultWeight<Cfg>>>           Path;
    typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>>     MPSolution;
    typedef std::pair<GroupRoadmapType*,VID>                     GroundedVertex;
    typedef std::vector<std::vector<std::shared_ptr<GroupTask>>> TransitionTaskSet;

    struct Transition {

      //std::unordered_map<Robot*,Path*> explicitPaths;
      std::unordered_map<Robot*,std::vector<Cfg>> explicitPaths;
      std::unordered_map<Robot*,std::pair<double,std::pair<VID,VID>>> implicitPaths;
      std::unordered_map<GroupTask*,std::unordered_set<Formation*>> taskFormations;
      TransitionTaskSet taskSet;
      std::pair<Action*,bool> action;
      bool onOff;
      double cost;

      bool operator==(const Transition& _other) const {
        return explicitPaths  == _other.explicitPaths
           and implicitPaths  == _other.implicitPaths
           and taskSet        == _other.taskSet
           and taskFormations == _other.taskFormations
           and cost           == _other.cost;
      }

      bool operator!=(const Transition& _other) const {
        return !(*this == _other);
      }

    };

    typedef Hypergraph<GroundedVertex,Transition>           GroundedHypergraphLocal;
    typedef Action::State                                   State;
    typedef VID                                             Vertex;
    typedef Hypergraph<Mode*,Transition>                    ModeTransitionHypergraph;

    typedef std::set<size_t>                           ActionHistory;
    struct ModeExtendedVertex {
      VID           modeVID;
      ActionHistory history;
      RobotGroup*   robotGroup;

      bool operator==(const ModeExtendedVertex& _other) const {
        return modeVID     == _other.modeVID
           and history     == _other.history
           and robotGroup  == _other.robotGroup;
      }
    };


    struct TransitionSwitch {
      HID           modeHID;
      bool          onOff;
      double        cost;

      bool operator==(const TransitionSwitch& _other) const {
        return modeHID  == _other.modeHID
           and onOff  == _other.onOff
           and cost   == _other.cost;
      }

      bool operator!=(const TransitionSwitch& _other) const {
        return !(*this == _other);
      }
    };
    typedef Hypergraph<ModeExtendedVertex,TransitionSwitch>     ModeExtendedHypergraph;


    ///@}
    ///@name Construction
    ///@{

    LazyModeGraph();

    LazyModeGraph(XMLNode& _node);

    virtual ~LazyModeGraph();

    ///@}
    ///@name Interface
    ///@{

    void Initialize() override;


    //void GenerateRepresentation(const State& _start);
    void GenerateRepresentation();


    ///@}
    ///@name Accessors
    ///@{

    ModeHypergraph& GetModeHypergraph();

    // ModeExtendedHypergraph& GetModeExtendedHypergraph();

    // void SetModeExtendedHypergraph(ModeExtendedHypergraph _modeExtendedHypergraph);

    void ImproveTaskSpace();

    GroundedHypergraphLocal& GetGroundedHypergraphLocal();
    ModeTransitionHypergraph& GetModeTransitionHypergraph();
    std::set<VID> GetGoalModeVIDs();
    std::set<VID> GetSinkModeVIDs();
    size_t GetSourceModeTransitionVID();
    size_t GetSinkModeTransitionVID();
    set<VID> GetIgnitionModeTransitionVIDs();
    set<VID> GetTerminationModeTransitionVIDs();
    set<VID> GetActiveModeTransitionVIDs();
    set<VID> GetPassiveModeTransitionVIDs();
    std::unordered_map<RobotGroup*,std::unordered_map<Robot*,std::string>> GetGoalRobots();
    void SetExtraTaskSpaceCandidates(std::set<std::pair<size_t,size_t>> _nmc);
    void SetTaskSpaceImprovementCandidates(std::unordered_map<size_t,std::set<size_t>> _tsic);
    
    void SetRelevantMTHIDVector(std::vector<HID> _relevantMTHIDs);
    void SetRelevantMTVIDVector(std::vector<VID> _relevantMTVIDs);
    void SetRelevantMTHIDs(std::set<HID> _relevantMTHIDs);
    void SetRelevantMTVIDs(std::set<VID> _relevantMTVIDs);
    void SetActiveTaskPlan(std::unordered_map<VID,std::set<std::pair<HID,HID>>> _activeTaskPlan);

    std::vector<HID> GetRelevantMTHIDVector();
    HID GetIgnitionMTHID();
    HID GetTerminationMTHID();

    std::unordered_map<HID,std::set<std::pair<std::set<VID>,std::set<VID>>>> GetGroundedTransitionMap();

    GroupRoadmapType* GetGroupRoadmap(RobotGroup* _group);

    //MPSolution* GetMPSolution();
  
    VID GetModeOfGroundedVID(const VID& _vid) const;
    VID GetTransitionModeOfGroundedVID(const VID& _vid) const;

    std::set<VID> GetGoalGroundedVIDs();
    // RobotGroup* GetRobotGroupOfGroundedVID(const VID& _vid) const;

    std::unordered_map<size_t,size_t> GetHidConversionMap() const;
    std::unordered_map<size_t,size_t> GetVidConversionMap() const;

    ///@}

    Transition GetTransition(const HID& _hid);

    Transition GetTransition(const std::set<VID>& _tail, const std::set<VID>& _head);

    std::set<VID> GetSubsinkVIDs();
    std::vector<HID> GetMotionHistory();

    void SampleTransitions(std::set<HID> _targetHIDs={});
    void ResampleTransitions(HID _targetHID, std::pair<std::set<VID>,std::set<VID>>& _nextAssignment);
    void InitializeTransitions();

    void GenerateRoadmaps();

    std::set<VID> ConnectTransitions();
    void ConnectSource();
    void ConnectSink(std::set<VID> _goalGroundedSet);

    void PrintModeCapability();

    void GenerateCombinations(std::unordered_map<RobotGroup*, std::set<VID>> robotsMap,
                           std::set<std::set<VID>>& combinations,
                           std::set<VID> currentCombination,
                           size_t currentRobotIndex);

    bool SampleHyperpath();

    void SampleNonActuatedCfgs(const State& _start);
    void SampleStartGoal(const State& _start,std::set<VID>& _startVIDs,std::set<VID>& _goalVIDs);

    void GenerateRoadmaps(const State& _start,std::set<VID>& _startVIDs,std::set<VID>& _goalVIDs);
    void SampleCfgs(const State& _start,std::unordered_map<RobotGroup*,VID>& _startRobotVIDs);
    void GenerateMotionHypergraph();
    void InitializeMotionHypergraph();
    bool PropagateMotions();

    void SetGeometricConstraints(std::unordered_map<size_t,std::set<size_t>> _gcSet);
    void SetGeometricConstraints2(std::unordered_map<size_t,std::set<size_t>> _gcSet);
    void SetTaskOrderConstraints(std::set<std::vector<HID>> _tcSet);
    void SetMotionConstraints(std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> _mcSet);
    std::unordered_map<size_t,std::set<size_t>> GetGeometricConstraints();
    std::unordered_map<size_t,std::set<size_t>> GetGeometricConstraints2();
    std::unordered_map<VID,std::set<VID>> GetNonMonotonicConstraints();
    std::set<std::vector<HID>> GetTaskOrderConstraints();
    std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> GetMotionConstraints();
    HID GetReplanSource();
    std::set<HID> GetInteractionConstraints();
    std::unordered_map<VID,std::set<VID>> GetGroundedVIDMap();
    void ClearConstraints();
    std::vector<std::pair<HID,std::pair<std::set<VID>,std::set<VID>>>> GetGroundedVertexHistory();


  private:

    ///@name Helper Functions
    ///@{

    std::vector<VID> AddStartState(const State& _start);


    void GenerateModeHypergraph(const std::vector<VID>& _initialModes);


    void IdentifyStartAndGoalVIDs(const State& _start, std::set<VID>& _startVIDs, std::set<VID>& _goalVIDs);

    void GenerateModeTransitionHypergraph(const State& _start);

    void ComputePath(std::unordered_map<VID,VID>& _startRobotGroundedVIDs,
                    std::set<VID> _startGraspGroundedVIDs,
                    std::set<VID> _goalGraspGroundedVIDs,
                    std::set<HID> _candidateHIDs);

    double ComputeEdgeCost(const VID _start, const VID _goal);
    
    void ConfigureGoalSets(const size_t& _sink, std::set<VID>& _goalVIDs);

    std::vector<std::set<SemanticTask*>> BuildTaskSets(std::set<SemanticTask*> _taskSet, size_t _index, 
                                                 const std::vector<std::set<SemanticTask*>>& _buckets);



    void ApplyAction(Action* _action, std::set<std::vector<VID>>& _applied,
                     std::vector<VID>& _newModes, bool _forward=true);

    std::vector<std::vector<LazyModeGraph::VID>> CollectModeSets(
               const std::vector<std::vector<VID>>& _formationModes, 
               size_t _index, 
               const std::vector<VID>& _partialSet);


    std::vector<std::vector<Mode*>> CollectModeSetCombinations(
                        const std::vector<std::vector<std::vector<Robot*>>>& _possibleAssignments,
                        size_t _index, std::vector<Mode*> _partial, 
                        const std::set<Robot*>& _used);


    std::vector<Mode*> CollectModeCombinations(const std::vector<std::vector<Robot*>>& _possibleModeAssignments,
                        size_t _index, const std::vector<Robot*> _partial,
                        const std::set<Robot*>& _used);

    void SaveInteractionPaths(size_t _hid, Interaction* _interaction, State& _start, State& _end,
                                 std::unordered_map<RobotGroup*,Mode*> _startModeMap,
                                 std::unordered_map<RobotGroup*,Mode*> _endModeMap);

    VID AddMode(Mode* _mode);

    std::set<VID> AddStateToGroundedHypergraphLocal(const State& _state, std::unordered_map<RobotGroup*,Mode*> _modeMap);

    bool CanReach(const State& _state);
    Cfg CanReach(Interaction* _interaction, const RobotGroup* _activeRobot, GroupCfgType _gcfg);

    bool ContainsSolution(std::set<VID>& _startVIDs);

    double GetDistance(RobotGroup* _active, GroupCfgType _gcfg);


    ///@}
    ///@name Internal State
    ///@{

    // Main set of representations

    ModeHypergraph m_modeHypergraph;
    ModeTransitionHypergraph m_modeTransitionHypergraph;
    // ModeExtendedHypergraph m_modeExtendedHypergraph;
    GroundedHypergraphLocal m_groundedHypergraph;
    GroundedHypergraphLocal m_groundedHypergraphLocal;

    //std::unique_ptr<MPSolution> m_solution;

    // Additional info

    std::vector<unique_ptr<Mode>> m_modes;

    std::unordered_map<VID,std::unordered_set<VID>> m_modeGroundedVertices;
    std::unordered_map<VID,std::unordered_set<VID>> m_modeStartGroundedVIDs;
    std::unordered_map<VID,std::unordered_set<VID>> m_modeGoalGroundedVertices;
    std::unordered_map<VID,std::set<VID>> m_modeTransitionGroundedVertices;

    std::map<size_t,size_t> m_groundedInstanceTracker;

    std::set<size_t> m_unactuatedModes;
    std::set<size_t> m_unactuatedMTVIDs;

    // XML Parameters

    std::string m_unactuatedSM;

    std::string m_querySM;

    std::string m_queryStrategy;

    std::string m_queryStrategyStatic;

    std::string m_expansionStrategy;

    size_t m_numUnactuatedSamples;

    size_t m_numInteractionSamples;

    size_t m_maxAttempts;

    std::set<size_t> m_entryVertices;
    std::set<size_t> m_exitVertices;

    std::unordered_map<SemanticTask*,size_t> m_goalVertexTaskMap;

    std::unordered_map<Robot*,Cfg> m_objectExtraPose;

    bool m_writeHypergraphs{false};

    // TEMP TO GET SEPARATE GROUNDED HYPERGRAPH
    std::string m_GH;
    std::string m_MH;
    bool m_roadmap;

    // Aggressive at the moment assuming objects are homogeneous
    bool m_copyRoadmaps{true}; ///< Flag to copy roadmaps for a robot for all objects

    bool m_complete{false};
    bool m_allGoals{false};

    std::unordered_map<size_t,size_t> m_hidConversionMap;
    std::unordered_map<size_t,size_t> m_vidConversionMap;

    std::vector<HID> m_motionHistory;

    State m_start;
    std::set<VID> m_startGroundedVIDs;
    std::set<VID> m_goalGroundedVIDs;
    // std::set<VID> m_startModeVIDs;
    // std::set<VID> m_goalModeVIDs;
    std::set<VID> m_sinkModeVIDs;
    VID m_sourceGroundedVID;
    VID m_sinkGroundedVID;
    VID m_sourceModeTransitionVID;
    VID m_sinkModeTransitionVID; 
    HID m_ignitionMTHID;
    HID m_terminationMTHID;
    std::set<VID> m_ignitionModeTransitionVIDs;
    std::set<VID> m_terminationModeTransitionVIDs;
    std::set<VID> m_activeModeTransitionVIDs;
    std::set<VID> m_passiveModeTransitionVIDs;
    std::unordered_map<RobotGroup*,VID> m_startRobotGroundedVIDs;
    
    std::set<HID> m_modeExtendedRelevantHIDs;
    std::set<HID> m_relevantMHIDs;
    std::set<VID> m_relevantMVIDs;
    std::set<HID> m_relevantMTHIDs;
    std::set<VID> m_relevantMTVIDs;
    std::set<HID> m_computedInteraction;
    std::vector<HID> m_relevantMTHIDVector;
    std::vector<VID> m_relevantMTVIDVector;
    std::set<VID> m_relevantGVIDs;
    std::unordered_map<VID,std::set<std::pair<HID,HID>>> m_activeTaskPlan;
    std::unordered_map<HID,HID> m_modeHIDReverseMap;
    std::unordered_map<HID,HID> m_relevantMTHIDsToMHIDs;
    std::unordered_map<HID,std::set<std::pair<std::set<VID>,std::set<VID>>>> m_groundedTransitionMap;

    std::set<VID> m_subsinkVIDs;
    std::vector<std::pair<std::vector<std::set<HID>>,int>> m_validHistorySets;
    size_t currentHistory{0};

    std::unordered_map<RobotGroup*,std::vector<std::unique_ptr<Constraint>>> m_constraintMap;

    HID m_replanSource{MAX_UINT};
    std::unordered_map<size_t,std::set<size_t>> m_geometricConstraintSet;
    std::unordered_map<size_t,std::set<size_t>> m_geometricConstraintSet2;
    std::unordered_map<size_t,std::set<size_t>> m_prevGeometricConstraintSet;
    std::unordered_map<size_t,std::set<size_t>> m_prevGeometricConstraintSet2;
    std::unordered_map<VID,std::set<VID>> m_nonMonotonicConstraintSet;
    std::set<std::vector<HID>> m_taskOrderConstraintSet;
    std::set<HID> m_interactionConstraintSet;
    std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> m_motionConstraintSet;
    std::map<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>,int> m_motionConstraintCount;
    std::set<std::pair<size_t,size_t>> m_extraTaskSpaceCandidates;
    std::unordered_map<size_t,std::set<size_t>> m_taskSpaceImprovementCandidates;
    
    int m_iterationCall{0};

    size_t m_resampleAttempts;

    double m_robotBaseMaxDistance;

    double m_robotRange{1.0};

    bool m_monotonic{true};

    std::set<RobotGroup*> m_improveComputed;

    std::set<RobotGroup*> m_computedRoadmap;

    ///@}

};

std::ostream& operator<<(std::ostream& _os, const LazyModeGraph::Mode* _mode);
std::istream& operator>>(std::istream& _is, const LazyModeGraph::Mode* _mode);

std::ostream& operator<<(std::ostream& _os, const LazyModeGraph::ReversibleAction _ra);
std::istream& operator>>(std::istream& _is, const LazyModeGraph::ReversibleAction _ra);

//std::ostream& operator<<(std::ostream& _os, const LazyModeGraph::GroundedVertex _vertex);
//std::istream& operator>>(std::istream& _is, const LazyModeGraph::GroundedVertex _vertex);

std::ostream& operator<<(std::ostream& _os, const LazyModeGraph::Transition _t);
std::istream& operator>>(std::istream& _is, const LazyModeGraph::Transition _t);

std::ostream& operator<<(std::ostream& _os, const LazyModeGraph::TransitionSwitch _t);
std::istream& operator>>(std::istream& _is, const LazyModeGraph::TransitionSwitch _t);

std::ostream& operator<<(std::ostream& _os, const LazyModeGraph::ModeExtendedVertex& _vertex);
std::istream& operator>>(std::istream& _is, const LazyModeGraph::ModeExtendedVertex& _vertex);

#endif