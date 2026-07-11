#ifndef PPL_MODE_QUERY_H_
#define PPL_MODE_QUERY_H_

#include "TaskEvaluatorMethod.h"

#include "TMPLibrary/StateGraphs/GroundedHypergraph.h"
#include "TMPLibrary/StateGraphs/LazyModeGraph.h"

#include "Utilities/Hypergraph.h"
#include "Utilities/SSSHP.h"

class ModeQuery : public TaskEvaluatorMethod { 

  public:

    ///@name Local Types
    ///@{
    typedef LazyModeGraph::ModeExtendedVertex                        ModeExtendedVertex;
    typedef LazyModeGraph::TransitionSwitch                          TransitionSwitch;
    typedef LazyModeGraph::Vertex                                    ModeVertex;
    typedef LazyModeGraph::ModeTransitionHypergraph                  ModeTransitionHypergraph;
    typedef LazyModeGraph::ModeExtendedHypergraph                    ModeExtendedHypergraph;

    typedef std::set<size_t>                           ActionHistory;
    typedef std::pair<std::set<size_t>,ActionHistory>  PartiallyExtendedHyperarc;
    typedef std::pair<bool,size_t> HPElem;

    struct PartiallyScheduledHyperarc {
      size_t modeHID;
      PartiallyExtendedHyperarc pgh;
      std::set<size_t> constraintTail;
      std::set<size_t> blockingVertices;

      bool operator<(const PartiallyScheduledHyperarc& _other) const {
        if(modeHID != _other.modeHID) 
          return modeHID < _other.modeHID;
        return true;
      }

      bool operator==(const PartiallyScheduledHyperarc& other) const {
        return modeHID == other.modeHID
               and pgh == other.pgh
               and constraintTail == other.constraintTail
               and blockingVertices == other.blockingVertices;
      }
    };

    struct SchedulingConstraint {
      bool vertex{false}; // true = vertex, false = hyperarc
      size_t id;

      bool operator<(const SchedulingConstraint& _other) const {
        if(id != _other.id) 
          return id < _other.id;
        return vertex < _other.vertex;
      }

      bool operator==(const SchedulingConstraint& _other) const {
        if(id == _other.id and vertex == _other.vertex) 
          return true;
      }
      
    };


    /// Hyperarcs are hids in grounded hypergraph
    typedef size_t                               VID;
    typedef size_t                               HID;

    ///@}
    ///@name Construction
    ///@{

    ModeQuery();

    ModeQuery(XMLNode& _node);

    virtual ~ModeQuery();

    ///@}
    ///@name Task Evaluator Interface
    ///@{

    virtual void Initialize();

    ///@}

    void AddSchedulingConstraint(SemanticTask* _task, SemanticTask* _constraint);

    void SampleHyperpath();


  protected:

    ///@name Helper Functions
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    ActionHistory CombineHistories(size_t _vid, const std::set<size_t>& _pgh,
                                   const ActionHistory& _history,
                                   size_t hid,
                                   bool _greedy=true);

    void ConvertToPlan(Plan* _plan);
    
    void ConvertToTaskPlan(Plan* _plan);

    std::vector<HPElem> ConstructPath(size_t _sink, 
                std::set<HPElem>& _parents, const MBTOutput& _mbt);

    std::vector<HPElem> AddBranches(std::vector<HPElem> _path, 
                std::set<HPElem>& _parents, const MBTOutput& _mbt);

    std::vector<HPElem> AddDanglingNodes(std::vector<HPElem> _path,
                std::set<HPElem>& _parents);

    std::vector<HPElem> OrderPath(std::vector<HPElem> _path);

    void ComputeHeuristicValues();
    void ComputeCostToGoValues();

    std::set<size_t> GetFrontier(ActionHistory _history);

    std::set<size_t> GetExtendedFrontier(ActionHistory _history);

    void CheckSchedulingConstraints(size_t _hid, std::set<size_t> _tail, ActionHistory _history, std::set<size_t>& _fullyExtendedHyperarcs);

    size_t AddHyperarc(std::set<size_t> _tail, std::set<size_t> _head, size_t _HID, std::set<size_t>& _fullyExtendedHyperarcs);

    ///@}
    ///@name Hyperpath Functions
    ///@{

    void HyperpathQuery();

    SSSHPTermination HyperpathTermination(const size_t& _vid, const MBTOutput& _mbt);

    double HyperpathPathWeightFunction(
          const typename ModeExtendedHypergraph::Hyperarc& _hyperarc,
          const std::unordered_map<size_t,double> _weightMap,
          const size_t _target);

    std::set<size_t> HyperpathForwardStar(const size_t& _vid, ModeExtendedHypergraph* _h, const std::set<size_t>& _interactionConstraints={}, bool _greedy=true);


    double HyperpathHeuristic(const size_t& _target);

    void ClearData();

    size_t ConnectSink();

    // ModeExtendedHypergraph& GetModeExtendedHypergraph();

    ///@name Internal State
    ///@{

    ModeExtendedHypergraph m_modeExtendedHypergraph;

    /// Set of action extended vertices that contained previous solutions

    /// Map from grounded hypergraph vertices to action extended vertices

    /// Map from grounded hypergraph hyperarcs to action extended hyperarcs

    /// Map from grounded hypergraph hyperarcs to action extended partial groundings

    size_t m_goalVID;

    std::unordered_map<SemanticTask*,size_t> m_vertexTasks;
    std::unordered_map<SemanticTask*,size_t> m_hyperarcTasks;

    /// Map from grouneded hypergraph vertex to heuristic value
    std::unordered_map<size_t,double> m_costToGoMap; 

    std::unordered_map<size_t,double> m_searchTimeHeuristicMap;

    double m_maxDistance;

    bool m_reverseActions{false};

    bool m_writeHypergraph{false};

    bool m_monotonic;

    // Saving frontier of hypergraph search
    MBTOutput m_mbt;
    
    std::priority_queue<SSSHPElement,std::vector<SSSHPElement>,std::greater<SSSHPElement>> m_pq;


    std::string m_mgLabel;

    std::string m_ghLabel;


    
    size_t m_counter{0};

    std::set<size_t> m_previousSolutions;

    std::unordered_map<size_t,std::set<size_t>> m_vertexMap;

    std::unordered_map<size_t,std::set<size_t>> m_hyperarcMap;

    std::unordered_map<size_t,std::vector<PartiallyExtendedHyperarc>> m_partiallyExtendedHyperarcs;

    std::vector<PartiallyScheduledHyperarc> m_blockedHyperarcs;

    std::unordered_map<size_t,std::set<size_t>> m_blockingMap;

    std::set<size_t> m_computedFS;
 
    std::map<SchedulingConstraint,std::set<SchedulingConstraint>> m_constraintMap;

    std::unordered_map<size_t,std::set<size_t>> m_hyperarcConstraintTails;

    // std::unordered_map<size_t,std::map<std::set<size_t>,int>> m_blockCount;
    size_t m_prevHistoryGraphGoalVID;

    std::unordered_map<Robot*,size_t> m_objectLevelMap;
    
    ///@}
};


#endif
