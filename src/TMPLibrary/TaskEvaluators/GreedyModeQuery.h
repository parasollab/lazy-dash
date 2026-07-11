
#ifndef PPL_GREEDY_MODE_QUERY_H_
#define PPL_GREEDY_MODE_QUERY_H_

#include "ModeQuery.h"

#include "ConfigurationSpace/GenericStateGraph.h"

#include "TMPLibrary/Solution/Plan.h"

#include "TMPLibrary/StateGraphs/LazyModeGraph.h"

#include "TMPLibrary/TaskEvaluators/SubmodeQuery.h"


class GreedyModeQuery : public ModeQuery {

  public:

    ///@name Local Types
    ///@{

    typedef ModeQuery::ActionHistory             ActionHistory;
    // typedef ModeQuery::ActionHistoryInfo         ActionHistoryInfo;
    typedef size_t                               VID;
    typedef size_t                               HID;


    typedef GenericStateGraph<ActionHistory,HID> HistoryGraph;

    ///@}
    ///@name Construction
    ///@{

    GreedyModeQuery();

    GreedyModeQuery(XMLNode& _node);

    ~GreedyModeQuery();

    ///@}
    ///@name Task Evaluator Interface
    ///@{

    virtual void Initialize();

    ///@}

  protected:

    ///@name Task Evaluator Functions
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    ///@}
    ///@name Helper Functions
    ///@{

    VID DFS(const VID _source);

    VID Termination(const VID _vid);

    std::vector<GreedyModeQuery::VID> Frontier(const VID _vid);

    std::set<VID> BuildQuantumFrontier(std::set<VID> _frontier, ActionHistory _history, std::set<size_t> _interactionConstraints);

    void ExpandVertex(const VID _source, const VID _vid, const std::set<VID> _frontier, 
                      const ActionHistory _history,
                      std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>>& _transNeighbors,
                      std::priority_queue<std::pair<double,VID>,std::vector<std::pair<double,VID>>>& _motionNeighbors,
                      std::set<size_t> _interactionConstraints);

    bool IsValidHistory(const ActionHistory& _history);
    
    void SetMotionConstraints(std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> _mcSet);

    virtual void SetReplanSource(HID _rs) override;
    
    virtual void SetInteractionConstraints(std::set<HID> _icSet) override;
    
    virtual void SetGeometricConstraints(std::unordered_map<size_t,std::set<size_t>> _gcSet) override;
    
    virtual void SetGeometricConstraints2(std::unordered_map<size_t,std::set<size_t>> _gcSet) override;

    virtual void SetNonMonotonicConstraints(std::unordered_map<VID,std::set<VID>> _nmcSet) override;

    virtual void SetTaskOrderConstraints(std::set<std::vector<size_t>> _tcSet) override;

    std::unordered_map<size_t,std::set<size_t>> GetGeometricConstraints();

    std::unordered_map<size_t,std::set<size_t>> GetGeometricConstraints2();

    std::unordered_map<VID,std::set<VID>> GetNonMonotonicConstraints();

    std::set<std::vector<size_t>> GetTaskOrderConstraints();

    std::set<size_t> GetInteractionConstraints();

    std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> GetMotionConstraints();

    void ClearData();


    ///@}
    ///@name Internal State
    ///@{

    std::unique_ptr<HistoryGraph> m_historyGraph;

    std::unordered_map<VID,double> m_heuristicValues;

    std::unordered_map<size_t,std::set<size_t>> m_geometricConstraintSet;
    
    std::unordered_map<size_t,std::set<size_t>> m_geometricConstraintSet2;

    std::unordered_map<VID,std::set<VID>> m_nonMonotonicConstraintSet;

    std::set<std::vector<size_t>> m_taskOrderConstraintSet;

    std::set<size_t> m_interactionConstraintSet;

    size_t m_prevHistoryGraphGoalVID;
    
    HID m_replanSource{MAX_UINT};

    VID m_replanHistoryGraphVID{MAX_UINT};

    std::set<std::vector<VID>> m_priorityQueueSet;

    ///@}

};

#endif
