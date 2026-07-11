#ifndef PPL_SCHEDULED_CBS_H_
#define PPL_SCHEDULED_CBS_H_

#include "TaskEvaluatorMethod.h"

#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupPath.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "Traits/CfgTraits.h"
#include "TMPLibrary/TaskEvaluators/SubmodeQuery.h"

#include "Utilities/CBS.h"

#include <map>
#include <unordered_map>
#include <vector>

class ScheduledCBS : public TaskEvaluatorMethod {

  public:

    ///@name Local Types
    ///@{

    typedef TMPBaseObject::GroupCfgType                       GroupCfgType;
    typedef TMPBaseObject::GroupLocalPlanType                 GroupLocalPlanType;
    typedef TMPBaseObject::GroupRoadmapType                   GroupRoadmapType;
    typedef MPTraits<Cfg>::GroupPathType                      GroupPathType;

    // Edge <Source, Target>, Time Interval <Start, End> 
    typedef std::pair<std::pair<size_t,size_t>,Range<size_t>> Constraint;
    typedef CBSNode<SemanticTask,Constraint,GroupPathType>    Node;

    typedef std::map<size_t,std::vector<Range<size_t>>> VertexIntervals;
    typedef std::map<std::pair<size_t,size_t>,std::vector<Range<size_t>>> EdgeIntervals;

    typedef GenericStateGraph<SemanticTask*,size_t> ScheduleGraph;

    ///@}
    ///@name Construction
    ///@{

    ScheduledCBS();

    ScheduledCBS(XMLNode& _node);

    ~ScheduledCBS();

    ///@}
    ///@name Task Evaluator Overrides
    ///@{

    virtual void Initialize() override;

    ///@}

    //TODO::Add function to set upper bound for quitting early
    void SetUpperBound(double _upperBound); 

    std::set<std::pair<size_t,size_t>> GetGeometricConstraints();
    std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> GetMotionConstraints();


  private:

    ///@name Overrides
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    ///@}
    ///@name CBS Functions
    ///@{

    bool LowLevelPlanner(Node& _node, SemanticTask* _task, std::string _queryStrategy);

    std::vector<std::pair<SemanticTask*,Constraint>> ValidationFunction(Node& _node);

    std::vector<Node> SplitNodeFunction(Node& _node, 
          std::vector<std::pair<SemanticTask*,Constraint>> _constraints, 
          CBSLowLevelPlannerWithQueryStrategy<SemanticTask,Constraint,GroupPathType>& _lowLevel,
          CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost);

    double CostFunction(Node& _node);

    void InitialSolutionFunction(std::vector<Node>& _root, std::vector<SemanticTask*> _tasks,
                        CBSLowLevelPlannerWithQueryStrategy<SemanticTask,Constraint,GroupPathType>& _lowLevel,
                        CBSCostFunction<SemanticTask,Constraint,GroupPathType>& _cost);

    bool EarlyTermination(const size_t& _numNodes, std::vector<std::pair<SemanticTask*,Constraint>> _constraints);

    ///@}
    ///@name Helper Functions
    ///@{

    GroupPathType* QueryPath(SemanticTask* _task, const size_t _startTime,
                             const Node& _node, std::string _queryStrategy);

    void ConvertToPlan(const Node& _node, Plan* _plan);

    void ConvertToModifiedPlan(const Node& _node, Plan* _plan);

    size_t FindStartTime(SemanticTask* _task, std::set<SemanticTask*> _solved, 
                         std::map<SemanticTask*,size_t> _endTimes);

    void ComputeIntervals(SemanticTask* _task, const Node& _node);

    std::vector<Range<size_t>> ConstructSafeIntervals(std::vector<Range<size_t>>& _unsafeIntervals);

    std::vector<std::vector<std::pair<SemanticTask*,Constraint>>> FindConflicts(Node& _node, bool _getAll = false);

    bool HandleFailure(std::vector<SemanticTask*> _tasks);

    ///@}
    ///@name Critical Path Functions
    ///@{

    void BuildScheduleGraph(Plan* _plan);

    void ComputeScheduleAtomicDistances();

    std::vector<std::vector<size_t>> ComputeCriticalPaths(const Node& _node);

    std::unordered_map<size_t,double> ComputeScheduleSlack(const Node& _node);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_vcLabel;

    std::string m_queryLabel;

    std::string m_queryStrategy;

    std::string m_initialQueryStrategy;

    std::string m_sqLabel;

    std::string m_mqLabel;

    std::string m_pmLabel; ///< The list of path modifiers to use.


    std::map<GroupPathType*,size_t> m_startTimes;

    std::map<GroupPathType*,size_t> m_endTimes;

    double m_upperBound;

    bool m_initial{true};

    typedef std::map<size_t,std::vector<Range<size_t>>> UnsafeVertexIntervals;

    typedef std::map<std::pair<size_t,size_t>,std::vector<Range<size_t>>> UnsafeEdgeIntervals;

    std::vector<std::map<SemanticTask*,UnsafeVertexIntervals>> m_unsafeVertexIntervalMap;

    std::vector<std::map<SemanticTask*,UnsafeEdgeIntervals>> m_unsafeEdgeIntervalMap;

    VertexIntervals m_vertexIntervals;

    EdgeIntervals m_edgeIntervals;

    std::unique_ptr<ScheduleGraph> m_scheduleGraph;

    std::unordered_map<SemanticTask*,std::pair<size_t,size_t>> m_currentScheduleGraph;

    std::map<size_t,size_t> m_scheduleAtomicDistances;

    bool m_bypass{true};

    bool m_quit{false};

    size_t m_buffer{0};

    double m_alpha;

    double m_X;

    size_t m_quitTimes{0};

    bool m_writeSolution{false};

    std::set<std::pair<size_t,size_t>> m_geometricConstraintSet;

    std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> m_motionConstraintSet;

    std::set<std::pair<std::pair<SubmodeQuery::SchedulingConstraint,SubmodeQuery::SchedulingConstraint>,Cfg>> m_motionConstraintCount;

    std::unordered_map<Robot*,SemanticTask*> m_passiveEndTaskMap;



};

#endif
