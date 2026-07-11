#ifndef PMPL_GROUP_LAZY_QUERY_H_
#define PMPL_GROUP_LAZY_QUERY_H_

#include "GroupQuery.h"
#include "SIPPMethod.h"

#include <algorithm>
#include <functional>
#include <unordered_map>


////////////////////////////////////////////////////////////////////////////////
/// First assumes all nodes and edges are valid, then checks for validity of the
/// nodes/edges used in the path.
///
/// Reference:
///   Robert Bohlin and Lydia E. Kavraki. "Path Planning Using Lazy PRM".
///   ICRA 2000.
///
/// @note Node enhancement does not work like in the paper. Here we use a
///       flat gaussian distribution with fixed distance.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupLazyQuery : virtual public GroupQuery<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType                   CfgType;
    typedef typename MPTraits::GroupCfgType              GroupCfgType;
    typedef typename MPTraits::GroupRoadmapType          GroupRoadmapType;
    typedef typename GroupRoadmapType::VID               VID;
    typedef typename GroupRoadmapType::EdgeID            EdgeID;
    typedef typename MPTraits::GoalTracker               GoalTracker;
    typedef typename GoalTracker::VIDSet                 VIDSet;
    typedef typename GroupRoadmapType::EID               ED;
    typedef typename GroupRoadmapType::VI                VI;
    typedef typename GroupRoadmapType::EI                EI;
    typedef typename GroupRoadmapType::CVI               CVI;
    typedef typename GroupRoadmapType::CEI               CEI;

    ///@}
    ///@name Construction
    ///@{

    GroupLazyQuery();
    GroupLazyQuery(XMLNode& _node);
    virtual ~GroupLazyQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}
    ///@name GroupQuery Overrides
    ///@{

    /// Set an alternate path weight function to use when searching the roadmap
    /// @param _f The path weight function to use.
    virtual void SetPathWeightFunction(SSSPPathWeightFunction<GroupRoadmapType> _f);

    ///@}

  protected:

    ///@name Internal Types
    ///@{

    typedef std::unordered_set<VID>    VertexSet;
    typedef std::unordered_set<EdgeID> EdgeSet;

    ///@}
    ///@name GroupQuery Overrides
    ///@{

    /// Reset the path and list of undiscovered goals
    /// @param _r The roadmap to use.
    virtual void Reset(GroupRoadmapType* const _r) override;

    virtual bool PerformSubQuery(const VID _start, const VIDSet& _goals);

    virtual double StaticPathWeight(
        typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const
        override;

    virtual double DynamicPathWeight(
        typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const
        override;

    ///@}
    ///@name Helpers
    ///@{

    /// Checks validity of nodes and edges and deletes any invalid ones.
    /// @return True if the path was valid.
    bool ValidatePath();

    /// Check each vertex and ensure it is valid. Upon discovering an invalid
    /// vertex, delete it and return.
    /// @return True if a vertex was deleted.
    bool PruneInvalidVertices();

    /// Check each edge and ensure it is valid. Upon discovering an invalid edge,
    /// delete it and return.
    /// @return True if an edge was deleted.
    bool PruneInvalidEdges();

    /// Choose a random deleted edge and generate nodes with a gaussian
    /// distribution around the edge's midpoint.
    virtual void NodeEnhance();

    /// Additional handling of invalid vertices.
    /// @param _cfg The invalid configuration to handle.
    virtual void ProcessInvalidNode(const GroupCfgType& _cfg) { }

    /// Invalidate or delete a roadmap configuration according to the deletion
    /// option.
    /// @param _vid The vertex descriptor.
    void InvalidateVertex(const VID _vid);

    /// Invalidate or delete a roadmap edge according to the deletion option.
    /// @param _source The source vertex descriptor.
    /// @param _target The target vertex descriptor.
    void InvalidateEdge(const VID _source, const VID _target);

    ///@}
    ///@name Lazy Invalidation
    ///@{

    /// Set a vertex as invalidated.
    /// @param _vid The vertex descriptor.
    void SetVertexInvalidated(const VID _vid) noexcept;

    /// Check if a vertex is lazily invalidated.
    /// @param _vid The vertex descriptor.
    /// @return     True if _vid is lazily invalidated.
    bool IsVertexInvalidated(const VID _vid) const noexcept;

    /// Check if an edge is lazily invalidated.
    /// @param _eid The edge ID.
    /// @return     True if _eid is lazily invalidated.
    bool IsEdgeInvalidated(const EdgeID _eid) const noexcept;

    /// @overload This version takes the source and target VIDs for an edge.
    /// @param _source The VID of the source vertex.
    /// @param _target The VID of the target vertex.
    /// @return        True if (_source, _target) is lazily invalidated.
    bool IsEdgeInvalidated(const VID _source, const VID _target) const noexcept;

    /// Set an edge as invalidated.
    /// @param _eid The edge ID.
    void SetEdgeInvalidated(const EdgeID _eid) noexcept;

    /// @overload
    /// @param _source  The VID of the source vertex.
    /// @param _target  The VID of the target vertex.
    void SetEdgeInvalidated(const VID _source, const VID _target) noexcept;


    /// Get a random configuration to grow towards.
    CfgType SelectIndividualTarget();
    std::vector<CfgType> SelectFormationTarget(Formation* _formation);
    GroupCfgType SelectTarget();


    ///@}
    ///@name MP Object Labels
    ///@{

    std::string m_vcLabel;         ///< The lazy validity checker label.
    std::string m_lpLabel;         ///< The lazy local planner label.
    std::string m_enhanceDmLabel;  ///< The distance metric for enhancement.
    bool m_debug{false};                  ///< Print debug info?
    bool m_doubleCheckVertices{false};
    
    std::vector<std::string> m_ncLabels; ///< The connectors for enhancement.

    ///@}
    ///@name Internal State
    ///@{

    bool m_deleteInvalid{true};   ///< Remove invalid vertices from the roadmap?

    std::vector<int> m_resolutions{1}; ///< List of resolution multiples to check.
    size_t m_numEnhance{0};       ///< Number of enhancement nodes to generate.
    double m_d{0};                ///< Gaussian distance for enhancement sampling.
    size_t m_maxAttempts{10};
    size_t m_maxIterations{1};

    std::string m_extraEvalLabel{""};

    /// Candidate edges for enhancement sampling.
    std::vector<std::pair<GroupCfgType, GroupCfgType>> m_edges;

    /// Lazy-invalidated vertices.
    std::unordered_map<GroupRoadmapType*, VertexSet> m_invalidVertices;
    /// Lazy-invalidated edges.
    std::unordered_map<GroupRoadmapType*, EdgeSet> m_invalidEdges;

    /// Stuff for sample checking for guiding spaces things
    bool m_trackVerified{false};
    std::unique_ptr<GroupRoadmapType> m_verifiedRoadmap;
    std::unique_ptr<GroupRoadmapType> m_validRoadmap;
    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GroupLazyQuery<MPTraits>::
GroupLazyQuery() : GroupQuery<MPTraits>() {
  this->SetName("GroupLazyQuery");
}


template <typename MPTraits>
GroupLazyQuery<MPTraits>::
GroupLazyQuery(XMLNode& _node) : GroupQuery<MPTraits>(_node) {
  this->SetName("GroupLazyQuery");

  m_vcLabel = _node.Read("vcLabel", true, "", "Lazy validity checker method.");
  m_lpLabel = _node.Read("lpLabel", true, "", "Local planner method.");

  m_deleteInvalid = _node.Read("deleteInvalid", false, m_deleteInvalid,
      "Remove invalid vertices from the roadmap?");

  m_numEnhance = _node.Read("numEnhance", false, m_numEnhance,
      size_t(0), std::numeric_limits<size_t>::max(),
      "Number of nodes to generate in node enhancement");
  m_d = _node.Read("d", false, m_d,
      0., std::numeric_limits<double>::max(),
      "Gaussian d value for node enhancement");
  m_enhanceDmLabel = _node.Read("enhanceDmLabel", m_numEnhance, "",
      "Distance metric method for generating enhancement nodes.");

  m_trackVerified = _node.Read("trackVerified",false,m_trackVerified,
      "Flag indicating if all verified vertices should be tracked.");

  m_maxAttempts = _node.Read("maxAttempts", false, m_maxAttempts,
      size_t(0), std::numeric_limits<size_t>::max(),
      "The maximum number of attempts for enhancements");
  
  m_maxIterations = _node.Read("maxIterations", false, m_maxIterations,
      size_t(1), std::numeric_limits<size_t>::max(),
      "The maximum number of attempts for enhancements");
  
  m_extraEvalLabel = _node.Read("extraEvalLabel", false, m_extraEvalLabel, 
      "Extra evaluation label");

  m_debug = _node.Read("debug", false, m_debug, "Show run-time debug info?");

  m_doubleCheckVertices = _node.Read("doubleCheckVertices", false, m_doubleCheckVertices, "Show run-time debug info?");

  for(auto& child : _node) {
    if(child.Name() == "Resolution")
      m_resolutions.push_back(child.Read("mult", true, 1, 1, MAX_INT,
          "Multiple of finest resolution checked, >= 1. Higher resolutions are "
          "coarser initial checks."));
    else if(child.Name() == "NodeConnectionMethod")
      m_ncLabels.push_back(child.Read("label", true, "",
          "Connector method for enhancement nodes."));
  }

  // Sort resolutions in decreasing order.
  std::sort(m_resolutions.begin(), m_resolutions.end(), std::greater<int>());
  auto iter = std::unique(m_resolutions.begin(), m_resolutions.end());
  m_resolutions.erase(iter, m_resolutions.end());

  // Ensure that resolution '1' is included.
  if(m_resolutions.back() != 1)
    throw RunTimeException(WHERE) << "Last resolution should be '1', but it is '"
                                  << m_resolutions.back() << "'.";
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
GroupLazyQuery<MPTraits>::
Print(std::ostream& _os) const {
  GroupQuery<MPTraits>::Print(_os);
  _os << "\tEnhancement distance Metric: " << m_enhanceDmLabel
      << "\n\tLocal Planner: " << m_lpLabel
      << "\n\tValidity Checker: " << m_vcLabel
      << "\n\tDelete Invalid: " << m_deleteInvalid
      << "\n\tnumEnhance: " << m_numEnhance
      << "\n\td: " << m_d
      << "\n\tresolutions:";
  for(const auto r : m_resolutions)
    _os << " " << r;
  _os << std::endl;
}


template <typename MPTraits>
void
GroupLazyQuery<MPTraits>::
Initialize() {
  GroupQuery<MPTraits>::Initialize();
  m_edges.clear();
  m_invalidVertices.clear();
  m_invalidEdges.clear();

  auto group = this->GetGroupTask()->GetRobotGroup();
  // MPSolution* solution = new MPSolution(group);
  auto solution = this->GetMPSolution();

  if(m_trackVerified) {
    m_verifiedRoadmap = std::unique_ptr<GroupRoadmapType>(new GroupRoadmapType(group,solution));
    m_validRoadmap = std::unique_ptr<GroupRoadmapType>(new GroupRoadmapType(group,solution));
  }
}

/*--------------------------- GroupQuery Overrides --------------------------*/

template <typename MPTraits>
void
GroupLazyQuery<MPTraits>::
SetPathWeightFunction(SSSPPathWeightFunction<GroupRoadmapType> _f) {
  using EI = typename GroupRoadmapType::adj_edge_iterator;

  // Wrap the requested weight function with a preceding check on invalidation.
  this->m_weightFunction = [this, _f](EI& _ei,
                                      const double _sourceDistance,
                                      const double _targetDistance) {
    if(this->IsEdgeInvalidated(_ei->id()))
      return std::numeric_limits<double>::infinity();
    return _f(_ei, _sourceDistance, _targetDistance);
  };
}


template <typename MPTraits>
void
GroupLazyQuery<MPTraits>::
Reset(GroupRoadmapType* const _r) {
  GroupQuery<MPTraits>::Reset(_r);

  // Create storage for this roadmap's invalidations.
  m_invalidVertices[_r];
  m_invalidEdges[_r];
}


template <typename MPTraits>
bool
GroupLazyQuery<MPTraits>::
PerformSubQuery(const VID _start, const VIDSet& _goals) {
  size_t i = 0;
  
  if(m_debug)
    std::cout << "\nGroupLazyQuery SubQuery. Attempt " << i << std::endl;

  if(m_extraEvalLabel != "") {
    if(m_debug)
      std::cout << "RUN SIPP: " << _start << " --> " << _goals << std::endl;

    auto q = dynamic_cast<SIPPMethod<MPTraits>*>(this->GetMPLibrary()->GetMapEvaluator(m_extraEvalLabel));
    q->operator()();

    if(ValidatePath()) {
      if(m_debug) {
        std::cout << "** VALID PATH FOUND IN SIPP: ";
        auto path = this->GetGroupPath();
        auto rm = path->GetRoadmap();
        for(const auto vid : path->VIDs())
          std::cout << vid << ": " << rm->GetVertex(vid).PrettyPrint() << std::endl;
      }
      return true;
    }
    else {
      NodeEnhance();
      while(q->operator()()) {
        i++;
        if(m_debug)
          std::cout << "GOT TRUE. VALIDATE PATH. Attempt #" << i << std::endl;
        if(ValidatePath()) {
          if(m_debug) {
            std::cout << "** VALID PATH FOUND IN SIPP after iteration " << std::endl;
            std::cout << "Valid Path is: " ;
            auto path = this->GetGroupPath();
            auto rm = path->GetRoadmap();
            for(const auto vid : path->VIDs())
              std::cout << vid << ": " << rm->GetVertex(vid).PrettyPrint() << std::endl;
          }
          return true;
        }
        else {
          NodeEnhance();
        }
      }
    }
    if(m_debug)
      std::cout << "** FAILED TO FIND A VALID PATH IN SIPP" << std::endl;
  }
  else {
    if(m_debug)
      std::cout << "RUN GROUP LAZY QUERY: " << _start << " --> " << _goals << std::endl;
    while(GroupQuery<MPTraits>::PerformSubQuery(_start, _goals)) {
      if(m_debug)
        std::cout << "GOT TRUE. VALIDATE PATH " << std::endl;
      if(ValidatePath()) {
        if(m_trackVerified) {
          const std::string base = this->GetBaseFilename();
          m_verifiedRoadmap->Write(base + ".verification.map",
              this->GetEnvironment());
          m_verifiedRoadmap->Write(base + ".valid.map",
              this->GetEnvironment());
        }
        if(m_debug)
          std::cout << "** VALID PATH FOUND IN GROUP LAZY QUERY" << std::endl;
        return true;
      }
    }
    if(m_debug) {
      std::cout << "** FAILED TO FIND A VALID PATH IN LAZY QUERY" << std::endl;
      std::cout << "** ALL STRATEGY FAILED TO FIND A VALID PATH" << std::endl;
      std::cout << "ENHANCE" << std::endl;
    }
    // There are no valid paths, enhance and return false.
    NodeEnhance();
  }
    
  // }
  return false;
}


template <typename MPTraits>
double
GroupLazyQuery<MPTraits>::
StaticPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // First check if the edge is lazily invalidated. If so, the distance is
  // infinite.
  if(this->IsEdgeInvalidated(_ei->id()))
    return std::numeric_limits<double>::infinity();

  return GroupQuery<MPTraits>::StaticPathWeight(_ei, _sourceDistance,
      _targetDistance);
}


template <typename MPTraits>
double
GroupLazyQuery<MPTraits>::
DynamicPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) const {
  // First check if the edge is lazily invalidated. If so, the distance is
  // infinite.
  if(this->IsEdgeInvalidated(_ei->id()))
    return std::numeric_limits<double>::infinity();

  return GroupQuery<MPTraits>::DynamicPathWeight(_ei, _sourceDistance,
      _targetDistance);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
GroupLazyQuery<MPTraits>::
ValidatePath() {
  auto path = this->GetGroupPath();

  if(this->m_debug) {
    std::cout << "\tValidating path for " << path->GetRoadmap()->GetGroup()->GetLabel() << " " << path->VIDs() << " " << path->GetRoadmap() << "\n\t";
    auto vc = this->GetValidityChecker(m_vcLabel);
    for(auto v : path->VIDs()) {
      std::cout << vc->IsValid(this->m_roadmap->GetVertex(v),this->GetNameAndLabel()) << " ";
    }
    std::cout << std::endl;
  }

  if(path->Size() == 0 or PruneInvalidVertices() or PruneInvalidEdges()) {
    path->Clear();
    if(this->m_debug)
      std::cout << "\tPath is invalid." << std::endl;
    return false;
  }

  if(this->m_debug)
    std::cout << "\tPath is valid." << std::endl;

  return true;
}


template <typename MPTraits>
bool
GroupLazyQuery<MPTraits>::
PruneInvalidVertices() {
  if(this->m_debug)
    std::cout << "\t\tChecking vertices..." << std::endl;

  auto vc = this->GetValidityChecker(m_vcLabel);
  auto path = this->GetGroupPath();

  // Check each vertex in the path.
  for(size_t i = 0; i < path->VIDs().size(); ++i) {
    
    // Work from the outside towards the middle.
    const size_t index = i % 2 ? path->Size() - i / 2 - 1 : i / 2;
    auto vid = path->VIDs()[index];

    GroupCfgType& cfg = this->m_roadmap->GetVertex(vid);

    if(cfg.IsLabel("VALID")) {
      continue;
    }

    // Validate cfg. Move on to the next if it is valid.
    auto valid = vc->IsValid(cfg, "GroupLazyQuery::ValidatePath");

    if(m_trackVerified)
      m_verifiedRoadmap->AddVertex(cfg);
    if(m_trackVerified and valid)
      m_validRoadmap->AddVertex(cfg);


    if(valid) {
      continue;
    }

    // If we're here, the cfg is invalid.
    if(this->m_debug)
      std::cout << "\t\tNode " << vid << " found invalid during path validation. "
                << "\n\t" << cfg.PrettyPrint()
                << std::endl;

    // Invalidate the cfg.
    InvalidateVertex(vid);
    return true;
  }

  if(this->m_debug)
    std::cout << "\t\t\tVertices are ok." << std::endl;

  return false;
}


template <typename MPTraits>
bool
GroupLazyQuery<MPTraits>::
PruneInvalidEdges() {
  auto env = this->GetEnvironment();
  auto lp = this->GetLocalPlanner(m_lpLabel);
  auto path = this->GetGroupPath();

  if(this->m_debug)
    std::cout << "\t\tChecking edges..." << std::endl;

  // Perform the check for each resolution.
  for(const auto res : m_resolutions) {
    if(this->m_debug)
      std::cout << "\t\tChecking with resolution " << res << "..." << std::endl;

    for(size_t i = 0; i < path->Size() - 1; ++i) {
      if(m_doubleCheckVertices) {
        if(i==0 or i==path->Size()-1)
          continue;
      }
      // Check from outside to middle
      const size_t index = i % 2 ? path->Size() - i / 2 - 2 : i / 2;
      const VID v1 = path->VIDs()[index],
                v2 = path->VIDs()[index + 1];
      if(this->m_debug)
        std::cout << "\t\t\tEdge: " << v1 << " --> " << v2 << std::endl;

      // if(path->Size() > 2 and (i==0 or i==path->Size()-1)) {
      //   std::cout << "First and last" << std::endl;
      //   continue;
      // }
      
      if(v1==v2)
        continue;

      // Skip checks if already checked and valid.
      auto& edge = this->m_roadmap->GetEdge(v1,v2);
      const auto& sourceCfg = this->m_roadmap->GetVertex(v1),
                & targetCfg = this->m_roadmap->GetVertex(v2);

      bool isAllChecked = true;
      // auto edge = ei->property();
      auto& descriptors = edge.GetEdgeDescriptors();
      for(size_t i = 0; i < descriptors.size(); ++i) {
        auto roadmap = this->m_roadmap->GetIndividualGraph(i);

        const VID individualSourceVID = sourceCfg.GetVID(i),
                  individualTargetVID = targetCfg.GetVID(i);
        if(this->m_debug)
          std::cout << "\t\t\tIsChecked " << roadmap->GetRobot()->GetLabel() << " "
                    << individualSourceVID <<  " --> " 
                    << individualTargetVID << std::endl;
        if(!roadmap->IsEdge(individualSourceVID,individualTargetVID)) {
          if(this->m_debug)
            std::cout << "\t\t\t\tEdge not exist" << std::endl;
          continue; 
        }

        auto& e1 = roadmap->GetEdge(individualSourceVID,individualTargetVID);
        auto& e2 = roadmap->GetEdge(individualTargetVID,individualSourceVID);
                  
        if(this->m_debug)
          std::cout << "\t\t\t\twith res "
                    << res << ": "
                    << e1.IsChecked(res) 
                    << std::endl;
        if(e1.IsChecked(res)) {
          if(this->m_debug)
            std::cout << "\t\t\t\t" << roadmap->GetRobot()->GetLabel() << ": " 
                      << individualSourceVID << " --> " << individualTargetVID 
                      << " res " << res << " already computed" << std::endl;
          continue;
        }
        e1.SetChecked(res);
        e2.SetChecked(res);
        isAllChecked = false;
      }

      if(isAllChecked) {
        if(this->m_debug)
          std::cout << "\t\t\tAll Checked: " << this->m_roadmap->GetGroup()->GetLabel() << std::endl;
        continue;
      }

      // Validate edge with local planner.
      GroupCfgType witness(this->m_roadmap);
      GroupLPOutput<MPTraits> lpo(this->m_roadmap);
      const bool valid = lp->IsConnected(
          this->m_roadmap->GetVertex(v1), this->m_roadmap->GetVertex(v2),
          witness, &lpo,
          env->GetPositionRes() * res, env->GetOrientationRes() * res, true);

      // If the edge is valid, move on.
      if(valid)
        continue;

      if(this->m_debug) {
        std::cout << "\n\t\tEdge (" << v1 << ", " << v2 << ") is invalid at "
                  << "resolultion factor " << res << "." << std::endl;
        std::cout << "\t\t" << v1 << ": " << this->m_roadmap->GetVertex(v1).PrettyPrint() << std::endl;
        std::cout << "\t\t" << v2 << ": " << this->m_roadmap->GetVertex(v2).PrettyPrint() << std::endl;
        std::cout << "\t\t" << "at " << witness.PrettyPrint() << std::endl;
        auto vc = this->GetValidityChecker(m_vcLabel);
        std::cout << vc->IsValid(this->m_roadmap->GetVertex(v1),this->GetNameAndLabel()) << std::endl;
        std::cout << vc->IsValid(this->m_roadmap->GetVertex(v2),this->GetNameAndLabel()) << std::endl;
      }

      // Invalidate the edge.
      // ProcessInvalidNode(witness);
      InvalidateEdge(v1, v2);
      return true;
    }

    if(this->m_debug)
      std::cout << "ok." << std::endl;
  }

  if(this->m_debug)
    std::cout << "\t\tEdges are ok." << std::endl;

  return false;
}


template <typename MPTraits>
void
GroupLazyQuery<MPTraits>::
NodeEnhance() {
  if(!m_numEnhance || m_edges.empty())
    return;

  if(this->m_debug)
    std::cout << "\tGroupLazyQuery is enhancing nodes...\n\t  Generated VIDs:\n";

  if(this->m_debug) {
    std::cout << "number of enhance: " << m_edges.size() << std::endl;
    for(auto pair : m_edges) {
      std::cout << pair.first.PrettyPrint() << " and " << pair.second.PrettyPrint() << std::endl;
    }
  }

  auto dm = this->GetDistanceMetric(m_enhanceDmLabel);
  auto roadmap = this->GetGroupRoadmap();
  // auto env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);

  // for(size_t i = 0; i < m_numEnhance and !m_edges.empty(); ++i) {
  for(size_t j = 0; j < m_numEnhance; ++j) {
    // Pick a random edge from m_edges.
    const size_t index = LRand() % m_edges.size();

    // Get its midpoint and a random ray.
    GroupCfgType midpoint = (m_edges[index].first + m_edges[index].second) / 2.;

    roadmap->SetAllFormationsInactive();
    for(auto f : midpoint.GetFormations()) {
      roadmap->AddFormation(f);
    }

    Robot* active = nullptr;
    // double ee;
    if(this->m_debug)
      std::cout << "Group: " << midpoint.GetGroup()->GetLabel() << std::endl;
    for(auto robot : midpoint.GetGroup()->GetRobots()) {
      if(this->m_debug)
        std::cout << "Check robot: " << robot->GetLabel() << std::endl;
      if(!robot->GetMultiBody()->IsPassive()) {
        active = robot;
        if(this->m_debug)
          std::cout << "\t This is active robot" << std::endl;
      }
    }


    for(size_t j = 0 ; j < m_maxAttempts ; ++j) {  
      std::unordered_map<Robot*,std::unique_ptr<CSpaceConstraint>> constraintMap;
      auto group = roadmap->GetGroup();
      if(this->m_debug) {
        std::cout << "group: " << group->GetLabel() << std::endl;
        std::cout << "active: " << active->GetLabel() << std::endl;
      }
      auto activeCfg = midpoint.GetRobotCfg(active);
      double eeCfg = activeCfg[1];
      CfgType randomActiveCfg(active);
      for(auto robot : group->GetRobots()) {
        if(robot != active)
          continue;
        // Get Path Constraint;
        for(auto iter = this->GetGroupTask()->begin(); iter!=this->GetGroupTask()->end(); iter++) {
          if(iter->GetRobot() != robot)
            continue;
          for(const auto& c : iter->GetPathConstraints()) {
            if(this->m_debug)
              std::cout << "path constraint exists" << std::endl;
            auto boundary = c->GetBoundary();
            randomActiveCfg.GetRandomCfg(boundary);
          }
          if(iter->GetPathConstraints().size()==0) {
            randomActiveCfg.GetRandomCfg(this->GetEnvironment()->GetBoundary());
          }
        }

        // std::vector<double> data;
        std::vector<double> dataCopy;
        for(size_t i = 0 ; i < randomActiveCfg.GetData().size() ; ++i)
          dataCopy.push_back(randomActiveCfg[i]);
        while(true) {
          for(size_t i = 0 ; i < dataCopy.size() ; ++i) {
            if(i==1) {
              dataCopy[i] = eeCfg;
              continue;
            }
            if(dataCopy[i] > 1) {
              dataCopy[i] = -2 + dataCopy[i];
            }
            else if(dataCopy[i] < -1) {
              dataCopy[i] = 2 + dataCopy[i];
            }
            else {
              dataCopy[i] = dataCopy[i];
            }
          }
          bool inBound = true;
          for(size_t i = 0 ; i < dataCopy.size() ; ++i) {
            if(dataCopy[i] < -1. or dataCopy[i] > 1.) {
              inBound = false;
              break;
            }
          }
          if(inBound)
            break;
        }
        randomActiveCfg.SetData(dataCopy);

        if(this->m_debug) {
          std::cout << "random active cfg: " << randomActiveCfg.PrettyPrint() << std::endl;
          std::cout << "active cfg: " << activeCfg.PrettyPrint() << std::endl;
        }
        break;
      }

      auto diff = randomActiveCfg - activeCfg;
      if(this->m_debug)
        std::cout << "\tdiff: " << diff.PrettyPrint() << std::endl;
      dm->ScaleCfg(std::abs(GaussianDistribution(0, m_d)), diff);
      if(this->m_debug)
        std::cout << "\tscaled: " << diff.PrettyPrint() << std::endl;
      auto enhanceActive = activeCfg + diff;
      // std::vector<double> data;
      std::vector<double> dataCopy;
      for(size_t i = 0 ; i < enhanceActive.GetData().size() ; ++i)
        dataCopy.push_back(enhanceActive[i]);
      while(true) {
        for(size_t i = 0 ; i < dataCopy.size() ; ++i) {
          if(i==1) {
            dataCopy[i] = eeCfg;
            continue;
          }
          if(dataCopy[i] > 1) {
            dataCopy[i] = -2 + dataCopy[i];
          }
          else if(dataCopy[i] < -1) {
            dataCopy[i] = 2 + dataCopy[i];
          }
          else {
            dataCopy[i] = dataCopy[i];
          }
        }
        bool inBound = true;
        for(size_t i = 0 ; i < dataCopy.size() ; ++i) {
          if(dataCopy[i] < -1. or dataCopy[i] > 1.) {
            inBound = false;
            break;
          }
        }
        if(inBound)
          break;
      }
      enhanceActive.SetData(dataCopy);

      constraintMap[active] = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(active,enhanceActive));
      
      GroupCfgType enhance(roadmap);
      enhance.GetRandomGroupCfg(constraintMap[active]->GetBoundary());
      if(this->m_debug) {
        std::cout << "\tEnhance Group: " << enhance.PrettyPrint() << std::endl;
      }
      enhance.SetLabel("Enhance", true);
      // ray.GetRandomGroupCfg(constraintMap[active]->GetBoundary());

      // std::cout << "midpoint: " << midpoint.PrettyPrint() << std::endl;
      // std::cout << "\trandom ray: " << ray.PrettyPrint() << std::endl;

      // auto diff = ray - midpoint;
      // std::cout << "\tdiff: " << diff.PrettyPrint() << std::endl;
      // dm->ScaleCfg(std::abs(GaussianDistribution(0, m_d)), diff);
      // std::cout << "\tscaled: " << diff.PrettyPrint() << std::endl;
      // GroupCfgType enhance = midpoint + diff;
      // std::cout << "\tEnhance: " << enhance.PrettyPrint() << std::endl;

      // enhance.SetLabel("Enhance", true);

      // If enchancement cfg is in bounds, add it to the roadmap and connect.
      // if(enhance.InBounds(this->GetEnvironment())) {
      if(vc->IsValid(enhance,this->GetNameAndLabel())) {
        const VID newVID = roadmap->AddVertex(enhance);
        if(this->m_debug) 
          std::cout << "\tNew vid: " << newVID << ": " << enhance.PrettyPrint() << std::endl;
        for(auto& label : m_ncLabels)
          this->GetConnector(label)->Connect(roadmap, newVID);
        break;
      }
      else {
        if(this->m_debug) 
          std::cout << "\tInvalid new cfg " << std::endl;
      }
    }

    // // Release the enhancement edge.
    // std::cout << "Erase the edge" << std::endl;
    // m_edges.erase(m_edges.begin() + index);
  }
  m_edges.clear();

}


template <typename MPTraits>
void
GroupLazyQuery<MPTraits>::
InvalidateVertex(const VID _vid) {
  // This part for LazyToggleQuery.
  // const GroupCfgType& cfg = this->m_roadmap->GetVertex(_vid);
  // ProcessInvalidNode(cfg);

  // Collect the edge list (deleting as we go will invalidate our iterators).
  auto vi = this->m_roadmap->find_vertex(_vid);
  std::vector<std::pair<VID, VID>> edgeList;
  edgeList.reserve(vi->size());
  for(auto ei = vi->begin(); ei != vi->end(); ++ei)
    edgeList.emplace_back(ei->source(), ei->target());

  // Invalidate the edges.
  for(auto ei = edgeList.begin(); ei != edgeList.end(); ++ei) {
    if(ei->first == ei->second) 
      continue;
    InvalidateEdge(ei->first, ei->second);
  }

  // Delete or invalidate the vertex as appropriate.
  if(m_deleteInvalid)
    this->m_roadmap->DeleteVertex(_vid);
  else
    SetVertexInvalidated(_vid);
}


template <typename MPTraits>
void
GroupLazyQuery<MPTraits>::
InvalidateEdge(const VID _source, const VID _target) {
  // Add the invalid edge to enhancement sampling list.
  if(m_numEnhance) {
    const GroupCfgType& cfg1 = this->m_roadmap->GetVertex(_source),
                 & cfg2 = this->m_roadmap->GetVertex(_target);
    if(!cfg1.IsLabel("Enhance") and !cfg2.IsLabel("Enhance")) {
      m_edges.emplace_back(cfg1, cfg2);
      if(this->m_debug)
        std::cout << "\t\t\tAdding edge that needs enhancement (" << _source << ", "
                  << _target << ")." << std::endl;
    }
  }

  // If we are deleting invalid edges, delete (_source, _target) and return.
  // Also delete the reverse edge if it exists.
  if(m_deleteInvalid) {
    if(this->m_debug) {
      std::cout << "\t\t\tDelete edge (" << _source << ", " << _target << "). " << std::endl;
      std::cout << "Recover edges" << std::endl;
    }
    auto& edge = this->m_roadmap->GetEdge(_source,_target);
    const auto& sourceCfg = this->m_roadmap->GetVertex(_source),
              & targetCfg = this->m_roadmap->GetVertex(_target);

    auto& descriptors = edge.GetEdgeDescriptors();
    for(size_t i = 0; i < descriptors.size(); ++i) {
      auto roadmap = this->m_roadmap->GetIndividualGraph(i);

      const VID individualSourceVID = sourceCfg.GetVID(i),
                individualTargetVID = targetCfg.GetVID(i);
      
      if(!roadmap->IsEdge(individualSourceVID,individualTargetVID)) {
        if(this->m_debug)
          std::cout << "recover: " << individualSourceVID << " --> " << individualTargetVID << std::endl;
        roadmap->AddEdge(individualSourceVID,individualTargetVID);
      }
      if(!roadmap->IsEdge(individualTargetVID,individualSourceVID)) {
        if(this->m_debug)
          std::cout << "recover: " << individualTargetVID << " --> " << individualSourceVID << std::endl;
        roadmap->AddEdge(individualTargetVID,individualSourceVID);
      }
    }

    if(this->m_debug)
      std::cout << "Check edges existance" << std::endl;
    for(size_t i = 0; i < descriptors.size(); ++i) {
      auto roadmap = this->m_roadmap->GetIndividualGraph(i);

      const VID individualSourceVID = sourceCfg.GetVID(i),
                individualTargetVID = targetCfg.GetVID(i);
      if(this->m_debug) {
        std::cout << "\tIsEdge " << individualSourceVID << " --> " << individualTargetVID << ": " << roadmap->IsEdge(individualSourceVID,individualTargetVID) << std::endl;      
        std::cout << "\tIsEdge " << individualTargetVID << " --> " << individualSourceVID << ": " << roadmap->IsEdge(individualTargetVID,individualSourceVID) << std::endl;      
      }
    }

    if(this->m_debug)
      std::cout << "Now we remove the group edge " << "(" << _source << ", " << _target << ")" << std::endl;
    if(this->m_roadmap->IsEdge(_source, _target))
      this->m_roadmap->DeleteEdge(_source, _target);
    if(this->m_roadmap->IsEdge(_target, _source))
      this->m_roadmap->DeleteEdge(_target, _source);
    if(this->m_debug) {
      std::cout << "Does edge " << _source << " " << _target << " exist?: " << this->m_roadmap->IsEdge(_source, _target) << std::endl;
      std::cout << "Does edge " << _target << " " << _source << " exist?: " << this->m_roadmap->IsEdge(_target, _source) << std::endl;
    }
  }
  // Otherwise, mark this edge (and its counterpart if applicable) as lazy
  // invalid.
  else {
    SetEdgeInvalidated(_source, _target);
    if(this->m_roadmap->IsEdge(_target, _source))
      SetEdgeInvalidated(_target, _source);
  }
}


template <typename MPTraits>
std::vector<typename MPTraits::CfgType>
GroupLazyQuery<MPTraits>::
SelectFormationTarget(Formation* _formation) {
  
  auto groupTask = this->GetGroupTask();
  this->GetMPLibrary()->SetGroupTask(nullptr);

  // Heavy assumption that only leader has dofs for now
  auto leader = _formation->GetLeader();
  for(auto& task : *(groupTask)) {
    if(task.GetRobot() == leader) {
      this->GetMPLibrary()->SetTask(&task);
    }
  }

  auto cfg = SelectIndividualTarget();

  this->GetMPLibrary()->SetGroupTask(groupTask);
  auto boundary = std::unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(cfg.DOF()));
  boundary->ShrinkToPoint(cfg);
  
  auto target = _formation->GetRandomFormationCfg(boundary.get());
  return target;
}


template <typename MPTraits>
typename MPTraits::CfgType
GroupLazyQuery<MPTraits>::
SelectIndividualTarget() {

  CfgType target(this->GetTask()->GetRobot());

  // Get the sampler and boundary.
  // std::cout << "robot: " << this->GetTask()->GetRobot()->GetLabel() << std::endl;
  const Boundary* samplingBoundary = this->GetTask()->GetPathConstraints().size() ?
                                    this->GetTask()->GetPathConstraints()[0]->GetBoundary() :
                                    this->GetEnvironment()->GetBoundary();

  // std::cout << "Sampling path constraint size: " << this->GetTask()->GetPathConstraints().size() << std::endl;
  // std::cout << "Start constraint size: " << this->GetTask()->GetStartConstraint() << std::endl;
  // std::cout << "Boundary Center: " << samplingBoundary->GetCenter() << std::endl;
  // const std::string* samplerLabel = &m_samplerLabel;
  // std::vector<CfgType> samples;
  // auto s = this->GetSampler(*samplerLabel);
  // while(samples.empty())
  //   s->Sample(1, 100, samplingBoundary, std::back_inserter(samples));
  // target = samples.front();

  target.GetRandomCfg(samplingBoundary);

  // if(this->m_debug)
  //   std::cout << "\t" << target.PrettyPrint() << std::endl;

  return target;
}





/*---------------------------- Lazy Invalidation -----------------------------*/

template <typename MPTraits>
inline
bool
GroupLazyQuery<MPTraits>::
IsVertexInvalidated(const VID _vid) const noexcept {
  return m_invalidVertices.at(this->m_roadmap).count(_vid);
}


template <typename MPTraits>
inline
void
GroupLazyQuery<MPTraits>::
SetVertexInvalidated(const VID _vid) noexcept {
  m_invalidVertices.at(this->m_roadmap).insert(_vid);
}


template <typename MPTraits>
inline
bool
GroupLazyQuery<MPTraits>::
IsEdgeInvalidated(const EdgeID _eid) const noexcept {
  return m_invalidEdges.at(this->m_roadmap).count(_eid);
}


template <typename MPTraits>
inline
bool
GroupLazyQuery<MPTraits>::
IsEdgeInvalidated(const VID _source, const VID _target) const noexcept {
  typename GroupRoadmapType::CEI ei;
  if(!this->m_roadmap->GetEdge(_source, _target, ei))
    // return true;
    throw RunTimeException(WHERE) << "Requested non-existent edge ("
                                  << _source << ", " << _target << ")...,,";
  return IsEdgeInvalidated(ei->id());
}


template <typename MPTraits>
inline
void
GroupLazyQuery<MPTraits>::
SetEdgeInvalidated(const EdgeID _eid) noexcept {
  m_invalidEdges.at(this->m_roadmap).insert(_eid);
}


template <typename MPTraits>
inline
void
GroupLazyQuery<MPTraits>::
SetEdgeInvalidated(const VID _source, const VID _target) noexcept {
  typename GroupRoadmapType::EI ei;
  if(!this->m_roadmap->GetEdge(_source, _target, ei))
    throw RunTimeException(WHERE) << "Requested non-existent edge ("
                                  << _source << ", " << _target << ")...";
  SetEdgeInvalidated(ei->id());
}

/*----------------------------------------------------------------------------*/

#endif