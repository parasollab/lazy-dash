#ifndef PMPL_TASK_EVALUATOR_METHOD_H_
#define PMPL_TASK_EVALUATOR_METHOD_H_

#include "TMPLibrary/TMPBaseObject.h"

#include <iostream>

class TaskEvaluatorMethod : public TMPBaseObject {
  public:

    ///@name Construction
    ///@{

    TaskEvaluatorMethod();

    TaskEvaluatorMethod(XMLNode& _node);

    virtual ~TaskEvaluatorMethod();

    ///@}
    ///@name Task Evaluator Interface
    ///@{

    /// Evaluate a stateGraph.
    /// @return True if this stateGraph meets the evaluation criteria.
    bool operator()(Plan* _plan = nullptr);

    virtual void Initialize();

    virtual void SetGeometricConstraints(std::unordered_map<size_t,std::set<size_t>> _gc);
    
    virtual void SetGeometricConstraints2(std::unordered_map<size_t,std::set<size_t>> _gc);
    
    virtual void SetNonMonotonicConstraints(std::unordered_map<size_t,std::set<size_t>> _nmcSet);
    
    virtual void SetTaskOrderConstraints(std::set<std::vector<size_t>> _tc);

    virtual void SetReplanSource(size_t _rs);

    virtual void SetInteractionConstraints(std::set<size_t> _tcSet);


    ///@}
  protected:

    ///@name Helper Functions
    ///@{

    ///Exectute
    ///@param _plan pointer
    ///@return True if exectuion is successful
    virtual bool Run(Plan* _plan = nullptr);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_sgLabel; ///< StateGraph Label

    ///@}

};

/*----------------------------------------------------------------------------*/

#endif
