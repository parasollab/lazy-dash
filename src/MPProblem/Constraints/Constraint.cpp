#include "Constraint.h"
#include "BoundaryConstraint.h"
#include "CSpaceConstraint.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*--------------------------------- Construction -----------------------------*/

Constraint::
Constraint(Robot* const _r) : m_robot(_r) { }


Constraint::
~Constraint() = default;


std::unique_ptr<Constraint>
Constraint::
Factory(Robot* const _r, XMLNode& _node) {
  std::cout << "** Constraints Factory **: " << std::endl;
  std::unique_ptr<Constraint> output;

  if(_node.Name() == "CSpaceConstraint") {
    std::cout << "setting CSpaceConstraint" << std::endl;
    output = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(_r, _node));
  }
  else if(_node.Name() == "BoundaryConstraint") {
    std::cout << "setting BoundaryConsraints" << std::endl;
    output = std::unique_ptr<BoundaryConstraint>(
        new BoundaryConstraint(_r, _node));
  }
  else
    throw RunTimeException(_node.Where()) << "Unrecognized constraint type '"
                                          << _node.Name() << "'.";
  std::cout << "Returning the constraint" << std::endl;
  return output;
}

/*---------------------------- Constraint Interface --------------------------*/

void
Constraint::
SetRobot(Robot* const _r) {
  m_robot = _r;
}

Robot*
Constraint::
GetRobot() const {
  return m_robot;
}

// std::string
// Constraint::
// GetLabel() {
//   std::cout << "returning constraint label: " << m_constraint_label << std::endl;
//   return m_constraint_label;
// }

/*----------------------------------------------------------------------------*/
