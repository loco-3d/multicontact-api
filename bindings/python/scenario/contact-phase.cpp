// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/scenario/contact-phase.hpp"

namespace multicontact_api {
namespace python {
void exposeContactPhase() {
  ContactPhasePythonVisitor<multicontact_api::scenario::ContactPhase4>::expose("ContactPhase4");
  //ContactPhaseHumanoidPythonVisitor<multicontact_api::scenario::ContactPhaseHumanoid>::expose("ContactPhaseHumanoid");
}
}

CurveMap ContactPhaseTpl::contact_forces() const
{
    return m_contact_forces;
}

void ContactPhaseTpl::setContact_forces(const CurveMap &contact_forces)
{
    m_contact_forces = contact_forces;
}

// namespace python
}  // namespace multicontact_api
