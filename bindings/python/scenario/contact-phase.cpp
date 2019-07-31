// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/scenario/contact-phase.hpp"

<<<<<<< f9c313c597929adfebcc22f273c67361c3dfef6c
namespace multicontact_api {
namespace python {
void exposeContactPhase() {
  ContactPhasePythonVisitor<multicontact_api::scenario::ContactPhase4>::expose("ContactPhase4");
  //ContactPhaseHumanoidPythonVisitor<multicontact_api::scenario::ContactPhaseHumanoid>::expose("ContactPhaseHumanoid");
}
}  // namespace python
}  // namespace multicontact_api
