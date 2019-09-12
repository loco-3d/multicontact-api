// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/scenario/contact-sequence.hpp"
#include "multicontact-api/bindings/python/scenario/contact-phase-humanoid.hpp"
#include "multicontact-api/bindings/python/scenario/ms-interval.hpp"

namespace multicontact_api {
namespace python {
void exposeContactSequence() {
  //      ContactSequencePythonVisitor<multicontact_api::scenario::ContactSequence4>::expose("ContactSequence4");
  ContactSequencePythonVisitor<multicontact_api::scenario::ContactSequenceHumanoid>::expose("ContactSequenceHumanoid");
  MSIntervalPythonVisitor<multicontact_api::scenario::ContactSequenceHumanoid::MSIntervalData>::expose(
      "MSIntervalData");
}
}  // namespace python
}  // namespace multicontact_api
