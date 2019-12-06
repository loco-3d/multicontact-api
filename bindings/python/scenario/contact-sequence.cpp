// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/scenario/contact-sequence.hpp"

namespace multicontact_api {
namespace python {
void exposeContactSequence() {
  ContactSequencePythonVisitor<multicontact_api::scenario::ContactSequence>::expose("ContactSequence");
}
}  // namespace python
}  // namespace multicontact_api
