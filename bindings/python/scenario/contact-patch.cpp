// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#include "multicontact-api/bindings/python/scenario/contact-patch.hpp"

#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"

namespace multicontact_api {
namespace python {
void exposeContactPatch() {
  ContactPatchPythonVisitor<multicontact_api::scenario::ContactPatch>::expose(
      "ContactPatch");
}
}  // namespace python
}  // namespace multicontact_api
