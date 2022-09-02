// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/scenario/contact-model.hpp"

#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"

namespace multicontact_api {
namespace python {
void exposeContactModels() {
  ContactModelPythonVisitor<multicontact_api::scenario::ContactModel>::expose(
      "ContactModel");
}
}  // namespace python
}  // namespace multicontact_api
