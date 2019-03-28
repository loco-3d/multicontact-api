// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/scenario/contact-model-planar.hpp"

namespace multicontact_api
{
  namespace python
  {
    void exposeContactModels()
    {
      ContactModelPlanarPythonVisitor<multicontact_api::scenario::ContactModelPlanar>::expose("ContactModelPlanar");
    }
  }
}
