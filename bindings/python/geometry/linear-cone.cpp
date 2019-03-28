// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/geometry/linear-cone.hpp"
#include "multicontact-api/bindings/python/geometry/expose-geometry.hpp"

namespace multicontact_api
{
  namespace python
  {
    void exposeLinearCone()
    {
      ForceConePythonVisitor<multicontact_api::geometry::ForceCone>::expose("ForceCone");
      WrenchConePythonVisitor<multicontact_api::geometry::WrenchCone>::expose("WrenchCone");
    }
  }
}
