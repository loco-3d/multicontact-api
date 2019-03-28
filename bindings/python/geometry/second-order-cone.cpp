// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/geometry/second-order-cone.hpp"
#include "multicontact-api/bindings/python/geometry/expose-geometry.hpp"

namespace multicontact_api
{
  namespace python
  {
    void exposeSecondOrderCone()
    {
      SOCPythonVisitor<multicontact_api::geometry::SOC6d>::expose("SOC6");
      SOCPythonVisitor<multicontact_api::geometry::SOC3d>::expose("SOC3");
    }
  }
}
