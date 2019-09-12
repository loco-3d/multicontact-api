// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/geometry/ellipsoid.hpp"
#include "multicontact-api/bindings/python/geometry/expose-geometry.hpp"

namespace multicontact_api {
namespace python {
void exposeEllipsoid() { EllipsoidPythonVisitor<multicontact_api::geometry::Ellipsoid3d>::expose("Ellipsoid3d"); }
}  // namespace python
}  // namespace multicontact_api
