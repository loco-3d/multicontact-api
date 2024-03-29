// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_expose_geometry_hpp__
#define __multicontact_api_python_expose_geometry_hpp__

namespace multicontact_api {
namespace python {

void exposeEllipsoid();
void exposeLinearCone();
void exposeSecondOrderCone();

inline void exposeGeometry() {
  exposeEllipsoid();
  exposeLinearCone();
  exposeSecondOrderCone();
}

}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_expose_geometry_hpp__
