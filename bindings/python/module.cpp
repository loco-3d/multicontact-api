// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include <eigenpy/eigenpy.hpp>
#include <boost/python.hpp>

#include "multicontact-api/bindings/python/geometry/expose-geometry.hpp"
#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/trajectories/expose-trajectories.hpp"

namespace bp = boost::python;

BOOST_PYTHON_MODULE(libmulticontact_api)
{
  eigenpy::enableEigenPy();

  using namespace multicontact_api::python;
  exposeGeometry();
  exposeScenario();
  exposeTrajectories();
}
