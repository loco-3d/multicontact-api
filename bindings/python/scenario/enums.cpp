// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include <boost/python.hpp>
#include <boost/python/enum.hpp>

namespace multicontact_api {
namespace python {
namespace bp = boost::python;

using namespace multicontact_api::scenario;

void exposeEnumConicType() {
  bp::enum_<ConicType>("ConicType")
      .value("CONIC_SOWC", CONIC_SOWC)
      .value("CONIC_DOUBLE_DESCRIPTION", CONIC_DOUBLE_DESCRIPTION)
      .value("CONIC_UNDEFINED", CONIC_UNDEFINED);
}

void exposeScenarioEnums() {
  exposeEnumConicType();
}
}  // namespace python
}  // namespace multicontact_api
