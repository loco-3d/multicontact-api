// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/scenario/enums.hpp"

#include <boost/python/enum.hpp>

namespace multicontact_api
{
  namespace python
  {
    namespace bp = boost::python;

    using namespace multicontact_api::scenario;

    void exposeEnumHumanoidPhaseType()
    {
      bp::enum_<HumanoidPhaseType>("HumanoidPhaseType")
      .value("SINGLE_SUPPORT",SINGLE_SUPPORT)
      .value("DOUBLE_SUPPORT",DOUBLE_SUPPORT)
      .value("TRIPLE_SUPPORT",TRIPLE_SUPPORT)
      .value("QUADRUPLE_SUPPORT",QUADRUPLE_SUPPORT)
      .value("HUMANOID_PHASE_UNDEFINED",HUMANOID_PHASE_UNDEFINED)
      ;
    }

    void exposeEnumConicType()
    {
      bp::enum_<ConicType>("ConicType")
      .value("CONIC_SOWC",CONIC_SOWC)
      .value("CONIC_DOUBLE_DESCRIPTION",CONIC_DOUBLE_DESCRIPTION)
      .value("CONIC_UNDEFINED",CONIC_UNDEFINED)
      ;
    }

    void exposeScenarioEnums()
    {
      exposeEnumHumanoidPhaseType();
      exposeEnumConicType();
    }
  }
}
