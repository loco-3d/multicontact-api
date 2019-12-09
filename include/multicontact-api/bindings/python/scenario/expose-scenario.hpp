// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#ifndef __multicontact_api_python_scenario_expose_scenario_hpp__
#define __multicontact_api_python_scenario_expose_scenario_hpp__

namespace multicontact_api {
namespace python {

void exposeContactPatch();
void exposeContactPhase();
void exposeContactSequence();
void exposeScenarioEnums();
void exposeContactModels();

inline void exposeScenario() {
  exposeContactPatch();
  exposeContactPhase();
  //exposeContactSequence();
  exposeScenarioEnums();
  exposeContactModels();
}

}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_expose_scenario_hpp__
