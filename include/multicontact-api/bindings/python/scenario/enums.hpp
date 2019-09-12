// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenerario_enum_hpp__
#define __multicontact_api_python_scenerario_enum_hpp__

#include <boost/python.hpp>
#include <string>

namespace multicontact_api {
namespace python {

void exposeEnumHumanoidPhaseType(const std::string& enum_name);
}
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenerario_enum_hpp__
