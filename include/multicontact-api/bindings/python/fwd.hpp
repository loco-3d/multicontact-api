// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_geometry_ellipsoid_hpp__
#define __multicontact_api_python_geometry_ellipsoid_hpp__

namespace multicontact_api {
namespace python {
namespace internal {

template <typename T>
struct build_type_name {
  static const char* name();
  static const char* shortname();
};

template <>
const char* build_type_name<double>::shortname() {
  return "d";
}

}  // namespace internal
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_geometry_ellipsoid_hpp__
