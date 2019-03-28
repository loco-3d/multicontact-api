// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_utils_base_hpp__
#define __multicontact_api_python_utils_base_hpp__

#include <boost/python.hpp>
#include <string>

namespace multicontact_api
{
  namespace python
  {

    namespace bp = boost::python;

    template<typename Derived>
    struct BasePythonVisitor : public bp::def_visitor< BasePythonVisitor<Derived> >
    {
      static void expose(const std::string & derived_class_name)
      {
        const std::string base_class_name = derived_class_name + "Base";
        const std::string base_doc = "Base class of " + derived_class_name;
        bp::class_<typename Derived::Base>(base_class_name.c_str(),
                                           base_doc.c_str(),
                                           bp::no_init);
      }
    };


  }
}

#endif // ifndef __multicontact_api_python_utils_base_hpp__
