// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_serialization_archive_hpp__
#define __multicontact_api_python_serialization_archive_hpp__

#include <boost/python.hpp>

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename Derived>
struct SerializableVisitor : public boost::python::def_visitor<SerializableVisitor<Derived> > {
  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def("saveAsText", &Derived::saveAsText, bp::args("filename"), "Saves *this inside a text file.")
        .def("loadFromText", &Derived::loadFromText, bp::args("filename"), "Loads *this from a text file.")
        .def("saveAsXML", &Derived::saveAsXML, bp::args("filename", "tag_name"), "Saves *this inside a XML file.")
        .def("loadFromXML", &Derived::loadFromXML, bp::args("filename", "tag_name"), "Loads *this from a XML file.")
        .def("saveAsBinary", &Derived::saveAsBinary, bp::args("filename"), "Saves *this inside a binary file.")
        .def("loadFromBinary", &Derived::loadFromBinary, bp::args("filename"), "Loads *this from a binary file.");
  }
};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_serialization_archive_hpp__
