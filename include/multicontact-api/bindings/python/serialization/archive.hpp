// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_serialization_archive_hpp__
#define __multicontact_api_python_serialization_archive_hpp__

#include <boost/python.hpp>

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename Derived>
struct cs_pickle_suite : bp::pickle_suite {
  static bp::object getstate(const Derived& cs) {
    std::ostringstream os;
    boost::archive::text_oarchive oa(os);
    oa << cs;
    return bp::str(os.str());
  }

  static void setstate(Derived& cs, bp::object entries) {
    bp::str s = bp::extract<bp::str>(entries)();
    std::string st = bp::extract<std::string>(s)();
    std::istringstream is(st);
    boost::archive::text_iarchive ia(is);
    ia >> cs;
  }
};

template <typename Derived>
struct SerializableVisitor : public boost::python::def_visitor<SerializableVisitor<Derived> > {
  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def("saveAsText", &Derived::saveAsText, bp::args("filename"), "Saves *this inside a text file.")
        .def("loadFromText", &Derived::loadFromText, bp::args("filename"), "Loads *this from a text file.")
        .def("saveAsXML", &Derived::saveAsXML, bp::args("filename", "tag_name"), "Saves *this inside a XML file.")
        .def("loadFromXML", &Derived::loadFromXML, bp::args("filename", "tag_name"), "Loads *this from a XML file.")
        .def("saveAsBinary", &Derived::saveAsBinary, bp::args("filename"), "Saves *this inside a binary file.")
        .def("loadFromBinary", &Derived::loadFromBinary, bp::args("filename"), "Loads *this from a binary file.")
        .def_pickle(cs_pickle_suite<Derived>());
  }
};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_serialization_archive_hpp__
