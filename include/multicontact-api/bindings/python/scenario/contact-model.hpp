// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_contact_model_planar_hpp__
#define __multicontact_api_python_scenario_contact_model_planar_hpp__

#include <string>

#include "multicontact-api/scenario/contact-model.hpp"
#include "multicontact-api/bindings/python/serialization/archive.hpp"
#include "multicontact-api/bindings/python/utils/printable.hpp"

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename ContactModel>
struct ContactModelPythonVisitor
    : public boost::python::def_visitor<ContactModelPythonVisitor<ContactModel> > {
  typedef typename ContactModel::Scalar Scalar;

  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<>())
        .def(bp::init<Scalar, Scalar>(bp::args("mu", "ZMP_radius")))
        .def(bp::init<ContactModel>(bp::args("other"), "Copy contructor."))
        .def_readwrite("mu", &ContactModel::m_mu, "Friction coefficient.")
        .def_readwrite("ZMP_radius", &ContactModel::m_ZMP_radius, "Radius of the ZMP region.")

        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def("copy", &copy, "Returns a copy of *this.");
  }

  static void expose(const std::string& class_name) {
    std::string doc = "Contact Model Planar";
    bp::class_<ContactModel>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(ContactModelPythonVisitor<ContactModel>())
        .def(SerializableVisitor<ContactModel>())
        .def(PrintableVisitor<ContactModel>());
  }

 private:
  static ContactModel copy(const ContactModel& self) { return ContactModel(self); }
};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_model_planar_hpp__
