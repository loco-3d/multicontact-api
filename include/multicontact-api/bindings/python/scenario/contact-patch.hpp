// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_contact_patch_hpp__
#define __multicontact_api_python_scenario_contact_patch_hpp__

#include <string>

#include "multicontact-api/scenario/contact-patch.hpp"
#include "multicontact-api/bindings/python/serialization/archive.hpp"
#include "multicontact-api/bindings/python/utils/printable.hpp"

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename ContactPatch>
struct ContactPatchPythonVisitor : public boost::python::def_visitor<ContactPatchPythonVisitor<ContactPatch> > {
  typedef typename ContactPatch::Scalar Scalar;
  typedef typename ContactPatch::SE3 SE3;

  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<>(bp::arg(""), "Default constructor."))
        .def(bp::init<SE3>(bp::arg("placement"), "Init with a given placement."))
        .def(bp::init<SE3, Scalar>(bp::args("placement", "friction"),
                                   "Init with a given placement and friction coefficient."))
        .def(bp::init<ContactPatch>(bp::arg("other"), "Copy contructor."))
        .add_property("placement",
                      // getter require to use "make_function" to pass the return_internal_reference policy (return ref
                      // to custom object)
                      bp::make_function(&getPlacement, bp::return_internal_reference<>()), &setPlacement,
                      "Placement of the patch represented as a pinocchio SE3 object.")
        .add_property("friction", &getFriction, &setFriction,
                      "Friction coefficient between the robot and the environment for this contact.")
        .def_readwrite("contact_model", &ContactPatch::m_contact_model, "The contact model defining this contact.")
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def("copy", &copy, "Returns a copy of *this.");
  }

  static void expose(const std::string& class_name) {
    std::string doc = "Contact Patch";
    bp::class_<ContactPatch>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(ContactPatchPythonVisitor<ContactPatch>())
        .def(SerializableVisitor<ContactPatch>())
        .def(PrintableVisitor<ContactPatch>());
  }

 protected:
  // define setter and getter
  static SE3& getPlacement(ContactPatch& self) { return self.placement(); }
  static void setPlacement(ContactPatch& self, const SE3& placement) { self.placement() = placement; }
  static Scalar getFriction(ContactPatch& self) { return self.friction(); }
  static void setFriction(ContactPatch& self, const Scalar& friction) { self.friction() = friction; }
  static ContactPatch copy(const ContactPatch& self) { return ContactPatch(self); }
};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_patch_hpp__
