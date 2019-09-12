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
  typedef typename ContactPatch::LinearWrenchCone LWC;
  typedef typename ContactPatch::ContactModel ContactModel;

  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<SE3>(bp::arg("placement"), "Init with a given placement."))
        .def(bp::init<ContactPatch>(bp::args("other"), "Copy contructor."))
        .add_property("placement",
                      bp::make_function((SE3 & (ContactPatch::*)(void)) & ContactPatch::placement,
                                        bp::return_internal_reference<>()),
                      &setPlacement, "Placement of the patch.")
        .add_property("oMcp",
                      bp::make_function((SE3 & (ContactPatch::*)(void)) & ContactPatch::placement,
                                        bp::return_internal_reference<>()),
                      &setPlacement, "Placement of the contact model with respect to the world.")
        .add_property("active", bp::make_function((bool (ContactPatch::*)(void) const) & ContactPatch::active),
                      &setActive, "Active property.")
        .add_property(
            "lwc",
            bp::make_function((LWC & (ContactPatch::*)(void)) & ContactPatch::lwc, bp::return_internal_reference<>()),
            &setLwc, "Linear Wrench Cone associated to the patch.")
        .add_property("contactModel",
                      bp::make_function((ContactModel & (ContactPatch::*)(void)) & ContactPatch::contactModel,
                                        bp::return_internal_reference<>()),
                      &setContactModel, "Contact Model associated to the patch.")
        .add_property("contactModelPlacement",
                      bp::make_function((SE3 & (ContactPatch::*)(void)) & ContactPatch::contactModelPlacement,
                                        bp::return_internal_reference<>()),
                      &setContactModelPlacement, "Placement of the patch.")

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
  static void setPlacement(ContactPatch& self, const SE3& placement) { self.placement() = placement; }
  static void setContactModelPlacement(ContactPatch& self, const SE3& placement) {
    self.contactModelPlacement() = placement;
  }
  static void setActive(ContactPatch& self, const bool value) { self.active() = value; }
  static void setContactModel(ContactPatch& self, const ContactModel& contactModel) {
    self.contactModel() = contactModel;
  }
  static void setLwc(ContactPatch& self, const LWC& lwc) { self.lwc() = lwc; }
  static ContactPatch copy(const ContactPatch& self) { return ContactPatch(self); }
};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_patch_hpp__
