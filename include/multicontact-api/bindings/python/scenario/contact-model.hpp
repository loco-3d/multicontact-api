// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_contact_model_planar_hpp__
#define __multicontact_api_python_scenario_contact_model_planar_hpp__

#include <string>
#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/scenario/contact-model.hpp"
#include "multicontact-api/bindings/python/serialization/archive.hpp"
#include "multicontact-api/bindings/python/utils/printable.hpp"
#include <eigenpy/eigenpy.hpp>

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename ContactModel>
struct ContactModelPythonVisitor : public boost::python::def_visitor<ContactModelPythonVisitor<ContactModel> > {
  typedef typename ContactModel::Scalar Scalar;
  typedef scenario::ContactType ContactType;
  typedef typename ContactModel::Matrix3X Matrix3X;

  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<>())
        .def(bp::init<Scalar>(bp::args("mu")))
        .def(bp::init<Scalar, ContactType>(bp::args("mu", "contact_type")))
        .def(bp::init<ContactModel>(bp::args("other"), "Copy contructor."))
        .def_readwrite("mu", &ContactModel::m_mu, "Friction coefficient.")
        .def_readwrite("contact_type", &ContactModel::m_contact_type, "Enum that define the type of contact.")
        .add_property("num_contact_points", &getNumContact, &setNumContact,
                      "The number of contact points used to model this contact. \n"
                      "Changing this value will clear the contact_points_positions matrix")
        .add_property("contact_points_positions", &getContactPositions, &setContactPositions,
                      "3xnum_contact_points matrix defining the contact points positions in the frame of the contact "
                      "placement. \n"
                      "num_contact_points is automatically updated to the number of columns of this matrix.")
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def("copy", &copy, "Returns a copy of *this.");
  }

  static void expose(const std::string& class_name) {
    std::string doc = "Contact Model";
    bp::class_<ContactModel>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(ContactModelPythonVisitor<ContactModel>())
        .def(SerializableVisitor<ContactModel>())
        .def(PrintableVisitor<ContactModel>());

    ENABLE_SPECIFIC_MATRIX_TYPE(Matrix3X);
  }

 private:
  static ContactModel copy(const ContactModel& self) { return ContactModel(self); }
  // define setter and getter
  static size_t getNumContact(ContactModel& self) { return self.num_contact_points(); }
  static void setNumContact(ContactModel& self, const size_t num) { self.num_contact_points(num); }
  static Matrix3X getContactPositions(ContactModel& self) { return self.contact_points_positions(); }
  static void setContactPositions(ContactModel& self, const Matrix3X& pos) { self.contact_points_positions(pos); }
};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_model_planar_hpp__
