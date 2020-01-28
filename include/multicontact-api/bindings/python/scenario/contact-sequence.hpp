// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_contact_sequence_hpp__
#define __multicontact_api_python_scenario_contact_sequence_hpp__

#include <pinocchio/fwd.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/bindings/python/utils/std-aligned-vector.hpp>

#include "multicontact-api/scenario/contact-sequence.hpp"
#include "multicontact-api/bindings/python/serialization/archive.hpp"

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename CS>
struct ContactSequencePythonVisitor : public bp::def_visitor<ContactSequencePythonVisitor<CS> > {
  typedef typename CS::ContactPhaseVector ContactPhaseVector;

  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(cs_breakContact_overloads, CS::breakContact, 1, 2)
  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(cs_createContact_overloads, CS::createContact, 2, 3)
  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(cs_moveEffectorToPlacement_overloads, CS::moveEffectorToPlacement, 2, 4)
  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(cs_moveEffectorOf_overloads, CS::moveEffectorOf, 2, 4)

  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<size_t>(bp::arg("size"), "Default constructor from a given size."))
        .def(bp::init<>(bp::arg(""), "Default constructor."))
        .def(bp::init<CS>(bp::args("other"), "Copy contructor."))
        .def("size", &CS::size, "Return the size of the contact sequence.")
        .def("resize", &CS::resize, bp::arg("size"), "Resize the vector of ContactPhases.")
        .def("append", &CS::append, bp::arg("ContactPhase"),
             "Add the given ContactPhase at the end of the sequence. \n"
             "Return the new id of this ContactPhase inside the sequence.")
        .add_property("contactPhases", bp::make_getter(&CS::m_contact_phases, bp::return_internal_reference<>()),
                      "Vector of Contact Phases contained in the sequence")
        .def("breakContact", &CS::breakContact,
             cs_breakContact_overloads(
                 (bp::arg("eeName"), bp::arg("phaseDuration") = -1),
                 " Add a new contactPhase at the end of the current ContactSequence,"
                 "The new ContactPhase have the same ContactPatchs as the last phase of the sequence,"
                 "with the exeption of the given contact removed."
                 "It copy all the 'final' values of the last phase as 'initial' values of the new phase."
                 "It also set the duration of the previous last phase. \n"
                 "phaseDuration : if provided, the duration of the previous last phase of the sequence is set to this "
                 "value"
                 "(it is thus the duration BEFORE breaking the contact) \n"
                 "Raise value_error if the phaseDuration is provided but the last phase do not have a time-range "
                 "defined.\n"
                 "Raise value_error if eeName is not in contact in the last phase of the sequence.\n"))
        .def("createContact", &CS::createContact,
             cs_createContact_overloads(
                 (bp::arg("eeName"), bp::arg("contactPatch"), bp::arg("phaseDuration") = -1),
                 "Add a new contactPhase at the end of the current ContactSequence,"
                 "The new ContactPhase have the same ContactPatchs as the last phase of the sequence,"
                 "with the exeption of the given contact added.\n"
                 "phaseDuration: if provided, the duration of the previous last phase of the sequence is set to this "
                 "value"
                 "(it is thus the duration BEFORE creating the contact)\n"
                 "Raise value_error if the phaseDuration is provided but the last phase do not have a time-range "
                 "defined\n"
                 "Raise value_error if eeName is already in contact in the last phase of the sequence"))
        .def("moveEffectorToPlacement", &CS::moveEffectorToPlacement,
             cs_moveEffectorToPlacement_overloads(
                 (bp::arg("eeName"), bp::arg("placement"), bp::arg("durationBreak") = -1,
                  bp::arg("durationCreate") = -1),
                 "Add two new phases at the end of the current ContactSequence:\n"
                 "- it break the contact with eeName\n"
                 "- it create the contact with eeName at the given placement.\n"
                 "It copy all the 'final' values of the last phase as 'initial' values of the new phase."
                 "It also set the duration of the previous last phase.\n"
                 "placement: the new placement for the contact of eeName, defined as a pinocchio::SE3\n"
                 "durationBreak: the duration of the previous last phase of the sequence"
                 "(it is thus the duration BEFORE breaking the contact)\n"
                 "durationCreate: the duration of the first new ContactPhase"
                 "(it is thus the duration BEFORE creating the contact)\n"
                 "Raise value_error if the phaseDuration is provided but the last phase do not have a time-range "
                 "defined\n"
                 "Raise value_error if eeName is not in contact in the last phase of the sequence\n"))
        .def(
            "moveEffectorOf", &CS::moveEffectorOf,
            cs_moveEffectorOf_overloads(
                (bp::arg("eeName"), bp::arg("transform"), bp::arg("durationBreak") = -1,
                 bp::arg("durationCreate") = -1),
                "Similar to moveEffectorToPlacement"
                "exept that the new placement is defined from the previous placement and a given transform applied.\n"
                "transform: the transform applied to the placement of the contact in the last phase of the sequence.\n"
                "durationBreak: the duration of the previous last phase of the sequence"
                "(it is thus the duration BEFORE breaking the contact)\n"
                "durationCreate: the duration of the first new ContactPhase"
                "(it is thus the duration BEFORE creating the contact)\n"
                "Raise value_error if the phaseDuration is provided but the last phase do not have a time-range "
                "defined\n"
                "Raise value_error if eeName is not in contact in the last phase of the sequence\n"))
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def("copy", &copy, "Returns a copy of *this.");
    ;
  }

  static void expose(const std::string& class_name) {
    std::string doc = "Contact Sequence of dynamic size";
    bp::class_<CS>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(ContactSequencePythonVisitor<CS>())
        .def(SerializableVisitor<CS>());

    bp::class_<ContactPhaseVector>("ContactPhaseVector").def(bp::vector_indexing_suite<ContactPhaseVector>());
  }

 protected:
  static CS copy(const CS& self) { return CS(self); }
};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_sequence_hpp__
