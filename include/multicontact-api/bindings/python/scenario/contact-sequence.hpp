// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_contact_sequence_hpp__
#define __multicontact_api_python_scenario_contact_sequence_hpp__

#include <eigenpy/eigenpy.hpp>
#include <pinocchio/bindings/python/utils/std-aligned-vector.hpp>
#include <pinocchio/fwd.hpp>

#include "multicontact-api/bindings/python/serialization/archive.hpp"
#include "multicontact-api/scenario/contact-sequence.hpp"

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename CS>
struct ContactSequencePythonVisitor
    : public bp::def_visitor<ContactSequencePythonVisitor<CS> > {
  typedef typename CS::ContactPhaseVector ContactPhaseVector;

  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(cs_breakContact_overloads,
                                         CS::breakContact, 1, 2)
  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(cs_createContact_overloads,
                                         CS::createContact, 2, 3)
  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(cs_moveEffectorToPlacement_overloads,
                                         CS::moveEffectorToPlacement, 2, 4)
  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(cs_moveEffectorOf_overloads,
                                         CS::moveEffectorOf, 2, 4)
  BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(cs_haveEffectorTrajectories_overloads,
                                         CS::haveEffectorsTrajectories, 0, 2)

  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<size_t>(bp::arg("size"),
                            "Default constructor from a given size."))
        .def(bp::init<>(bp::arg(""), "Default constructor."))
        .def(bp::init<CS>(bp::args("other"), "Copy contructor."))
        .def("size", &CS::size, "Return the size of the contact sequence.")
        .def("resize", &CS::resize, bp::arg("size"),
             "Resize the vector of ContactPhases.")
        .def("append", &CS::append, bp::arg("ContactPhase"),
             "Add the given ContactPhase at the end of the sequence. \n"
             "Return the new id of this ContactPhase inside the sequence.")
        .add_property("contactPhases",
                      bp::make_getter(&CS::m_contact_phases,
                                      bp::return_internal_reference<>()),
                      "Vector of Contact Phases contained in the sequence")
        .def("breakContact", &CS::breakContact,
             cs_breakContact_overloads(
                 (bp::arg("eeName"), bp::arg("phaseDuration") = -1),
                 " Add a new contactPhase at the end of the current "
                 "ContactSequence,"
                 "The new ContactPhase have the same ContactPatchs as the last "
                 "phase of the sequence,"
                 "with the exeption of the given contact removed."
                 "It copy all the 'final' values of the last phase as "
                 "'initial' values of the new phase."
                 "It also set the duration of the previous last phase. \n"
                 "phaseDuration : if provided, the duration of the previous "
                 "last phase of the sequence is set to this "
                 "value"
                 "(it is thus the duration BEFORE breaking the contact) \n"
                 "Raise value_error if the phaseDuration is provided but the "
                 "last phase do not have a time-range "
                 "defined.\n"
                 "Raise value_error if eeName is not in contact in the last "
                 "phase of the sequence.\n"))
        .def("createContact", &CS::createContact,
             cs_createContact_overloads(
                 (bp::arg("eeName"), bp::arg("contactPatch"),
                  bp::arg("phaseDuration") = -1),
                 "Add a new contactPhase at the end of the current "
                 "ContactSequence,"
                 "The new ContactPhase have the same ContactPatchs as the last "
                 "phase of the sequence,"
                 "with the exeption of the given contact added.\n"
                 "phaseDuration: if provided, the duration of the previous "
                 "last phase of the sequence is set to this "
                 "value"
                 "(it is thus the duration BEFORE creating the contact)\n"
                 "Raise value_error if the phaseDuration is provided but the "
                 "last phase do not have a time-range "
                 "defined\n"
                 "Raise value_error if eeName is already in contact in the "
                 "last phase of the sequence"))
        .def(
            "moveEffectorToPlacement", &CS::moveEffectorToPlacement,
            cs_moveEffectorToPlacement_overloads(
                (bp::arg("eeName"), bp::arg("placement"),
                 bp::arg("durationBreak") = -1, bp::arg("durationCreate") = -1),
                "Add two new phases at the end of the current "
                "ContactSequence:\n"
                "- it break the contact with eeName\n"
                "- it create the contact with eeName at the given placement.\n"
                "It copy all the 'final' values of the last phase as 'initial' "
                "values of the new phase."
                "It also set the duration of the previous last phase.\n"
                "placement: the new placement for the contact of eeName, "
                "defined as a pinocchio::SE3\n"
                "durationBreak: the duration of the previous last phase of the "
                "sequence"
                "(it is thus the duration BEFORE breaking the contact)\n"
                "durationCreate: the duration of the first new ContactPhase"
                "(it is thus the duration BEFORE creating the contact)\n"
                "Raise value_error if the phaseDuration is provided but the "
                "last phase do not have a time-range "
                "defined\n"
                "Raise value_error if eeName is not in contact in the last "
                "phase of the sequence\n"))
        .def(
            "moveEffectorOf", &CS::moveEffectorOf,
            cs_moveEffectorOf_overloads(
                (bp::arg("eeName"), bp::arg("transform"),
                 bp::arg("durationBreak") = -1, bp::arg("durationCreate") = -1),
                "Similar to moveEffectorToPlacement"
                "exept that the new placement is defined from the previous "
                "placement and a given transform applied.\n"
                "transform: the transform applied to the placement of the "
                "contact in the last phase of the sequence.\n"
                "durationBreak: the duration of the previous last phase of the "
                "sequence"
                "(it is thus the duration BEFORE breaking the contact)\n"
                "durationCreate: the duration of the first new ContactPhase"
                "(it is thus the duration BEFORE creating the contact)\n"
                "Raise value_error if the phaseDuration is provided but the "
                "last phase do not have a time-range "
                "defined\n"
                "Raise value_error if eeName is not in contact in the last "
                "phase of the sequence\n"))
        .def("haveTimings", &CS::haveTimings,
             "Check if all the time intervals are defined and consistent"
             "(ie. the time always increase and the final time of one phase is "
             "equal to the initial one of the newt "
             "phase) \n"
             "Return true if the sequence is consistent, false otherwise")
        .def("haveConsistentContacts", &CS::haveConsistentContacts,
             "check that there is always one contact change between adjacent "
             "phases in the sequence.\n"
             "and that there isn't any phase without any contact.")
        .def("haveCOMvalues", &CS::haveCOMvalues,
             "Check that the initial and final CoM position values are defined "
             "for all phases.\n"
             "Also check that the initial values of one phase correspond to "
             "the final values of the previous ones.")
        .def("haveAMvalues", &CS::haveAMvalues,
             "Check that the initial and final AM values are defined for all "
             "phases.\n"
             "Also check that the initial values of one phase correspond to "
             "the final values of the previous ones.")
        .def("haveCentroidalValues", &CS::haveCentroidalValues,
             "Check that the initial and final CoM position and AM values are "
             "defined for all phases.\n"
             "Also check that the initial values of one phase correspond to "
             "the final values of the previous ones.")
        .def("haveConfigurationsValues", &CS::haveConfigurationsValues,
             "Check that the initial and final configuration are defined for "
             "all phases.\n"
             "Also check that the initial values of one phase correspond to "
             "the final values of the previous ones.")
        .def("haveCOMtrajectories", &CS::haveCOMtrajectories,
             "check that a c, dc and ddc trajectories are defined for each "
             "phases.\n"
             "Also check that the time interval of this trajectories matches "
             "the one of the phase.\n"
             "and that the trajectories start and end and the correct values "
             "defined in each phase.")
        .def("haveAMtrajectories", &CS::haveAMtrajectories,
             "check that a L and dL trajectories are defined for each phases.\n"
             "Also check that the time interval of this trajectories matches "
             "the one of the phase.\n"
             "and that the trajectories start and end and the correct values "
             "defined in each phase.")
        .def("haveCentroidalTrajectories", &CS::haveCentroidalTrajectories,
             "check that all centroidal trajectories are defined for each "
             "phases.\n"
             "Also check that the time interval of this trajectories matches "
             "the one of the phase.\n"
             "and that the trajectories start and end and the correct values "
             "defined in each phase.")
        .def("haveEffectorsTrajectories", &CS::haveEffectorsTrajectories,
             cs_haveEffectorTrajectories_overloads(
                 (bp::args("precision_threshold") =
                      Eigen::NumTraits<typename CS::Scalar>::dummy_precision(),
                  bp::args("use_rotation") = true),
                 "check that for each phase preceeding a contact creation,"
                 "an SE3 trajectory is defined for the effector that will be "
                 "in contact.\n"
                 "Also check that this trajectory is defined on the "
                 "time-interval of the phase.\n"
                 "Also check that the trajectory correctly end at the "
                 "placement defined for the contact in the next "
                 "phase.\n"
                 "If this effector was in contact in the previous phase,"
                 "it check that the trajectory start at the previous contact "
                 "placement.\n"
                 "If use_rotation == false, only the translation part of the "
                 "transforms are compared. "))
        .def("haveJointsTrajectories", &CS::haveJointsTrajectories,
             "Check that a q trajectory is defined for each phases.\n"
             "Also check that the time interval of this trajectories matches "
             "the one of the phase.\n"
             "and that the trajectories start and end and the correct values "
             "defined in each phase.")
        .def("haveJointsDerivativesTrajectories",
             &CS::haveJointsDerivativesTrajectories,
             "Check that a  dq and ddq trajectories are defined for each "
             "phases.\n"
             "Also check that the time interval of this trajectories matches "
             "the one of the phase.\n"
             "and that the trajectories start and end and the correct values "
             "defined in each phase.")
        .def("haveTorquesTrajectories", &CS::haveTorquesTrajectories,
             "Check that a joint torque trajectories are defined for each "
             "phases.\n"
             "Also check that the time interval of this trajectories matches "
             "the one of the phase.\n"
             "and that the trajectories start and end and the correct values "
             "defined in each phase")
        .def("haveContactForcesTrajectories",
             &CS::haveContactForcesTrajectories,
             "Check that a contact force trajectory exist for each active "
             "contact.\n"
             "Also check that the time interval of this trajectories matches "
             "the one of the phase.\n"
             "and that the trajectories start and end and the correct values "
             "defined in each phase.")
        .def("haveRootTrajectories", &CS::haveRootTrajectories,
             "check that a root trajectory exist for each contact phases.\n"
             "Also check that it start and end at the correct time interval.")
        .def("haveFriction", &CS::haveFriction,
             "check that all the contact patch used in the sequence have"
             "a friction coefficient initialized.")
        .def("haveContactModelDefined", &CS::haveContactModelDefined,
             "haveContactModelDefined check that all the contact patch have a "
             "contact_model defined")
        .def("haveZMPtrajectories", &CS::haveZMPtrajectories,
             "check that all the contact phases have a ZMP trajectory.")
        .def("getAllEffectorsInContact", &getAllEffectorsInContactAsList,
             "return a list of names of all the effectors used to create "
             "contacts during the sequence")
        .def("concatenateCtrajectories", &CS::concatenateCtrajectories,
             "Return a piecewise curve wchich is the concatenation of the m_c "
             "curves"
             " for each contact phases in the sequence.")
        .def("concatenateDCtrajectories", &CS::concatenateDCtrajectories,
             "Return a piecewise curve wchich is the concatenation of the m_dc "
             "curves"
             " for each contact phases in the sequence.")
        .def("concatenateDDCtrajectories", &CS::concatenateDDCtrajectories,
             "Return a piecewise curve wchich is the concatenation of the "
             "m_ddc curves"
             " for each contact phases in the sequence.")
        .def("concatenateLtrajectories", &CS::concatenateLtrajectories,
             "Return a piecewise curve wchich is the concatenation of the m_L "
             "curves"
             " for each contact phases in the sequence.")
        .def("concatenateDLtrajectories", &CS::concatenateDLtrajectories,
             "Return a piecewise curve wchich is the concatenation of the m_dL "
             "curves"
             " for each contact phases in the sequence.")
        .def("concatenateZMPtrajectories", &CS::concatenateZMPtrajectories,
             "Return a piecewise curve wchich is the concatenation of the "
             "m_zmp curves"
             " for each contact phases in the sequence.")
        .def("concatenateWrenchTrajectories",
             &CS::concatenateWrenchTrajectories,
             "Return a piecewise curve wchich is the concatenation of the "
             "m_wrench curves"
             " for each contact phases in the sequence.")
        .def("concatenateQtrajectories", &CS::concatenateQtrajectories,
             "Return a piecewise curve wchich is the concatenation of the m_q "
             "curves"
             " for each contact phases in the sequence.")
        .def("concatenateDQtrajectories", &CS::concatenateDQtrajectories,
             "Return a piecewise curve wchich is the concatenation of the m_dq "
             "curves"
             " for each contact phases in the sequence.")
        .def("concatenateDDQtrajectories", &CS::concatenateDDQtrajectories,
             "Return a piecewise curve wchich is the concatenation of the "
             "m_ddq curves"
             " for each contact phases in the sequence.")
        .def("concatenateTauTrajectories", &CS::concatenateTauTrajectories,
             "Return a piecewise curve wchich is the concatenation of the "
             "m_tau curves"
             " for each contact phases in the sequence.")
        .def("concatenateRootTrajectories", &CS::concatenateRootTrajectories,
             "Return a piecewise curve wchich is the concatenation of the "
             "m_root curves"
             " for each contact phases in the sequence.")
        .def("concatenateEffectorTrajectories",
             &CS::concatenateEffectorTrajectories, bp::arg("eeName"),
             "Return a piecewise curve which is the concatenation"
             "of the effectors trajectories curves for the given effector"
             "for each contact phases in the sequence.\n"
             "During the phases where no effector trajectories are defined,"
             "the trajectory is constant  with the value of"
             "the last phase where it was defined.")
        .def("concatenateContactForceTrajectories",
             &CS::concatenateContactForceTrajectories, bp::arg("eeName"),
             "Return a piecewise curve which"
             "is the concatenation of the contact forces for the given effector"
             "for each contact phases in the sequence.\n"
             "During the phases where no contact forces are defined,"
             "the trajectory is constant with the value of 0.")
        .def("concatenateNormalForceTrajectories",
             &CS::concatenateNormalForceTrajectories, bp::arg("eeName"),
             "Return a piecewise curve which"
             "is the concatenation of the contact normal forces for the given "
             "effector"
             "for each contact phases in the sequence.\n"
             "During the phases where no contact normal forces are defined,"
             "the trajectory is constant with the value of 0.")
        .def("phaseIdAtTime", &CS::phaseIdAtTime, bp::arg("time"),
             "return the index of a phase in the sequence such that "
             "phase.timeInitial <= t < phase.timeFinal \n"
             "if t equal to the last phase timeFinal, this index is returned.")
        .def("phaseAtTime", &CS::phaseAtTime, bp::arg("time"),
             bp::return_internal_reference<>(),
             "return a phase of the sequence such that "
             "phase.timeInitial <= t < phase.timeFinal \n"
             "if t equal to the last phase timeFinal, this index is returned.")
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def("copy", &copy, "Returns a copy of *this.");
  }

  static void expose(const std::string& class_name) {
    std::string doc = "Contact Sequence of dynamic size";
    bp::class_<CS>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(ContactSequencePythonVisitor<CS>())
        .def(SerializableVisitor<CS>());

    bp::class_<ContactPhaseVector>("ContactPhaseVector")
        .def(bp::vector_indexing_suite<ContactPhaseVector>());
  }

 protected:
  static CS copy(const CS& self) { return CS(self); }

  // Converts a C++ vector to a python list
  // Note : lot of overhead, should not be used for large vector and/or
  // operations called frequently. prefer the direct bindings with
  // std_vector_strings for this cases.
  template <class T>
  static bp::list toPythonList(std::vector<T> vector) {
    typename std::vector<T>::const_iterator iter;
    boost::python::list list;
    for (iter = vector.begin(); iter != vector.end(); ++iter) {
      list.append(*iter);
    }
    return list;
  }

  static bp::list getAllEffectorsInContactAsList(CS& self) {
    return toPythonList<std::string>(self.getAllEffectorsInContact());
  }
};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_sequence_hpp__
