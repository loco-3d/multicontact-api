// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_contact_phase_hpp__
#define __multicontact_api_python_scenario_contact_phase_hpp__

#include <pinocchio/fwd.hpp>
#include <eigenpy/eigenpy.hpp>
#include <typeinfo>

#include "multicontact-api/scenario/contact-phase.hpp"
#include "multicontact-api/bindings/python/serialization/archive.hpp"
#include "multicontact-api/bindings/python/utils/printable.hpp"

#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>


namespace multicontact_api{
namespace python{

  namespace bp = boost::python;

  template<typename ContactPhase>
  struct ContactPhasePythonVisitor
      : public bp::def_visitor< ContactPhasePythonVisitor<ContactPhase> >
  {
    typedef typename ContactPhase::Scalar Scalar;
    typedef typename ContactPhase::ContactPatch ContactPatch;
    typedef typename ContactPhase::SE3 SE3;
    typedef typename ContactPhase::t_strings t_strings;
    typedef typename ContactPhase::curve_ptr curve_ptr;
    typedef typename ContactPhase::curve_SE3_ptr curve_SE3_ptr;
    typedef typename ContactPhase::point3_t point3_t;
    typedef typename ContactPhase::point6_t point6_t;
    typedef typename ContactPhase::pointX_t pointX_t;


    // call macro for all ContactPhase methods that can be overloaded
    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(isConsistent_overloads,ContactPhase::isConsistent,0,1)



    template<class PyClass>
    void visit(PyClass & cl) const
    {



      cl
          .def(bp::init<>(bp::arg(""),"Default constructor."))
          .def(bp::init<Scalar,Scalar>(bp::args("t_init","t_final"),"Constructor with time interval."))
          .def(bp::init<ContactPhase>(bp::arg("other"),"Copy contructor."))
          .add_property("timeInitial", &getTimeInitial, &setTimeInitial,
                        "The time at the begining of this contact phase.")
          .add_property("timeFinal", &getTimeFinal, &setTimeFinal,
                        "The time at the end of this contact phase.")
          .add_property("duration", &getDuration, &setDuration,
                        "The duration this contact phase.")
          // expose public members :
          .add_property("c_init", bp::make_getter(&ContactPhase::m_c_init,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_c_init),
                         "Initial 3D position of the center of mass for this contact phase")
          .add_property("dc_init", bp::make_getter(&ContactPhase::m_dc_init,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_dc_init),
                        "Initial linear velocity of the center of mass for this contact phase")
          .add_property("ddc_init",  bp::make_getter(&ContactPhase::m_ddc_init,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_ddc_init),
                        "Initial linear acceleration of the center of mass for this contact phase")
          .add_property("L_init",  bp::make_getter(&ContactPhase::m_L_init,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_L_init),
                        "Initial angular momentum for this contact phase")
          .add_property("dL_init",  bp::make_getter(&ContactPhase::m_dL_init,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_dL_init),
                        "Initial angular momentum derivative for this contact phase")
          .add_property("q_init",  bp::make_getter(&ContactPhase::m_q_init,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_q_init),
                        "Initial whole body configuration of this phase")
          .add_property("c_final",  bp::make_getter(&ContactPhase::m_c_final,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_c_final),
                        "Final 3D position of the center of mass for this contact phase")
          .add_property("dc_final",  bp::make_getter(&ContactPhase::m_dc_final,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_dc_final),
                        "Final linear velocity of the center of mass for this contact phase")
          .add_property("ddc_final",  bp::make_getter(&ContactPhase::m_ddc_final,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_ddc_final),
                        "Final linear acceleration of the center of mass for this contact phase")
          .add_property("L_final", bp::make_getter(&ContactPhase::m_L_final,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_L_final),
                        "Final angular momentum for this contact phase")
          .add_property("dL_final", bp::make_getter(&ContactPhase::m_dL_final,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_dL_final),
                        "Final angular momentum derivative for this contact phase")
          .add_property("q_final",  bp::make_getter(&ContactPhase::m_q_final,bp::return_value_policy<bp::return_by_value>()),
                                  bp::make_setter(&ContactPhase::m_q_final),
                        "Final whole body configuration of this phase")
          .def_readwrite("q_t", &ContactPhase::m_q)
          .def_readwrite("dq_t", &ContactPhase::m_dq)
          .def_readwrite("ddq_t", &ContactPhase::m_ddq)
          .def_readwrite("tau_t", &ContactPhase::m_tau)
          .def_readwrite("c_t", &ContactPhase::m_c)
          .def_readwrite("dc_t", &ContactPhase::m_dc)
          .def_readwrite("ddc_t", &ContactPhase::m_ddc)
          .def_readwrite("L_t", &ContactPhase::m_L)
          .def_readwrite("dL_t", &ContactPhase::m_dL)
          .def_readwrite("wrench_t", &ContactPhase::m_wrench)
          .def_readwrite("zmp_t", &ContactPhase::m_zmp)
          .def_readwrite("root_t", &ContactPhase::m_root)
          // accessor to map with key
          .def("contactForce", &contactForcesFromKey,bp::arg("effector_name"),
               "Return a pointer to the contact force trajectory for this effector.\n"
               "Throw a ValueError if the effector is not in contact.")
          .def("contactNormalForce", &contactNormalForcesFromKey,bp::arg("effector_name"),
               "Return a pointer to the contact normal force trajectory for this effector.\n"
               "Throw a ValueError if the effector is not in contact.")
          .def("effectorTrajectory", &effectorTrajectoriesFromKey,bp::arg("effector_name"),
               "Return a pointer to the effector trajectory (in SE3) for this effector.\n"
               "Throw a ValueError if the effector is in contact.")
          .def("contactPatch", &contactPatchFromKey,bp::arg("effector_name"),bp::return_internal_reference<>(),
               "Return the ContactPatch object for this effector.\n"
               "Throw a ValueError if the effector is not in contact.")
          // Bindings of the maps:
          .def("contactPatches", &contactPatchesAsDict,
               "Return a CONST dict EffectorName:ContactPatch")
          .def("contactForces", &contactForcesAsDict,
               "Return a CONST dict EffectorName:contact force")
          .def("contactNormalForces", &contactNormalForcesAsDict,
               "Return a CONST dict EffectorName:contact normal force")
          .def("effectorTrajectories", &effectorTrajectoriesAsDict,
               "Return a CONST dict EffectorName:effector trajectory")
          // adding trajectory to map :
          .def("addContactForceTrajectory", &ContactPhase::addContactForceTrajectory,bp::args("effector_name","trajectory"),
               "Add a trajectory to the map of contact forces.\n"
               "If a trajectory already exist for this effector, it is overwritted.\n"
               "Throw invalid_argument if eeName is not defined in contact for this phase.\n"
               "Return false if a trajectory already existed (and have been overwrited) true otherwise.")
          .def("addContactNormalForceTrajectory", &ContactPhase::addContactNormalForceTrajectory,bp::args("effector_name","trajectory"),
               "Add a trajectory to the map of contact normal forces.\n"
               "If a trajectory already exist for this effector, it is overwritted.\n"
               "Throw a ValueError if eeName is not defined in contact for this phase.\n"
               "Throw a ValueError if trajectory is not of dimension 1.\n"
               "Return false if a trajectory already existed (and have been overwrited) true otherwise.")
          .def("addEffectorTrajectory", &ContactPhase::addEffectorTrajectory,bp::args("effector_name","trajectory"),
               "Add a trajectory to the map of effector trajectories.\n"
               "If a trajectory already exist for this effector, it is overwritted.\n"
               "Throw a ValueError if eeName is defined in contact for this phase.\n"
               "Return false if a trajectory already existed (and have been overwrited) true otherwise.")
          // contacts
          .def("addContact", &ContactPhase::addContact,bp::args("effector_name","patch"),
               "Add a new contact patch for effector_name to this contact phase\n"
               "If a contact phase already exist for this effector, it is overwritted.\n"
               "If an end effector trajectory exist for this contact, it is removed.\n"
               "Return false if a contact for this effector already existed (and have been overwrited) true otherwise.")
          .def("removeContact", &ContactPhase::removeContact,bp::arg("effector_name"),
               "Remove the contact for effector_name.\n"
               "This will also remove the contact_patch, all the contact_forces and contact_normal_forces related to this contact.\n"
               "Return true if the effector was in contact, false otherwise")
          .def("numContacts",&ContactPhase::numContacts,"Returns the number of active contacts.")
          .def("isEffectorInContact",&ContactPhase::isEffectorInContact,bp::arg("effector_name"),
               "Returns True if the given effector_name is in contact for this phase, False otherwise.")
          .def("effectorsInContact",&effectorsInContactAsList,"Returns the names of the effectors in contact.")
          .def("effectorHaveAtrajectory",&ContactPhase::effectorHaveAtrajectory,bp::arg("effector_name"),
               "Returns True if the given effector_name have an effector_trajectory defined in this phase, False otherwise.")
          .def("effectorsWithTrajectory",&effectorsWithTrajectoryAsList,
               "Returns the names of the effectors for which an end effector trajectory have been defined in this phase.")
          .def("isConsistent",&ContactPhase::isConsistent,isConsistent_overloads( bp::arg("throw_if_invalid"),
                  "isConsistent check if all the members of the phase are consistent together.\n"
                  "if throw_if_invalid == True it raise an error instead of returning False."))
          .def(bp::self == bp::self)
          .def(bp::self != bp::self)
          .def("copy", &copy, "Returns a copy of *this.");

    }

    static void expose(const std::string& class_name) {
      std::string doc = "Contact Phase";
      bp::class_<ContactPhase>(class_name.c_str(), doc.c_str(), bp::no_init)
          .def(ContactPhasePythonVisitor<ContactPhase>())
          .def(SerializableVisitor<ContactPhase>())
          .def(PrintableVisitor<ContactPhase>());

      ENABLE_SPECIFIC_MATRIX_TYPE(point3_t);
      ENABLE_SPECIFIC_MATRIX_TYPE(point6_t);
      ENABLE_SPECIFIC_MATRIX_TYPE(pointX_t);
    }


  protected:


    // define getter and setter (because they are overloaded in c++)
    static Scalar getTimeInitial(ContactPhase& self){return self.timeInitial();}
    static void setTimeInitial(ContactPhase& self, const Scalar& time) { self.timeInitial(time); }
    static Scalar getTimeFinal(ContactPhase& self){return self.timeFinal();}
    static void setTimeFinal(ContactPhase& self, const Scalar& time) { self.timeFinal(time);  }
    static Scalar getDuration(ContactPhase& self){return self.duration();}
    static void setDuration(ContactPhase& self, const Scalar& time) { self.duration(time);  }

    // accessor to map with key:
    static curve_ptr contactForcesFromKey(ContactPhase& self,const std::string& eeName){return self.contactForces(eeName);}
    static curve_ptr contactNormalForcesFromKey(ContactPhase& self,const std::string& eeName){return self.contactNormalForces(eeName);}
    static curve_SE3_ptr effectorTrajectoriesFromKey(ContactPhase& self,const std::string& eeName){return self.effectorTrajectories(eeName);}
    static ContactPatch& contactPatchFromKey(ContactPhase& self,const std::string& eeName){return self.contactPatch(eeName);}

    // Converts a C++ vector to a python list
    // Note : lot of overhead, should not be used for large vector and/or operations called frequently.
    // prefer the direct bindings with std_vector_strings for this cases.
    template <class T>
    static bp::list toPythonList(std::vector<T> vector) {
        typename std::vector<T>::const_iterator iter;
        boost::python::list list;
        for (iter = vector.begin(); iter != vector.end(); ++iter) {
            list.append(*iter);
        }
        return list;
    }
    static bp::list effectorsInContactAsList(ContactPhase& self){return toPythonList<std::string>(self.effectorsInContact());}
    static bp::list effectorsWithTrajectoryAsList(ContactPhase& self){return toPythonList<std::string>(self.effectorsWithTrajectory());}

    // Converts a C++ map to a python dict
    // Note : lot of overhead, should not be used for large map and/or operations called frequently.
    // prefer the direct bindings with std_map_* for this cases.
    template <class T>
    static bp::dict toPythonDict(std::map<std::string,T> map) {
        typename std::map<std::string,T>::const_iterator iter;
        bp::dict dict;
        for (iter = map.begin(); iter != map.end(); ++iter) {
            dict[iter->first] = iter->second;
        }
        return dict;
    }

    static bp::dict contactPatchesAsDict(ContactPhase& self){return toPythonDict<ContactPatch>(self.contactPatches());}
    static bp::dict contactForcesAsDict(ContactPhase& self){return toPythonDict<curve_ptr>(self.contactForces());}
    static bp::dict contactNormalForcesAsDict(ContactPhase& self){return toPythonDict<curve_ptr>(self.contactNormalForces());}
    static bp::dict effectorTrajectoriesAsDict(ContactPhase& self){return toPythonDict<curve_SE3_ptr>(self.effectorTrajectories());}
    static ContactPhase copy(const ContactPhase& self) { return ContactPhase(self); }


  };
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_phase_hpp__
