// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_contact_phase_hpp__
#define __multicontact_api_python_scenario_contact_phase_hpp__

#include <pinocchio/fwd.hpp>
#include <eigenpy/memory.hpp>
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

    // call macro for all ContactPhase methods that can be overloaded
    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(isConsistent_overloads,ContactPhase::isConsistent,0,1)



    template<class PyClass>
    void visit(PyClass & cl) const
    {
//      bp::class_<t_strings>("std_vector_strings")
//          .def(bp::vector_indexing_suite<t_strings>() );

      // define bindings for maps
      bp::class_< typename ContactPhase::ContactPatchMap >("StdMap_string_contactPatch")
          .def(bp::map_indexing_suite< typename ContactPhase::ContactPatchMap >() )
          ;

      bp::class_< typename ContactPhase::CurveMap >("StdMap_string_curve")
          .def(bp::map_indexing_suite< typename ContactPhase::CurveMap >() )
          ;

      bp::class_< typename ContactPhase::CurveSE3Map >("StdMap_string_curveSE3")
          .def(bp::map_indexing_suite< typename ContactPhase::CurveSE3Map >() )
          ;


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
          .def("numContacts",&ContactPhase::numContacts,"Returns the number of active contacts.")
          .def("effectorsInContact",&effectorsInContactAsList,"Returns the names of the effectors in contact.")
          .def("isConsistent",&ContactPhase::isConsistent,isConsistent_overloads( bp::arg("throw_if_invalid"),
                  "isConsistent check if all the members of the phase are consistent together."
                  "if throw_if_invalid == True it raise an error instead of returning False."))
          .def(bp::self == bp::self)
          .def(bp::self != bp::self);

    }

    static void expose(const std::string& class_name) {
      std::string doc = "Contact Phase";
      bp::class_<ContactPhase>(class_name.c_str(), doc.c_str(), bp::no_init)
          .def(ContactPhasePythonVisitor<ContactPhase>())
          .def(SerializableVisitor<ContactPhase>())
          .def(PrintableVisitor<ContactPhase>());
    }


  protected:

    // Converts a C++ vector to a python list
    // Note : lot of overhead, should not be used for large vector and/or operations called frequently.
    // prefer the direct bindings with StdMap_string_contactPatch for this cases.
    template <class T>
    static bp::list toPythonList(std::vector<T> vector) {
        typename std::vector<T>::const_iterator iter;
        boost::python::list list;
        for (iter = vector.begin(); iter != vector.end(); ++iter) {
            list.append(*iter);
        }
        return list;
    }

    // define getter and setter (because they are overloaded in c++)
    static Scalar getTimeInitial(ContactPhase& self){return self.timeInitial();}
    static void setTimeInitial(ContactPhase& self, const Scalar& time) { self.timeInitial(time); }
    static Scalar getTimeFinal(ContactPhase& self){return self.timeFinal();}
    static void setTimeFinal(ContactPhase& self, const Scalar& time) { self.timeFinal(time);  }
    static Scalar getDuration(ContactPhase& self){return self.duration();}
    static void setDuration(ContactPhase& self, const Scalar& time) { self.duration(time);  }

    static bp::list effectorsInContactAsList(ContactPhase& self){return toPythonList<std::string>(self.effectorsInContact());}
  };
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_phase_hpp__
