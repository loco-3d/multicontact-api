// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_contact_phase_hpp__
#define __multicontact_api_python_scenario_contact_phase_hpp__

#include <pinocchio/fwd.hpp>
#include <eigenpy/memory.hpp>
#include <typeinfo>

#include "multicontact-api/scenario/contact-phase.hpp"
#include "multicontact-api/bindings/python/serialization/archive.hpp"
#include "multicontact-api/bindings/python/container/array.hpp"
#include "multicontact-api/bindings/python/container/visitor.hpp"
#include "multicontact-api/bindings/python/container/reference-wrapper.hpp"


namespace multicontact_api
{
  namespace python
  {

    namespace bp = boost::python;

    template<typename ContactPhase>
    struct ContactPhasePythonVisitor
    : public boost::python::def_visitor< ContactPhasePythonVisitor<ContactPhase> >
    {
      typedef typename ContactPhase::Scalar Scalar;
      typedef typename ContactPhase::SOC6 SOC6;
      typedef typename ContactPhase::WrenchCone WrenchCone;
      typedef typename ContactPhase::ContactPatchVector ContactPatchVector;
      typedef typename ContactPhase::ContactPatch ContactPatch;
      typedef typename ContactPhase::SE3 SE3;
      typedef typename ContactPhase::Matrix6x Matrix6x;

      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl
        .def(bp::init<>(bp::arg(""),"Default constructor."))
        .def(bp::init<ContactPhase>(bp::args("other"),"Copy contructor."))
        .add_property("contact_patches", bp::make_array(&ContactPhase::m_contact_patches)) // TODO
        .def("numActivePatches",&ContactPhase::numActivePatches,"Returns the number of active patches.")
        .def("getActivePatches",&ContactPhase::getActivePatches,"Returns the vector of active patches.")

        .add_property("sowc",
                      bp::make_function(&getSOWC,bp::return_internal_reference<>()),
                      &setSOWC,
                      "Second order conic constraint representing the Wrench Cone of contact forces of the patches."
                      )

        .add_property("sowc_placement",
                      bp::make_function(&getSOWCPlacement,bp::return_internal_reference<>()),
                      &setSOWCPlacement,
                      "Returns the placement of the SDWC."
                      )

        .add_property("double_description",
                      &getDoubleDescription,
                      &setDoubleDescription,
                      "Returns the double description of the LWC."
                      )

        .add_property("lwc",
                      bp::make_function(&getLWC,bp::return_internal_reference<>()),
                      &setLWC,
                      "Linear cone constraint representing the Wrench Cone of contact forces of the patches."
                      )

        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "Contact Phase";
        bp::class_<ContactPhase>(class_name.c_str(),
                       doc.c_str(),
                       bp::no_init)
        .def(ContactPhasePythonVisitor<ContactPhase>())
        .def(SerializableVisitor<ContactPhase>())
        ;

        // Expose related types
        related();
      }

      static void related()
      {
        VectorPythonVisitor<ContactPatchVector,true>::expose(typeid(ContactPatchVector).name());
        reference_wrapper_converter<typename ContactPatchVector::value_type>::expose();
      }

    protected:

      static SOC6 & getSOWC(ContactPhase & self) { return self.sowc(); }
      static void setSOWC(ContactPhase & self, const SOC6 & cone) { self.sowc() = cone; }

      static Matrix6x getDoubleDescription(const ContactPhase & self) { return self.doubleDescription(); }
      static void setDoubleDescription(ContactPhase & self, const Matrix6x & mat) { self.doubleDescription() = mat; }

      static SE3 & getSOWCPlacement(ContactPhase & self) { return self.sowcPlacement(); }
      static void setSOWCPlacement(ContactPhase & self, const SE3 & placement) { self.sowcPlacement() = placement; }

      static WrenchCone & getLWC(ContactPhase & self) { return self.lwc(); }
      static void setLWC(ContactPhase & self, const WrenchCone & cone) { self.lwc() = cone; }
    };
  }
}


#endif // ifndef __multicontact_api_python_scenario_contact_phase_hpp__
