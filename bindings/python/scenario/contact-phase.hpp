// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
// Simplified BSD license :
//Redistribution and use in source and binary forms, with or without modification,
//are permitted provided that the following conditions are met:

//1. Redistributions of source code must retain the above copyright notice,
//this list of conditions and the following disclaimer.

//2. Redistributions in binary form must reproduce the above copyright notice,
//this list of conditions and the following disclaimer in the documentation
//and/or other materials provided with the distribution.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
//OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
//ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef __locomote_python_scenario_contact_phase_hpp__
#define __locomote_python_scenario_contact_phase_hpp__

#include <eigenpy/memory.hpp>
#include <typeinfo>

#include "locomote/scenario/contact-phase.hpp"
#include "locomote/bindings/python/serialization/archive.hpp"
#include "locomote/bindings/python/container/array.hpp"
#include "locomote/bindings/python/container/visitor.hpp"
#include "locomote/bindings/python/container/reference-wrapper.hpp"


namespace locomote
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


#endif // ifndef __locomote_python_scenario_contact_phase_hpp__
