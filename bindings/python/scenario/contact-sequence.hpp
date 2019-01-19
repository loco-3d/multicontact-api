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
#ifndef __locomote_python_scenario_contact_sequence_hpp__
#define __locomote_python_scenario_contact_sequence_hpp__

#include <eigenpy/eigenpy.hpp>
#include <pinocchio/bindings/python/utils/std-aligned-vector.hpp>

#include "locomote/scenario/contact-sequence.hpp"
#include "locomote/bindings/python/serialization/archive.hpp"
#include "locomote/bindings/python/container/visitor.hpp"

namespace locomote
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename CS>
    struct ContactSequencePythonVisitor
    : public boost::python::def_visitor< ContactSequencePythonVisitor<CS> >
    {
      typedef typename CS::ContactPhaseVector ContactPhaseVector;
      
      typedef typename CS::MSIntervalDataVector MSIntervalDataVector;
      typedef typename CS::MSIntervalData MSIntervalData;
      
      
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl
        .def(bp::init<size_t>(bp::arg("size"),"Default constructor from a given size."))
        .def(bp::init<CS>(bp::args("other"),"Copy contructor."))
        .add_property("contact_phases", bp::make_getter(&CS::m_contact_phases,bp::return_internal_reference<>()))
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        .def("size",&CS::size,"Size of the contact sequence.")
        .def_readwrite("ms_interval_data",&CS::m_ms_interval_data)
        
        .def_readwrite("conic_type",&CS::m_conic_type)
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Contact Sequence of dynamic size";
        bp::class_<CS>(class_name.c_str(),
                        doc.c_str(),
                        bp::no_init)
        .def(ContactSequencePythonVisitor<CS>())
        .def(SerializableVisitor<CS>())
        ;
        
        // Expose related vector
        VectorPythonVisitor<ContactPhaseVector>::expose("ContactPhaseVector");
        se3::python::StdAlignedVectorPythonVisitor<MSIntervalData,true>::expose("MSIntervalDataVector");
        
      }
    };
  }
}


#endif // ifndef __locomote_python_scenario_contact_sequence_hpp__
