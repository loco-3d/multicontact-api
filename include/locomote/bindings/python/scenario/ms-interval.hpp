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
#ifndef __locomote_python_scenario_ms_interval_hpp__
#define __locomote_python_scenario_ms_interval_hpp__

#include <eigenpy/eigenpy.hpp>

#include "locomote/scenario/contact-phase-humanoid.hpp"
#include "locomote/scenario/contact-phase.hpp"
#include "locomote/bindings/python/container/visitor.hpp"
#include "locomote/bindings/python/container/array.hpp"

#include <pinocchio/bindings/python/utils/std-aligned-vector.hpp>

namespace locomote
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename MSInterval>
    struct MSIntervalPythonVisitor
    : public boost::python::def_visitor< MSIntervalPythonVisitor<MSInterval> >
    {
      
      typedef typename MSInterval::TimeVector TimeVector;
      typedef typename MSInterval::StateVectorTrajectory StateVectorTrajectory;
      typedef typename MSInterval::ControlVectorTrajectory ControlVectorTrajectory;
      
      template<class PyClass>
      void visit(PyClass & cl) const
      {
        cl
        .def(bp::init<>(bp::arg(""),"Default constructor."))
        .def(bp::init<MSInterval>(bp::args("other"),"Copy contructor."))
        
        .add_property("time_trajectory",
                      bp::make_function(&time_trajectory,
                      bp::return_internal_reference<>()))
        .add_property("state_trajectory",
                      bp::make_function(&state_trajectory,
                      bp::return_internal_reference<>()))
        .add_property("dot_state_trajectory",
                      bp::make_function(&dot_state_trajectory,
                      bp::return_internal_reference<>()))
        .add_property("control_trajectory",
                      bp::make_function(&control_trajectory,
                      bp::return_internal_reference<>()))
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        .def("copy",&copy,"Returns a copy of *this.")
        ;
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Multiple Shootin inveral data: state and dot state trajectories, time trajectory and control trajectory.";
        bp::class_<MSInterval>(class_name.c_str(),
                               doc.c_str(),
                               bp::no_init)
        .def(MSIntervalPythonVisitor<MSInterval>())
        ;
        
      }
      
    protected:
      
      static MSInterval copy(const MSInterval & self) { return MSInterval(self); }
      
      static TimeVector & time_trajectory(MSInterval & self) { return self.time_trajectory(); }
      static StateVectorTrajectory & state_trajectory(MSInterval & self) { return self.state_trajectory(); }
      static StateVectorTrajectory & dot_state_trajectory(MSInterval & self) { return self.dot_state_trajectory(); }
      static ControlVectorTrajectory & control_trajectory(MSInterval & self) { return self.control_trajectory(); }
    };
  }
}


#endif // ifndef __locomote_python_scenario_ms_interval_hpp__
