// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_ms_interval_hpp__
#define __multicontact_api_python_scenario_ms_interval_hpp__

#include <eigenpy/eigenpy.hpp>

#include "multicontact-api/scenario/contact-phase-humanoid.hpp"
#include "multicontact-api/scenario/contact-phase.hpp"
#include "multicontact-api/bindings/python/container/visitor.hpp"
#include "multicontact-api/bindings/python/container/array.hpp"

#include <pinocchio/bindings/python/utils/std-aligned-vector.hpp>

namespace multicontact_api
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


#endif // ifndef __multicontact_api_python_scenario_ms_interval_hpp__
