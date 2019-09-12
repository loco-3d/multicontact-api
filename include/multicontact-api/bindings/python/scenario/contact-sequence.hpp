// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_python_scenario_contact_sequence_hpp__
#define __multicontact_api_python_scenario_contact_sequence_hpp__

#include <pinocchio/fwd.hpp>
#include <eigenpy/eigenpy.hpp>
#include <pinocchio/bindings/python/utils/std-aligned-vector.hpp>

#include "multicontact-api/scenario/contact-sequence.hpp"
#include "multicontact-api/bindings/python/serialization/archive.hpp"
#include "multicontact-api/bindings/python/container/visitor.hpp"

namespace multicontact_api {
namespace python {

namespace bp = boost::python;

template <typename CS>
struct ContactSequencePythonVisitor : public boost::python::def_visitor<ContactSequencePythonVisitor<CS> > {
  typedef typename CS::ContactPhaseVector ContactPhaseVector;

  typedef typename CS::MSIntervalDataVector MSIntervalDataVector;
  typedef typename CS::MSIntervalData MSIntervalData;

  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<size_t>(bp::arg("size"), "Default constructor from a given size."))
        .def(bp::init<CS>(bp::args("other"), "Copy contructor."))
        .add_property("contact_phases", bp::make_getter(&CS::m_contact_phases, bp::return_internal_reference<>()))

        .def(bp::self == bp::self)
        .def(bp::self != bp::self)

        .def("size", &CS::size, "Size of the contact sequence.")
        .def_readwrite("ms_interval_data", &CS::m_ms_interval_data)

        .def_readwrite("conic_type", &CS::m_conic_type);
  }

  static void expose(const std::string& class_name) {
    std::string doc = "Contact Sequence of dynamic size";
    bp::class_<CS>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(ContactSequencePythonVisitor<CS>())
        .def(SerializableVisitor<CS>());

    // Expose related vector
    VectorPythonVisitor<ContactPhaseVector>::expose("ContactPhaseVector");
    pinocchio::python::StdAlignedVectorPythonVisitor<MSIntervalData, true>::expose("MSIntervalDataVector");
  }
};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_sequence_hpp__
