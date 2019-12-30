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
struct ContactSequencePythonVisitor : public bp::def_visitor<ContactSequencePythonVisitor<CS> > {
  typedef typename CS::ContactPhaseVector ContactPhaseVector;



  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<size_t>(bp::arg("size"), "Default constructor from a given size."))
        .def(bp::init<>(bp::arg(""),"Default constructor."))
        .def(bp::init<CS>(bp::args("other"), "Copy contructor."))
        .def("size", &CS::size, "Return the size of the contact sequence.")
        .def("resize", &CS::resize,bp::arg("size"), "Resize the vector of ContactPhases.")
        .def("append",&CS::append,bp::arg("ContactPhase"),
        "Add the given ContactPhase at the end of the sequence. \n"
        "Return the new id of this ContactPhase inside the sequence.")
        .add_property("contactPhases",
        bp::make_getter(&CS::m_contact_phases,bp::return_internal_reference<>()),
        "Vector of Contact Phases contained in the sequence")
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

    bp::class_<ContactPhaseVector>("ContactPhaseVector")
        .def(bp::vector_indexing_suite<ContactPhaseVector>() );
  }

  protected:

    static CS copy(const CS& self) { return CS(self); }


};
}  // namespace python
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_python_scenario_contact_sequence_hpp__
