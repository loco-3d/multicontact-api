// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/scenario/contact-sequence.hpp"

// required because of the serialization of the curves pointer :
#include <curves/fwd.h>
#include <curves/so3_linear.h>
#include <curves/se3_curve.h>
#include <curves/polynomial.h>
#include <curves/bezier_curve.h>
#include <curves/piecewise_curve.h>
#include <curves/exact_cubic.h>
#include <curves/cubic_hermite_spline.h>


namespace multicontact_api {
namespace python {
void exposeContactSequence() {
  ContactSequencePythonVisitor<multicontact_api::scenario::ContactSequence>::expose("ContactSequence");
}
}  // namespace python
}  // namespace multicontact_api
