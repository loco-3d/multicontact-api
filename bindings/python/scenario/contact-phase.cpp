// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/scenario/contact-phase.hpp"

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
void exposeContactPhase() {
  ContactPhasePythonVisitor<multicontact_api::scenario::ContactPhase>::expose("ContactPhase");
}
}


// namespace python
}  // namespace multicontact_api
