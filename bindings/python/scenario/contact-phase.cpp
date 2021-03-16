// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"
#include "multicontact-api/bindings/python/scenario/contact-phase.hpp"

// required because of the serialization of the curves pointer :
#include <ndcurves/fwd.h>
#include <ndcurves/so3_linear.h>
#include <ndcurves/se3_curve.h>
#include <ndcurves/polynomial.h>
#include <ndcurves/bezier_curve.h>
#include <ndcurves/piecewise_curve.h>
#include <ndcurves/exact_cubic.h>
#include <ndcurves/cubic_hermite_spline.h>

namespace multicontact_api {
namespace python {
void exposeContactPhase() {
  ContactPhasePythonVisitor<multicontact_api::scenario::ContactPhase>::expose("ContactPhase");
}
}  // namespace python

// namespace python
}  // namespace multicontact_api
