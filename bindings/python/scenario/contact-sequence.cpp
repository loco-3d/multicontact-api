// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
//
#include "multicontact-api/bindings/python/scenario/contact-sequence.hpp"

#include "multicontact-api/bindings/python/scenario/expose-scenario.hpp"

// required because of the serialization of the curves pointer :
#include <ndcurves/bezier_curve.h>
#include <ndcurves/cubic_hermite_spline.h>
#include <ndcurves/exact_cubic.h>
#include <ndcurves/fwd.h>
#include <ndcurves/piecewise_curve.h>
#include <ndcurves/polynomial.h>
#include <ndcurves/se3_curve.h>
#include <ndcurves/so3_linear.h>

namespace multicontact_api {
namespace python {
void exposeContactSequence() {
  ContactSequencePythonVisitor<
      multicontact_api::scenario::ContactSequence>::expose("ContactSequence");
}
}  // namespace python
}  // namespace multicontact_api
