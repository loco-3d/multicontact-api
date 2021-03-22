// Copyright (c) 2019-2020, CNRS
// Authors: Pierre Fernbach <pierre.fernbach@laas.fr>,

#include <iostream>

#define BOOST_TEST_MODULE StatsTests
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "multicontact-api/scenario/contact-sequence.hpp"
#include "multicontact-api/scenario/fwd.hpp"
#include <ndcurves/fwd.h>
#include <ndcurves/so3_linear.h>
#include <ndcurves/se3_curve.h>
#include <ndcurves/polynomial.h>
#include <ndcurves/bezier_curve.h>
#include <ndcurves/piecewise_curve.h>
#include <ndcurves/exact_cubic.h>
#include <ndcurves/cubic_hermite_spline.h>

/**
 * This unit test try to deserialize the ContactSequences in the examples/previous_versions folder
 * And check that they are compatible with the current version
 */

using namespace multicontact_api::scenario;

const std::string path = TEST_DATA_PATH;
BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(api_0) {
  ContactSequence cs;
  cs.loadFromBinary(path + "previous_versions/api_0.cs");
  BOOST_CHECK_EQUAL(cs.size(), 9);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(cs.haveEffectorsTrajectories(1e-1));
  BOOST_CHECK(cs.haveJointsTrajectories());
  BOOST_CHECK(cs.haveJointsDerivativesTrajectories());
  BOOST_CHECK(cs.haveContactForcesTrajectories());
  BOOST_CHECK(cs.haveZMPtrajectories());
  BOOST_CHECK(cs.haveFriction());
  BOOST_CHECK(!cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(api_1) {
  ContactSequence cs;
  cs.loadFromBinary(path + "previous_versions/api_1.cs");
  BOOST_CHECK_EQUAL(cs.size(), 9);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(cs.haveEffectorsTrajectories(1e-1));
  BOOST_CHECK(cs.haveJointsTrajectories());
  BOOST_CHECK(cs.haveJointsDerivativesTrajectories());
  BOOST_CHECK(cs.haveContactForcesTrajectories());
  BOOST_CHECK(cs.haveZMPtrajectories());
  BOOST_CHECK(cs.haveFriction());
  BOOST_CHECK(cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_SUITE_END()
