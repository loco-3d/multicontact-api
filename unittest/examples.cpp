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
 * This unit test try to deserialize the ContactSequences in the examples folder
 * and check if they have the given data set.
 * If this test fail, it probably mean that an update of multicontact-api broke the backward compatibility with
 * serialized objects The objects need to be re-generated.
 */

using namespace multicontact_api::scenario;

const std::string path = TEST_DATA_PATH;
BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(com_motion_above_feet_COM) {
  ContactSequence cs;
  cs.loadFromBinary(path + "com_motion_above_feet_COM.cs");
  BOOST_CHECK_EQUAL(cs.size(), 1);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
}

BOOST_AUTO_TEST_CASE(com_motion_above_feet_WB) {
  ContactSequence cs;
  cs.loadFromBinary(path + "com_motion_above_feet_WB.cs");
  BOOST_CHECK_EQUAL(cs.size(), 1);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(cs.haveJointsTrajectories());
  BOOST_CHECK(cs.haveJointsDerivativesTrajectories());
  BOOST_CHECK(cs.haveContactForcesTrajectories());
  BOOST_CHECK(cs.haveZMPtrajectories());
}

BOOST_AUTO_TEST_CASE(step_in_place) {
  ContactSequence cs;
  cs.loadFromBinary(path + "step_in_place.cs");
  BOOST_CHECK_EQUAL(cs.size(), 9);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(!cs.haveFriction());
  BOOST_CHECK(!cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(step_in_place_COM) {
  ContactSequence cs;
  cs.loadFromBinary(path + "step_in_place_COM.cs");
  BOOST_CHECK_EQUAL(cs.size(), 9);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(!cs.haveFriction());
  BOOST_CHECK(!cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(step_in_place_REF) {
  ContactSequence cs;
  cs.loadFromBinary(path + "step_in_place_REF.cs");
  BOOST_CHECK_EQUAL(cs.size(), 9);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(cs.haveEffectorsTrajectories());
  BOOST_CHECK(cs.haveEffectorsTrajectories(1e-6, false));
  BOOST_CHECK(cs.haveFriction());
  BOOST_CHECK(cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(step_in_place_WB) {
  ContactSequence cs;
  cs.loadFromBinary(path + "step_in_place_WB.cs");
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

BOOST_AUTO_TEST_CASE(step_in_place_quasistatic) {
  ContactSequence cs;
  cs.loadFromBinary(path + "step_in_place_quasistatic.cs");
  BOOST_CHECK_EQUAL(cs.size(), 9);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(!cs.haveFriction());
  BOOST_CHECK(!cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(step_in_place_quasistatic_COM) {
  ContactSequence cs;
  cs.loadFromBinary(path + "step_in_place_quasistatic_COM.cs");
  BOOST_CHECK_EQUAL(cs.size(), 9);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(!cs.haveFriction());
  BOOST_CHECK(!cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(step_in_place_quasistatic_REF) {
  ContactSequence cs;
  cs.loadFromBinary(path + "step_in_place_quasistatic_REF.cs");
  BOOST_CHECK_EQUAL(cs.size(), 9);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(cs.haveEffectorsTrajectories());
  BOOST_CHECK(cs.haveFriction());
  BOOST_CHECK(cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(step_in_place_quasistatic_WB) {
  ContactSequence cs;
  cs.loadFromBinary(path + "step_in_place_quasistatic_WB.cs");
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

BOOST_AUTO_TEST_CASE(walk_20cm) {
  ContactSequence cs;
  cs.loadFromBinary(path + "walk_20cm.cs");
  BOOST_CHECK_EQUAL(cs.size(), 23);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(!cs.haveFriction());
  BOOST_CHECK(!cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(walk_20cm_COM) {
  ContactSequence cs;
  cs.loadFromBinary(path + "walk_20cm_COM.cs");
  BOOST_CHECK_EQUAL(cs.size(), 23);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(!cs.haveFriction());
  BOOST_CHECK(!cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(walk_20cm_REF) {
  ContactSequence cs;
  cs.loadFromBinary(path + "walk_20cm_REF.cs");
  BOOST_CHECK_EQUAL(cs.size(), 23);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(cs.haveEffectorsTrajectories());
  BOOST_CHECK(cs.haveFriction());
  BOOST_CHECK(cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(walk_20cm_WB) {
  ContactSequence cs;
  cs.loadFromBinary(path + "walk_20cm_WB.cs");
  BOOST_CHECK_EQUAL(cs.size(), 23);
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

BOOST_AUTO_TEST_CASE(walk_20cm_quasistatic) {
  ContactSequence cs;
  cs.loadFromBinary(path + "walk_20cm_quasistatic.cs");
  BOOST_CHECK_EQUAL(cs.size(), 23);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(!cs.haveFriction());
  BOOST_CHECK(!cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(walk_20cm_quasistatic_COM) {
  ContactSequence cs;
  cs.loadFromBinary(path + "walk_20cm_quasistatic_COM.cs");
  BOOST_CHECK_EQUAL(cs.size(), 23);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(!cs.haveFriction());
  BOOST_CHECK(!cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(walk_20cm_quasistatic_REF) {
  ContactSequence cs;
  cs.loadFromBinary(path + "walk_20cm_quasistatic_REF.cs");
  BOOST_CHECK_EQUAL(cs.size(), 23);
  BOOST_CHECK(cs.haveConsistentContacts());
  BOOST_CHECK(cs.haveTimings());
  BOOST_CHECK(cs.haveCentroidalValues());
  BOOST_CHECK(cs.haveCentroidalTrajectories());
  BOOST_CHECK(cs.haveEffectorsTrajectories());
  BOOST_CHECK(cs.haveFriction());
  BOOST_CHECK(cs.haveContactModelDefined());
}

BOOST_AUTO_TEST_CASE(walk_20cm_quasistatic_WB) {
  ContactSequence cs;
  cs.loadFromBinary(path + "walk_20cm_quasistatic_WB.cs");
  BOOST_CHECK_EQUAL(cs.size(), 23);
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
