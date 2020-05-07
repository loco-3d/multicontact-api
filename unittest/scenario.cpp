// Copyright (c) 2019-2020, CNRS
// Authors: Pierre Fernbach <pierre.fernbach@laas.fr>,

#include <iostream>

#define BOOST_TEST_MODULE StatsTests
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "multicontact-api/scenario/contact-model.hpp"
#include "multicontact-api/scenario/contact-patch.hpp"
#include "multicontact-api/scenario/contact-phase.hpp"
#include "multicontact-api/scenario/contact-sequence.hpp"

#include <curves/fwd.h>
#include <curves/se3_curve.h>
#include <curves/polynomial.h>
#include <curves/bezier_curve.h>
#include <curves/piecewise_curve.h>

typedef Eigen::Matrix<double, 1, 1> point1_t;
using curves::point3_t;
using curves::point6_t;
typedef Eigen::Matrix<double, 12, 1> point12_t;
using curves::curve_ptr_t;
using curves::curve_SE3_ptr_t;
using curves::matrix3_t;
using curves::piecewise_SE3_t;
using curves::piecewise_t;
using curves::pointX_t;
using curves::quaternion_t;
using curves::t_point3_t;
using curves::t_pointX_t;
using namespace multicontact_api::scenario;
typedef ContactSequence::ContactPhaseVector ContactPhaseVector;
typedef ContactModel::Matrix3X Matrix3X;

typedef pinocchio::SE3Tpl<double> SE3;

curve_ptr_t buildPiecewisePolynomialC2() {
  point3_t p0(0., 0., 0.);
  point3_t p1(1., 2., 3.);
  point3_t p2(4., 4., 4.);
  point3_t p3(10., 10., 10.);
  point3_t d0(1., 1., 1.);
  point3_t d1(2., 2., 2.);
  point3_t d2(3., 3., 3.);
  point3_t d3(5., 5., 5.);
  point3_t dd0(1.5, 1.5, 1.5);
  point3_t dd1(2.5, 2.5, 2.5);
  point3_t dd2(3.5, 3.5, 3.5);
  point3_t dd3(5.5, 5.5, 5.5);
  double t0 = 1.0;
  double t1 = 1.5;
  double t2 = 3.0;
  double t3 = 10.0;
  t_pointX_t points;
  points.push_back(p0);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  t_pointX_t points_derivative;
  points_derivative.push_back(d0);
  points_derivative.push_back(d1);
  points_derivative.push_back(d2);
  points_derivative.push_back(d3);
  t_pointX_t points_second_derivative;
  points_second_derivative.push_back(dd0);
  points_second_derivative.push_back(dd1);
  points_second_derivative.push_back(dd2);
  points_second_derivative.push_back(dd3);
  std::vector<double> time_points;
  time_points.push_back(t0);
  time_points.push_back(t1);
  time_points.push_back(t2);
  time_points.push_back(t3);

  curves::piecewise_t ppc_C2 = curves::piecewise_t::convert_discrete_points_to_polynomial<curves::polynomial_t>(
      points, points_derivative, points_second_derivative, time_points);
  curve_ptr_t res(new curves::piecewise_t(ppc_C2));
  return res;
}

template <typename Point>
curve_ptr_t buildRandomPolynomial(double min = -1, double max = -1) {
  pointX_t a = Point::Random();
  pointX_t b = Point::Random();
  pointX_t c = Point::Random();
  pointX_t d = Point::Random();
  if (min < 0 && max < 0) {
    const double t1 = Eigen::internal::random<double>(0., 10.);
    const double t2 = Eigen::internal::random<double>(0., 10.);
    min = std::min(t1, t2);
    max = std::max(t1, t2);
  }
  t_pointX_t vec;
  vec.push_back(a);
  vec.push_back(b);
  vec.push_back(c);
  vec.push_back(d);
  curve_ptr_t res(new curves::polynomial_t(vec.begin(), vec.end(), min, max));
  return res;
}

curve_ptr_t buildRandomPolynomial3D(double min = -1, double max = -1) {
  return buildRandomPolynomial<point3_t>(min, max);
}

curve_ptr_t buildRandomPolynomial12D(double min = -1, double max = -1) {
  return buildRandomPolynomial<point12_t>(min, max);
}

curve_ptr_t buildRandomPolynomial1D(double min = -1, double max = -1) {
  return buildRandomPolynomial<point1_t>(min, max);
}

curve_SE3_ptr_t buildPiecewiseSE3() {
  const double min = 0.5;
  const double mid = 1.2;
  const double max = 2.;
  quaternion_t q0(1, 0, 0, 0);
  quaternion_t q1(0., 1., 0, 0);
  pointX_t p0 = point3_t(1., 1.5, -2.);
  pointX_t p1 = point3_t(3., 0, 1.);
  curve_SE3_ptr_t cLinear(new curves::SE3Curve_t(p0, p1, q0, q1, min, mid));
  point3_t a(1, 2, 3);
  point3_t b(2, 3, 4);
  point3_t c(3, 4, 5);
  point3_t d(3, 6, 7);
  std::vector<point3_t> params;
  params.push_back(a);
  params.push_back(b);
  params.push_back(c);
  params.push_back(d);
  boost::shared_ptr<curves::bezier_t> translation_bezier(new curves::bezier_t(params.begin(), params.end(), mid, max));
  curve_SE3_ptr_t cBezier(new curves::SE3Curve_t(translation_bezier, q0.toRotationMatrix(), q1.toRotationMatrix()));

  curves::piecewise_SE3_t piecewiseSE3(cLinear);
  piecewiseSE3.add_curve_ptr(cBezier);
  curve_SE3_ptr_t res = boost::make_shared<curves::piecewise_SE3_t>(piecewiseSE3);
  return res;
}

quaternion_t randomQuaternion() {  // already included in newest eigen release
  const double u1 = Eigen::internal::random<double>(0, 1), u2 = Eigen::internal::random<double>(0, 2 * EIGEN_PI),
               u3 = Eigen::internal::random<double>(0, 2 * EIGEN_PI);
  const double a = sqrt(1 - u1), b = sqrt(u1);
  return quaternion_t(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3)).normalized();
}

curve_SE3_ptr_t buildRandomSE3LinearTraj(const double min, const double max) {
  quaternion_t q0 = randomQuaternion();
  quaternion_t q1 = randomQuaternion();
  q0.normalize();
  q1.normalize();
  pointX_t p0 = point3_t::Random();
  pointX_t p1 = point3_t::Random();

  curve_SE3_ptr_t res(new curves::SE3Curve_t(p0, p1, q0, q1, min, max));
  return res;
}

void addRandomPointsValues(ContactPhase& cp) {
  point3_t c_init = point3_t::Random();
  point3_t dc_init = point3_t::Random();
  pointX_t ddc_init = point3_t::Random();  // test with point X for automatic conversion
  pointX_t L_init = point3_t::Random();
  point3_t dL_init = point3_t::Random();
  pointX_t q_init = point12_t::Random();
  pointX_t c_final = point3_t::Random();
  pointX_t dc_final = point3_t::Random();
  point3_t ddc_final = point3_t::Random();
  pointX_t L_final = point3_t::Random();
  point3_t dL_final = point3_t::Random();
  pointX_t q_final = point12_t::Random();
  cp.m_c_init = c_init;
  cp.m_dc_init = dc_init;
  cp.m_ddc_init = ddc_init;
  cp.m_L_init = L_init;
  cp.m_dL_init = dL_init;
  cp.m_q_init = q_init;
  cp.m_c_final = c_final;
  cp.m_dc_final = dc_final;
  cp.m_ddc_final = ddc_final;
  cp.m_L_final = L_final;
  cp.m_dL_final = dL_final;
  cp.m_q_final = q_final;
}

void addRandomCurvesValues(ContactPhase& cp) {
  curve_ptr_t q = buildRandomPolynomial12D();
  curve_ptr_t dq = buildRandomPolynomial12D();
  curve_ptr_t ddq = buildRandomPolynomial12D();
  curve_ptr_t tau = buildRandomPolynomial12D();
  curve_ptr_t c = buildRandomPolynomial3D();
  curve_ptr_t dc = buildRandomPolynomial3D();
  curve_ptr_t ddc = buildRandomPolynomial3D();
  curve_ptr_t L = buildRandomPolynomial3D();
  curve_ptr_t dL = buildRandomPolynomial3D();
  curve_ptr_t wrench = buildRandomPolynomial3D();
  curve_ptr_t zmp = buildRandomPolynomial3D();
  curve_SE3_ptr_t root = buildRandomSE3LinearTraj(1, 5.5);
  cp.m_q = q;
  cp.m_dq = dq;
  cp.m_ddq = ddq;
  cp.m_tau = tau;
  cp.m_c = c;
  cp.m_dc = dc;
  cp.m_ddc = ddc;
  cp.m_L = L;
  cp.m_dL = dL;
  cp.m_wrench = wrench;
  cp.m_zmp = zmp;
  cp.m_root = root;
}

void addRandomContacts(ContactPhase& cp) {
  cp.addContact("right_hand", ContactPatch(SE3::Identity().setRandom()));
  cp.addContact("left_foot", ContactPatch(SE3::Identity().setRandom()));
}

void addRandomForcesTrajs(ContactPhase& cp) {
  cp.addContactForceTrajectory("right_hand", buildRandomPolynomial12D());
  cp.addContactForceTrajectory("left_foot", buildRandomPolynomial12D());
  cp.addContactNormalForceTrajectory("right_hand", buildRandomPolynomial1D());
  cp.addContactNormalForceTrajectory("left_foot", buildRandomPolynomial1D());
}

void addRandomEffectorTrajectories(ContactPhase& cp) {
  cp.addEffectorTrajectory("left_hand", buildRandomSE3LinearTraj(0, 2));
  cp.addEffectorTrajectory("right_foot", buildRandomSE3LinearTraj(0, 2));
}

ContactPhase buildRandomContactPhase(const double min, const double max) {
  ContactPhase cp(min, max);
  addRandomPointsValues(cp);
  addRandomCurvesValues(cp);
  addRandomContacts(cp);
  addRandomForcesTrajs(cp);
  addRandomEffectorTrajectories(cp);
  return cp;
}

ContactPhase buildRandomContactPhase() {
  ContactPhase cp;
  addRandomPointsValues(cp);
  addRandomCurvesValues(cp);
  addRandomContacts(cp);
  addRandomForcesTrajs(cp);
  addRandomEffectorTrajectories(cp);
  return cp;
}

void explicitContactPhaseAssertEqual(ContactPhase& cp1, ContactPhase& cp2) {
  BOOST_CHECK(cp1.m_c_init == cp2.m_c_init);
  BOOST_CHECK(cp1.m_dc_init == cp2.m_dc_init);
  BOOST_CHECK(cp1.m_ddc_init == cp2.m_ddc_init);
  BOOST_CHECK(cp1.m_L_init == cp2.m_L_init);
  BOOST_CHECK(cp1.m_dL_init == cp2.m_dL_init);
  BOOST_CHECK(cp1.m_q_init == cp2.m_q_init);
  BOOST_CHECK(cp1.m_c_final == cp2.m_c_final);
  BOOST_CHECK(cp1.m_dc_final == cp2.m_dc_final);
  BOOST_CHECK(cp1.m_ddc_final == cp2.m_ddc_final);
  BOOST_CHECK(cp1.m_L_final == cp2.m_L_final);
  BOOST_CHECK(cp1.m_dL_final == cp2.m_dL_final);
  BOOST_CHECK(cp1.m_q_final == cp2.m_q_final);
  BOOST_CHECK(cp1.m_c_init == cp2.m_c_init);
  BOOST_CHECK(cp1.m_dc_init == cp2.m_dc_init);
  BOOST_CHECK(cp1.m_ddc_init == cp2.m_ddc_init);
  BOOST_CHECK(cp1.m_L_init == cp2.m_L_init);
  BOOST_CHECK(cp1.m_dL_init == cp2.m_dL_init);
  BOOST_CHECK(cp1.m_q_init == cp2.m_q_init);
  BOOST_CHECK(cp1.m_c_final == cp2.m_c_final);
  BOOST_CHECK(cp1.m_dc_final == cp2.m_dc_final);
  BOOST_CHECK(cp1.m_ddc_final == cp2.m_ddc_final);
  BOOST_CHECK(cp1.m_L_final == cp2.m_L_final);
  BOOST_CHECK(cp1.m_dL_final == cp2.m_dL_final);
  BOOST_CHECK(cp1.m_q_final == cp2.m_q_final);
  BOOST_CHECK((*cp1.m_q)(cp2.m_q->min()) == (*cp2.m_q)(cp2.m_q->min()));
  BOOST_CHECK((*cp1.m_dq)(cp2.m_dq->min()) == (*cp2.m_dq)(cp2.m_dq->min()));
  BOOST_CHECK((*cp1.m_ddq)(cp2.m_ddq->min()) == (*cp2.m_ddq)(cp2.m_ddq->min()));
  BOOST_CHECK((*cp1.m_tau)(cp2.m_tau->min()) == (*cp2.m_tau)(cp2.m_tau->min()));
  BOOST_CHECK((*cp1.m_c)(cp2.m_c->min()) == (*cp2.m_c)(cp2.m_c->min()));
  BOOST_CHECK((*cp1.m_dc)(cp2.m_dc->min()) == (*cp2.m_dc)(cp2.m_dc->min()));
  BOOST_CHECK((*cp1.m_ddc)(cp2.m_ddc->min()) == (*cp2.m_ddc)(cp2.m_ddc->min()));
  BOOST_CHECK((*cp1.m_L)(cp2.m_L->min()) == (*cp2.m_L)(cp2.m_L->min()));
  BOOST_CHECK((*cp1.m_dL)(cp2.m_dL->min()) == (*cp2.m_dL)(cp2.m_dL->min()));
  BOOST_CHECK((*cp1.m_wrench)(cp2.m_wrench->min()) == (*cp2.m_wrench)(cp2.m_wrench->min()));
  BOOST_CHECK((*cp1.m_zmp)(cp2.m_zmp->min()) == (*cp2.m_zmp)(cp2.m_zmp->min()));
  BOOST_CHECK((*cp1.m_root)(cp2.m_root->min()).isApprox((*cp2.m_root)(cp2.m_root->min())));
  BOOST_CHECK(cp1.m_q->isApprox(cp2.m_q.get()));
  BOOST_CHECK(cp1.m_dq->isApprox(cp2.m_dq.get()));
  BOOST_CHECK(cp1.m_ddq->isApprox(cp2.m_ddq.get()));
  BOOST_CHECK(cp1.m_tau->isApprox(cp2.m_tau.get()));
  BOOST_CHECK(cp1.m_c->isApprox(cp2.m_c.get()));
  BOOST_CHECK(cp1.m_dc->isApprox(cp2.m_dc.get()));
  BOOST_CHECK(cp1.m_ddc->isApprox(cp2.m_ddc.get()));
  BOOST_CHECK(cp1.m_L->isApprox(cp2.m_L.get()));
  BOOST_CHECK(cp1.m_dL->isApprox(cp2.m_dL.get()));
  BOOST_CHECK(cp1.m_wrench->isApprox(cp2.m_wrench.get()));
  BOOST_CHECK(cp1.m_zmp->isApprox(cp2.m_zmp.get()));
  BOOST_CHECK(cp1.m_root->isApprox(cp2.m_root.get()));
  BOOST_CHECK(cp1.contactForces() == cp2.contactForces());
  BOOST_CHECK(cp1.contactNormalForces() == cp2.contactNormalForces());
  BOOST_CHECK(cp1.effectorTrajectories() == cp2.effectorTrajectories());
  BOOST_CHECK(cp1.effectorsInContact() == cp2.effectorsInContact());
  BOOST_CHECK(cp1.contactPatches() == cp2.contactPatches());
  const ContactPhase::t_strings eeNames = cp2.effectorsInContact();
  for (ContactPhase::t_strings::const_iterator ee = eeNames.begin(); ee != eeNames.end(); ++ee) {
    BOOST_CHECK(cp2.isEffectorInContact(*ee));
    BOOST_CHECK(cp1.contactPatch(*ee) == cp2.contactPatch(*ee));
    if (cp1.contactForces().count(*ee)) {
      BOOST_CHECK(cp2.contactForces().count(*ee));
      BOOST_CHECK(cp1.contactForces(*ee)->isApprox(cp2.contactForces(*ee).get()));
      BOOST_CHECK((*cp1.contactForces(*ee))(cp1.contactForces(*ee)->min()) ==
                  (*cp2.contactForces(*ee))(cp2.contactForces(*ee)->min()));
      BOOST_CHECK((*cp1.contactForces(*ee))(cp1.contactForces(*ee)->max()) ==
                  (*cp2.contactForces(*ee))(cp2.contactForces(*ee)->max()));
    }
    if (cp1.contactNormalForces().count(*ee)) {
      BOOST_CHECK(cp2.contactNormalForces().count(*ee));
      BOOST_CHECK(cp1.contactNormalForces(*ee)->isApprox(cp2.contactNormalForces(*ee).get()));
      BOOST_CHECK((*cp1.contactNormalForces(*ee))(cp1.contactNormalForces(*ee)->min()) ==
                  (*cp2.contactNormalForces(*ee))(cp2.contactNormalForces(*ee)->min()));
      BOOST_CHECK((*cp1.contactNormalForces(*ee))(cp1.contactNormalForces(*ee)->max()) ==
                  (*cp2.contactNormalForces(*ee))(cp2.contactNormalForces(*ee)->max()));
    }
  }
  const ContactPhase::CurveSE3Map_t trajMap = cp2.effectorTrajectories();
  for (ContactPhase::CurveSE3Map_t::const_iterator mit = trajMap.begin(); mit != trajMap.end(); ++mit) {
    BOOST_CHECK(cp2.effectorTrajectories().count(mit->first));
    BOOST_CHECK(cp1.effectorTrajectories(mit->first)->isApprox(cp2.effectorTrajectories(mit->first).get()));
    BOOST_CHECK((*cp1.effectorTrajectories(mit->first))(cp1.effectorTrajectories(mit->first)->min())
                    .isApprox((*cp2.effectorTrajectories(mit->first))(cp2.effectorTrajectories(mit->first)->min())));
    BOOST_CHECK((*cp1.effectorTrajectories(mit->first))(cp1.effectorTrajectories(mit->first)->max())
                    .isApprox((*cp2.effectorTrajectories(mit->first))(cp2.effectorTrajectories(mit->first)->max())));
  }
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_model) {
  ContactModel mp;
  BOOST_CHECK(mp.m_mu == -1.);
  BOOST_CHECK(mp.m_contact_type == ContactType::CONTACT_UNDEFINED);
  BOOST_CHECK(mp.num_contact_points() == 1);
  BOOST_CHECK(mp.contact_points_positions().cols() == 1);
  BOOST_CHECK(mp.contact_points_positions().isZero());

  const double mu = 0.3;
  ContactModel mp_mu(mu);
  BOOST_CHECK(mp_mu.m_mu == mu);
  BOOST_CHECK(mp_mu.m_contact_type == ContactType::CONTACT_UNDEFINED);
  BOOST_CHECK(mp_mu.num_contact_points() == 1);
  BOOST_CHECK(mp_mu.contact_points_positions().cols() == 1);
  BOOST_CHECK(mp_mu.contact_points_positions().isZero());

  ContactModel mp1(mu, ContactType::CONTACT_PLANAR);
  BOOST_CHECK(mp1.m_mu == mu);
  BOOST_CHECK(mp1.m_contact_type == ContactType::CONTACT_PLANAR);
  BOOST_CHECK(mp1.num_contact_points() == 1);
  BOOST_CHECK(mp1.contact_points_positions().cols() == 1);
  BOOST_CHECK(mp1.contact_points_positions().isZero());

  ContactModel mp2(mp1);
  BOOST_CHECK(mp2.m_mu == mu);
  BOOST_CHECK(mp2.m_contact_type == ContactType::CONTACT_PLANAR);
  BOOST_CHECK(mp2.num_contact_points() == 1);
  BOOST_CHECK(mp2.contact_points_positions().cols() == 1);
  BOOST_CHECK(mp2.contact_points_positions().isZero());
}

BOOST_AUTO_TEST_CASE(contact_model_points_positions) {
  const double mu = 0.3;
  ContactModel mp(mu, ContactType::CONTACT_PLANAR);

  mp.num_contact_points(4);
  BOOST_CHECK_EQUAL(mp.num_contact_points(), 4);
  BOOST_CHECK_EQUAL(mp.contact_points_positions().cols(), 4);
  BOOST_CHECK(mp.contact_points_positions().isZero());

  Matrix3X positions = Matrix3X::Random(3, 6);
  mp.contact_points_positions(positions);
  BOOST_CHECK_EQUAL(mp.num_contact_points(), 6);
  BOOST_CHECK_EQUAL(mp.contact_points_positions().cols(), 6);
  BOOST_CHECK(mp.contact_points_positions().isApprox(positions));

  mp.num_contact_points(2);
  BOOST_CHECK_EQUAL(mp.num_contact_points(), 2);
  BOOST_CHECK_EQUAL(mp.contact_points_positions().cols(), 2);
  BOOST_CHECK(mp.contact_points_positions().isZero());
}

BOOST_AUTO_TEST_CASE(contact_model_operator_equal) {
  ContactModel mp1(0.3, ContactType::CONTACT_PLANAR);
  Matrix3X positions = Matrix3X::Random(3, 4);
  mp1.contact_points_positions(positions);

  ContactModel mp2(0.3, ContactType::CONTACT_PLANAR);
  mp2.contact_points_positions(positions);

  ContactModel mp3(mp1);

  BOOST_CHECK(mp1 == mp2);
  BOOST_CHECK(mp1 == mp3);

  ContactModel mp_n1(0.3, ContactType::CONTACT_PLANAR);
  ContactModel mp_n2(1., ContactType::CONTACT_PLANAR);
  mp1.contact_points_positions(positions);
  ContactModel mp_n3(0.3, ContactType::CONTACT_UNDEFINED);
  mp1.contact_points_positions(positions);
  ContactModel mp_n4(0.3, ContactType::CONTACT_PLANAR);
  mp_n4.num_contact_points(4);

  BOOST_CHECK(mp1 != mp_n1);
  BOOST_CHECK(mp1 != mp_n2);
  BOOST_CHECK(mp1 != mp_n3);
  BOOST_CHECK(mp1 != mp_n4);
}

BOOST_AUTO_TEST_CASE(contact_model_serialization) {
  ContactModel mp1(0.3, ContactType::CONTACT_PLANAR);
  Matrix3X positions = Matrix3X::Random(3, 4);
  mp1.contact_points_positions(positions);

  std::string fileName("fileTest_contactModel");
  mp1.saveAsText(fileName + ".txt");
  ContactModel mp_from_text;
  mp_from_text.loadFromText(fileName + ".txt");
  BOOST_CHECK(mp1 == mp_from_text);

  mp1.saveAsXML(fileName + ".xml", "ContactModel");
  ContactModel mp_from_xml;
  mp_from_xml.loadFromXML(fileName + ".xml", "ContactModel");
  BOOST_CHECK(mp1 == mp_from_xml);

  mp1.saveAsBinary(fileName);
  ContactModel mp_from_bin;
  mp_from_bin.loadFromBinary(fileName);
  BOOST_CHECK(mp1 == mp_from_bin);
}

BOOST_AUTO_TEST_CASE(contact_patch) {
  // check default constructor :
  ContactPatch cp;
  BOOST_CHECK(cp.placement() == SE3::Identity());
  BOOST_CHECK(cp.friction() == -1.);
  SE3 p = SE3::Identity();
  p.setRandom();
  cp.placement() = p;
  BOOST_CHECK(cp.placement() == p);
  cp.friction() = 0.7;
  BOOST_CHECK(cp.friction() == 0.7);

  // constructor with placement :
  p.setRandom();
  ContactPatch cp1(p);
  BOOST_CHECK(cp1.placement() == p);
  BOOST_CHECK(cp1.friction() == -1.);

  // constructor with placement and friction
  p.setRandom();
  ContactPatch cp2(p, 0.9);
  BOOST_CHECK(cp2.placement() == p);
  BOOST_CHECK(cp2.friction() == 0.9);

  // check comparison operator
  BOOST_CHECK(cp1 != cp2);
  ContactPatch cp3(p, 0.9);
  BOOST_CHECK(cp3 == cp2);
  cp2.friction() = 0.1;
  BOOST_CHECK(cp3 != cp2);

  // copy constructor
  ContactPatch cp4(cp3);
  BOOST_CHECK(cp4 == cp3);
  BOOST_CHECK(cp4.placement() == p);
  BOOST_CHECK(cp4.friction() == 0.9);
  cp4.placement() = SE3::Identity();
  BOOST_CHECK(cp3 != cp4);

  // operator =
  ContactPatch cp5 = cp4;
  BOOST_CHECK(cp4 == cp5);
  cp5.friction() = 2.;
  BOOST_CHECK(cp4 != cp5);

  // serialization :
  std::string fileName("fileTest");
  cp3.saveAsText(fileName);
  ContactPatch cp_from_text;
  cp_from_text.loadFromText(fileName);
  BOOST_CHECK(cp3 == cp_from_text);

  cp3.saveAsXML(fileName, "ContactPatch");
  ContactPatch cp_from_xml;
  cp_from_xml.loadFromXML(fileName, "ContactPatch");
  BOOST_CHECK(cp3 == cp_from_xml);

  cp3.saveAsBinary(fileName);
  ContactPatch cp_from_bin;
  cp_from_bin.loadFromBinary(fileName);
  BOOST_CHECK(cp3 == cp_from_bin);
}

BOOST_AUTO_TEST_CASE(contact_phase) {
  // default constructor
  ContactPhase cp1;
  // test timings setter/getter
  BOOST_CHECK(cp1.timeInitial() == -1);
  BOOST_CHECK(cp1.timeFinal() == -1);
  BOOST_CHECK(cp1.duration() == 0);
  // cp1.timeInitial() = 2; // should not compile : only const return for times
  // cp1.timeInitial() = 2; // should not compile : only const return for times
  cp1.timeInitial(0.5);
  cp1.timeFinal(2.);
  BOOST_CHECK(cp1.timeInitial() == 0.5);
  BOOST_CHECK(cp1.timeFinal() == 2.);
  BOOST_CHECK(cp1.duration() == 1.5);
  cp1.duration(3.);
  BOOST_CHECK(cp1.timeInitial() == 0.5);
  BOOST_CHECK(cp1.timeFinal() == 3.5);
  BOOST_CHECK(cp1.duration() == 3.);
  BOOST_CHECK(cp1.numContacts() == 0);

  // check exeptions related to wrong timing
  BOOST_CHECK_THROW(cp1.timeFinal(0.1), std::invalid_argument);
  BOOST_CHECK_THROW(cp1.duration(-2.), std::invalid_argument);
  // constructor with timings, should throw :
  BOOST_CHECK_THROW(ContactPhase(0.9, 0.1), std::invalid_argument);
  BOOST_CHECK_THROW(ContactPhase(0., -1), std::invalid_argument);

  // Constructor with timings :
  ContactPhase cp2(0.5, 2.);
  BOOST_CHECK(cp2.timeInitial() == 0.5);
  BOOST_CHECK(cp2.timeFinal() == 2.);
  BOOST_CHECK(cp2.duration() == 1.5);

  // Check contacts creation / removal
  ContactPatch patchL(SE3::Identity().setRandom());
  bool newContact = cp2.addContact("left_leg", patchL);
  BOOST_CHECK(newContact);
  BOOST_CHECK(cp2.isEffectorInContact("left_leg"));
  BOOST_CHECK(!cp2.isEffectorInContact("other"));
  BOOST_CHECK(cp2.contactPatch("left_leg") == patchL);
  BOOST_CHECK(cp2.contactPatches()["left_leg"] == patchL);
  BOOST_CHECK(cp2.numContacts() == 1);
  BOOST_CHECK_THROW(cp2.contactPatch("other"), std::invalid_argument);

  ContactPatch patchR(SE3::Identity().setRandom());
  newContact = cp2.addContact("right_leg", patchR);
  BOOST_CHECK(newContact);
  BOOST_CHECK(cp2.isEffectorInContact("left_leg"));
  BOOST_CHECK(cp2.isEffectorInContact("right_leg"));
  BOOST_CHECK(cp2.contactPatch("left_leg") == patchL);
  BOOST_CHECK(cp2.contactPatch("right_leg") == patchR);
  BOOST_CHECK(cp2.contactPatches()["left_leg"] == patchL);
  BOOST_CHECK(cp2.contactPatches()["right_leg"] == patchR);

  cp2.contactPatches().insert(
      std::pair<std::string, ContactPatch>("other", patchR));  // Why does it compile ? there is only const getter
  BOOST_CHECK(cp2.contactPatches().count("other") == 0);  // previous line should have no effect, only const getter
  BOOST_CHECK(!cp2.isEffectorInContact("other"));
  ContactPatch patchR2(SE3::Identity().setRandom(), 1.5);
  //  cp2.contactPatches

  newContact = cp2.addContact("left_leg", ContactPatch(SE3::Identity().setRandom(), 0.5));
  BOOST_CHECK(!newContact);
  BOOST_CHECK(cp2.isEffectorInContact("left_leg"));
  BOOST_CHECK(cp2.isEffectorInContact("right_leg"));
  BOOST_CHECK(cp2.contactPatch("left_leg") != patchL);
  BOOST_CHECK(cp2.numContacts() == 2);

  bool exist = cp2.removeContact("right_leg");
  BOOST_CHECK(exist);
  BOOST_CHECK(cp2.isEffectorInContact("left_leg"));
  BOOST_CHECK(!cp2.isEffectorInContact("right_leg"));
  BOOST_CHECK(cp2.numContacts() == 1);
  exist = cp2.removeContact("other");
  BOOST_CHECK(!exist);
  exist = cp2.removeContact("right_leg");
  BOOST_CHECK(!exist);

  // check adding forces trajectory :
  pointX_t f0 = point12_t::Random();
  pointX_t f1 = point12_t::Random();
  const double min = 0.5;
  const double max = 2.;
  curve_ptr_t force12d(new curves::polynomial_t(f0, f1, min, max));
  bool newTraj = cp2.addContactForceTrajectory("left_leg", force12d);
  BOOST_CHECK(newTraj);
  newTraj = cp2.addContactForceTrajectory("left_leg", force12d);
  BOOST_CHECK(!newTraj);
  BOOST_CHECK((*cp2.contactForces()["left_leg"])(min) == f0);
  BOOST_CHECK((*cp2.contactForces()["left_leg"])(max) == (*force12d)(max));
  BOOST_CHECK((*cp2.contactForces()["left_leg"])(1.2) == (*force12d)(1.2));
  BOOST_CHECK((*cp2.contactForces("left_leg"))(min) == f0);
  BOOST_CHECK((*cp2.contactForces("left_leg"))(max) == (*force12d)(max));
  BOOST_CHECK((*cp2.contactForces("left_leg"))(1.2) == (*force12d)(1.2));
  pointX_t f0_1 = point12_t::Random();
  pointX_t f1_1 = point12_t::Random();
  curve_ptr_t force12d_1(new curves::polynomial_t(f0_1, f1_1, min, max));
  cp2.contactForces().insert(
      std::pair<std::string, curve_ptr_t>("right_leg", force12d_1));  // should not have any effect only const getter
  BOOST_CHECK(cp2.contactForces().count("right_leg") == 0);
  cp2.contactForces().erase("left_leg");  // should not have any effect
  BOOST_CHECK((*cp2.contactForces("left_leg"))(min) == f0);
  BOOST_CHECK_THROW(cp2.addContactForceTrajectory("right_leg", force12d),
                    std::invalid_argument);  // right leg is not in contact, cannot add force
  BOOST_CHECK_THROW(cp2.contactForces("right_leg"), std::invalid_argument);

  // check with piecewise curve :
  curve_ptr_t force_C2 = buildPiecewisePolynomialC2();
  cp2.addContactForceTrajectory("left_leg", force_C2);
  BOOST_CHECK((*cp2.contactForces("left_leg"))(1.) == (*force_C2)(1.));
  BOOST_CHECK((*cp2.contactForces("left_leg"))(10.) == (*force_C2)(10.));
  BOOST_CHECK((*cp2.contactForces("left_leg"))(5.7) == (*force_C2)(5.7));

  pointX_t nf0(1);
  nf0 << 56.3;
  pointX_t nf1(1);
  nf1 << 5893.2;
  curve_ptr_t force1d(new curves::polynomial_t(nf0, nf1, min, max));
  newTraj = cp2.addContactNormalForceTrajectory("left_leg", force1d);
  BOOST_CHECK(newTraj);
  newTraj = cp2.addContactNormalForceTrajectory("left_leg", force1d);
  BOOST_CHECK(!newTraj);
  BOOST_CHECK((*cp2.contactNormalForces()["left_leg"])(min) == nf0);
  BOOST_CHECK((*cp2.contactNormalForces()["left_leg"])(max) == (*force1d)(max));
  BOOST_CHECK((*cp2.contactNormalForces()["left_leg"])(1.2) == (*force1d)(1.2));
  BOOST_CHECK((*cp2.contactNormalForces("left_leg"))(min) == nf0);
  BOOST_CHECK((*cp2.contactNormalForces("left_leg"))(max) == (*force1d)(max));
  BOOST_CHECK((*cp2.contactNormalForces("left_leg"))(1.2) == (*force1d)(1.2));
  pointX_t nf0_1(1);
  nf0_1 << 147.2;
  pointX_t nf1_1(1);
  nf1_1 << 562;
  curve_ptr_t force1d_1(new curves::polynomial_t(nf0_1, nf1_1, min, max));
  cp2.contactNormalForces().insert(
      std::pair<std::string, curve_ptr_t>("right_leg", force1d_1));  // should not have any effect only const getter
  BOOST_CHECK(cp2.contactNormalForces().count("right_leg") == 0);
  cp2.contactNormalForces().erase("left_leg");  // should not have any effect
  BOOST_CHECK((*cp2.contactNormalForces("left_leg"))(min) == nf0);
  BOOST_CHECK_THROW(cp2.contactNormalForces("right_leg"), std::invalid_argument);
  BOOST_CHECK_THROW(cp2.addContactNormalForceTrajectory("right_leg", force12d),
                    std::invalid_argument);  // right leg is not in contact, cannot add force
  BOOST_CHECK_THROW(cp2.addContactNormalForceTrajectory("left_leg", force12d),
                    std::invalid_argument);  // should be of dimension 1

  // Check effector trajectory :
  curve_SE3_ptr_t effR = buildPiecewiseSE3();
  newTraj = cp2.addEffectorTrajectory("right_leg", effR);
  BOOST_CHECK(newTraj);
  newTraj = cp2.addEffectorTrajectory("right_leg", effR);
  BOOST_CHECK(!newTraj);
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(min).isApprox((*effR)(min)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(max).isApprox((*effR)(max)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(1.2).isApprox((*effR)(1.2)));
  BOOST_CHECK((*cp2.effectorTrajectories("right_leg"))(min).isApprox((*effR)(min)));
  BOOST_CHECK((*cp2.effectorTrajectories("right_leg"))(max).isApprox((*effR)(max)));
  BOOST_CHECK((*cp2.effectorTrajectories("right_leg"))(1.2).isApprox((*effR)(1.2)));
  BOOST_CHECK_THROW(cp2.effectorTrajectories("left_leg"), std::invalid_argument);
  BOOST_CHECK_THROW(cp2.addEffectorTrajectory("left_leg", effR),
                    std::invalid_argument);  // right leg is not in contact, cannot add force
  // cp2.addEffectorTrajectory("right_leg",force12d); // should not compile : not the right return type for the curve

  // check setting init / final data :
  // public members :
  /*
  point3_t m_c_init;
  point3_t m_dc_init;
  point3_t m_ddc_init;
  point3_t m_L_init;
  point3_t m_dL_init;
  pointX_t m_q_init;
  point3_t m_c_final;
  point3_t m_dc_final;
  point3_t m_ddc_final;
  point3_t m_L_final;
  point3_t m_dL_final;
  pointX_t m_q_final;
  */

  point3_t c_init = point3_t::Random();
  point3_t dc_init = point3_t::Random();
  pointX_t ddc_init = point3_t::Random();  // test with point X for automatic conversion
  pointX_t L_init = point3_t::Random();
  point3_t dL_init = point3_t::Random();
  pointX_t q_init = point12_t::Random();
  pointX_t c_final = point3_t::Random();
  pointX_t dc_final = point3_t::Random();
  point3_t ddc_final = point3_t::Random();
  pointX_t L_final = point3_t::Random();
  point3_t dL_final = point3_t::Random();
  pointX_t q_final = point12_t::Random();
  cp2.m_c_init = c_init;
  cp2.m_dc_init = dc_init;
  cp2.m_ddc_init = ddc_init;
  cp2.m_L_init = L_init;
  cp2.m_dL_init = dL_init;
  cp2.m_q_init = q_init;
  cp2.m_c_final = c_final;
  cp2.m_dc_final = dc_final;
  cp2.m_ddc_final = ddc_final;
  cp2.m_L_final = L_final;
  cp2.m_dL_final = dL_final;
  cp2.m_q_final = q_final;
  BOOST_CHECK(cp2.m_c_init == c_init);
  BOOST_CHECK(cp2.m_dc_init == dc_init);
  BOOST_CHECK(cp2.m_ddc_init == ddc_init);
  BOOST_CHECK(cp2.m_L_init == L_init);
  BOOST_CHECK(cp2.m_dL_init == dL_init);
  BOOST_CHECK(cp2.m_q_init == q_init);
  BOOST_CHECK(cp2.m_c_final == c_final);
  BOOST_CHECK(cp2.m_dc_final == dc_final);
  BOOST_CHECK(cp2.m_ddc_final == ddc_final);
  BOOST_CHECK(cp2.m_L_final == L_final);
  BOOST_CHECK(cp2.m_dL_final == dL_final);
  BOOST_CHECK(cp2.m_q_final == q_final);

  // check adding trajectory to public members :
  /*
    curve_ptr m_q;
    curve_ptr m_dq;
    curve_ptr m_ddq;
    curve_ptr m_tau;
    curve_ptr m_c;
    curve_ptr m_dc;
    curve_ptr m_ddc;
    curve_ptr m_L;
    curve_ptr m_dL;
    curve_ptr m_wrench;
    curve_ptr m_zmp;
    curve_SE3_ptr_t m_root;
    */
  {  // inner scope to check that the curves are not destroyed
    curve_ptr_t q = buildRandomPolynomial12D();
    curve_ptr_t dq = buildRandomPolynomial12D();
    curve_ptr_t ddq = buildRandomPolynomial12D();
    curve_ptr_t tau = buildRandomPolynomial12D();
    curve_ptr_t c = buildRandomPolynomial3D();
    curve_ptr_t dc = buildRandomPolynomial3D();
    curve_ptr_t ddc = buildRandomPolynomial3D();
    curve_ptr_t L = buildRandomPolynomial3D();
    curve_ptr_t dL = buildRandomPolynomial3D();
    curve_ptr_t wrench = buildRandomPolynomial3D();
    curve_ptr_t zmp = buildRandomPolynomial3D();
    curve_SE3_ptr_t root = buildRandomSE3LinearTraj(1, 5.5);
    cp2.m_q = q;
    cp2.m_dq = dq;
    cp2.m_ddq = ddq;
    cp2.m_tau = tau;
    cp2.m_c = c;
    cp2.m_dc = dc;
    cp2.m_ddc = ddc;
    cp2.m_L = L;
    cp2.m_dL = dL;
    cp2.m_wrench = wrench;
    cp2.m_zmp = zmp;
    cp2.m_root = root;

    BOOST_CHECK((*cp2.m_q)(q->min()) == (*q)(q->min()));
    BOOST_CHECK((*cp2.m_dq)(dq->min()) == (*dq)(dq->min()));
    BOOST_CHECK((*cp2.m_ddq)(ddq->min()) == (*ddq)(ddq->min()));
    BOOST_CHECK((*cp2.m_tau)(tau->min()) == (*tau)(tau->min()));
    BOOST_CHECK((*cp2.m_c)(c->min()) == (*c)(c->min()));
    BOOST_CHECK((*cp2.m_dc)(dc->min()) == (*dc)(dc->min()));
    BOOST_CHECK((*cp2.m_ddc)(ddc->min()) == (*ddc)(ddc->min()));
    BOOST_CHECK((*cp2.m_L)(L->min()) == (*L)(L->min()));
    BOOST_CHECK((*cp2.m_dL)(dL->min()) == (*dL)(dL->min()));
    BOOST_CHECK((*cp2.m_wrench)(wrench->min()) == (*wrench)(wrench->min()));
    BOOST_CHECK((*cp2.m_zmp)(zmp->min()) == (*zmp)(zmp->min()));
    BOOST_CHECK((*cp2.m_root)(root->min()).isApprox((*root)(root->min())));
    BOOST_CHECK((*cp2.m_q)(q->max()) == (*q)(q->max()));
    BOOST_CHECK((*cp2.m_dq)(dq->max()) == (*dq)(dq->max()));
    BOOST_CHECK((*cp2.m_ddq)(ddq->max()) == (*ddq)(ddq->max()));
    BOOST_CHECK((*cp2.m_tau)(tau->max()) == (*tau)(tau->max()));
    BOOST_CHECK((*cp2.m_c)(c->max()) == (*c)(c->max()));
    BOOST_CHECK((*cp2.m_dc)(dc->max()) == (*dc)(dc->max()));
    BOOST_CHECK((*cp2.m_ddc)(ddc->max()) == (*ddc)(ddc->max()));
    BOOST_CHECK((*cp2.m_L)(L->max()) == (*L)(L->max()));
    BOOST_CHECK((*cp2.m_dL)(dL->max()) == (*dL)(dL->max()));
    BOOST_CHECK((*cp2.m_wrench)(wrench->max()) == (*wrench)(wrench->max()));
    BOOST_CHECK((*cp2.m_zmp)(zmp->max()) == (*zmp)(zmp->max()));
    BOOST_CHECK((*cp2.m_root)(root->max()).isApprox((*root)(root->max())));
  }
  BOOST_CHECK_NO_THROW(
      cp2.m_q->operator()(cp2.m_q->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_dq->operator()(cp2.m_dq->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_ddq->operator()(cp2.m_ddq->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_tau->operator()(cp2.m_tau->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_c->operator()(cp2.m_c->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_dc->operator()(cp2.m_dc->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_ddc->operator()(cp2.m_ddc->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_L->operator()(cp2.m_L->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_dL->operator()(cp2.m_dL->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_wrench->operator()(cp2.m_wrench->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_root->operator()(cp2.m_root->min()));  // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(
      cp2.m_zmp->operator()(cp2.m_zmp->min()));  // check that the curves still exist after leaving the scope

  // add more contact and trajectories :
  cp2.addContact("right_hand", ContactPatch(SE3::Identity().setRandom()));
  cp2.addContactForceTrajectory("right_hand", buildRandomPolynomial12D());
  cp2.addContactNormalForceTrajectory("right_hand", buildRandomPolynomial1D());
  int num_ctc = 0;
  for (ContactPhase::CurveMap_t::const_iterator mit = cp2.contactForces().begin(); mit != cp2.contactForces().end();
       ++mit) {
    BOOST_CHECK(mit->first == "right_hand" || mit->first == "left_leg");
    num_ctc++;
  }
  BOOST_CHECK(num_ctc == 2);

  curve_SE3_ptr_t eff_knee(buildRandomSE3LinearTraj(cp2.timeInitial(), cp2.timeFinal()));
  double min_knee = eff_knee->min();
  double max_knee = eff_knee->max();
  newTraj = cp2.addEffectorTrajectory("knee", eff_knee);
  BOOST_CHECK(newTraj);
  BOOST_CHECK((*cp2.effectorTrajectories()["knee"])(min_knee).isApprox((*eff_knee)(min_knee)));
  BOOST_CHECK((*cp2.effectorTrajectories()["knee"])(max_knee).isApprox((*eff_knee)(max_knee)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(min).isApprox((*effR)(min)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(max).isApprox((*effR)(max)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(1.2).isApprox((*effR)(1.2)));
  BOOST_CHECK(cp2.effectorTrajectories().size() == 2);
  int num_eff_traj = 0;
  const ContactPhase::CurveSE3Map_t trajMap = cp2.effectorTrajectories();
  for (ContactPhase::CurveSE3Map_t::const_iterator mit = trajMap.begin(); mit != trajMap.end(); ++mit) {
    BOOST_CHECK(mit->first == "knee" || mit->first == "right_leg");
    num_eff_traj++;
  }
  BOOST_CHECK(num_eff_traj == 2);
  BOOST_CHECK(cp2.effectorsWithTrajectory().size() == 2);
  BOOST_CHECK(cp2.effectorHaveAtrajectory("knee"));
  BOOST_CHECK(cp2.effectorHaveAtrajectory("right_leg"));
  BOOST_CHECK(!cp2.effectorHaveAtrajectory("left_leg"));

  // check copy constructor and operator ==
  ContactPhase cp4(cp2);
  ContactPhase cp5 = cp2;
  BOOST_CHECK(cp2 != cp1);
  BOOST_CHECK(cp4 == cp2);
  BOOST_CHECK(cp5 == cp2);
  BOOST_CHECK(cp4.m_c_init == c_init);
  BOOST_CHECK(cp4.m_dc_init == dc_init);
  BOOST_CHECK(cp4.m_ddc_init == ddc_init);
  BOOST_CHECK(cp4.m_L_init == L_init);
  BOOST_CHECK(cp4.m_dL_init == dL_init);
  BOOST_CHECK(cp4.m_q_init == q_init);
  BOOST_CHECK(cp4.m_c_final == c_final);
  BOOST_CHECK(cp4.m_dc_final == dc_final);
  BOOST_CHECK(cp4.m_ddc_final == ddc_final);
  BOOST_CHECK(cp4.m_L_final == L_final);
  BOOST_CHECK(cp4.m_dL_final == dL_final);
  BOOST_CHECK(cp4.m_q_final == q_final);
  BOOST_CHECK(cp5.m_c_init == c_init);
  BOOST_CHECK(cp5.m_dc_init == dc_init);
  BOOST_CHECK(cp5.m_ddc_init == ddc_init);
  BOOST_CHECK(cp5.m_L_init == L_init);
  BOOST_CHECK(cp5.m_dL_init == dL_init);
  BOOST_CHECK(cp5.m_q_init == q_init);
  BOOST_CHECK(cp5.m_c_final == c_final);
  BOOST_CHECK(cp5.m_dc_final == dc_final);
  BOOST_CHECK(cp5.m_ddc_final == ddc_final);
  BOOST_CHECK(cp5.m_L_final == L_final);
  BOOST_CHECK(cp5.m_dL_final == dL_final);
  BOOST_CHECK(cp5.m_q_final == q_final);
  BOOST_CHECK((*cp5.m_q)(cp2.m_q->min()) == (*cp2.m_q)(cp2.m_q->min()));
  BOOST_CHECK((*cp5.m_dq)(cp2.m_dq->min()) == (*cp2.m_dq)(cp2.m_dq->min()));
  BOOST_CHECK((*cp5.m_ddq)(cp2.m_ddq->min()) == (*cp2.m_ddq)(cp2.m_ddq->min()));
  BOOST_CHECK((*cp5.m_tau)(cp2.m_tau->min()) == (*cp2.m_tau)(cp2.m_tau->min()));
  BOOST_CHECK((*cp5.m_c)(cp2.m_c->min()) == (*cp2.m_c)(cp2.m_c->min()));
  BOOST_CHECK((*cp5.m_dc)(cp2.m_dc->min()) == (*cp2.m_dc)(cp2.m_dc->min()));
  BOOST_CHECK((*cp5.m_ddc)(cp2.m_ddc->min()) == (*cp2.m_ddc)(cp2.m_ddc->min()));
  BOOST_CHECK((*cp5.m_L)(cp2.m_L->min()) == (*cp2.m_L)(cp2.m_L->min()));
  BOOST_CHECK((*cp5.m_dL)(cp2.m_dL->min()) == (*cp2.m_dL)(cp2.m_dL->min()));
  BOOST_CHECK((*cp5.m_wrench)(cp2.m_wrench->min()) == (*cp2.m_wrench)(cp2.m_wrench->min()));
  BOOST_CHECK((*cp5.m_zmp)(cp2.m_zmp->min()) == (*cp2.m_zmp)(cp2.m_zmp->min()));
  BOOST_CHECK((*cp5.m_root)(cp2.m_root->min()).isApprox((*cp2.m_root)(cp2.m_root->min())));
  BOOST_CHECK(cp5.contactForces() == cp2.contactForces());
  BOOST_CHECK(cp5.contactNormalForces() == cp2.contactNormalForces());
  BOOST_CHECK(cp5.effectorTrajectories() == cp2.effectorTrajectories());
  BOOST_CHECK(cp5.effectorsInContact() == cp2.effectorsInContact());
  BOOST_CHECK(cp5.contactPatches() == cp2.contactPatches());
  BOOST_CHECK(cp5.contactForces("left_leg") == cp2.contactForces("left_leg"));
  BOOST_CHECK(cp5.contactNormalForces("left_leg") == cp2.contactNormalForces("left_leg"));
  BOOST_CHECK(cp5.contactForces("right_hand") == cp2.contactForces("right_hand"));
  BOOST_CHECK(cp5.contactNormalForces("right_hand") == cp2.contactNormalForces("right_hand"));
  BOOST_CHECK(cp5.effectorTrajectories("right_leg") == cp2.effectorTrajectories("right_leg"));
  BOOST_CHECK(cp5.effectorTrajectories("knee") == cp2.effectorTrajectories("knee"));
  BOOST_CHECK(cp5.contactPatch("left_leg") == cp2.contactPatch("left_leg"));
  BOOST_CHECK(cp5.contactPatch("right_hand") == cp2.contactPatch("right_hand"));

  // check operator != and ==
  pointX_t p = point3_t::Random();
  cp5.m_c_init = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_c_init == p);
  BOOST_CHECK(cp2.m_c_init != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point3_t::Random();
  cp5.m_dc_init = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_dc_init == p);
  BOOST_CHECK(cp2.m_dc_init != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point3_t::Random();
  cp5.m_ddc_init = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_ddc_init == p);
  BOOST_CHECK(cp2.m_ddc_init != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point3_t::Random();
  cp5.m_L_init = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_L_init == p);
  BOOST_CHECK(cp2.m_L_init != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point3_t::Random();
  cp5.m_dL_init = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_dL_init == p);
  BOOST_CHECK(cp2.m_dL_init != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point12_t::Random();
  cp5.m_q_init = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_q_init == p);
  BOOST_CHECK(cp2.m_q_init != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point3_t::Random();
  cp5.m_c_final = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_c_final == p);
  BOOST_CHECK(cp2.m_c_final != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point3_t::Random();
  cp5.m_dc_final = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_dc_final == p);
  BOOST_CHECK(cp2.m_dc_final != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point3_t::Random();
  cp5.m_ddc_final = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_ddc_final == p);
  BOOST_CHECK(cp2.m_ddc_final != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point3_t::Random();
  cp5.m_L_final = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_L_final == p);
  BOOST_CHECK(cp2.m_L_final != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point3_t::Random();
  cp5.m_dL_final = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_dL_final == p);
  BOOST_CHECK(cp2.m_dL_final != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  p = point12_t::Random();
  cp5.m_q_final = p;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_q_final == p);
  BOOST_CHECK(cp2.m_q_final != p);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);

  curve_ptr_t curve;
  curve = buildRandomPolynomial12D();
  cp5.m_q = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_q == curve);
  BOOST_CHECK(cp2.m_q != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial12D();
  cp5.m_dq = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_dq == curve);
  BOOST_CHECK(cp2.m_dq != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial12D();
  cp5.m_ddq = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_ddq == curve);
  BOOST_CHECK(cp2.m_ddq != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial12D();
  cp5.m_tau = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_tau == curve);
  BOOST_CHECK(cp2.m_tau != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial3D();
  cp5.m_c = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_c == curve);
  BOOST_CHECK(cp2.m_c != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial3D();
  cp5.m_dc = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_dc == curve);
  BOOST_CHECK(cp2.m_dc != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial3D();
  cp5.m_ddc = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_ddc == curve);
  BOOST_CHECK(cp2.m_ddc != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial3D();
  cp5.m_L = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_L == curve);
  BOOST_CHECK(cp2.m_L != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial3D();
  cp5.m_dL = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_dL == curve);
  BOOST_CHECK(cp2.m_dL != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial3D();
  cp5.m_wrench = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_wrench == curve);
  BOOST_CHECK(cp2.m_wrench != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve = buildRandomPolynomial3D();
  cp5.m_zmp = curve;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_zmp == curve);
  BOOST_CHECK(cp2.m_zmp != curve);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  curve_SE3_ptr_t curveSE3 = buildRandomSE3LinearTraj(1.5, 2.9);
  cp5.m_root = curveSE3;
  BOOST_CHECK(cp5 != cp2);
  BOOST_CHECK(cp5.m_root == curveSE3);
  BOOST_CHECK(cp2.m_root != curveSE3);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);

  cp5.removeContact("left_leg");
  BOOST_CHECK(cp5 != cp2);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  cp5.addContact("test", ContactPatch(SE3::Identity().setRandom()));
  BOOST_CHECK(cp5 != cp2);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  cp5.addEffectorTrajectory("knee", buildRandomSE3LinearTraj(cp5.timeInitial(), cp5.timeFinal()));
  BOOST_CHECK(cp5 != cp2);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  cp5.addContactForceTrajectory("left_leg", buildRandomPolynomial12D());
  BOOST_CHECK(cp5 != cp2);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  cp5.addContactNormalForceTrajectory("left_leg", buildRandomPolynomial1D());
  BOOST_CHECK(cp5 != cp2);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);

  // check that when removing the contact the trajectory are correctly deleted :
  cp5.removeContact("left_leg");
  BOOST_CHECK(!cp5.isEffectorInContact("left_leg"));
  BOOST_CHECK(cp5.contactForces().count("left_leg") == 0);
  BOOST_CHECK(cp5.contactNormalForces().count("left_leg") == 0);
  // check that when adding a contact the effector trajectory is correctly deleted :
  cp5.addContact("right_leg", patchR);
  BOOST_CHECK(cp5.effectorTrajectories().count("right_leg") == 0);

  // check serialization
  std::string fileName("fileTest");
  ContactPhase cp_from_txt, cp_from_xml, cp_from_bin;
  std::cout << "cp2 before serialization : " << std::endl << cp2 << std::endl;

  cp2.saveAsText(fileName + ".txt");
  cp_from_txt.loadFromText(fileName + ".txt");
  BOOST_CHECK(cp2 == cp_from_txt);
  std::cout << "cp2 after deserialization : " << std::endl << cp_from_txt << std::endl;

  cp2.saveAsXML(fileName + ".xml", "ContactPhase");
  cp_from_xml.loadFromXML(fileName + ".xml", "ContactPhase");
  BOOST_CHECK(cp2 == cp_from_xml);

  cp2.saveAsBinary(fileName);
  cp_from_bin.loadFromBinary(fileName);
  BOOST_CHECK(cp2 == cp_from_bin);

  explicitContactPhaseAssertEqual(cp2, cp_from_txt);
}

BOOST_AUTO_TEST_CASE(contact_phase_helpers_tarjectories) {
  point3_t p0(0., 0., 0.);
  point3_t p1(1., 2., 3.);
  point3_t p2(4., 4., 4.);
  point3_t p3(10., 10., 10.);
  point3_t d0(1., 1., 1.);
  point3_t d1(2., 2., 2.);
  point3_t d2(3., 3., 3.);
  point3_t d3(5., 5., 5.);
  point3_t dd0(1.5, 1.5, 1.5);
  point3_t dd1(2.5, 2.5, 2.5);
  point3_t dd2(3.5, 3.5, 3.5);
  point3_t dd3(5.5, 5.5, 5.5);
  double t0 = 1.0;
  double t1 = 1.5;
  double t2 = 3.0;
  double t3 = 10.0;
  t_pointX_t points;
  points.push_back(p0);
  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  t_pointX_t points_derivative;
  points_derivative.push_back(d0);
  points_derivative.push_back(d1);
  points_derivative.push_back(d2);
  points_derivative.push_back(d3);
  t_pointX_t points_second_derivative;
  points_second_derivative.push_back(dd0);
  points_second_derivative.push_back(dd1);
  points_second_derivative.push_back(dd2);
  points_second_derivative.push_back(dd3);
  std::vector<double> time_points;
  time_points.push_back(t0);
  time_points.push_back(t1);
  time_points.push_back(t2);
  time_points.push_back(t3);

  // check COM trajectory :
  ContactPhase cp;
  cp.setCOMtrajectoryFromPoints(points, points_derivative, points_second_derivative, time_points);
  BOOST_CHECK_EQUAL(cp.m_c->dim(), 3);
  BOOST_CHECK_EQUAL(cp.m_dc->dim(), 3);
  BOOST_CHECK_EQUAL(cp.m_ddc->dim(), 3);
  BOOST_CHECK_EQUAL(cp.m_c->min(), t0);
  BOOST_CHECK_EQUAL(cp.m_dc->min(), t0);
  BOOST_CHECK_EQUAL(cp.m_ddc->min(), t0);
  BOOST_CHECK_EQUAL(cp.m_c->max(), t3);
  BOOST_CHECK_EQUAL(cp.m_dc->max(), t3);
  BOOST_CHECK_EQUAL(cp.m_ddc->max(), t3);
  BOOST_CHECK((*cp.m_c)(t0).isApprox(p0));
  BOOST_CHECK((*cp.m_c)(t1).isApprox(p1));
  BOOST_CHECK((*cp.m_c)(t2).isApprox(p2));
  BOOST_CHECK((*cp.m_c)(t3).isApprox(p3));
  BOOST_CHECK((*cp.m_dc)(t0).isApprox(d0));
  BOOST_CHECK((*cp.m_dc)(t1).isApprox(d1));
  BOOST_CHECK((*cp.m_dc)(t2).isApprox(d2));
  BOOST_CHECK((*cp.m_dc)(t3).isApprox(d3));
  BOOST_CHECK((*cp.m_ddc)(t0).isApprox(dd0));
  BOOST_CHECK((*cp.m_ddc)(t1).isApprox(dd1));
  BOOST_CHECK((*cp.m_ddc)(t2).isApprox(dd2));
  BOOST_CHECK((*cp.m_ddc)(t3).isApprox(dd3));

  BOOST_CHECK(cp.m_c_init.isApprox(p0));
  BOOST_CHECK(cp.m_c_final.isApprox(p3));
  BOOST_CHECK(cp.m_dc_init.isApprox(d0));
  BOOST_CHECK(cp.m_dc_final.isApprox(d3));
  BOOST_CHECK(cp.m_ddc_init.isApprox(dd0));
  BOOST_CHECK(cp.m_ddc_final.isApprox(dd3));

  // check that init/final are not modified if they ere already set :
  // NOT THE CORRECT BEHAVIOR ANYMORE
  //  ContactPhase cp2 = buildRandomContactPhase(0, 2);
  //  point3_t c_init, c_final, dc_init, dc_final, ddc_init, ddc_final;
  //  c_init = cp2.m_c_init;
  //  dc_init = cp2.m_dc_init;
  //  ddc_init = cp2.m_ddc_init;
  //  c_final = cp2.m_c_final;
  //  dc_final = cp2.m_dc_final;
  //  ddc_final = cp2.m_ddc_final;
  //  cp2.setCOMtrajectoryFromPoints(points, points_derivative, points_second_derivative, time_points);
  //  BOOST_CHECK(cp2.m_c_init.isApprox(c_init));
  //  BOOST_CHECK(cp2.m_c_final.isApprox(c_final));
  //  BOOST_CHECK(cp2.m_dc_init.isApprox(dc_init));
  //  BOOST_CHECK(cp2.m_dc_final.isApprox(dc_final));
  //  BOOST_CHECK(cp2.m_ddc_init.isApprox(ddc_init));
  //  BOOST_CHECK(cp2.m_ddc_final.isApprox(ddc_final));
  //  BOOST_CHECK(!cp2.m_c_init.isApprox(p0));
  //  BOOST_CHECK(!cp2.m_c_final.isApprox(p3));
  //  BOOST_CHECK(!cp2.m_dc_init.isApprox(d0));
  //  BOOST_CHECK(!cp2.m_dc_final.isApprox(d3));
  //  BOOST_CHECK(!cp2.m_ddc_init.isApprox(dd0));
  //  BOOST_CHECK(!cp2.m_ddc_final.isApprox(dd3));

  // check AM trajectory

  cp.setAMtrajectoryFromPoints(points, points_derivative, time_points);
  BOOST_CHECK_EQUAL(cp.m_L->dim(), 3);
  BOOST_CHECK_EQUAL(cp.m_dL->dim(), 3);
  BOOST_CHECK_EQUAL(cp.m_L->min(), t0);
  BOOST_CHECK_EQUAL(cp.m_dL->min(), t0);
  BOOST_CHECK_EQUAL(cp.m_L->max(), t3);
  BOOST_CHECK_EQUAL(cp.m_dL->max(), t3);
  BOOST_CHECK((*cp.m_L)(t0).isApprox(p0));
  BOOST_CHECK((*cp.m_L)(t1).isApprox(p1));
  BOOST_CHECK((*cp.m_L)(t2).isApprox(p2));
  BOOST_CHECK((*cp.m_L)(t3).isApprox(p3));
  BOOST_CHECK((*cp.m_dL)(t0).isApprox(d0));
  BOOST_CHECK((*cp.m_dL)(t1).isApprox(d1));
  BOOST_CHECK((*cp.m_dL)(t2).isApprox(d2));
  BOOST_CHECK((*cp.m_dL)(t3).isApprox(d3));

  BOOST_CHECK(cp.m_L_init.isApprox(p0));
  BOOST_CHECK(cp.m_L_final.isApprox(p3));
  BOOST_CHECK(cp.m_dL_init.isApprox(d0));
  BOOST_CHECK(cp.m_dL_final.isApprox(d3));

  // check that init/final are not modified if they ere already set :
  // NOT THE CORRECT BEHAVIOR ANYMORE
  //  point3_t L_init, L_final, dL_init, dL_final;
  //  L_init = cp2.m_L_init;
  //  dL_init = cp2.m_dL_init;
  //  L_final = cp2.m_L_final;
  //  dL_final = cp2.m_dL_final;
  //  cp2.setAMtrajectoryFromPoints(points, points_derivative, time_points);
  //  BOOST_CHECK(cp2.m_L_init.isApprox(L_init));
  //  BOOST_CHECK(cp2.m_L_final.isApprox(L_final));
  //  BOOST_CHECK(cp2.m_dL_init.isApprox(dL_init));
  //  BOOST_CHECK(cp2.m_dL_final.isApprox(dL_final));
  //  BOOST_CHECK(!cp2.m_L_init.isApprox(p0));
  //  BOOST_CHECK(!cp2.m_L_final.isApprox(p3));
  //  BOOST_CHECK(!cp2.m_dL_init.isApprox(d0));
  //  BOOST_CHECK(!cp2.m_dL_final.isApprox(d3));

  // check q trajectory :
  pointX_t q0 = point12_t::Random();
  pointX_t q1 = point12_t::Random();
  pointX_t q2 = point12_t::Random();
  pointX_t q3 = point12_t::Random();
  pointX_t dq0 = point12_t::Random();
  pointX_t dq1 = point12_t::Random();
  pointX_t dq2 = point12_t::Random();
  pointX_t dq3 = point12_t::Random();
  pointX_t ddq0 = point12_t::Random();
  pointX_t ddq1 = point12_t::Random();
  pointX_t ddq2 = point12_t::Random();
  pointX_t ddq3 = point12_t::Random();
  t_pointX_t points_q;
  points_q.push_back(q0);
  points_q.push_back(q1);
  points_q.push_back(q2);
  points_q.push_back(q3);
  t_pointX_t points_derivative_q;
  points_derivative_q.push_back(dq0);
  points_derivative_q.push_back(dq1);
  points_derivative_q.push_back(dq2);
  points_derivative_q.push_back(dq3);
  t_pointX_t points_second_derivative_q;
  points_second_derivative_q.push_back(ddq0);
  points_second_derivative_q.push_back(ddq1);
  points_second_derivative_q.push_back(ddq2);
  points_second_derivative_q.push_back(ddq3);

  cp.setJointsTrajectoryFromPoints(points_q, points_derivative_q, points_second_derivative_q, time_points);
  BOOST_CHECK_EQUAL(cp.m_q->dim(), 12);
  BOOST_CHECK_EQUAL(cp.m_dq->dim(), 12);
  BOOST_CHECK_EQUAL(cp.m_ddq->dim(), 12);
  BOOST_CHECK_EQUAL(cp.m_q->min(), t0);
  BOOST_CHECK_EQUAL(cp.m_dq->min(), t0);
  BOOST_CHECK_EQUAL(cp.m_ddq->min(), t0);
  BOOST_CHECK_EQUAL(cp.m_q->max(), t3);
  BOOST_CHECK_EQUAL(cp.m_dq->max(), t3);
  BOOST_CHECK_EQUAL(cp.m_ddq->max(), t3);
  BOOST_CHECK((*cp.m_q)(t0).isApprox(q0));
  BOOST_CHECK((*cp.m_q)(t1).isApprox(q1));
  BOOST_CHECK((*cp.m_q)(t2).isApprox(q2));
  BOOST_CHECK((*cp.m_q)(t3).isApprox(q3));
  BOOST_CHECK((*cp.m_dq)(t0).isApprox(dq0));
  BOOST_CHECK((*cp.m_dq)(t1).isApprox(dq1));
  BOOST_CHECK((*cp.m_dq)(t2).isApprox(dq2));
  BOOST_CHECK((*cp.m_dq)(t3).isApprox(dq3));
  BOOST_CHECK((*cp.m_ddq)(t0).isApprox(ddq0));
  BOOST_CHECK((*cp.m_ddq)(t1).isApprox(ddq1));
  BOOST_CHECK((*cp.m_ddq)(t2).isApprox(ddq2));
  BOOST_CHECK((*cp.m_ddq)(t3).isApprox(ddq3));

  BOOST_CHECK(cp.m_q_init.isApprox(q0));
  BOOST_CHECK(cp.m_q_final.isApprox(q3));

  // check that init/final are not modified if they ere already set :
  // NOT THE CORRECT BEHAVIOR ANYMORE
  //  pointX_t q_init, q_final;
  //  q_init = cp2.m_q_init;
  //  q_final = cp2.m_q_final;
  //  cp2.setJointsTrajectoryFromPoints(points_q, points_derivative_q, points_second_derivative_q, time_points);
  //  BOOST_CHECK(cp2.m_q_init.isApprox(q_init));
  //  BOOST_CHECK(cp2.m_q_final.isApprox(q_final));
  //  BOOST_CHECK(!cp2.m_q_init.isApprox(q0));
  //  BOOST_CHECK(!cp2.m_q_final.isApprox(q3));

  // check that errors are correctly throw when required :
  // not the correct size
  BOOST_CHECK_THROW(cp.setCOMtrajectoryFromPoints(points_q, points_derivative, points_second_derivative, time_points),
                    std::invalid_argument);
  BOOST_CHECK_THROW(cp.setCOMtrajectoryFromPoints(points, points_derivative_q, points_second_derivative, time_points),
                    std::invalid_argument);
  BOOST_CHECK_THROW(cp.setCOMtrajectoryFromPoints(points, points_derivative, points_second_derivative_q, time_points),
                    std::invalid_argument);
  BOOST_CHECK_THROW(
      cp.setCOMtrajectoryFromPoints(points_q, points_derivative_q, points_second_derivative_q, time_points),
      std::invalid_argument);
  BOOST_CHECK_THROW(cp.setAMtrajectoryFromPoints(points_q, points_derivative_q, time_points), std::invalid_argument);
  BOOST_CHECK_THROW(cp.setAMtrajectoryFromPoints(points, points_derivative_q, time_points), std::invalid_argument);
  BOOST_CHECK_THROW(cp.setAMtrajectoryFromPoints(points_q, points_derivative, time_points), std::invalid_argument);
  // not the same number of points :
  t_pointX_t points2 = points;
  points2.push_back(d0);
  t_pointX_t points_derivative2 = points_derivative;
  points_derivative2.push_back(d0);
  t_pointX_t points_second_derivative2 = points_second_derivative;
  points_second_derivative2.push_back(d0);
  std::vector<double> time_points2 = time_points;
  time_points2.push_back(50.);
  BOOST_CHECK_THROW(cp.setCOMtrajectoryFromPoints(points2, points_derivative, points_second_derivative, time_points),
                    std::invalid_argument);
  BOOST_CHECK_THROW(cp.setCOMtrajectoryFromPoints(points, points_derivative2, points_second_derivative, time_points),
                    std::invalid_argument);
  BOOST_CHECK_THROW(cp.setCOMtrajectoryFromPoints(points, points_derivative, points_second_derivative2, time_points),
                    std::invalid_argument);
  BOOST_CHECK_THROW(cp.setCOMtrajectoryFromPoints(points, points_derivative, points_second_derivative, time_points2),
                    std::invalid_argument);
  BOOST_CHECK_THROW(cp.setAMtrajectoryFromPoints(points2, points_derivative, time_points), std::invalid_argument);
  BOOST_CHECK_THROW(cp.setAMtrajectoryFromPoints(points, points_derivative2, time_points), std::invalid_argument);
  BOOST_CHECK_THROW(cp.setAMtrajectoryFromPoints(points, points_derivative, time_points2), std::invalid_argument);
  t_pointX_t points_q2 = points_q;
  points_q2.push_back(q0);
  t_pointX_t points_derivative_q2 = points_derivative_q;
  points_derivative_q2.push_back(q0);
  t_pointX_t points_second_derivative_q2 = points_second_derivative_q;
  points_second_derivative_q2.push_back(q0);
  BOOST_CHECK_THROW(
      cp.setJointsTrajectoryFromPoints(points_q2, points_derivative_q, points_second_derivative_q, time_points),
      std::invalid_argument);
  BOOST_CHECK_THROW(
      cp.setJointsTrajectoryFromPoints(points_q, points_derivative_q2, points_second_derivative_q, time_points),
      std::invalid_argument);
  BOOST_CHECK_THROW(
      cp.setJointsTrajectoryFromPoints(points_q, points_derivative_q, points_second_derivative_q2, time_points),
      std::invalid_argument);
  BOOST_CHECK_THROW(
      cp.setJointsTrajectoryFromPoints(points_q, points_derivative_q, points_second_derivative_q, time_points2),
      std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(contact_phase_helper_contact_variation) {
  typedef ContactPhase::t_strings t_strings;
  // # contacts repositioned :
  ContactPhase cp1 = buildRandomContactPhase();
  ContactPhase cp2 = buildRandomContactPhase();
  t_strings repo = cp1.getContactsRepositioned(cp2);
  BOOST_CHECK(repo.size() == 2);
  BOOST_CHECK(repo[0] == "right_hand");
  BOOST_CHECK(repo[1] == "left_foot");
  t_strings repo1 = cp2.getContactsRepositioned(cp1);
  BOOST_CHECK(repo1.size() == 2);
  BOOST_CHECK(repo1[0] == "right_hand");
  BOOST_CHECK(repo1[1] == "left_foot");
  t_strings vars = cp1.getContactsVariations(cp2);
  BOOST_CHECK(vars.size() == 2);

  // # contacts broken :
  ContactPhase cp3;
  ContactPatch RH_patch(SE3::Identity().setRandom());
  cp3.addContact("right_hand", RH_patch);
  cp3.addContact("left_foot", ContactPatch(SE3::Identity().setRandom()));
  ContactPhase cp4;
  cp4.addContact("right_hand", RH_patch);
  t_strings broken = cp3.getContactsBroken(cp4);
  BOOST_CHECK(broken.size() == 1);
  BOOST_CHECK(broken[0] == "left_foot");
  t_strings broken1 = cp4.getContactsBroken(cp3);
  BOOST_CHECK(broken1.size() == 0);

  t_strings created = cp4.getContactsCreated(cp3);
  BOOST_CHECK(created.size() == 1);
  BOOST_CHECK(created[0] == "left_foot");
  t_strings created1 = cp3.getContactsCreated(cp4);
  BOOST_CHECK(created1.size() == 0);

  vars = cp3.getContactsVariations(cp4);
  BOOST_CHECK(vars.size() == 1);
  BOOST_CHECK(vars[0] == "left_foot");
  vars = cp4.getContactsVariations(cp3);
  BOOST_CHECK(vars.size() == 1);
  BOOST_CHECK(vars[0] == "left_foot");
}

BOOST_AUTO_TEST_CASE(contact_sequence) {
  // test without specifying size and using 'append' :
  ContactSequence cs1 = ContactSequence(0);
  BOOST_CHECK(cs1.size() == 0);
  ContactPhase cp0 = buildRandomContactPhase(0, 2);
  ContactPhase cp1 = buildRandomContactPhase(2, 4.);
  size_t id = cs1.append(cp0);
  BOOST_CHECK(cs1.size() == 1);
  BOOST_CHECK(id == 0);
  BOOST_CHECK(cs1.contactPhase(0) == cp0);
  id = cs1.append(cp1);
  BOOST_CHECK(cs1.size() == 2);
  BOOST_CHECK(id == 1);
  BOOST_CHECK(cs1.contactPhase(0) == cp0);
  BOOST_CHECK(cs1.contactPhase(1) == cp1);
  // check with accessor to the contact phases vector
  ContactPhaseVector phases = cs1.contactPhases();
  BOOST_CHECK(phases.size() == 2);
  BOOST_CHECK(phases[0] == cp0);
  BOOST_CHECK(phases[1] == cp1);

  // check that the accessor to contactPhases() create a copy :
  ContactPhase cp2 = buildRandomContactPhase(0, 2);
  phases.push_back(cp2);
  BOOST_CHECK(phases.size() == 3);
  BOOST_CHECK(cs1.size() == 2);  // original contact sequence should not be modified
  phases[1].duration(3.);
  BOOST_CHECK(cs1.contactPhase(1) == cp1);  // original contact sequence should not be modified

  // check that contactPhase(id) getter return non const reference :
  cs1.contactPhase(1).timeFinal(10.);
  BOOST_CHECK(cs1.contactPhase(1) != cp1);
  BOOST_CHECK(cs1.contactPhase(1).timeFinal() == 10.);

  cs1.contactPhase(0) = cp2;
  BOOST_CHECK(cs1.contactPhase(0) == cp2);

  // check with creating a contact sequence with a given size :
  ContactPhase cp_default;
  ContactSequence cs2 = ContactSequence(3);
  BOOST_CHECK(cs2.size() == 3);
  for (size_t i = 0; i < 3; ++i) BOOST_CHECK(cs2.contactPhase(i) == cp_default);

  // try to modify the uninitialized contact phase inside the sequence from the reference
  ContactPhase& cp2_0 = cs2.contactPhase(0);
  point3_t c_init = point3_t::Random();
  cp2_0.m_c_init = c_init;
  cp2_0.duration(10);
  BOOST_CHECK(cs2.contactPhase(0) != cp_default);
  BOOST_CHECK(cs2.contactPhase(0).duration() == 10);
  BOOST_CHECK(cs2.contactPhase(0).m_c_init == c_init);

  cs2.contactPhase(1) = cp1;
  BOOST_CHECK(cs2.contactPhase(1) == cp1);

  // try resize method :
  // with smaller value than current :
  cs2.resize(1);
  BOOST_CHECK(cs2.size() == 1);
  BOOST_CHECK(cs2.contactPhase(0).duration() == 10);
  BOOST_CHECK(cs2.contactPhase(0).m_c_init == c_init);

  // check with greater size than current :
  cs2.resize(4);
  BOOST_CHECK(cs2.size() == 4);
  BOOST_CHECK(cs2.contactPhase(0).duration() == 10);
  BOOST_CHECK(cs2.contactPhase(0).m_c_init == c_init);
  for (size_t i = 1; i < 4; ++i) BOOST_CHECK(cs2.contactPhase(i) == cp_default);

  // test operator == :
  ContactSequence cs3;
  ContactSequence cs4;

  BOOST_CHECK(cs3 == cs4);
  ContactPhase cp3_0 = buildRandomContactPhase(0, 2);
  cs3.append(cp3_0);
  BOOST_CHECK(cs3 != cs4);
  BOOST_CHECK(!(cs3 == cs4));
  cs4.append(cp3_0);
  BOOST_CHECK(cs3 == cs4);
  ContactPhase cp3_1 = buildRandomContactPhase(0, 2);
  cs3.append(cp3_1);
  BOOST_CHECK(cs3 != cs4);
  cs4.append(cp3_1);
  BOOST_CHECK(cs3 == cs4);
  cs4.contactPhase(1).duration(10);
  BOOST_CHECK(cs4.contactPhase(1) != cp3_1);
  BOOST_CHECK(cs3 != cs4);
  ContactSequence cs5(2);
  cs5.contactPhase(0) = cp3_0;
  BOOST_CHECK(cs3 != cs5);
  cs5.contactPhase(1) = cp3_1;
  BOOST_CHECK(cs3 == cs5);

  // test copy constructor :
  ContactSequence cs6;
  for (size_t i = 0; i < 10; ++i) {
    ContactPhase cp6 = buildRandomContactPhase(0, 2);
    cs6.append(cp6);
  }
  BOOST_CHECK(cs6.size() == 10);

  ContactSequence cs7(cs6);
  BOOST_CHECK(cs7 == cs6);
  for (size_t i = 0; i < 10; ++i) {
    BOOST_CHECK(cs6.contactPhase(i) == cs7.contactPhase(i));
  }

  // test serialization :
  std::string fileName("fileTest");
  ContactSequence cs_from_txt, cs_from_xml, cs_from_bin;

  cs6.saveAsText(fileName + ".txt");
  cs_from_txt.loadFromText(fileName + ".txt");
  BOOST_CHECK(cs6 == cs_from_txt);

  cs6.saveAsXML(fileName + ".xml", "ContactSequence");
  cs_from_xml.loadFromXML(fileName + ".xml", "ContactSequence");
  BOOST_CHECK(cs6 == cs_from_xml);

  cs6.saveAsBinary(fileName);
  cs_from_bin.loadFromBinary(fileName);
  BOOST_CHECK(cs6 == cs_from_bin);
}

BOOST_AUTO_TEST_CASE(contact_sequence_helper) {
  ContactSequence cs1 = ContactSequence(0);
  BOOST_CHECK(cs1.size() == 0);
  ContactPhase cp0 = buildRandomContactPhase(0, 2);
  ContactPhase cp1 = buildRandomContactPhase(2, 4.);
  cs1.append(cp0);
  cs1.append(cp1);
  // # test break contact :
  BOOST_CHECK(cs1.size() == 2);
  cs1.breakContact("left_foot");
  BOOST_CHECK(cs1.size() == 3);
  BOOST_CHECK(!cs1.contactPhase(2).isEffectorInContact("left_foot"));
  BOOST_CHECK(cs1.contactPhase(1).timeFinal() == 4.);  // time final of previous phase should not have been modified
  // check that the final value of the previous phase have been copied in the initial value of the new one
  BOOST_CHECK(cs1.contactPhase(1).m_c_final == cs1.contactPhase(2).m_c_init);
  BOOST_CHECK(cs1.contactPhase(1).m_dc_final == cs1.contactPhase(2).m_dc_init);
  BOOST_CHECK(cs1.contactPhase(1).m_ddc_final == cs1.contactPhase(2).m_ddc_init);
  BOOST_CHECK(cs1.contactPhase(1).m_L_final == cs1.contactPhase(2).m_L_init);
  BOOST_CHECK(cs1.contactPhase(1).m_dL_final == cs1.contactPhase(2).m_dL_init);
  BOOST_CHECK(cs1.contactPhase(1).m_q_final == cs1.contactPhase(2).m_q_init);
  BOOST_CHECK(cs1.contactPhase(1).timeFinal() == cs1.contactPhase(2).timeInitial());
  // check that the other contactPatch have been copied :
  BOOST_CHECK(cs1.contactPhase(1).contactPatch("right_hand") == cs1.contactPhase(2).contactPatch("right_hand"));

  // # test create contact :
  ContactPatch target(SE3::Identity().setRandom());
  cs1.createContact("left_foot", target, 2.5);
  BOOST_CHECK(cs1.size() == 4);
  BOOST_CHECK(cs1.contactPhase(2).timeFinal() == 6.5);  // time final of previous phase should have been modified
  BOOST_CHECK(cs1.contactPhase(3).contactPatch("left_foot") == target);
  // check that the final value of the previous phase have been copied in the initial value of the new one
  BOOST_CHECK(cs1.contactPhase(2).m_c_final == cs1.contactPhase(3).m_c_init);
  BOOST_CHECK(cs1.contactPhase(2).m_dc_final == cs1.contactPhase(3).m_dc_init);
  BOOST_CHECK(cs1.contactPhase(2).m_ddc_final == cs1.contactPhase(3).m_ddc_init);
  BOOST_CHECK(cs1.contactPhase(2).m_L_final == cs1.contactPhase(3).m_L_init);
  BOOST_CHECK(cs1.contactPhase(2).m_dL_final == cs1.contactPhase(3).m_dL_init);
  BOOST_CHECK(cs1.contactPhase(2).m_q_final == cs1.contactPhase(3).m_q_init);
  BOOST_CHECK(cs1.contactPhase(2).timeFinal() == cs1.contactPhase(3).timeInitial());
  // check that the other contactPatch have been copied :
  BOOST_CHECK(cs1.contactPhase(2).contactPatch("right_hand") == cs1.contactPhase(3).contactPatch("right_hand"));

  // # test break with duration :
  cs1.breakContact("left_foot", 1.);
  BOOST_CHECK(cs1.size() == 5);
  BOOST_CHECK(!cs1.contactPhase(4).isEffectorInContact("left_foot"));
  BOOST_CHECK(cs1.contactPhase(3).timeFinal() == 7.5);  // time final of previous phase should have been modified

  // # test  create contact with no duration:
  cs1.contactPhase(4).duration(1.);
  BOOST_CHECK(cs1.contactPhase(4).timeFinal() == 8.5);  // time final of previous phase should have been modified
  target = ContactPatch(SE3::Identity().setRandom());
  cs1.createContact("left_foot", target);
  BOOST_CHECK(cs1.size() == 6);
  BOOST_CHECK(cs1.contactPhase(4).timeFinal() == 8.5);    // time final of previous phase should have been modified
  BOOST_CHECK(cs1.contactPhase(5).timeInitial() == 8.5);  // time final of previous phase should have been modified

  // # test move effector to placement :
  SE3 target_placement = SE3::Identity().setRandom();
  addRandomPointsValues(cs1.contactPhase(5));
  cs1.contactPhase(5).contactPatch("right_hand").friction() = 2.;
  cs1.moveEffectorToPlacement("right_hand", target_placement, 1., 1.5);
  BOOST_CHECK(cs1.size() == 8);
  BOOST_CHECK(!cs1.contactPhase(6).isEffectorInContact("right_hand"));
  BOOST_CHECK(cs1.contactPhase(7).isEffectorInContact("right_hand"));
  BOOST_CHECK(cs1.contactPhase(7).contactPatch("right_hand").placement() == target_placement);
  // check that previous patch have not been modified :
  BOOST_CHECK(cs1.contactPhase(5).contactPatch("right_hand").placement() != target_placement);
  BOOST_CHECK(cs1.contactPhase(7).contactPatch("right_hand").friction() == 2.);
  BOOST_CHECK(cs1.contactPhase(5).timeFinal() == 9.5);
  BOOST_CHECK(cs1.contactPhase(6).timeInitial() == 9.5);
  BOOST_CHECK(cs1.contactPhase(6).timeFinal() == 11.);
  BOOST_CHECK(cs1.contactPhase(7).timeInitial() == 11.);
  // check that the final value of the previous phase have been copied in the initial value of the new one
  BOOST_CHECK(cs1.contactPhase(5).m_c_final == cs1.contactPhase(6).m_c_init);
  BOOST_CHECK(cs1.contactPhase(5).m_dc_final == cs1.contactPhase(6).m_dc_init);
  BOOST_CHECK(cs1.contactPhase(5).m_ddc_final == cs1.contactPhase(6).m_ddc_init);
  BOOST_CHECK(cs1.contactPhase(5).m_L_final == cs1.contactPhase(6).m_L_init);
  BOOST_CHECK(cs1.contactPhase(5).m_dL_final == cs1.contactPhase(6).m_dL_init);
  BOOST_CHECK(cs1.contactPhase(5).m_q_final == cs1.contactPhase(6).m_q_init);
  // with MoveEffector, the middle phase should have the same initial and final point :
  BOOST_CHECK(cs1.contactPhase(6).m_c_final == cs1.contactPhase(6).m_c_init);
  BOOST_CHECK(cs1.contactPhase(6).m_dc_final == cs1.contactPhase(6).m_dc_init);
  BOOST_CHECK(cs1.contactPhase(6).m_ddc_final == cs1.contactPhase(6).m_ddc_init);
  BOOST_CHECK(cs1.contactPhase(6).m_L_final == cs1.contactPhase(6).m_L_init);
  BOOST_CHECK(cs1.contactPhase(6).m_dL_final == cs1.contactPhase(6).m_dL_init);
  BOOST_CHECK(cs1.contactPhase(6).m_q_final == cs1.contactPhase(6).m_q_init);
  // check that the final value of the previous phase have been copied in the initial value of the new one
  BOOST_CHECK(cs1.contactPhase(6).m_c_final == cs1.contactPhase(7).m_c_init);
  BOOST_CHECK(cs1.contactPhase(6).m_dc_final == cs1.contactPhase(7).m_dc_init);
  BOOST_CHECK(cs1.contactPhase(6).m_ddc_final == cs1.contactPhase(7).m_ddc_init);
  BOOST_CHECK(cs1.contactPhase(6).m_L_final == cs1.contactPhase(7).m_L_init);
  BOOST_CHECK(cs1.contactPhase(6).m_dL_final == cs1.contactPhase(7).m_dL_init);
  BOOST_CHECK(cs1.contactPhase(6).m_q_final == cs1.contactPhase(7).m_q_init);
  // check that the other contactPatch have been copied :
  BOOST_CHECK(cs1.contactPhase(5).contactPatch("left_foot") == cs1.contactPhase(6).contactPatch("left_foot"));
  BOOST_CHECK(cs1.contactPhase(6).contactPatch("left_foot") == cs1.contactPhase(7).contactPatch("left_foot"));

  // # test move effector of:
  SE3 target_transform = SE3::Identity().setRandom();
  cs1.contactPhase(7).contactPatch("left_foot").friction() = 10.;
  cs1.moveEffectorOf("left_foot", target_transform, 1., 1.5);
  BOOST_CHECK(cs1.size() == 10);
  BOOST_CHECK(!cs1.contactPhase(8).isEffectorInContact("left_foot"));
  BOOST_CHECK(cs1.contactPhase(9).isEffectorInContact("left_foot"));
  target_placement = target_transform.act(cs1.contactPhase(7).contactPatch("left_foot").placement());
  BOOST_CHECK(cs1.contactPhase(9).contactPatch("left_foot").placement() == target_placement);
  BOOST_CHECK(cs1.contactPhase(9).contactPatch("left_foot").friction() == 10.);
  // check that the other contactPatch have been copied :
  BOOST_CHECK(cs1.contactPhase(7).contactPatch("right_hand") == cs1.contactPhase(8).contactPatch("right_hand"));
  BOOST_CHECK(cs1.contactPhase(8).contactPatch("right_hand") == cs1.contactPhase(9).contactPatch("right_hand"));
}

BOOST_AUTO_TEST_CASE(contact_sequence_helper_throw) {
  using std::invalid_argument;
  // # check that break contact correctly throw error when needed :
  ContactSequence cs1 = ContactSequence(0);
  BOOST_CHECK(cs1.size() == 0);
  ContactPhase cp0 = buildRandomContactPhase(0, 2);
  ContactPhase cp1 = buildRandomContactPhase(2, 4.);
  cp1.removeContact("left_foot");
  cs1.append(cp0);
  cs1.append(cp1);
  BOOST_CHECK(cs1.size() == 2);
  BOOST_CHECK_THROW(cs1.breakContact("left_foot"), invalid_argument);  // contact do not exist
  BOOST_CHECK(cs1.size() == 2);
  ContactPhase cp2 = buildRandomContactPhase();
  cs1.append(cp2);
  BOOST_CHECK(cs1.size() == 3);
  BOOST_CHECK_THROW(cs1.breakContact("left_foot", 1.5), invalid_argument);  // time interval not defined for last phase
  BOOST_CHECK(cs1.size() == 3);

  // # check that create contact correctly throw error when needed :
  ContactPatch target(SE3::Identity().setRandom());
  BOOST_CHECK_THROW(cs1.createContact("left_foot", target), invalid_argument);  // contact already exist
  BOOST_CHECK(cs1.size() == 3);
  cs1.breakContact("left_foot");
  BOOST_CHECK(cs1.size() == 4);
  BOOST_CHECK_THROW(cs1.createContact("left_foot", target, 2.), invalid_argument);  // time interval not defined
  BOOST_CHECK(cs1.size() == 4);
}

BOOST_AUTO_TEST_CASE(contact_sequence_is_time_consistent) {
  ContactSequence cs1 = ContactSequence(0);
  ContactPhase cp0 = buildRandomContactPhase(0, 2);
  ContactPhase cp1 = buildRandomContactPhase(2, 4.);
  cs1.append(cp0);
  cs1.append(cp1);
  bool consistent = cs1.haveTimings();
  BOOST_CHECK(consistent);

  ContactSequence cs2 = ContactSequence(0);
  ContactPhase cp2 = buildRandomContactPhase(0, 2);
  ContactPhase cp3 = buildRandomContactPhase(1.5, 4.);
  cs2.append(cp2);
  cs2.append(cp3);
  consistent = cs2.haveTimings();
  BOOST_CHECK(!consistent);

  ContactSequence cs3 = ContactSequence(0);
  ContactPhase cp4 = buildRandomContactPhase(0, 2);
  ContactPhase cp5 = buildRandomContactPhase();
  cs3.append(cp4);
  cs3.append(cp5);
  consistent = cs3.haveTimings();
  BOOST_CHECK(!consistent);

  ContactSequence cs4 = ContactSequence(0);
  ContactPhase cp6 = buildRandomContactPhase();
  ContactPhase cp7 = buildRandomContactPhase(1, 3);
  cs4.append(cp6);
  cs4.append(cp7);
  consistent = cs4.haveTimings();
  BOOST_CHECK(!consistent);
}

BOOST_AUTO_TEST_CASE(contact_sequence_concatenate_com_traj) {
  ContactSequence cs1 = ContactSequence(0);
  ContactPhase cp0 = buildRandomContactPhase(0, 2);
  ContactPhase cp1 = buildRandomContactPhase(2, 4.);
  point3_t p0(0., 0., 0.);
  point3_t p1(1., 2., 3.);
  point3_t p2(4., 4., 4.);
  point3_t p3(10., 10., 10.);
  point3_t p4(-2., 3.6, 4.);
  double t0 = 0.;
  double t1 = 0.8;
  double t2 = 2.;
  double t3 = 3.2;
  double t4 = 4.;
  t_pointX_t points1, points2;
  points1.push_back(p0);
  points1.push_back(p1);
  points1.push_back(p2);
  points2.push_back(p2);
  points2.push_back(p3);
  points2.push_back(p4);
  std::vector<double> time_points1, time_points2;
  time_points1.push_back(t0);
  time_points1.push_back(t1);
  time_points1.push_back(t2);
  time_points2.push_back(t2);
  time_points2.push_back(t3);
  time_points2.push_back(t4);

  curves::piecewise_t c1 =
      curves::piecewise_t::convert_discrete_points_to_polynomial<curves::polynomial_t>(points1, time_points1);
  curve_ptr_t c1_ptr(new curves::piecewise_t(c1));
  curves::piecewise_t c2 =
      curves::piecewise_t::convert_discrete_points_to_polynomial<curves::polynomial_t>(points2, time_points2);
  curve_ptr_t c2_ptr(new curves::piecewise_t(c2));
  cp0.m_c = c1_ptr;
  cp1.m_c = c2_ptr;
  BOOST_CHECK(cp0.m_c->min() == 0.);
  BOOST_CHECK(cp0.m_c->max() == 2.);
  BOOST_CHECK(cp1.m_c->min() == 2.);
  BOOST_CHECK(cp1.m_c->max() == 4.);
  cs1.append(cp0);
  cs1.append(cp1);
  piecewise_t c_t = cs1.concatenateCtrajectories();
  BOOST_CHECK(c_t.min() == 0.);
  BOOST_CHECK(c_t.max() == 4.);
  BOOST_CHECK(c_t(0) == cp0.m_c->operator()(0));
  BOOST_CHECK(c_t(0.5) == cp0.m_c->operator()(0.5));
  BOOST_CHECK(c_t(2.) == cp0.m_c->operator()(2.));
  BOOST_CHECK(c_t(3) == cp1.m_c->operator()(3));
  BOOST_CHECK(c_t(4.) == cp1.m_c->operator()(4.));
}

BOOST_AUTO_TEST_CASE(contact_sequence_concatenate_effector_traj) {
  ContactSequence cs1 = ContactSequence(0);
  ContactPhase cp0 = ContactPhase(0, 2);
  ContactPhase cp1 = ContactPhase(2, 4.);
  ContactPhase cp2 = ContactPhase(4, 8.);
  quaternion_t q0 = randomQuaternion();
  quaternion_t q1 = randomQuaternion();
  quaternion_t q2 = randomQuaternion();
  q0.normalize();
  q1.normalize();
  q2.normalize();
  pointX_t p0 = point3_t::Random();
  pointX_t p1 = point3_t::Random();
  pointX_t p2 = point3_t::Random();

  curve_SE3_ptr_t traj_0(new curves::SE3Curve_t(p0, p1, q0, q1, 0., 2.));
  curve_SE3_ptr_t traj_2(new curves::SE3Curve_t(p1, p2, q1, q2, 4., 8.));
  cp0.addEffectorTrajectory("right_leg", traj_0);
  cp2.addEffectorTrajectory("right_leg", traj_2);

  cs1.append(cp0);
  cs1.append(cp1);
  cs1.append(cp2);

  piecewise_SE3_t traj = cs1.concatenateEffectorTrajectories("right_leg");
  BOOST_CHECK(traj.min() == 0.);
  BOOST_CHECK(traj.max() == 8.);
  BOOST_CHECK(traj(0.).isApprox(traj_0->operator()(0.)));
  BOOST_CHECK(traj(1.5).isApprox(traj_0->operator()(1.5)));
  BOOST_CHECK(traj(2.).isApprox(traj_0->operator()(2.)));
  BOOST_CHECK(traj(4.).isApprox(traj_2->operator()(4.)));
  BOOST_CHECK(traj(6.).isApprox(traj_2->operator()(6.)));
  BOOST_CHECK(traj(8.).isApprox(traj_2->operator()(8.)));
  BOOST_CHECK(traj(2.5).isApprox(traj_0->operator()(2.)));
  BOOST_CHECK(traj(3.8).isApprox(traj_0->operator()(2.)));
}

BOOST_AUTO_TEST_CASE(contact_sequence_concatenate_force_traj) {
  ContactSequence cs1 = ContactSequence(0);
  ContactPhase cp0 = ContactPhase(0, 2);
  ContactPhase cp1 = ContactPhase(2, 4.);
  ContactPhase cp2 = ContactPhase(4, 8.);

  cp0.addContact("right_leg", ContactPatch());
  cp2.addContact("right_leg", ContactPatch());
  curve_ptr_t f_0 = buildRandomPolynomial12D(0, 2);
  curve_ptr_t f_2 = buildRandomPolynomial12D(4, 8);
  cp0.addContactForceTrajectory("right_leg", f_0);
  cp2.addContactForceTrajectory("right_leg", f_2);

  cs1.append(cp0);
  cs1.append(cp1);
  cs1.append(cp2);

  piecewise_t forces = cs1.concatenateContactForceTrajectories("right_leg");
  BOOST_CHECK(forces.min() == 0.);
  BOOST_CHECK(forces.max() == 8.);
  BOOST_CHECK(forces(0.).isApprox(f_0->operator()(0.)));
  BOOST_CHECK(forces(1.5).isApprox(f_0->operator()(1.5)));
  BOOST_CHECK(forces(1.999).isApprox(f_0->operator()(1.999)));
  BOOST_CHECK(forces(4.).isApprox(f_2->operator()(4.)));
  BOOST_CHECK(forces(6.).isApprox(f_2->operator()(6.)));
  BOOST_CHECK(forces(8.).isApprox(f_2->operator()(8.)));
  BOOST_CHECK(forces(2.).isApprox(point12_t::Zero()));
  BOOST_CHECK(forces(2.5).isApprox(point12_t::Zero()));
  BOOST_CHECK(forces(3.8).isApprox(point12_t::Zero()));
}

BOOST_AUTO_TEST_CASE(contact_sequence_concatenate_normal_force_traj) {
  ContactSequence cs1 = ContactSequence(0);
  ContactPhase cp0 = ContactPhase(0, 2);
  ContactPhase cp1 = ContactPhase(2, 4.);
  ContactPhase cp2 = ContactPhase(4, 8.);

  cp1.addContact("right_leg", ContactPatch());
  curve_ptr_t f_1 = buildRandomPolynomial1D(2., 4.);
  cp1.addContactNormalForceTrajectory("right_leg", f_1);

  cs1.append(cp0);
  cs1.append(cp1);
  cs1.append(cp2);

  piecewise_t forces = cs1.concatenateNormalForceTrajectories("right_leg");
  BOOST_CHECK(forces.min() == 0.);
  BOOST_CHECK(forces.max() == 8.);
  BOOST_CHECK(forces(2.).isApprox(f_1->operator()(2.)));
  BOOST_CHECK(forces(2.5).isApprox(f_1->operator()(2.5)));
  BOOST_CHECK(forces(3.999).isApprox(f_1->operator()(3.999)));
  BOOST_CHECK(forces(0.).isApprox(point1_t::Zero()));
  BOOST_CHECK(forces(1.5).isApprox(point1_t::Zero()));
  BOOST_CHECK(forces(4.).isApprox(point1_t::Zero()));
  BOOST_CHECK(forces(7.5).isApprox(point1_t::Zero()));
}

BOOST_AUTO_TEST_CASE(contact_sequence_phase_at_time) {
  ContactSequence cs1 = ContactSequence(0);
  ContactPhase cp0 = ContactPhase(0, 2);
  ContactPhase cp1 = ContactPhase(2, 4.);
  ContactPhase cp2 = ContactPhase(4, 8.);

  cs1.append(cp0);
  cs1.append(cp1);
  cs1.append(cp2);

  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(0.), 0);
  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(1.), 0);
  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(1.9), 0);
  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(2.), 1);
  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(3.5), 1);
  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(4.), 2);
  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(5.), 2);
  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(8.), 2);
  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(-0.5), -1);
  BOOST_CHECK_EQUAL(cs1.phaseIdAtTime(10.), -1);

  BOOST_CHECK_EQUAL(cs1.phaseAtTime(0.), cp0);
  BOOST_CHECK_EQUAL(cs1.phaseAtTime(1.), cp0);
  BOOST_CHECK_EQUAL(cs1.phaseAtTime(1.9), cp0);
  BOOST_CHECK_EQUAL(cs1.phaseAtTime(2.), cp1);
  BOOST_CHECK_EQUAL(cs1.phaseAtTime(3.5), cp1);
  BOOST_CHECK_EQUAL(cs1.phaseAtTime(4.), cp2);
  BOOST_CHECK_EQUAL(cs1.phaseAtTime(5.), cp2);
  BOOST_CHECK_EQUAL(cs1.phaseAtTime(8.), cp2);
  BOOST_CHECK_THROW(cs1.phaseAtTime(-0.5), std::invalid_argument);
  BOOST_CHECK_THROW(cs1.phaseAtTime(10.), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()
