// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#include <iostream>

#define BOOST_TEST_MODULE StatsTests
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "multicontact-api/scenario/contact-model-planar.hpp"
#include "multicontact-api/scenario/contact-patch.hpp"
#include "multicontact-api/scenario/contact-phase.hpp"
//#include "multicontact-api/scenario/contact-sequence.hpp"

#include <curves/fwd.h>
#include <curves/so3_linear.h>
#include <curves/se3_curve.h>
#include <curves/polynomial.h>
#include <curves/bezier_curve.h>
#include <curves/piecewise_curve.h>
#include <curves/exact_cubic.h>
#include <curves/cubic_hermite_spline.h>

typedef Eigen::Matrix<double, 1, 1> point1_t;
using curves::point3_t;
using curves::point6_t;
typedef Eigen::Matrix<double, 12, 1> point12_t;
using curves::pointX_t;
using curves::matrix3_t;
using curves::quaternion_t;
using curves::t_pointX_t;
using curves::curve_ptr_t;
using curves::curve_SE3_ptr_t;

using namespace multicontact_api::scenario;

template <typename Scalar>
struct ATpl {
  typedef pinocchio::SE3Tpl<Scalar> SE3;

  explicit ATpl() : data() {}
  explicit ATpl(const ATpl& other) : data(other.data){};

  bool operator==(const ATpl& other) { return data == other.data; }

 protected:
  SE3 data;
};

typedef ATpl<double> Ad;
typedef pinocchio::SE3Tpl<double> SE3;

curve_ptr_t buildPiecewisePolynomialC2(){
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
curve_ptr_t buildRandomPolynomial(){
  pointX_t a = Point::Random();
  pointX_t b = Point::Random();
  pointX_t c = Point::Random();
  pointX_t d = Point::Random();
  const double t1 = Eigen::internal::random<double>(0.,10.);
  const double t2 = Eigen::internal::random<double>(0.,10.);
  const double min = std::min(t1,t2);
  const double max = std::max(t1,t2);
  t_pointX_t vec;
  vec.push_back(a);
  vec.push_back(b);
  vec.push_back(c);
  vec.push_back(d);
  curve_ptr_t res (new curves::polynomial_t(vec.begin(), vec.end(), min, max));
  return  res;
}

curve_ptr_t buildRandomPolynomial3D(){
  return buildRandomPolynomial<point3_t>();
}

curve_ptr_t buildRandomPolynomial12D(){
  return buildRandomPolynomial<point12_t>();
}

curve_ptr_t buildRandomPolynomial1D(){
  return buildRandomPolynomial<point1_t>();
}




curve_SE3_ptr_t buildPiecewiseSE3(){
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

quaternion_t randomQuaternion(){ // already included in newest eigen release
  const double u1 = Eigen::internal::random<double>(0, 1),
      u2 = Eigen::internal::random<double>(0, 2*EIGEN_PI),
      u3 = Eigen::internal::random<double>(0, 2*EIGEN_PI);
  const double a = sqrt(1 - u1),
      b = sqrt(u1);
  return quaternion_t(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3)).normalized();
}

curve_SE3_ptr_t buildRandomSE3LinearTraj(const double min, const double max){
  quaternion_t q0 = randomQuaternion();
  quaternion_t q1 = randomQuaternion();
  q0.normalize();
  q1.normalize();
  pointX_t p0 = point3_t::Random();
  pointX_t p1 = point3_t::Random();

  curve_SE3_ptr_t res(new curves::SE3Curve_t(p0, p1, q0, q1, min, max));
  return res;
}


void explicitContactPhaseAssertEqual(ContactPhase& cp1, ContactPhase& cp2){
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
  BOOST_CHECK((*cp1.m_q)(cp2.m_q->min())==(*cp2.m_q)(cp2.m_q->min()));
  BOOST_CHECK((*cp1.m_dq)(cp2.m_dq->min())==(*cp2.m_dq)(cp2.m_dq->min()));
  BOOST_CHECK((*cp1.m_ddq)(cp2.m_ddq->min())==(*cp2.m_ddq)(cp2.m_ddq->min()));
  BOOST_CHECK((*cp1.m_tau)(cp2.m_tau->min())==(*cp2.m_tau)(cp2.m_tau->min()));
  BOOST_CHECK((*cp1.m_c)(cp2.m_c->min())==(*cp2.m_c)(cp2.m_c->min()));
  BOOST_CHECK((*cp1.m_dc)(cp2.m_dc->min())==(*cp2.m_dc)(cp2.m_dc->min()));
  BOOST_CHECK((*cp1.m_ddc)(cp2.m_ddc->min())==(*cp2.m_ddc)(cp2.m_ddc->min()));
  BOOST_CHECK((*cp1.m_L)(cp2.m_L->min())==(*cp2.m_L)(cp2.m_L->min()));
  BOOST_CHECK((*cp1.m_dL)(cp2.m_dL->min())==(*cp2.m_dL)(cp2.m_dL->min()));
  BOOST_CHECK((*cp1.m_wrench)(cp2.m_wrench->min())==(*cp2.m_wrench)(cp2.m_wrench->min()));
  BOOST_CHECK((*cp1.m_zmp)(cp2.m_zmp->min())==(*cp2.m_zmp)(cp2.m_zmp->min()));
  BOOST_CHECK((*cp1.m_root)(cp2.m_root->min()).isApprox((*cp2.m_root)(cp2.m_root->min())));
  BOOST_CHECK(*cp1.m_q == *cp2.m_q);
  BOOST_CHECK(*cp1.m_dq == *cp2.m_dq);
  BOOST_CHECK(*cp1.m_ddq == *cp2.m_ddq);
  BOOST_CHECK(*cp1.m_tau == *cp2.m_tau);
  BOOST_CHECK(*cp1.m_c == *cp2.m_c);
  BOOST_CHECK(*cp1.m_dc == *cp2.m_dc);
  BOOST_CHECK(*cp1.m_ddc == *cp2.m_ddc);
  BOOST_CHECK(*cp1.m_L == *cp2.m_L);
  BOOST_CHECK(*cp1.m_dL == *cp2.m_dL);
  BOOST_CHECK(*cp1.m_wrench == *cp2.m_wrench);
  BOOST_CHECK(*cp1.m_zmp == *cp2.m_zmp);
  BOOST_CHECK(*cp1.m_root == *cp2.m_root);
  BOOST_CHECK(cp1.contactForces() == cp2.contactForces());
  BOOST_CHECK(cp1.contactNormalForces() == cp2.contactNormalForces());
  BOOST_CHECK(cp1.effectorTrajectories() == cp2.effectorTrajectories());
  BOOST_CHECK(cp1.effectorsInContact() == cp2.effectorsInContact());
  BOOST_CHECK(cp1.contactPatches() == cp2.contactPatches());
  const ContactPhase::t_strings eeNames = cp2.effectorsInContact();
  for(ContactPhase::t_strings::const_iterator ee = eeNames.begin() ; ee != eeNames.end() ; ++ee){
    std::cout<<"## For effector "<<*ee<<std::endl;
    BOOST_CHECK(cp2.isEffectorInContact(*ee));
    BOOST_CHECK(cp1.contactPatch(*ee) == cp2.contactPatch(*ee));
    if(cp1.contactForces().count(*ee)){
      BOOST_CHECK(cp2.contactForces().count(*ee));
      BOOST_CHECK(*cp1.contactForces(*ee) == *cp2.contactForces(*ee));
      BOOST_CHECK((*cp1.contactForces(*ee))(cp1.contactForces(*ee)->min()) == (*cp2.contactForces(*ee))(cp2.contactForces(*ee)->min()));
      BOOST_CHECK((*cp1.contactForces(*ee))(cp1.contactForces(*ee)->max()) == (*cp2.contactForces(*ee))(cp2.contactForces(*ee)->max()));
    }
    if(cp1.contactNormalForces().count(*ee)){
      BOOST_CHECK(cp2.contactNormalForces().count(*ee));
      BOOST_CHECK(*cp1.contactNormalForces(*ee) == *cp2.contactNormalForces(*ee));
      BOOST_CHECK((*cp1.contactNormalForces(*ee))(cp1.contactNormalForces(*ee)->min()) == (*cp2.contactNormalForces(*ee))(cp2.contactNormalForces(*ee)->min()));
      BOOST_CHECK((*cp1.contactNormalForces(*ee))(cp1.contactNormalForces(*ee)->max()) == (*cp2.contactNormalForces(*ee))(cp2.contactNormalForces(*ee)->max()));
    }
  }
  const ContactPhase::CurveSE3Map trajMap = cp2.effectorTrajectories();
  for (ContactPhase::CurveSE3Map::const_iterator mit = trajMap.begin() ; mit != trajMap.end(); ++mit)
  {
    std::cout<<"## For effector trajectory "<<mit->first<<std::endl;
    BOOST_CHECK(cp2.effectorTrajectories().count(mit->first));
    BOOST_CHECK(*cp1.effectorTrajectories(mit->first) == *cp2.effectorTrajectories(mit->first));
    BOOST_CHECK((*cp1.effectorTrajectories(mit->first))(cp1.effectorTrajectories(mit->first)->min()).isApprox((*cp2.effectorTrajectories(mit->first))(cp2.effectorTrajectories(mit->first)->min())));
    BOOST_CHECK((*cp1.effectorTrajectories(mit->first))(cp1.effectorTrajectories(mit->first)->max()).isApprox((*cp2.effectorTrajectories(mit->first))(cp2.effectorTrajectories(mit->first)->max())));
  }
}


BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_model) {
  const double mu = 0.3;
  const double ZMP_radius = 0.01;

  ContactModelPlanar mp1(mu, ZMP_radius);
  ContactModelPlanar mp2(mp1);
  BOOST_CHECK(mp1.m_mu == mu);
  BOOST_CHECK(mp1.m_ZMP_radius == ZMP_radius);

  BOOST_CHECK(mp1 == mp2);
  mp1.m_mu = 0.5;
  BOOST_CHECK(mp1 != mp2);

  //TODO : check serialization
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
  ContactPatch cp2(p,0.9);
  BOOST_CHECK(cp2.placement() == p);
  BOOST_CHECK(cp2.friction() == 0.9);

  // check comparison operator
  BOOST_CHECK(cp1 != cp2);
  ContactPatch cp3(p,0.9);
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

  cp3.saveAsXML(fileName,"ContactPatch");
  ContactPatch cp_from_xml;
  cp_from_xml.loadFromXML(fileName,"ContactPatch");
  BOOST_CHECK(cp3 == cp_from_xml);

  cp3.saveAsBinary(fileName);
  ContactPatch cp_from_bin;
  cp_from_bin.loadFromBinary(fileName);
  BOOST_CHECK(cp3 == cp_from_bin);
}

BOOST_AUTO_TEST_CASE(contact_phase)
{
  // default constructor
  ContactPhase cp1;
  // test timings setter/getter
  BOOST_CHECK(cp1.timeInitial() == -1);
  BOOST_CHECK(cp1.timeFinal() == -1);
  BOOST_CHECK(cp1.duration() == 0);
  //cp1.timeInitial() = 2; // should not compile : only const return for times
  //cp1.timeInitial() = 2; // should not compile : only const return for times
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
  BOOST_CHECK_THROW(cp1.timeFinal(0.1),std::invalid_argument);
  BOOST_CHECK_THROW(cp1.duration(-2.),std::invalid_argument);
  // constructor with timings, should throw : 
  BOOST_CHECK_THROW(ContactPhase(0.9,0.1),std::invalid_argument);
  BOOST_CHECK_THROW(ContactPhase(0.,-1),std::invalid_argument);

  // Constructor with timings : 
  ContactPhase cp2(0.5,2.);
  BOOST_CHECK(cp2.timeInitial() == 0.5);
  BOOST_CHECK(cp2.timeFinal() == 2.);
  BOOST_CHECK(cp2.duration() == 1.5);

  // Check contacts creation / removal
  ContactPatch patchL(SE3::Identity().setRandom());
  bool newContact =  cp2.addContact("left_leg",patchL);
  BOOST_CHECK(newContact);
  BOOST_CHECK(cp2.isEffectorInContact("left_leg"));  BOOST_CHECK(!cp2.isEffectorInContact("other"));
  BOOST_CHECK(cp2.contactPatch("left_leg") == patchL);
  BOOST_CHECK(cp2.contactPatches()["left_leg"] == patchL);
  BOOST_CHECK(cp2.numContacts() == 1);
  BOOST_CHECK_THROW(cp2.contactPatch("other"),std::invalid_argument);

  ContactPatch patchR(SE3::Identity().setRandom());
  newContact =  cp2.addContact("right_leg",patchR);
  BOOST_CHECK(newContact);
  BOOST_CHECK(cp2.isEffectorInContact("left_leg"));
  BOOST_CHECK(cp2.isEffectorInContact("right_leg"));
  BOOST_CHECK(cp2.contactPatch("left_leg") == patchL);
  BOOST_CHECK(cp2.contactPatch("right_leg") == patchR);
  BOOST_CHECK(cp2.contactPatches()["left_leg"] == patchL);
  BOOST_CHECK(cp2.contactPatches()["right_leg"] == patchR);

  cp2.contactPatches().insert(std::pair<std::string,ContactPatch>("other",patchR)); // Why does it compile ? there is only const getter
  BOOST_CHECK(cp2.contactPatches().count("other") == 0); // previous line should have no effect, only const getter
  BOOST_CHECK(!cp2.isEffectorInContact("other"));
  ContactPatch patchR2(SE3::Identity().setRandom(),1.5);
//  cp2.contactPatches

  newContact = cp2.addContact("left_leg",ContactPatch(SE3::Identity().setRandom(),0.5));
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
  bool newTraj = cp2.addContactForceTrajectory("left_leg",force12d);
  BOOST_CHECK(newTraj);
  newTraj = cp2.addContactForceTrajectory("left_leg",force12d);
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
  cp2.contactForces().insert(std::pair<std::string,curve_ptr_t>("right_leg",force12d_1)); // should not have any effect only const getter
  BOOST_CHECK(cp2.contactForces().count("right_leg") == 0);
  cp2.contactForces().erase("left_leg"); // should not have any effect
  BOOST_CHECK((*cp2.contactForces("left_leg"))(min) == f0);
  BOOST_CHECK_THROW(cp2.addContactForceTrajectory("right_leg",force12d),std::invalid_argument); // right leg is not in contact, cannot add force
  BOOST_CHECK_THROW(cp2.contactForces("right_leg"),std::invalid_argument);

  // check with piecewise curve :
  curve_ptr_t force_C2 = buildPiecewisePolynomialC2();
  cp2.addContactForceTrajectory("left_leg",force_C2);
  BOOST_CHECK((*cp2.contactForces("left_leg"))(1.) == (*force_C2)(1.));
  BOOST_CHECK((*cp2.contactForces("left_leg"))(10.) == (*force_C2)(10.));
  BOOST_CHECK((*cp2.contactForces("left_leg"))(5.7) == (*force_C2)(5.7));


  pointX_t nf0(1);
  nf0 << 56.3;
  pointX_t nf1(1);
  nf1 << 5893.2;
  curve_ptr_t force1d(new curves::polynomial_t(nf0, nf1, min, max));
  newTraj = cp2.addContactNormalForceTrajectory("left_leg",force1d);
  BOOST_CHECK(newTraj);
  newTraj = cp2.addContactNormalForceTrajectory("left_leg",force1d);
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
  cp2.contactNormalForces().insert(std::pair<std::string,curve_ptr_t>("right_leg",force1d_1)); // should not have any effect only const getter
  BOOST_CHECK(cp2.contactNormalForces().count("right_leg") == 0);
  cp2.contactNormalForces().erase("left_leg"); // should not have any effect
  BOOST_CHECK((*cp2.contactNormalForces("left_leg"))(min) == nf0);
  BOOST_CHECK_THROW(cp2.contactNormalForces("right_leg"),std::invalid_argument);
  BOOST_CHECK_THROW(cp2.addContactNormalForceTrajectory("right_leg",force12d),std::invalid_argument); // right leg is not in contact, cannot add force
  BOOST_CHECK_THROW(cp2.addContactNormalForceTrajectory("left_leg",force12d),std::invalid_argument); // should be of dimension 1


  // Check effector trajectory : 
  curve_SE3_ptr_t effR = buildPiecewiseSE3();
  newTraj = cp2.addEffectorTrajectory("right_leg",effR);
  BOOST_CHECK(newTraj);
  newTraj = cp2.addEffectorTrajectory("right_leg",effR);
  BOOST_CHECK(!newTraj);
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(min).isApprox((*effR)(min)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(max).isApprox((*effR)(max)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(1.2).isApprox((*effR)(1.2)));
  BOOST_CHECK((*cp2.effectorTrajectories("right_leg"))(min).isApprox((*effR)(min)));
  BOOST_CHECK((*cp2.effectorTrajectories("right_leg"))(max).isApprox((*effR)(max)));
  BOOST_CHECK((*cp2.effectorTrajectories("right_leg"))(1.2).isApprox((*effR)(1.2)));
  BOOST_CHECK_THROW(cp2.effectorTrajectories("left_leg"),std::invalid_argument);
  BOOST_CHECK_THROW(cp2.addEffectorTrajectory("left_leg",effR),std::invalid_argument); // right leg is not in contact, cannot add force
  //cp2.addEffectorTrajectory("right_leg",force12d); // should not compile : not the right return type for the curve

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

  point3_t c_init= point3_t::Random();
  point3_t dc_init= point3_t::Random();
  pointX_t ddc_init= point3_t::Random(); // test with point X for automatic conversion
  pointX_t L_init= point3_t::Random();
  point3_t dL_init= point3_t::Random();
  pointX_t q_init= point12_t::Random();
  pointX_t c_final= point3_t::Random();
  pointX_t dc_final= point3_t::Random();
  point3_t ddc_final= point3_t::Random();
  pointX_t L_final= point3_t::Random();
  point3_t dL_final= point3_t::Random();
  pointX_t q_final= point12_t::Random();
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
  { // inner scope to check that the curves are not destroyed
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
  curve_SE3_ptr_t root = buildRandomSE3LinearTraj(1,5.5);
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
  BOOST_CHECK_NO_THROW(cp2.m_q->operator()(cp2.m_q->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_dq->operator()(cp2.m_dq->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_ddq->operator()(cp2.m_ddq->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_tau->operator()(cp2.m_tau->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_c->operator()(cp2.m_c->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_dc->operator()(cp2.m_dc->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_ddc->operator()(cp2.m_ddc->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_L->operator()(cp2.m_L->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_dL->operator()(cp2.m_dL->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_wrench->operator()(cp2.m_wrench->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_root->operator()(cp2.m_root->min())); // check that the curves still exist after leaving the scope
  BOOST_CHECK_NO_THROW(cp2.m_zmp->operator()(cp2.m_zmp->min())); // check that the curves still exist after leaving the scope


  //add more contact and trajectories :
  cp2.addContact("right_hand",ContactPatch(SE3::Identity().setRandom()));
  cp2.addContactForceTrajectory("right_hand",buildRandomPolynomial12D());
  cp2.addContactNormalForceTrajectory("right_hand",buildRandomPolynomial1D());
  int num_ctc = 0;
  for (ContactPhase::CurveMap::const_iterator mit = cp2.contactForces().begin() ; mit != cp2.contactForces().end(); ++mit)
  {
    BOOST_CHECK(mit->first == "right_hand" || mit->first == "left_leg");
    num_ctc ++;
  }
  BOOST_CHECK(num_ctc == 2);

  curve_SE3_ptr_t eff_knee(buildRandomSE3LinearTraj(cp2.timeInitial(),cp2.timeFinal()));
  double min_knee = eff_knee->min();
  double max_knee = eff_knee->max();
  newTraj = cp2.addEffectorTrajectory("knee",eff_knee);
  BOOST_CHECK(newTraj);
  BOOST_CHECK((*cp2.effectorTrajectories()["knee"])(min_knee).isApprox((*eff_knee)(min_knee)));
  BOOST_CHECK((*cp2.effectorTrajectories()["knee"])(max_knee).isApprox((*eff_knee)(max_knee)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(min).isApprox((*effR)(min)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(max).isApprox((*effR)(max)));
  BOOST_CHECK((*cp2.effectorTrajectories()["right_leg"])(1.2).isApprox((*effR)(1.2)));
  BOOST_CHECK(cp2.effectorTrajectories().size() == 2);
  int num_eff_traj = 0;
  const ContactPhase::CurveSE3Map trajMap = cp2.effectorTrajectories();
  for (ContactPhase::CurveSE3Map::const_iterator mit = trajMap.begin() ; mit != trajMap.end(); ++mit)
  {
    BOOST_CHECK(mit->first == "knee" || mit->first == "right_leg");
    num_eff_traj ++;
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
  BOOST_CHECK((*cp5.m_q)(cp2.m_q->min())==(*cp2.m_q)(cp2.m_q->min()));
  BOOST_CHECK((*cp5.m_dq)(cp2.m_dq->min())==(*cp2.m_dq)(cp2.m_dq->min()));
  BOOST_CHECK((*cp5.m_ddq)(cp2.m_ddq->min())==(*cp2.m_ddq)(cp2.m_ddq->min()));
  BOOST_CHECK((*cp5.m_tau)(cp2.m_tau->min())==(*cp2.m_tau)(cp2.m_tau->min()));
  BOOST_CHECK((*cp5.m_c)(cp2.m_c->min())==(*cp2.m_c)(cp2.m_c->min()));
  BOOST_CHECK((*cp5.m_dc)(cp2.m_dc->min())==(*cp2.m_dc)(cp2.m_dc->min()));
  BOOST_CHECK((*cp5.m_ddc)(cp2.m_ddc->min())==(*cp2.m_ddc)(cp2.m_ddc->min()));
  BOOST_CHECK((*cp5.m_L)(cp2.m_L->min())==(*cp2.m_L)(cp2.m_L->min()));
  BOOST_CHECK((*cp5.m_dL)(cp2.m_dL->min())==(*cp2.m_dL)(cp2.m_dL->min()));
  BOOST_CHECK((*cp5.m_wrench)(cp2.m_wrench->min())==(*cp2.m_wrench)(cp2.m_wrench->min()));
  BOOST_CHECK((*cp5.m_zmp)(cp2.m_zmp->min())==(*cp2.m_zmp)(cp2.m_zmp->min()));
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
  curve_SE3_ptr_t curveSE3 = buildRandomSE3LinearTraj(1.5,2.9);
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
  cp5.addContact("test",ContactPatch(SE3::Identity().setRandom()));
  BOOST_CHECK(cp5 != cp2);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  cp5.addEffectorTrajectory("knee",buildRandomSE3LinearTraj(cp5.timeInitial(),cp5.timeFinal()));
  BOOST_CHECK(cp5 != cp2);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  cp5.addContactForceTrajectory("left_leg",buildRandomPolynomial12D());
  BOOST_CHECK(cp5 != cp2);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);
  cp5.addContactNormalForceTrajectory("left_leg",buildRandomPolynomial1D());
  BOOST_CHECK(cp5 != cp2);
  cp5 = cp2;
  BOOST_CHECK(cp5 == cp2);


  // check that when removing the contact the trajectory are correctly deleted :
  cp5.removeContact("left_leg");
  BOOST_CHECK(!cp5.isEffectorInContact("left_leg"));
  BOOST_CHECK(cp5.contactForces().count("left_leg") == 0);
  BOOST_CHECK(cp5.contactNormalForces().count("left_leg") == 0);
  // check that when adding a contact the effector trajectory is correctly deleted :
  cp5.addContact("right_leg",patchR);
  BOOST_CHECK(cp5.effectorTrajectories().count("right_leg") == 0);

  // check serialization
  std::string fileName("fileTest");
  ContactPhase cp_from_txt,cp_from_xml,cp_from_bin;
  std::cout<<"cp2 before serialization : "<<std::endl<<cp2<<std::endl;

  cp2.saveAsText(fileName+".txt");
  cp_from_txt.loadFromText(fileName+".txt");
  BOOST_CHECK(cp2 == cp_from_txt);
  std::cout<<"cp2 after deserialization : "<<std::endl<<cp_from_txt<<std::endl;

  cp2.saveAsXML(fileName+".xml","ContactPhase");
  cp_from_xml.loadFromXML(fileName+".xml","ContactPhase");
  BOOST_CHECK(cp2 == cp_from_xml);

  cp2.saveAsBinary(fileName);
  cp_from_bin.loadFromBinary(fileName);
  BOOST_CHECK(cp2 == cp_from_bin);

  explicitContactPhaseAssertEqual(cp2,cp_from_txt);

}

//BOOST_AUTO_TEST_CASE(contact_sequence)
//{
//  ContactPhase4 cp;
//  ContactSequence4 cs(1);
//  ContactSequence4 cs_test(0);
//  cs.m_contact_phases[0] = cp;
//  // test serialization
//  cs.saveAsText("serialization_cs_test.test");
//  cs_test.loadFromText("serialization_cs_test.test");
//  remove("serialization_cp_test.test");
//  BOOST_CHECK(cs == cs_test);
//}

BOOST_AUTO_TEST_SUITE_END()
