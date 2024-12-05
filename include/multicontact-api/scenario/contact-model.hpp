// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_scenario_contact_model_planar_hpp__
#define __multicontact_api_scenario_contact_model_planar_hpp__

#include <Eigen/Dense>
#include <iostream>
#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/skew.hpp>

#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/serialization/archive.hpp"
#include "multicontact-api/serialization/eigen-matrix.hpp"

namespace multicontact_api {
namespace scenario {

template <typename _Scalar>
struct ContactModelTpl
    : public serialization::Serializable<ContactModelTpl<_Scalar> > {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
  typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix3X;
  typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic> Matrix6X;

  /// \brief Default constructor.
  ContactModelTpl()
      : m_mu(-1.),
        m_contact_type(ContactType::CONTACT_UNDEFINED),
        m_num_contact_points(1),
        m_contact_points_positions(Matrix3X::Zero(3, 1)) {}

  /// \brief Constructor with friction
  ContactModelTpl(const Scalar mu)
      : m_mu(mu),
        m_contact_type(ContactType::CONTACT_UNDEFINED),
        m_num_contact_points(1),
        m_contact_points_positions(Matrix3X::Zero(3, 1)) {}

  /// \brief Full constructor
  ContactModelTpl(const Scalar mu, const ContactType contact_type)
      : m_mu(mu),
        m_contact_type(contact_type),
        m_num_contact_points(1),
        m_contact_points_positions(Matrix3X::Zero(3, 1)) {}

  /// \brief Copy constructor
  template <typename S2>
  explicit ContactModelTpl(const ContactModelTpl<S2>& other)
      : m_mu(other.mu),
        m_contact_type(other.m_contact_type),
        m_contact_points_positions(other.m_contact_points_positions),
        m_num_contact_points(other.m_num_contact_points) {}

  template <typename S2>
  bool operator==(const ContactModelTpl<S2>& other) const {
    return m_mu == other.m_mu && m_contact_type == other.m_contact_type &&
           m_num_contact_points == other.m_num_contact_points &&
           m_contact_points_positions == other.m_contact_points_positions;
  }

  template <typename S2>
  bool operator!=(const ContactModelTpl<S2>& other) const {
    return !(*this == other);
  }

  void disp(std::ostream& os) const {
    os << "ContactType: " << m_contact_type << ", mu: " << m_mu << std::endl
       << "Number of contact points: " << m_num_contact_points
       << ", positions: " << std::endl
       << m_contact_points_positions << std::endl;
  }

  template <typename S2>
  friend std::ostream& operator<<(std::ostream& os,
                                  const ContactModelTpl<S2>& cp) {
    cp.disp(os);
    return os;
  }

  size_t num_contact_points() const { return m_num_contact_points; }

  void num_contact_points(const size_t num) {
    m_num_contact_points = num;
    m_contact_points_positions = Matrix3X::Zero(3, num);
  }

  Matrix3X contact_points_positions() const {
    return m_contact_points_positions;
  }

  void contact_points_positions(const Matrix3X& positions) {
    m_contact_points_positions = positions;
    m_num_contact_points = positions.cols();
  }

  /**
   * @brief generatorMatrix Return a 6x(num_contact_points*3) matrix
   * containing the generator used to compute contact forces.
   * @return
   */
  Matrix6X generatorMatrix() const {
    Matrix6X gen = Matrix6X::Zero(6, m_num_contact_points * 3);
    for (size_t i = 0; i < m_num_contact_points; i++) {
      gen.block(0, i * 3, 3, 3) = Matrix3::Identity();
      gen.block(3, i * 3, 3, 3) =
          pinocchio::skew(m_contact_points_positions.col(i));
    }
    return gen;
  }

  /// \brief Friction coefficient.
  Scalar m_mu;
  /// \brief ZMP radius.
  ContactType m_contact_type;

 private:
  /// \brief The number of contact points used to model this contact
  size_t m_num_contact_points;
  /// \brief 3xnum_contact_points matrix defining the contact points positions
  /// in the frame of the contact placement
  Matrix3X m_contact_points_positions;

  // Serialization of the class
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const {
    ar& boost::serialization::make_nvp("mu", m_mu);
    ar& boost::serialization::make_nvp("contact_type", m_contact_type);
    ar& boost::serialization::make_nvp("num_contact_points",
                                       m_num_contact_points);
    ar& boost::serialization::make_nvp("contact_points_positions",
                                       m_contact_points_positions);
  }

  template <class Archive>
  void load(Archive& ar, const unsigned int /*version*/) {
    ar >> boost::serialization::make_nvp("mu", m_mu);
    ar >> boost::serialization::make_nvp("contact_type", m_contact_type);
    ar >> boost::serialization::make_nvp("num_contact_points",
                                         m_num_contact_points);
    ar >> boost::serialization::make_nvp("contact_points_positions",
                                         m_contact_points_positions);
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()
};
}  // namespace scenario
}  // namespace multicontact_api

MULTICONTACT_API_DEFINE_CLASS_TEMPLATE_VERSION(
    typename Scalar, multicontact_api::scenario::ContactModelTpl<Scalar>)

#endif  // ifndef __multicontact_api_scenario_contact_model_planar_hpp__
