// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_scenario_contact_model_planar_hpp__
#define __multicontact_api_scenario_contact_model_planar_hpp__

#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/serialization/archive.hpp"

#include <iostream>
#include <Eigen/Dense>

namespace multicontact_api {
namespace scenario {

template <typename _Scalar>
struct ContactModelTpl : public serialization::Serializable<ContactModelTpl<_Scalar> > {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;

  /// \brief Default constructor.
  ContactModelTpl() : m_mu(-1.), m_ZMP_radius(-1.) {}

  /// \brief Default constructor.
  ContactModelTpl(const Scalar mu, const Scalar ZMP_radius) : m_mu(mu), m_ZMP_radius(ZMP_radius) {}

  /// \brief Copy constructor
  template <typename S2>
  explicit ContactModelTpl(const ContactModelTpl<S2>& other)
      : m_mu(other.mu), m_ZMP_radius(other.ZMP_radius) {}

  template <typename S2>
  bool operator==(const ContactModelTpl<S2>& other) const {
    return m_mu == other.m_mu && m_ZMP_radius == other.m_ZMP_radius;
  }

  template <typename S2>
  bool operator!=(const ContactModelTpl<S2>& other) const {
    return !(*this == other);
  }

  void disp(std::ostream& os) const {
    os << "mu: " << m_mu << std::endl << "ZMP radius: " << m_ZMP_radius << std::endl;
  }

  template <typename S2>
  friend std::ostream& operator<<(std::ostream& os, const ContactModelTpl<S2>& cp) {
    cp.disp(os);
    return os;
  }

  /// \brief Friction coefficient.
  Scalar m_mu;
  /// \brief ZMP radius.
  Scalar m_ZMP_radius;

 private:
  // Serialization of the class
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const {
    ar& boost::serialization::make_nvp("mu", m_mu);
    ar& boost::serialization::make_nvp("ZMP_radius", m_ZMP_radius);
  }

  template <class Archive>
  void load(Archive& ar, const unsigned int /*version*/) {
    ar >> boost::serialization::make_nvp("mu", m_mu);
    ar >> boost::serialization::make_nvp("ZMP_radius", m_ZMP_radius);
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()
};
}  // namespace scenario
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_scenario_contact_model_planar_hpp__
