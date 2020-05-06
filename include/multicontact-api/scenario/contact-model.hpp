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
  ContactModelTpl() : m_mu(-1.), m_contact_type(ContactType::UNDEFINED) {}

  /// \brief Constructor with friction
  ContactModelTpl(const Scalar mu) : m_mu(mu), m_contact_type(ContactType::UNDEFINED) {}

  /// \brief Full constructor
  ContactModelTpl(const Scalar mu, const ContactType contact_type) : m_mu(mu), m_contact_type(contact_type) {}

  /// \brief Copy constructor
  template <typename S2>
  explicit ContactModelTpl(const ContactModelTpl<S2>& other)
      : m_mu(other.mu), m_contact_type(other.m_contact_type) {}

  template <typename S2>
  bool operator==(const ContactModelTpl<S2>& other) const {
    return m_mu == other.m_mu && m_contact_type == other.m_contact_type;
  }

  template <typename S2>
  bool operator!=(const ContactModelTpl<S2>& other) const {
    return !(*this == other);
  }

  void disp(std::ostream& os) const {
    os << "ContactType : " << m_contact_type << ", mu: " << m_mu << std::endl;
  }

  template <typename S2>
  friend std::ostream& operator<<(std::ostream& os, const ContactModelTpl<S2>& cp) {
    cp.disp(os);
    return os;
  }

  /// \brief Friction coefficient.
  Scalar m_mu;
  /// \brief ZMP radius.
  ContactType m_contact_type;

 private:
  // Serialization of the class
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const {
    ar& boost::serialization::make_nvp("mu", m_mu);
    ar& boost::serialization::make_nvp("contact_type", m_contact_type);
  }

  template <class Archive>
  void load(Archive& ar, const unsigned int /*version*/) {
    ar >> boost::serialization::make_nvp("mu", m_mu);
    ar >> boost::serialization::make_nvp("contact_type", m_contact_type);
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()
};
}  // namespace scenario
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_scenario_contact_model_planar_hpp__
