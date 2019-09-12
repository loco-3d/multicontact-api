// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_scenario_contact_patch_hpp__
#define __multicontact_api_scenario_contact_patch_hpp__

#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/geometry/linear-cone.hpp"
#include "multicontact-api/serialization/archive.hpp"
#include "multicontact-api/serialization/spatial.hpp"
#include "multicontact-api/scenario/contact-model-planar.hpp"

#include <pinocchio/spatial/se3.hpp>
#include <iostream>

namespace multicontact_api {
namespace scenario {

template <typename _Scalar>
struct ContactPatchTpl : public serialization::Serializable<ContactPatchTpl<_Scalar> > {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _Scalar Scalar;
  typedef pinocchio::SE3Tpl<Scalar, 0> SE3;
  typedef geometry::WrenchConeTpl<Scalar> LinearWrenchCone;
  typedef ContactModelPlanarTpl<Scalar> ContactModel;

  /// \brief Default constructor.
  ContactPatchTpl()
      : m_placement(SE3::Identity()), m_active(false), m_contact_model_placement(SE3::Identity()), m_lwc(0) {}

  /// \brief Init contact patch from a given placement.
  explicit ContactPatchTpl(const SE3& placement)
      : m_placement(placement), m_active(false), m_contact_model_placement(SE3::Identity()), m_lwc(0) {}

  /// \brief Copy constructor
  ContactPatchTpl(const ContactPatchTpl& other)
      : m_placement(other.m_placement),
        m_active(other.m_active),
        m_contact_model_placement(other.m_contact_model_placement),
        m_oMcm(other.m_oMcm),
        m_contact_model(other.m_contact_model),
        m_lwc(other.m_lwc) {}

  //      ContactPatchTpl & operator=(const ContactPatchTpl & other)
  //      {
  //        std::cout << "Copy op" << std::endl;
  //      }

  const SE3& placement() const { return m_placement; }
  SE3& placement() { return m_placement; }

  bool active() const { return m_active; }
  bool& active() { return m_active; }

  const SE3& contactModelPlacement() const { return m_contact_model_placement; }
  SE3& contactModelPlacement() { return m_contact_model_placement; }

  const SE3& worldContactModelPlacement() const { return m_oMcm; }
  SE3& worldContactModelPlacement() { return m_oMcm; }

  const ContactModel& contactModel() const { return m_contact_model; }
  ContactModel& contactModel() { return m_contact_model; }

  const LinearWrenchCone& lwc() const { return m_lwc; }
  LinearWrenchCone& lwc() { return m_lwc; }

  template <typename S2>
  bool operator==(const ContactPatchTpl<S2>& other) const {
    return m_placement == other.m_placement && m_active == other.m_active &&
           m_contact_model_placement == other.m_contact_model_placement && m_contact_model == other.m_contact_model &&
           m_lwc == other.m_lwc;
  }

  template <typename S2>
  bool operator!=(const ContactPatchTpl<S2>& other) const {
    return !(*this != other);
  }

  void disp(std::ostream& os) const {
    os << "placement:\n"
       << m_placement << std::endl
       << "contact model placement:\n"
       << m_contact_model_placement << std::endl
       << "active: " << (m_active ? "True" : "False") << std::endl;
  }

  template <typename S2>
  friend std::ostream& operator<<(std::ostream& os, const ContactPatchTpl<S2>& cp) {
    cp.disp(os);
    return os;
  }

 protected:
  /// \brief Placement of the contact patch
  SE3 m_placement;
  /// \brief Is the contact patch active?
  bool m_active;

  // Relative to contact model
  /// \brief Placement of the contact model w.r.t the contact patch
  SE3 m_contact_model_placement;

  /// \brief Placement of the contact model w.r.t the world
  SE3 m_oMcm;

  /// \brief Contact model (Planar,Bilateral)
  ContactModel m_contact_model;
  LinearWrenchCone m_lwc;

 private:
  // Serialization of the class
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const {
    ar& boost::serialization::make_nvp("placement", m_placement);
    ar& boost::serialization::make_nvp("active", m_active);
    ar& boost::serialization::make_nvp("contact_model_placement", m_contact_model_placement);
    ar& boost::serialization::make_nvp("contact_model", m_contact_model);
    ar& boost::serialization::make_nvp("lwc", m_lwc);
  }

  template <class Archive>
  void load(Archive& ar, const unsigned int /*version*/) {
    ar >> boost::serialization::make_nvp("placement", m_placement);
    ar >> boost::serialization::make_nvp("active", m_active);
    ar >> boost::serialization::make_nvp("contact_model_placement", m_contact_model_placement);
    ar >> boost::serialization::make_nvp("contact_model", m_contact_model);
    ar >> boost::serialization::make_nvp("lwc", m_lwc);
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()
};
}  // namespace scenario
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_scenario_contact_patch_hpp__
