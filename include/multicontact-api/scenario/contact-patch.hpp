
#ifndef __multicontact_api_scenario_contact_patch_hpp__
#define __multicontact_api_scenario_contact_patch_hpp__

#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/scenario/contact-model.hpp"
#include <pinocchio/spatial/se3.hpp>
#include "multicontact-api/serialization/archive.hpp"
#include "multicontact-api/serialization/spatial.hpp"

namespace multicontact_api {
namespace scenario {

template <typename _Scalar>
struct ContactPatchTpl : public serialization::Serializable<ContactPatchTpl<_Scalar> > {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef ContactModelTpl<_Scalar> ContactModel;
  typedef pinocchio::SE3Tpl<Scalar, 0> SE3;

  /// \brief Default constructor.
  ContactPatchTpl() : m_contact_model(), m_placement(SE3::Identity()) {}

  /// \brief Init contact patch from a given placement.
  explicit ContactPatchTpl(const SE3& placement) : m_contact_model(), m_placement(placement) {}

  /// \brief Init contact patch from a given placement and a friction coefficient
  ContactPatchTpl(const SE3& placement, const Scalar mu) : m_contact_model(mu), m_placement(placement) {}

  /// \brief Init contact patch from a given placement and a contact model
  ContactPatchTpl(const SE3& placement, const ContactModel contact_model)
      : m_contact_model(contact_model), m_placement(placement) {}

  /// \brief Copy constructor
  ContactPatchTpl(const ContactPatchTpl& other)
      : m_contact_model(other.m_contact_model), m_placement(other.m_placement) {}

  const SE3& placement() const { return m_placement; }
  SE3& placement() { return m_placement; }

  const Scalar& friction() const { return m_contact_model.m_mu; }
  Scalar& friction() { return m_contact_model.m_mu; }

  template <typename S2>
  bool operator==(const ContactPatchTpl<S2>& other) const {
    return m_placement == other.m_placement && m_contact_model == other.m_contact_model;
  }

  template <typename S2>
  bool operator!=(const ContactPatchTpl<S2>& other) const {
    return !(*this == other);
  }

  void disp(std::ostream& os) const {
    os << "Placement:\n" << m_placement << std::endl << "ContactModel : " << m_contact_model << std::endl;
  }

  template <typename S2>
  friend std::ostream& operator<<(std::ostream& os, const ContactPatchTpl<S2>& cp) {
    cp.disp(os);
    return os;
  }

  /// \brief Contact model of this contact
  ContactModel m_contact_model;

 protected:
  /// \brief Placement of the contact patch
  SE3 m_placement;

 private:
  // Serialization of the class
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const {
    ar& boost::serialization::make_nvp("placement", m_placement);
    ar& boost::serialization::make_nvp("contact_model", m_contact_model);
  }

  template <class Archive>
  void load(Archive& ar, const unsigned int version) {
    ar >> boost::serialization::make_nvp("placement", m_placement);
    if (version >= 1) {
      ar >> boost::serialization::make_nvp("contact_model", m_contact_model);
    } else {
      double mu;
      ar >> boost::serialization::make_nvp("mu", mu);
      m_contact_model = ContactModel(mu);
    }
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()  // why is it required ? using only serialize() lead to compilation error,
                                      // probably because of the SE3

};  // struct ContactPatchTpl
}  // namespace scenario
}  // namespace multicontact_api

DEFINE_CLASS_TEMPLATE_VERSION(typename Scalar, multicontact_api::scenario::ContactPatchTpl<Scalar>)

#endif  // __multicontact_api_scenario_contact_patch_hpp__
