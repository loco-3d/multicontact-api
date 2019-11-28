
#ifndef __multicontact_api_scenario_contact_patch_hpp__
#define __multicontact_api_scenario_contact_patch_hpp__

#include "multicontact-api/scenario/fwd.hpp"
#include <pinocchio/spatial/se3.hpp>
#include "multicontact-api/serialization/archive.hpp"
#include "multicontact-api/serialization/spatial.hpp"

namespace multicontact_api {
namespace scenario {

template <typename _Scalar>
struct ContactPatchTpl : public serialization::Serializable<ContactPatchTpl<_Scalar> > {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef pinocchio::SE3Tpl<Scalar, 0> SE3;

  /// \brief Default constructor.
  ContactPatchTpl()
      : m_placement(SE3::Identity()), m_mu(-1.) {}

  /// \brief Init contact patch from a given placement.
  explicit ContactPatchTpl(const SE3& placement)
      : m_placement(placement), m_mu(-1.) {}

  /// \brief Init contact patch from a given placement and a friction coefficient
  ContactPatchTpl(const SE3& placement, const Scalar mu)
      : m_placement(placement), m_mu(mu) {}

  /// \brief Copy constructor
  ContactPatchTpl(const ContactPatchTpl& other)
      : m_placement(other.m_placement),
        m_mu(other.m_mu) {}

  const SE3& placement() const { return m_placement; }
  SE3& placement() { return m_placement; }

  const Scalar& friction() const { return m_mu; }
  Scalar& friction() { return m_mu; }


  template <typename S2>
  bool operator==(const ContactPatchTpl<S2>& other) const {
    return m_placement == other.m_placement && m_mu == other.m_mu;
  }

  template <typename S2>
  bool operator!=(const ContactPatchTpl<S2>& other) const {
    return !(*this == other);
  }


  void disp(std::ostream& os) const {
    os <<"Placement:\n"
       << m_placement << std::endl
       <<"Friction coefficient : "<<m_mu<<std::endl;
  }

  template <typename S2>
  friend std::ostream& operator<<(std::ostream& os, const ContactPatchTpl<S2>& cp) {
    cp.disp(os);
    return os;
  }


protected:
 /// \brief Placement of the contact patch
 SE3 m_placement;
 /// \brief friction coefficient for this contact
 Scalar m_mu;

private:
 // Serialization of the class
 friend class boost::serialization::access;

 template <class Archive>
 void save(Archive& ar, const unsigned int /*version*/) const {
   ar& boost::serialization::make_nvp("placement", m_placement);
   ar& boost::serialization::make_nvp("mu", m_mu);
 }

 template <class Archive>
 void load(Archive& ar, const unsigned int /*version*/) {
   ar >> boost::serialization::make_nvp("placement", m_placement);
   ar >> boost::serialization::make_nvp("mu", m_mu);

 }

 BOOST_SERIALIZATION_SPLIT_MEMBER() // why is it required ? using only serialize() lead to compilation error, probably because of the SE3

}; // struct ContactPatchTpl
}//scenario
}//multicontact_api


#endif // __multicontact_api_scenario_contact_patch_hpp__
