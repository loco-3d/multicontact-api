#ifndef __multicontact_api_scenario_contact_sequence_hpp__
#define __multicontact_api_scenario_contact_sequence_hpp__

#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/scenario/contact-phase.hpp"

#include "multicontact-api/serialization/archive.hpp"

#include <vector>
#include <boost/serialization/vector.hpp>

namespace multicontact_api {
namespace scenario {

template <class _ContactPhase>
struct ContactSequenceTpl : public serialization::Serializable<ContactSequenceTpl<_ContactPhase> > {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _ContactPhase ContactPhase;
  typedef typename ContactPhase::Scalar Scalar;
  typedef std::vector<ContactPhase> ContactPhaseVector;

  ContactSequenceTpl(const size_t size = 0) : m_contact_phases(size) {}

  /// \brief Copy contructor
  ContactSequenceTpl(const ContactSequenceTpl& other) : m_contact_phases(other.m_contact_phases) {}

  size_t size() const { return m_contact_phases.size(); }

  bool operator==(const ContactSequenceTpl& other) const { return m_contact_phases == other.m_contact_phases; }

  bool operator!=(const ContactSequenceTpl& other) const { return !(*this == other); }

  void resize(const size_t size) { m_contact_phases.resize(size); }

  /* Accessors to the contact Phases */

  /**
   * @brief append Add the given Phase at the end of the sequence
   * @param contactPhase the phase to end
   * @return The id of the phase added in the sequence
   */
  size_t append(const ContactPhase& contactPhase) {
    m_contact_phases.push_back(contactPhase);
    return m_contact_phases.size() - 1;
  }

  /**
   * @brief contactPhases return a Const copy of the contact phase vector in this sequence.
   * Prefer accessing the contact phases through the contactPhase(id) as this one create a copy
   * @return a Const copy of the contact phase vector in this sequence
   */
  const ContactPhaseVector contactPhases() const { return m_contact_phases; }

  /**
   * @brief contactPhase return a reference to the contactPhase stored at the given Id
   * @param id the desired Id in the contact sequence
   * @return a reference to the ContactPhase
   */
  ContactPhase& contactPhase(const size_t id) {
    if (id >= m_contact_phases.size())
      // throw std::invalid_argument("Contact Sequence size is "+m_contact_phases.size()+" given Id is "+id);
      throw std::invalid_argument("Given Id is greater than the vector size");
    return m_contact_phases.at(id);
  }

  /**
   * @brief removePhase remove the given contactPhase from the sequence
   * @param id the Id of the phase to remove
   */
  void removePhase(const size_t id) {
    if (id >= m_contact_phases.size()) throw std::invalid_argument("Given Id is greater than the vector size");
    m_contact_phases.erase(m_contact_phases.begin() + id);
  }

  /* End Accessors to the contact Phases */

  /* Helpers */
  /**
   * @brief breakContact Add a new contactPhase at the end of the current ContactSequence,
   * The new ContactPhase have the same ContactPatchs as the last phase of the sequence,
   * with the exeption of the given contact removed.
   * It copy all the 'final' values of the last phase as 'initial' values of the new phase.
   * It also set the duration of the previous last phase.
   * @param eeName the name of the effector to remove from contact
   * @param phaseDuration if provided, the duration of the previous last phase of the sequence is set to this value
   * (it is thus the duration BEFORE breaking the contact)
   * @throw invalid_argument if the phaseDuration is provided but the last phase do not have a time-range defined
   * @throw invalid_argument if eeName is not in contact in the last phase of the sequence
   */
  void breakContact(const std::string& eeName, const double phaseDuration = -1) {
    ContactPhase& lastPhase = m_contact_phases.back();
    if (!lastPhase.isEffectorInContact(eeName))
      throw std::invalid_argument("In breakContact : effector is not currently in contact : " + eeName);
    if (phaseDuration > 0) {
      if (lastPhase.timeInitial() < 0) {
        throw std::invalid_argument(
            "In breakContact : duration is specified but current phase interval in not initialised.");
      } else {
        lastPhase.duration(phaseDuration);
      }
    }
    ContactPhase phase;
    // set initial values from last values of previous phase :
    phase.timeInitial(lastPhase.timeFinal());
    phase.m_c_init = lastPhase.m_c_final;
    phase.m_dc_init = lastPhase.m_dc_final;
    phase.m_ddc_init = lastPhase.m_ddc_final;
    phase.m_L_init = lastPhase.m_L_final;
    phase.m_dL_init = lastPhase.m_dL_final;
    phase.m_q_init = lastPhase.m_q_final;

    // copy contact patchs :
    for (std::string name : lastPhase.effectorsInContact()) {
      if (name != eeName) {
        phase.addContact(name, lastPhase.contactPatch(name));
      }
    }
    // add new phase at the end of the sequence
    append(phase);
    return;
  }

  /**
   * @brief createContact Add a new contactPhase at the end of the current ContactSequence,
   * The new ContactPhase have the same ContactPatchs as the last phase of the sequence,
   * with the exeption of the given contact added.
   * @param eeName the name of the effector used to create contact
   * @param patch the ContactPatch of the new contact
   * @param phaseDuration if provided, the duration of the previous last phase of the sequence is set to this value
   * (it is thus the duration BEFORE creating the contact)
   * @throw invalid_argument if the phaseDuration is provided but the last phase do not have a time-range defined
   * @throw invalid_argument if eeName is already in contact in the last phase of the sequence
   */
  void createContact(const std::string& eeName, const ContactPatch& patch, const double phaseDuration = -1) {
    ContactPhase& lastPhase = m_contact_phases.back();
    if (lastPhase.isEffectorInContact(eeName))
      throw std::invalid_argument("In createContact : effector is already in contact : " + eeName);
    if (phaseDuration > 0) {
      if (lastPhase.timeInitial() < 0) {
        throw std::invalid_argument(
            "In createContact : duration is specified but current phase interval in not initialised.");
      } else {
        lastPhase.duration(phaseDuration);
      }
    }
    ContactPhase phase;
    // set initial values from last values of previous phase :
    phase.timeInitial(lastPhase.timeFinal());
    phase.m_c_init = lastPhase.m_c_final;
    phase.m_dc_init = lastPhase.m_dc_final;
    phase.m_ddc_init = lastPhase.m_ddc_final;
    phase.m_L_init = lastPhase.m_L_final;
    phase.m_dL_init = lastPhase.m_dL_final;
    phase.m_q_init = lastPhase.m_q_final;

    // copy contact patchs :
    for (std::string name : lastPhase.effectorsInContact()) {
      phase.addContact(name, lastPhase.contactPatch(name));
    }
    // add new contact to new phase :
    phase.addContact(eeName, patch);
    // add new phase at the end of the sequence
    append(phase);
    return;
  }

  /**
   * @brief moveEffectorToPlacement Add two new phases at the end of the current ContactSequence,
   *  - it break the contact with eeName
   *  - it create the contact with eeName at the given placement.
   * It copy all the 'final' values of the last phase as 'initial' values of the new phase.
   * It also set the duration of the previous last phase.
   * @param eeName the name of the effector used to create contact
   * @param placement the new placement for the contact of eeName
   * @param durationBreak the duration of the previous last phase of the sequence
   *  (it is thus the duration BEFORE breaking the contact)
   * @param durationCreate the duration of the first new ContactPhase
   *  (it is thus the duration BEFORE creating the contact)
   * @throw invalid_argument if the phaseDuration is provided but the last phase do not have a time-range defined
   * @throw invalid_argument if eeName is not in contact in the last phase of the sequence
   */
  void moveEffectorToPlacement(const std::string& eeName, const ContactPatch::SE3& placement,
                               const double durationBreak = -1, const double durationCreate = -1) {
    if (!m_contact_phases.back().isEffectorInContact(eeName))
      throw std::invalid_argument("In moveEffectorToPlacement : effector is not currently in contact : " + eeName);
    ContactPatch target(m_contact_phases.back().contactPatch(eeName));
    target.placement() = placement;
    breakContact(eeName, durationBreak);
    // copy all "init" value to "final" for the current last phase :
    ContactPhase& lastPhase = m_contact_phases.back();
    lastPhase.m_c_final = lastPhase.m_c_init;
    lastPhase.m_dc_final = lastPhase.m_dc_init;
    lastPhase.m_ddc_final = lastPhase.m_ddc_init;
    lastPhase.m_L_final = lastPhase.m_L_init;
    lastPhase.m_dL_final = lastPhase.m_dL_init;
    lastPhase.m_q_final = lastPhase.m_q_init;
    createContact(eeName, target, durationCreate);
    return;
  }

  /**
   * @brief moveEffectorOf similar to moveEffectorToPlacement
   * exept that the new placement is defined from the previous placement and a given transform applied.
   * @param eeName the name of the effector used to create contact
   * @param transform the transform applied to the placement of the contact in the last phase of the sequence
   * @param durationBreak the duration of the previous last phase of the sequence
   *  (it is thus the duration BEFORE breaking the contact)
   * @param durationCreate the duration of the first new ContactPhase
   *  (it is thus the duration BEFORE creating the contact)
   * @throw invalid_argument if the phaseDuration is provided but the last phase do not have a time-range defined
   * @throw invalid_argument if eeName is not in contact in the last phase of the sequence
   */
  void moveEffectorOf(const std::string& eeName, const ContactPatch::SE3& transform, const double durationBreak = -1,
                      const double durationCreate = -1) {
    if (!m_contact_phases.back().isEffectorInContact(eeName))
      throw std::invalid_argument("In moveEffectorToPlacement : effector is not currently in contact : " + eeName);
    ContactPatch::SE3 previous(m_contact_phases.back().contactPatch(eeName).placement());
    ContactPatch::SE3 target = previous.act(transform);
    return moveEffectorToPlacement(eeName, target, durationBreak, durationCreate);
  }

  /**
   * @brief isTimeConsistent Check if all the time intervals are defined and consistent
   * (ie. the time always increase and the final time of one phase is equal to the initial one of the newt phase)
   * @return true if the sequence is consistent, false otherwise
   */
  bool isTimeConsistent() const {
    double current_t = m_contact_phases.front().timeInitial();
    if (current_t < 0.) {
      std::cout << "Initial time is negative." << std::endl;
      return false;
    }
    for (size_t i = 0; i < size(); ++i) {
      const ContactPhase& phase = m_contact_phases.at(i);
      if (phase.timeInitial() != current_t) {
        std::cout << "For phase " << i << " initial time is not equal to previous final time." << std::endl;
        return false;
      }
      if (phase.timeInitial() > phase.timeFinal()) {
        std::cout << "For phase " << i << " final time is before initial time." << std::endl;
        return false;
      }
      current_t = phase.timeFinal();
    }
    return true;
  }

  /* End Helpers */

  /*Public Attributes*/
  ContactPhaseVector m_contact_phases;
  /*Public Attributes*/

 private:
  // Serialization of the class
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const {
    const size_t m_size = size();
    ar& boost::serialization::make_nvp("size", m_size);
    for (typename ContactPhaseVector::const_iterator it = m_contact_phases.begin(); it != m_contact_phases.end();
         ++it) {
      ar& boost::serialization::make_nvp("contact_phase", *it);
    }
  }

  template <class Archive>
  void load(Archive& ar, const unsigned int /*version*/) {
    size_t m_size;
    ar >> boost::serialization::make_nvp("size", m_size);
    assert(m_size > 0);
    resize(m_size);
    for (typename ContactPhaseVector::iterator it = m_contact_phases.begin(); it != m_contact_phases.end(); ++it) {
      ar >> boost::serialization::make_nvp("contact_phase", *it);
    }
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()

};  // end class ContactSequence

}  // namespace scenario
}  // namespace multicontact_api
#endif  // __multicontact_api_scenario_contact_sequence_hpp__
