#ifndef __multicontact_api_scenario_contact_sequence_hpp__
#define __multicontact_api_scenario_contact_sequence_hpp__


#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/scenario/contact-phase.hpp"

#include "multicontact-api/serialization/archive.hpp"


#include <vector>
#include <boost/serialization/vector.hpp>


namespace multicontact_api{
namespace scenario  {

template<class _ContactPhase>
struct ContactSequenceTpl : public serialization::Serializable< ContactSequenceTpl<_ContactPhase> >
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _ContactPhase ContactPhase;
  typedef typename ContactPhase::Scalar Scalar;
  typedef std::vector<ContactPhase> ContactPhaseVector;


    ContactSequenceTpl(const size_t size = 0)
    : m_contact_phases(size)
    {}

    /// \brief Copy contructor
    ContactSequenceTpl(const ContactSequenceTpl & other)
    : m_contact_phases(other.m_contact_phases)
    {}

    size_t size() const { return m_contact_phases.size(); }

    bool operator==(const ContactSequenceTpl & other) const
    {
      return
      m_contact_phases == other.m_contact_phases
      ;
    }

    bool operator!=(const ContactSequenceTpl & other) const
    { return !(*this == other); }

    void resize(const size_t size)
    {
      m_contact_phases.resize(size);
    }

    /* Accessors to the contact Phases */

    /**
     * @brief append Add the given Phase at the end of the sequence
     * @param contactPhase the phase to end
     * @return The id of the phase added in the sequence
     */
    size_t append(const ContactPhase& contactPhase){
      m_contact_phases.push_back(contactPhase);
      return m_contact_phases.size()-1;
    }

    /**
     * @brief contactPhases return a Const copy of the contact phase vector in this sequence.
     * Prefer accessing the contact phases through the contactPhase(id) as this one create a copy
     * @return a Const copy of the contact phase vector in this sequence
     */
    const ContactPhaseVector contactPhases() const{
      return m_contact_phases;
    }

    /**
     * @brief contactPhase return a reference to the contactPhase stored at the given Id
     * @param id the desired Id in the contact sequence
     * @return a reference to the ContactPhase
     */
    ContactPhase& contactPhase(const size_t id){
      if(id >= m_contact_phases.size())
        //throw std::invalid_argument("Contact Sequence size is "+m_contact_phases.size()+" given Id is "+id);
        throw std::invalid_argument("Given Id is greater than the vector size");
      return m_contact_phases.at(id);
    }

    /**
     * @brief removePhase remove the given contactPhase from the sequence
     * @param id the Id of the phase to remove
     */
    void removePhase(const size_t id){
      if(id >= m_contact_phases.size())
        throw std::invalid_argument("Given Id is greater than the vector size");
      m_contact_phases.erase(m_contact_phases.begin() + id);
    }

    /* End Accessors to the contact Phases */

    /* Helpers */



    /* End Helpers */

     /*Public Attributes*/
    ContactPhaseVector m_contact_phases;
    /*Public Attributes*/

  private:




    // Serialization of the class
    friend class boost::serialization::access;

    template<class Archive>
    void save(Archive & ar, const unsigned int /*version*/) const
    {
      const size_t m_size = size();
      ar & boost::serialization::make_nvp("size",m_size);
      for(typename ContactPhaseVector::const_iterator it = m_contact_phases.begin();
          it != m_contact_phases.end(); ++it)
      {
        ar & boost::serialization::make_nvp("contact_phase",*it);
      }
    }

    template<class Archive>
    void load(Archive & ar, const unsigned int /*version*/)
    {
      size_t m_size;
      ar >> boost::serialization::make_nvp("size",m_size);
      assert(m_size>0);
      resize(m_size);
      for(typename ContactPhaseVector::iterator it = m_contact_phases.begin();
          it != m_contact_phases.end(); ++it)
      {
        ar >> boost::serialization::make_nvp("contact_phase",*it);
      }
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

}; // end class ContactSequence

}//namespace scenario
} // namespace multicontact
#endif // __multicontact_api_scenario_contact_sequence_hpp__
