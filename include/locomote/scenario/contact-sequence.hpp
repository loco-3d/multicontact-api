// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>
// Simplified BSD license :
//Redistribution and use in source and binary forms, with or without modification,
//are permitted provided that the following conditions are met:

//1. Redistributions of source code must retain the above copyright notice,
//this list of conditions and the following disclaimer.

//2. Redistributions in binary form must reproduce the above copyright notice,
//this list of conditions and the following disclaimer in the documentation
//and/or other materials provided with the distribution.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
//OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
//ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef __locomote_scenario_contact_sequence_hpp__
#define __locomote_scenario_contact_sequence_hpp__

#include "locomote/scenario/fwd.hpp"
#include "locomote/scenario/contact-phase.hpp"
#include "locomote/scenario/ms-interval.hpp"

#include "locomote/serialization/archive.hpp"

#include <vector>
#include <Eigen/StdVector>
#include <pinocchio/container/aligned-vector.hpp>

namespace locomote
{
  namespace scenario
  {
    
    template<class _ContactPhase>
    struct ContactSequenceTpl : public serialization::Serializable< ContactSequenceTpl<_ContactPhase> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef _ContactPhase ContactPhase;
      typedef typename ContactPhase::Scalar Scalar;
      
      typedef typename ContactPhase::TimeVector TimeVector;
      typedef typename ContactPhase::StateVector StateVector;
      typedef typename ContactPhase::ConfigurationVector ConfigurationVector;
      
      typedef MSIntervalDataTpl<TimeVector,StateVector,ConfigurationVector> MSIntervalData;
      typedef se3::container::aligned_vector<MSIntervalData> MSIntervalDataVector;
      
      typedef se3::container::aligned_vector<ContactPhase> ContactPhaseVector;
//      typedef std::vector<ContactPhase, Eigen::aligned_allocator<ContactPhase> > ContactPhaseVector;
      
      ContactSequenceTpl(const size_t size = 0)
      : m_contact_phases(size)
      , m_ms_interval_data(0)
      , m_conic_type(CONIC_UNDEFINED)
      {}
      
      /// \brief Copy contructor
      ContactSequenceTpl(const ContactSequenceTpl & other)
      : m_contact_phases(other.m_contact_phases)
      , m_ms_interval_data(other.m_ms_interval_data)
      , m_conic_type(other.m_conic_type)
      {}
      
      size_t size() const { return m_contact_phases.size(); }
      
//    protected:
     
      ContactPhaseVector m_contact_phases;
      
      bool operator==(const ContactSequenceTpl & other) const
      {
        return
        m_contact_phases == other.m_contact_phases
        && m_ms_interval_data == other.m_ms_interval_data
        && m_conic_type == other.m_conic_type
        ;
      }
      
      bool operator!=(const ContactSequenceTpl & other) const
      { return !(*this == other); }
      
      void resize(const size_t size)
      {
        m_contact_phases.resize(size);
      }
      
      MSIntervalDataVector m_ms_interval_data;
      
      ConicType m_conic_type;
      
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
          ar & boost::serialization::make_nvp("contact_phase",*it);
        
        ar & boost::serialization::make_nvp("ms_interval_data",m_ms_interval_data);
        ar & boost::serialization::make_nvp("conic_type",m_conic_type);
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
        
        ar >> boost::serialization::make_nvp("ms_interval_data",m_ms_interval_data);
        ar >> boost::serialization::make_nvp("conic_type",m_conic_type);
        
      }
      
      BOOST_SERIALIZATION_SPLIT_MEMBER()
    };
  }
}

#endif // ifndef __locomote_scenario_contact_sequence_hpp__
