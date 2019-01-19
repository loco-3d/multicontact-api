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
#ifndef __locomote_scenario_contact_phase_hpp__
#define __locomote_scenario_contact_phase_hpp__

#include "locomote/scenario/fwd.hpp"
#include "locomote/scenario/contact-patch.hpp"

#include "locomote/serialization/archive.hpp"
#include "locomote/serialization/eigen-matrix.hpp"
#include "locomote/serialization/spatial.hpp"
#include "locomote/geometry/second-order-cone.hpp"
#include "locomote/geometry/linear-cone.hpp"
#include "locomote/container/ref.hpp"

#include <boost/array.hpp>

namespace locomote
{
  namespace scenario
  {
    
    template<typename _Scalar, int _dim>
    struct ContactPhaseTpl : public serialization::Serializable< ContactPhaseTpl<_Scalar,_dim> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef _Scalar Scalar;
      typedef ContactPatchTpl<Scalar> ContactPatch;
      typedef boost::array<ContactPatch,_dim> ContactPatchArray;
      typedef std::vector< container::comparable_reference_wrapper<ContactPatch> > ContactPatchVector;
      typedef std::vector< container::comparable_reference_wrapper<ContactPatch const> > ConstContactPatchVector;
      typedef geometry::SecondOrderCone<_Scalar,6> SOC6;
      typedef geometry::WrenchConeTpl<_Scalar> WrenchCone;
      typedef Eigen::DenseIndex Index;
      typedef typename ContactPatch::SE3 SE3;
      typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
      
      enum { dim = _dim };
      
      /// \brief Default constructor
      ContactPhaseTpl()
      : m_sowc_placement(SE3::Identity())
      {}
      
      /// \brief Copy constructor
      template<typename S2>
      ContactPhaseTpl(const ContactPhaseTpl<S2,dim> & other)
      : m_contact_patches(other.m_contact_patches)
      , m_sowc(other.m_sowc)
      , m_sowc_placement(other.m_sowc_placement)
      , m_lwc(other.m_lwc)
      {}
      
      const ContactPatchArray & contact_patches() const { return m_contact_patches; }
      ContactPatchArray & contact_patches() { return m_contact_patches; }
      
      /// \Returns a reference of the second order wrench cone
      const SOC6 & sowc() const { return m_sowc; }
      SOC6 & sowc() { return m_sowc; }
      
      const SE3 & sowcPlacement() const { return m_sowc_placement; }
      SE3 & sowcPlacement() { return m_sowc_placement; }
      
      const Matrix6x & doubleDescription() const { return m_double_description; }
      Matrix6x & doubleDescription() { return m_double_description; }
      
      const WrenchCone & lwc() const { return m_lwc; }
      WrenchCone & lwc() { return m_lwc; }
      
      /// \returns the number of active patches
      Index numActivePatches() const
      {
        Index num_active = 0;
        for(typename ContactPatchArray::const_iterator it = m_contact_patches.begin();
            it != m_contact_patches.end(); ++it)
          if(it->active()) num_active++;
        
        return num_active;
      }
      
      /// \returns the number of inactive patches
      Index numInactivePatches() const
      { return dim - numActivePatches(); }
      
      template<typename S2>
      bool operator==(const ContactPhaseTpl<S2,dim> & other) const
      {
        return
        m_contact_patches == other.m_contact_patches
        && m_lwc == other.m_lwc
        && m_sowc == other.m_sowc
        ;
      }
      
      template<typename S2>
      bool operator!=(const ContactPhaseTpl<S2,dim> & other) const
      { return !(*this == other); }
      
      ContactPatchArray m_contact_patches; // TODO: set protected
    
      ///
      /// \brief Returns the first active patch.
      ///
      /// \remark This method is useful one looks for the lonely active patch.
      ///
      /// \returns the first active patch.
      ///
      const ContactPatch & getActivePatch() const
      {
        for(typename ContactPatchArray::const_iterator it = m_contact_patches.begin();
            it != m_contact_patches.end(); ++it)
          if(it->active()) return *it;
      }
      ContactPatch & getActivePatch()
      { return const_cast<ContactPatch &>(static_cast<const ContactPhaseTpl*>(this)->getActivePatch()); }
      
      ContactPatchVector getActivePatches()
      {
        ContactPatchVector res; res.reserve((size_t)dim);
        for(typename ContactPatchArray::iterator it = m_contact_patches.begin();
            it != m_contact_patches.end(); ++it)
          if(it->active()) res.push_back(typename ContactPatchVector::value_type(*it));
        
        return res;
      }
      
    protected:
      
      /// \brief Second Order Wrench Cone (SOWC) representing the Minkoski sum of the patch linear wrench cone.
      SOC6 m_sowc;
      SE3 m_sowc_placement;
      
      Matrix6x m_double_description;
      
      
      /// \brief Linear Wrench Cone (LWC) representing the Minkoski sum of the patch linear wrench cone.
      WrenchCone m_lwc;
      
    private:
      
      // Serialization of the class
      friend class boost::serialization::access;
      
      template<class Archive>
      void save(Archive & ar, const unsigned int /*version*/) const
      {
        for(typename ContactPatchArray::const_iterator it = m_contact_patches.begin();
            it != m_contact_patches.end(); ++it)
          ar & boost::serialization::make_nvp("contact_patch",*it);
        
        ar & boost::serialization::make_nvp("lwc",m_lwc);
        ar & boost::serialization::make_nvp("sowc",m_sowc);
        ar & boost::serialization::make_nvp("sowc_placement",m_sowc_placement);
        ar & boost::serialization::make_nvp("double_description",m_double_description);
      }
      
      template<class Archive>
      void load(Archive & ar, const unsigned int /*version*/)
      {
        for(typename ContactPatchArray::iterator it = m_contact_patches.begin();
            it != m_contact_patches.end(); ++it)
          ar >> boost::serialization::make_nvp("contact_patch",*it);
        
        ar >> boost::serialization::make_nvp("lwc",m_lwc);
        ar >> boost::serialization::make_nvp("sowc",m_sowc);
        ar >> boost::serialization::make_nvp("sowc_placement",m_sowc_placement);
        ar >> boost::serialization::make_nvp("double_description",m_double_description);
      }
      
      BOOST_SERIALIZATION_SPLIT_MEMBER()
    };
  }
}

#endif // ifndef __locomote_scenario_contact_phase_hpp__
