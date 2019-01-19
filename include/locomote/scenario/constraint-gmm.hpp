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
#ifndef __locomote_scenario_constraint_gmm_hpp__
#define __locomote_scenario_constraint_gmm_hpp__

#include "locomote/scenario/constraint.hpp"
#include "locomote/serialization/archive.hpp"

namespace locomote
{
  namespace scenario
  {
    template<class GMM>
    struct traits< ContactConstraintGMM<GMM> >
    {
      typedef typename GMM::Scalar Scalar;
      enum
      {
        dim_in = GMM::dim,
        dim_out = 1
      };
      typedef Eigen::Matrix<Scalar,GMM::dim,1> ArgumentType;
      typedef Scalar ReturnType;
    };
    
    template<class _GMM>
    struct ContactConstraintGMM
    : public ConstraintBase< ContactConstraintGMM<_GMM> >
//    , public serialization::Serializable< ContactConstraintGMM<_GMM> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef _GMM GMM;
      typedef typename GMM::Scalar Scalar;
      typedef Eigen::DenseIndex Index;
      typedef typename traits<ContactConstraintGMM>::ArgumentType ArgumentType;
      typedef typename traits<ContactConstraintGMM>::ReturnType ReturnType;
      
      ContactConstraintGMM(const GMM * gmm_ptr = NULL, const ReturnType & offset = 0.)
      : m_gmm_ptr(gmm_ptr)
      , m_offset(offset)
      {}
      
      static Index inputSize() { return traits< ContactConstraintGMM<GMM> >::dim_in; }
      static Index ouputSize() { return traits< ContactConstraintGMM<GMM> >::dim_out; }
      static Index neq() { return 0; }
      
      ReturnType value(const ArgumentType & point) const
      { assert(m_gmm_ptr != NULL); return m_gmm_ptr->pdf(point)[0]; }
      
      ReturnType residu(const ArgumentType & point) const
      { assert(m_gmm_ptr != NULL); return m_gmm_ptr->pdf(point)[0] - m_offset; }
      
      const GMM * m_gmm_ptr;
      ReturnType m_offset;
      
    private:
      
//      // Serialization of the class
//      friend class boost::serialization::access;
//      
//      template<class Archive>
//      void save(Archive & ar, const unsigned int /*version*/) const
//      {
//        ar & boost::serialization::make_nvp("gmm",m_gmm);
//        ar & boost::serialization::make_nvp("offset",m_offset);
//      }
//      
//      template<class Archive>
//      void load(Archive & ar, const unsigned int /*version*/)
//      {
//        ar >> boost::serialization::make_nvp("gmm",m_gmm);
//        ar >> boost::serialization::make_nvp("offset",m_offset);
//      }
//      
//      BOOST_SERIALIZATION_SPLIT_MEMBER()
    };
  }
}

#endif // ifndef __locomote_scenario_constraint_gmm_hpp__
