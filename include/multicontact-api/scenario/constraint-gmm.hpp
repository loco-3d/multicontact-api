// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_scenario_constraint_gmm_hpp__
#define __multicontact_api_scenario_constraint_gmm_hpp__

#include "multicontact-api/scenario/constraint.hpp"
#include "multicontact-api/serialization/archive.hpp"

namespace multicontact_api {
namespace scenario {
template <class GMM>
struct traits<ContactConstraintGMM<GMM> > {
  typedef typename GMM::Scalar Scalar;
  enum { dim_in = GMM::dim, dim_out = 1 };
  typedef Eigen::Matrix<Scalar, GMM::dim, 1> ArgumentType;
  typedef Scalar ReturnType;
};

template <class _GMM>
struct ContactConstraintGMM : public ConstraintBase<ContactConstraintGMM<_GMM> >
//    , public serialization::Serializable< ContactConstraintGMM<_GMM> >
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _GMM GMM;
  typedef typename GMM::Scalar Scalar;
  typedef Eigen::DenseIndex Index;
  typedef typename traits<ContactConstraintGMM>::ArgumentType ArgumentType;
  typedef typename traits<ContactConstraintGMM>::ReturnType ReturnType;

  ContactConstraintGMM(const GMM* gmm_ptr = NULL, const ReturnType& offset = 0.)
      : m_gmm_ptr(gmm_ptr), m_offset(offset) {}

  static Index inputSize() { return traits<ContactConstraintGMM<GMM> >::dim_in; }
  static Index ouputSize() { return traits<ContactConstraintGMM<GMM> >::dim_out; }
  static Index neq() { return 0; }

  ReturnType value(const ArgumentType& point) const {
    assert(m_gmm_ptr != NULL);
    return m_gmm_ptr->pdf(point)[0];
  }

  ReturnType residu(const ArgumentType& point) const {
    assert(m_gmm_ptr != NULL);
    return m_gmm_ptr->pdf(point)[0] - m_offset;
  }

  const GMM* m_gmm_ptr;
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
}  // namespace scenario
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_scenario_constraint_gmm_hpp__
