// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_scenario_constraint_soc_hpp__
#define __multicontact_api_scenario_constraint_soc_hpp__

#include "multicontact-api/scenario/constraint.hpp"
#include "multicontact-api/serialization/archive.hpp"

namespace multicontact_api
{
  namespace scenario
  {
    template<class SOC>
    struct traits< ContactConstraintSOC<SOC> >
    {
      typedef typename SOC::Scalar Scalar;
      enum
      {
        dim_in = SOC::dim,
        dim_out = 1
      };
      typedef typename SOC::VectorD ArgumentType;
      typedef Scalar ReturnType;
    };

    template<class _SOC>
    struct ContactConstraintSOC
    : public ConstraintBase< ContactConstraintSOC<_SOC> >
    , public serialization::Serializable< ContactConstraintSOC<_SOC> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef _SOC SOC;
      typedef typename SOC::Scalar Scalar;
      typedef Eigen::DenseIndex Index;
      typedef typename traits<ContactConstraintSOC>::ArgumentType ArgumentType;
      typedef typename traits<ContactConstraintSOC>::ReturnType ReturnType;

      /// \brief Default constructor
      ContactConstraintSOC()
      : m_socw()
      , m_factor(1.)
      , m_threshold(0.)
      {}

      ContactConstraintSOC(const SOC & socw,
                           const Scalar factor = 1.,
                           const Scalar threshold = 0.)
      : m_socw(socw)
      , m_factor(factor)
      , m_threshold(threshold)
      {}

      static Index inputSize() { return traits< ContactConstraintSOC<SOC> >::dim_in; }
      static Index ouputSize() { return traits< ContactConstraintSOC<SOC> >::dim_out; }
      static Index neq() { return 0; }

      ReturnType value(const ArgumentType & point) const
      { return m_factor * m_socw.rhsValue(point) - m_socw.lhsValue(point); }

      ReturnType residu(const ArgumentType & point) const
      { return m_factor * m_socw.rhsValue(point) - m_socw.lhsValue(point) - m_threshold; }


      SOC m_socw;
      Scalar m_factor;
      Scalar m_threshold;

    private:

      // Serialization of the class
      friend class boost::serialization::access;

      template<class Archive>
      void save(Archive & ar, const unsigned int /*version*/) const
      {
        ar & boost::serialization::make_nvp("socw",m_socw);
        ar & boost::serialization::make_nvp("factor",m_factor);
        ar & boost::serialization::make_nvp("threshold",m_threshold);
      }

      template<class Archive>
      void load(Archive & ar, const unsigned int /*version*/)
      {
        ar >> boost::serialization::make_nvp("socw",m_socw);
        ar >> boost::serialization::make_nvp("factor",m_factor);
        ar >> boost::serialization::make_nvp("threshold",m_threshold);
      }

      BOOST_SERIALIZATION_SPLIT_MEMBER()
    };
  }
}

#endif // ifndef __multicontact_api_scenario_constraint_soc_hpp__
