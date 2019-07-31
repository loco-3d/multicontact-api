// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_scenario_contact_phase_humanoid_hpp__
#define __multicontact_api_scenario_contact_phase_humanoid_hpp__

#include "multicontact-api/scenario/contact-phase.hpp"
#include "multicontact-api/scenario/contact-sequence.hpp"
#include "multicontact-api/serialization/spatial.hpp"
#include "multicontact-api/serialization/aligned-vector.hpp"

#include <curves/cubic_hermite_spline.h>

#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/spatial/force.hpp>

#include <boost/serialization/vector.hpp>

#include <vector>
#include <Eigen/StdVector>

namespace multicontact_api
{
  namespace scenario
  {

    template<typename _Scalar>
    struct ContactPhaseHumanoidTpl
    : public ContactPhaseTpl<_Scalar,4>
    , public serialization::Serializable< ContactPhaseHumanoidTpl<_Scalar> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef ContactPhaseTpl<_Scalar,4> Base;
      typedef _Scalar Scalar;
      //typedef _Curve_abc curve_abc_t;
      typedef pinocchio::ForceTpl<Scalar> Force;
      enum {
        state_dim = 9,
        control_dim = 6,
        com_position_id = 0,
        com_velocity_id = 3,
        com_acceleration_id = 3,
        am_id = 6,
        am_variations_id = 6,
        linear_control_id = 0,
        angular_control_id = 3
      };

      typedef Eigen::Matrix<Scalar,state_dim,1> StateVector;
      typedef Eigen::Matrix<Scalar,control_dim,1> ControlVector;
      typedef Eigen::Matrix<Scalar,3,1> Vector3;
      typedef Eigen::Matrix<Scalar,24,1> Vector24;
      typedef Eigen::Map<Vector3> MapVector3;

      typedef Eigen::Map<StateVector> MapStateVector;
      typedef Eigen::Map<ControlVector> MapControlVector;

      typedef pinocchio::container::aligned_vector<StateVector> VectorStateVector;

      //      typedef pinocchio::container::aligned_vector<Force> VectorForce;
      typedef std::vector<Force, Eigen::aligned_allocator<Force> > VectorForce;
      typedef pinocchio::container::aligned_vector<ControlVector> VectorControlVector;
      typedef std::vector<Scalar> VectorScalar;
      typedef VectorScalar TimeVector;
      typedef Eigen::Matrix<_Scalar,Eigen::Dynamic,1> ConfigurationVector;
      typedef pinocchio::container::aligned_vector<ConfigurationVector> VectorConfigurationVector;

      typedef boost::array<VectorForce,4> ForceVectorArray;

      typedef curves::cubic_hermite_spline<Scalar,Scalar,3,true> CubicHermiteSpline3;
      typedef curves::cubic_hermite_spline<Scalar,Scalar,24,true> CubicHermiteSpline24;
      typedef std::pair<Vector3, Vector3> pair_point_tangent3_t;
      typedef std::pair<Vector24, Vector24> pair_point_tangent24_t;
      typedef std::vector<pair_point_tangent3_t,Eigen::aligned_allocator<pair_point_tangent3_t> > t_pair_point_tangent3_t;
      typedef std::vector<pair_point_tangent24_t,Eigen::aligned_allocator<pair_point_tangent24_t> > t_pair_point_tangent24_t;
      //typedef trajectories::CubicHermiteSplineTpl<Scalar,3> CubicHermiteSpline24;
      //typedef trajectories::CubicHermiteSplineTpl<Scalar,24> CubicHermiteSpline24;
      
      using Base::dim;
      using Base::operator==;
      using typename Base::ContactPatch;
      using Base::m_contact_patches;

      /// \brief default constructor
      ContactPhaseHumanoidTpl()
      : Base()
      , RF_patch(m_contact_patches[0])
      , LF_patch(m_contact_patches[1])
      , RH_patch(m_contact_patches[2])
      , LH_patch(m_contact_patches[3])
      , m_init_state()
      , m_final_state()
      //, m_state_trajectory(0)
      , m_dot_state_trajectory(0)
      //, m_control_trajectory(0)
      //, m_time_trajectory(0)
      , m_objective_trajectory(0)
      , m_raw_control_trajectory(0)
      , m_reference_configurations(0)
      {
        Vector3 p3_0;
        Vector24 p24_0;
        pair_point_tangent3_t pair3(p3_0,p3_0);
        pair_point_tangent24_t pair24(p24_0,p24_0);
        t_pair_point_tangent3_t cp3;
        cp3.push_back(pair3);
        t_pair_point_tangent24_t cp24;
        cp24.push_back(pair24);
        std::vector< double > time_control_points;
        time_control_points.push_back(0.);
        time_control_points.push_back(1.);
        // Replace cubic hermite spline
        m_angular_momentum_ref = new CubicHermiteSpline3(cp3.begin(), cp3.end(), time_control_points);
        m_com_ref = new CubicHermiteSpline3(cp3.begin(), cp3.end(), time_control_points);
        m_vcom_ref = new CubicHermiteSpline3(cp3.begin(), cp3.end(), time_control_points);
        m_forces_ref = new CubicHermiteSpline24(cp24.begin(), cp24.end(), time_control_points);
      }

      /// \brief copy constructor
      ContactPhaseHumanoidTpl(const ContactPhaseHumanoidTpl & other)
      : Base(other)
      , RF_patch(m_contact_patches[0])
      , LF_patch(m_contact_patches[1])
      , RH_patch(m_contact_patches[2])
      , LH_patch(m_contact_patches[3])
      , m_init_state(other.m_init_state)
      , m_final_state(other.m_final_state)
      //, m_state_trajectory(other.m_state_trajectory)
      , m_dot_state_trajectory(other.m_dot_state_trajectory)
      //, m_control_trajectory(other.m_control_trajectory)
      //, m_time_trajectory(other.m_time_trajectory)
      , m_objective_trajectory(other.m_objective_trajectory)
      , m_contact_forces_trajectories(other.m_contact_forces_trajectories)
      , m_raw_control_trajectory(other.m_raw_control_trajectory)
      , m_reference_configurations(other.m_reference_configurations)
      {
        m_angular_momentum_ref = new CubicHermiteSpline3( other.m_angular_momentum_ref->control_points_.begin(), 
                                                          other.m_angular_momentum_ref->control_points_.end(),
                                                          other.m_angular_momentum_ref->time_control_points_);
        m_com_ref = new CubicHermiteSpline3(  other.m_com_ref->control_points_.begin(), 
                                              other.m_com_ref->control_points_.end(),
                                              other.m_com_ref->time_control_points_);
        m_vcom_ref = new CubicHermiteSpline3( other.m_vcom_ref->control_points_.begin(), 
                                              other.m_vcom_ref->control_points_.end(),
                                              other.m_vcom_ref->time_control_points_);
        m_forces_ref = new CubicHermiteSpline24( other.m_forces_ref->control_points_.begin(), 
                                                other.m_forces_ref->control_points_.end(),
                                                other.m_forces_ref->time_control_points_);
      }

      /// \brief copy operator
//      template<typename S2>
      ContactPhaseHumanoidTpl & operator=(const ContactPhaseHumanoidTpl & other)
      {
        Base::operator=(other);
        m_init_state = other.m_init_state;
        m_final_state = other.m_final_state;
        //m_state_trajectory = other.m_state_trajectory;
        m_dot_state_trajectory = other.m_dot_state_trajectory;
        //m_control_trajectory = other.m_control_trajectory;
        //m_time_trajectory = other.m_time_trajectory;
        m_objective_trajectory = other.m_objective_trajectory;
        m_contact_forces_trajectories = other.m_contact_forces_trajectories;
        m_raw_control_trajectory = other.m_raw_control_trajectory;
        *(m_angular_momentum_ref) = *(other.m_angular_momentum_ref);
        *(m_com_ref) = *(other.m_com_ref);
        *(m_vcom_ref) = *(other.m_vcom_ref);
        *(m_forces_ref) = *(other.m_forces_ref);
        m_reference_configurations = other.m_reference_configurations;
        return *this;
      }

      ContactPatch & RF_patch;
      ContactPatch & LF_patch;
      ContactPatch & RH_patch;
      ContactPatch & LH_patch;

      StateVector m_init_state;
      StateVector m_final_state;

      //VectorStateVector m_state_trajectory;
      VectorStateVector m_dot_state_trajectory;
      //VectorControlVector m_control_trajectory;
      //TimeVector m_time_trajectory;
      VectorScalar m_objective_trajectory;
      VectorConfigurationVector m_reference_configurations;
      ForceVectorArray m_contact_forces_trajectories;
      VectorConfigurationVector m_raw_control_trajectory;
      CubicHermiteSpline3 * m_angular_momentum_ref;
      CubicHermiteSpline3 * m_com_ref;
      CubicHermiteSpline3 * m_vcom_ref;
      CubicHermiteSpline24 * m_forces_ref;

    private:

      // Serialization of the class
      friend class boost::serialization::access;

      template<class Archive>
      void save(Archive & ar, const unsigned int /*version*/) const
      {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
        ar & boost::serialization::make_nvp("init_state",m_init_state);
        ar & boost::serialization::make_nvp("final_state",m_final_state);
        //ar & boost::serialization::make_nvp("state_trajectory",m_state_trajectory);
        ar & boost::serialization::make_nvp("dot_state_trajectory",m_dot_state_trajectory);
        //ar & boost::serialization::make_nvp("control_trajectory",m_control_trajectory);
        //ar & boost::serialization::make_nvp("time_trajectory",m_time_trajectory);
        ar & boost::serialization::make_nvp("objective_trajectory",m_objective_trajectory);

        size_t reference_configurations_size = m_reference_configurations.size();
        ar & boost::serialization::make_nvp("reference_configurations_size",reference_configurations_size);
        for(typename VectorConfigurationVector::const_iterator it = m_reference_configurations.begin();
            it != m_reference_configurations.end(); ++it)
          ar & boost::serialization::make_nvp("reference_configuration",*it);

        for(typename ForceVectorArray::const_iterator it = m_contact_forces_trajectories.begin();
            it != m_contact_forces_trajectories.end(); ++it)
          ar & boost::serialization::make_nvp("contact_force_trajectory",*it);

//        const typename VectorConfigurationVector::vector_base & m_raw_control_trajectory_ =
//        static_cast<const typename VectorConfigurationVector::vector_base&> (m_raw_control_trajectory);
        ar & boost::serialization::make_nvp("raw_control",m_raw_control_trajectory);

        // Serialization to do
        /*
        ar & boost::serialization::make_nvp("angular_momentum_ref",m_angular_momentum_ref);
        ar & boost::serialization::make_nvp("com_ref",m_com_ref);
        ar & boost::serialization::make_nvp("vcom_ref",m_vcom_ref);
        ar & boost::serialization::make_nvp("forces_ref",m_forces_ref);     
        */   
      }

      template<class Archive>
      void load(Archive & ar, const unsigned int /*version*/)
      {
        ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
        ar >> boost::serialization::make_nvp("init_state",m_init_state);
        ar >> boost::serialization::make_nvp("final_state",m_final_state);
        //ar >> boost::serialization::make_nvp("state_trajectory",m_state_trajectory);
        ar >> boost::serialization::make_nvp("dot_state_trajectory",m_dot_state_trajectory);
        //ar >> boost::serialization::make_nvp("control_trajectory",m_control_trajectory);
        //ar >> boost::serialization::make_nvp("time_trajectory",m_time_trajectory);
        ar >> boost::serialization::make_nvp("objective_trajectory",m_objective_trajectory);

        size_t reference_configurations_size = 0;
        ar >> boost::serialization::make_nvp("reference_configurations_size",reference_configurations_size);
        m_reference_configurations.resize(reference_configurations_size);
        for(typename VectorConfigurationVector::iterator it = m_reference_configurations.begin();
            it != m_reference_configurations.end(); ++it)
          ar >> boost::serialization::make_nvp("reference_configuration",*it);

        for(typename ForceVectorArray::iterator it = m_contact_forces_trajectories.begin();
            it != m_contact_forces_trajectories.end(); ++it)
          ar >> boost::serialization::make_nvp("contact_force_trajectory",*it);

//        typename VectorConfigurationVector::vector_base & m_raw_control_trajectory_ =
//        static_cast<typename VectorConfigurationVector::vector_base&> (m_raw_control_trajectory);
        ar >> boost::serialization::make_nvp("raw_control",m_raw_control_trajectory);

        // Serialization to do
        /*
        ar >> boost::serialization::make_nvp("angular_momentum_ref",m_angular_momentum_ref);
        ar >> boost::serialization::make_nvp("com_ref",m_com_ref);
        ar >> boost::serialization::make_nvp("vcom_ref",m_vcom_ref);
        ar >> boost::serialization::make_nvp("forces_ref",m_forces_ref);
        */
      }

      BOOST_SERIALIZATION_SPLIT_MEMBER()

    };
  }

  ContactPatch& RF_patch;
  ContactPatch& LF_patch;
  ContactPatch& RH_patch;
  ContactPatch& LH_patch;

  StateVector m_init_state;
  StateVector m_final_state;

  VectorStateVector m_state_trajectory;
  VectorStateVector m_dot_state_trajectory;
  VectorControlVector m_control_trajectory;
  TimeVector m_time_trajectory;
  VectorScalar m_objective_trajectory;
  VectorConfigurationVector m_reference_configurations;
  ForceVectorArray m_contact_forces_trajectories;
  VectorConfigurationVector m_raw_control_trajectory;
  CubicHermiteSpline3 m_angular_momentum_ref;
  CubicHermiteSpline3 m_com_ref;
  CubicHermiteSpline3 m_vcom_ref;
  CubicHermiteSpline m_forces_ref;

 private:
  // Serialization of the class
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int /*version*/) const {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& boost::serialization::make_nvp("init_state", m_init_state);
    ar& boost::serialization::make_nvp("final_state", m_final_state);
    ar& boost::serialization::make_nvp("state_trajectory", m_state_trajectory);
    ar& boost::serialization::make_nvp("dot_state_trajectory", m_dot_state_trajectory);
    ar& boost::serialization::make_nvp("control_trajectory", m_control_trajectory);
    ar& boost::serialization::make_nvp("time_trajectory", m_time_trajectory);
    ar& boost::serialization::make_nvp("objective_trajectory", m_objective_trajectory);

    size_t reference_configurations_size = m_reference_configurations.size();
    ar& boost::serialization::make_nvp("reference_configurations_size", reference_configurations_size);
    for (typename VectorConfigurationVector::const_iterator it = m_reference_configurations.begin();
         it != m_reference_configurations.end(); ++it)
      ar& boost::serialization::make_nvp("reference_configuration", *it);

    for (typename ForceVectorArray::const_iterator it = m_contact_forces_trajectories.begin();
         it != m_contact_forces_trajectories.end(); ++it)
      ar& boost::serialization::make_nvp("contact_force_trajectory", *it);

    //        const typename VectorConfigurationVector::vector_base & m_raw_control_trajectory_ =
    //        static_cast<const typename VectorConfigurationVector::vector_base&> (m_raw_control_trajectory);
    ar& boost::serialization::make_nvp("raw_control", m_raw_control_trajectory);
    ar& boost::serialization::make_nvp("angular_momentum_ref", m_angular_momentum_ref);
    ar& boost::serialization::make_nvp("com_ref", m_com_ref);
    ar& boost::serialization::make_nvp("vcom_ref", m_vcom_ref);
    ar& boost::serialization::make_nvp("forces_ref", m_forces_ref);
  }

  template <class Archive>
  void load(Archive& ar, const unsigned int /*version*/) {
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar >> boost::serialization::make_nvp("init_state", m_init_state);
    ar >> boost::serialization::make_nvp("final_state", m_final_state);
    ar >> boost::serialization::make_nvp("state_trajectory", m_state_trajectory);
    ar >> boost::serialization::make_nvp("dot_state_trajectory", m_dot_state_trajectory);
    ar >> boost::serialization::make_nvp("control_trajectory", m_control_trajectory);
    ar >> boost::serialization::make_nvp("time_trajectory", m_time_trajectory);
    ar >> boost::serialization::make_nvp("objective_trajectory", m_objective_trajectory);

    size_t reference_configurations_size = 0;
    ar >> boost::serialization::make_nvp("reference_configurations_size", reference_configurations_size);
    m_reference_configurations.resize(reference_configurations_size);
    for (typename VectorConfigurationVector::iterator it = m_reference_configurations.begin();
         it != m_reference_configurations.end(); ++it)
      ar >> boost::serialization::make_nvp("reference_configuration", *it);

    for (typename ForceVectorArray::iterator it = m_contact_forces_trajectories.begin();
         it != m_contact_forces_trajectories.end(); ++it)
      ar >> boost::serialization::make_nvp("contact_force_trajectory", *it);

    //        typename VectorConfigurationVector::vector_base & m_raw_control_trajectory_ =
    //        static_cast<typename VectorConfigurationVector::vector_base&> (m_raw_control_trajectory);
    ar >> boost::serialization::make_nvp("raw_control", m_raw_control_trajectory);
    ar >> boost::serialization::make_nvp("angular_momentum_ref", m_angular_momentum_ref);
    ar >> boost::serialization::make_nvp("com_ref", m_com_ref);
    ar >> boost::serialization::make_nvp("vcom_ref", m_vcom_ref);
    ar >> boost::serialization::make_nvp("forces_ref", m_forces_ref);
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()
};
}  // namespace scenario
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_scenario_contact_phase_humanoid_hpp__
