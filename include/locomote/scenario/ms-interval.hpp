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
#ifndef __locomote_scenario_ms_interval_hpp__
#define __locomote_scenario_ms_interval_hpp__

#include "locomote/scenario/fwd.hpp"

#include "locomote/serialization/archive.hpp"
#include "locomote/serialization/eigen-matrix.hpp"

#include <pinocchio/container/aligned-vector.hpp>

namespace locomote
{
  namespace scenario
  {
    
    template<typename _TimeVector, typename _StateVector, typename _ControlVector>
    struct MSIntervalDataTpl : public serialization::Serializable< MSIntervalDataTpl<_TimeVector,_StateVector,_ControlVector> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef _TimeVector TimeVector;
      typedef _StateVector StateVector;
      typedef se3::container::aligned_vector<StateVector> StateVectorTrajectory;
      typedef _ControlVector ControlVector;
      typedef se3::container::aligned_vector<ControlVector> ControlVectorTrajectory;
      
      
      /// \brief Default constructor
      MSIntervalDataTpl() {}
      
      /// \brief Copy constructor
      MSIntervalDataTpl(const MSIntervalDataTpl & other)
      : m_time_trajectory(other.m_time_trajectory)
      , m_state_trajectory(other.m_state_trajectory)
      , m_dot_state_trajectory(other.m_dot_state_trajectory)
      , m_control_trajectory(other.m_control_trajectory)
      {}
      
      /// \Returns a reference of the time trajectory
      const TimeVector & time_trajectory() const { return m_time_trajectory; }
      /// \Returns a reference of the time trajectory
      TimeVector & time_trajectory() { return m_time_trajectory; }
      
      const StateVectorTrajectory & state_trajectory() const { return m_state_trajectory; }
      StateVectorTrajectory & state_trajectory() { return m_state_trajectory; }
      
      const StateVectorTrajectory & dot_state_trajectory() const { return m_dot_state_trajectory; }
      StateVectorTrajectory & dot_state_trajectory() { return m_dot_state_trajectory; }
      
      const ControlVectorTrajectory & control_trajectory() const { return m_control_trajectory; }
      ControlVectorTrajectory & control_trajectory() { return m_control_trajectory; }
      
      bool operator==(const MSIntervalDataTpl & other) const
      {
        return
        m_time_trajectory == other.m_time_trajectory
        && m_state_trajectory == other.m_state_trajectory
        && m_dot_state_trajectory == other.m_dot_state_trajectory
        && m_control_trajectory == other.m_control_trajectory
        ;
      }
      
      bool operator!=(const MSIntervalDataTpl & other) const
      { return !(*this == other); }
      
    protected:
      
      /// \brief Timeline of the shooting interval.
      TimeVector m_time_trajectory;
      
      /// \brief State and derivate of the state trajectories over the shooting intervals.
      StateVectorTrajectory m_state_trajectory, m_dot_state_trajectory;
      
      /// \brief Control trajectory over the shooting interval
      ControlVectorTrajectory m_control_trajectory;
      
    private:
      
      // Serialization of the class
      friend class boost::serialization::access;
      
      template<class Archive>
      void save(Archive & ar, const unsigned int /*version*/) const
      {
        ar & boost::serialization::make_nvp("time_trajectory",m_time_trajectory);
        ar & boost::serialization::make_nvp("state_trajectory",m_state_trajectory);
        ar & boost::serialization::make_nvp("dot_state_trajectory",m_dot_state_trajectory);
        ar & boost::serialization::make_nvp("control_trajectory",m_control_trajectory);
      }
      
      template<class Archive>
      void load(Archive & ar, const unsigned int /*version*/)
      {
        ar >> boost::serialization::make_nvp("time_trajectory",m_time_trajectory);
        ar >> boost::serialization::make_nvp("state_trajectory",m_state_trajectory);
        ar >> boost::serialization::make_nvp("dot_state_trajectory",m_dot_state_trajectory);
        ar >> boost::serialization::make_nvp("control_trajectory",m_control_trajectory);
      }
      
      BOOST_SERIALIZATION_SPLIT_MEMBER()
    };
  }
}

#endif // ifndef __locomote_scenario_ms_interval_hpp__
