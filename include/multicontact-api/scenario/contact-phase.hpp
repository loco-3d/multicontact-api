#ifndef __multicontact_api_scenario_contact_phase_hpp__
#define __multicontact_api_scenario_contact_phase_hpp__


#include "multicontact-api/scenario/fwd.hpp"
#include "multicontact-api/scenario/contact-patch.hpp"

#include "multicontact-api/serialization/archive.hpp"
#include "multicontact-api/serialization/eigen-matrix.hpp"
#include "multicontact-api/serialization/spatial.hpp"

#include <curves/curve_abc.h>
#include <map>
#include <set>
#include <string>
#include <sstream>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>

namespace multicontact_api{
namespace scenario  {

template<typename _Scalar>
struct ContactPhaseTpl : public serialization::Serializable< ContactPhaseTpl<_Scalar> >
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  // Eigen types
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> pointX_t;
  typedef Eigen::Matrix<Scalar,3, 1> point3_t;
  typedef Eigen::Matrix<Scalar, 6, 1> point6_t;
  typedef Eigen::Transform<Scalar, 3, Eigen::Affine> transform_t;

  // Curves types
  typedef curves::curve_abc<Scalar, Scalar, true, pointX_t> curve_t;
  //typedef curves::curve_abc<Scalar, Scalar, true, point3_t> curve_3_t;
  typedef curves::curve_abc<Scalar, Scalar, true, transform_t,point6_t> curve_SE3_t;
  typedef boost::shared_ptr<curve_t> curve_ptr;
  //typedef boost::shared_ptr<curve_3_t> curve_3_ptr;
  typedef boost::shared_ptr<curve_SE3_t> curve_SE3_ptr;

  typedef ContactPatchTpl<Scalar> ContactPatch;
  typedef typename ContactPatch::SE3 SE3;
  typedef std::map< std::string, ContactPatch > ContactPatchMap;
  typedef std::map< std::string, curve_ptr > CurveMap;
  typedef std::map< std::string, curve_SE3_ptr > CurveSE3Map;


  /// \brief Default constructor
  ContactPhaseTpl()
    : m_c_init(point3_t::Zero()),
      m_dc_init(point3_t::Zero()),
      m_ddc_init(point3_t::Zero()),
      m_L_init(point3_t::Zero()),
      m_dL_init(point3_t::Zero()),
      m_q_init(),
      m_c_final(point3_t::Zero()),
      m_dc_final(point3_t::Zero()),
      m_ddc_final(point3_t::Zero()),
      m_L_final(point3_t::Zero()),
      m_dL_final(point3_t::Zero()),
      m_q_final(),
      m_q(),
      m_dq(),
      m_ddq(),
      m_tau(),
      m_c(),
      m_dc(),
      m_ddc(),
      m_L(),
      m_dL(),
      m_wrench(),
      m_zmp(),
      m_contact_forces(),
      m_contact_normal_force(),
      m_effector_trajectories(),
      m_effector_in_contact(),
      m_contact_patches(),
      m_t_init(-1),
      m_t_final(-1)
  {}


  /**
   * @brief ContactPhaseTpl Constructor with time interval
   * @param t_begin the time at the beginning of this contact phase
   * @param t_final the time at the end of this contact phase
   */
  ContactPhaseTpl(const Scalar t_begin, const Scalar t_final)
    : m_c_init(point3_t::Zero()),
      m_dc_init(point3_t::Zero()),
      m_ddc_init(point3_t::Zero()),
      m_L_init(point3_t::Zero()),
      m_dL_init(point3_t::Zero()),
      m_q_init(),
      m_c_final(point3_t::Zero()),
      m_dc_final(point3_t::Zero()),
      m_ddc_final(point3_t::Zero()),
      m_L_final(point3_t::Zero()),
      m_dL_final(point3_t::Zero()),
      m_q_final(),
      m_q(),
      m_dq(),
      m_ddq(),
      m_tau(),
      m_c(),
      m_dc(),
      m_ddc(),
      m_L(),
      m_dL(),
      m_wrench(),
      m_zmp(),
      m_contact_forces(),
      m_contact_normal_force(),
      m_effector_trajectories(),
      m_effector_in_contact(),
      m_contact_patches(),
      m_t_init(t_begin),
      m_t_final(t_final)
  {}



  /// \brief Copy constructor
  template<typename S2>
  ContactPhaseTpl(const ContactPhaseTpl<S2> & other)
    : m_c_init(other.m_c_init),
      m_dc_init(other.m_dc_init),
      m_ddc_init(other.m_ddc_init),
      m_L_init(other.m_L_init),
      m_dL_init(other.m_dL_init),
      m_q_init(other.m_q_init),
      m_c_final(other.m_c_final),
      m_dc_final(other.m_dc_final),
      m_ddc_final(other.m_ddc_final),
      m_L_final(other.m_L_final),
      m_dL_final(other.m_dL_final),
      m_q_final(other.m_q_final),
      m_q(other.m_q),
      m_dq(other.m_dq),
      m_ddq(other.m_ddq),
      m_tau(other.m_tau),
      m_c(other.m_c),
      m_dc(other.m_dc),
      m_ddc(other.m_ddc),
      m_L(other.m_L),
      m_dL(other.m_dL),
      m_wrench(other.m_wrench),
      m_zmp(other.m_zmp),
      m_contact_forces(other.m_contact_forces),
      m_contact_normal_force(other.m_contact_normal_force),
      m_effector_trajectories(other.m_effector_trajectories),
      m_effector_in_contact(other.m_effector_in_contact),
      m_contact_patches(other.m_contact_patches),
      m_t_init(other.m_t_init),
      m_t_final(other.m_t_final)
  {}


  template <typename S2>
  bool operator==(const ContactPhaseTpl<S2>& other) const {
    return m_c_init == other.m_c_init &&
          m_dc_init == other.m_dc_init &&
          m_ddc_init == other.m_ddc_init &&
          m_L_init == other.m_L_init &&
          m_dL_init == other.m_dL_init &&
          m_q_init == other.m_q_init &&
          m_c_final == other.m_c_final &&
          m_dc_final == other.m_dc_final &&
          m_ddc_final == other.m_ddc_final &&
          m_L_final == other.m_L_final &&
          m_dL_final == other.m_dL_final &&
          m_q_final == other.m_q_final &&
          m_q == other.m_q &&
          m_dq == other.m_dq &&
          m_ddq == other.m_ddq &&
          m_tau == other.m_tau &&
          m_c == other.m_c &&
          m_dc == other.m_dc &&
          m_ddc == other.m_ddc &&
          m_L == other.m_L &&
          m_dL == other.m_dL &&
          m_wrench == other.m_wrench &&
          m_zmp == other.m_zmp &&
          m_contact_forces == other.m_contact_forces &&
          m_contact_normal_force == other.m_contact_normal_force &&
          m_effector_trajectories == other.m_effector_trajectories &&
          m_effector_in_contact == other.m_effector_in_contact &&
          m_contact_patches == other.m_contact_patches &&
          m_t_init == other.m_t_init &&
          m_t_final == other.m_t_final;
  }

  template <typename S2>
  bool operator!=(const ContactPhaseTpl<S2>& other) const {
    return !(*this == other);
  }


  // public members :
  /// \brief Initial position of the center of mass for this contact phase
  point3_t m_c_init;
  /// \brief Initial velocity of the center of mass for this contact phase
  point3_t m_dc_init;
  /// \brief Initial acceleration of the center of mass for this contact phase
  point3_t m_ddc_init;
  /// \brief Initial angular momentum for this contact phase
  point3_t m_L_init;
  /// \brief Initial angular momentum derivative for this contact phase
  point3_t m_dL_init;
  /// \brief Initial whole body configuration of this phase
  pointX_t m_q_init;
  /// \brief Final position of the center of mass for this contact phase
  point3_t m_c_final;
  /// \brief Final velocity of the center of mass for this contact phase
  point3_t m_dc_final;
  /// \brief Final acceleration of the center of mass for this contact phase
  point3_t m_ddc_final;
  /// \brief Final angular momentum for this contact phase
  point3_t m_L_final;
  /// \brief Final angular momentum derivative for this contact phase
  point3_t m_dL_final;
  /// \brief Final whole body configuration of this phase
  pointX_t m_q_final;

  /// \brief trajectory for the joint positions
  curve_ptr m_q;
  /// \brief trajectory for the joint velocities
  curve_ptr m_dq;
  /// \brief trajectory for the joint accelerations
  curve_ptr m_ddq;
  /// \brief trajectory for the joint torques
  curve_ptr m_tau;
  /// \brief trajectory for the center of mass position
  curve_ptr m_c;
  /// \brief trajectory for the center of mass velocity
  curve_ptr m_dc;
  /// \brief trajectory for the center of mass acceleration
  curve_ptr m_ddc;
  /// \brief trajectory for the angular momentum
  curve_ptr m_L;
  /// \brief trajectory for the angular momentum derivative
  curve_ptr m_dL;
  /// \brief trajectory for the centroidal wrench
  curve_ptr m_wrench;
  /// \brief trajectory for the zmp
  curve_ptr m_zmp;

  // getter and setter for the timings
  Scalar timeInitial() const {return m_t_init;}
  void timeInitial(const Scalar t){m_t_init = t;}
  Scalar timeFinal() const {return m_t_final;}
  void timeFinal(const Scalar t){
    if(t<=m_t_init)
      throw std::invalid_argument("t_final cannot be inferior to t_begin");
    m_t_final = t;
  }
  Scalar duration() const {return m_t_final - m_t_init;}
  void duration(const Scalar d){m_t_final = m_t_init + d;}

  // getter for the map trajectories
  CurveMap contactForces() const{return m_contact_forces;}
  CurveMap contactNormalForces() const{return m_contact_normal_force;}
  CurveSE3Map effectorTrajectories() const{return m_effector_trajectories;}
  curve_ptr contactForces(const std::string& eeName) {
    if(m_contact_forces.count(eeName) == 0){
      throw std::invalid_argument("This contact phase do not contain any contact force trajectory for the effector "+eeName);
    }else{
      return m_contact_forces.at(eeName);
    }
  }
  curve_ptr contactNormalForces(const std::string& eeName){
    if(m_contact_normal_force.count(eeName) == 0){
      throw std::invalid_argument("This contact phase do not contain any normal contact force trajectory for the effector "+eeName);
    }else{
      return m_contact_normal_force.at(eeName);
    }
  }
  curve_SE3_ptr effectorTrajectories(const std::string& eeName) {
    if(m_effector_trajectories.count(eeName) == 0){
      throw std::invalid_argument("This contact phase do not contain any effector trajectory for the effector "+eeName);
    }else{
      return m_effector_trajectories.at(eeName);
    }
  }
  /**
   * @brief addContactForceTrajectory add a trajectory to the map of contact forces.
   * If a trajectory already exist for this effector, it is overwritted.
   * @param eeName the name of the effector (key of the map)
   * @param trajectory the trajectory to add
   * @throw invalid_argument if eeName is not defined in contact for this phase
   * @return false if a trajectory already existed (and have been overwrited) true otherwise
   */
  bool addContactForceTrajectory(const std::string& eeName, const curve_ptr trajectory){
    if(m_effector_in_contact.count(eeName) == 0)
      throw std::invalid_argument("Cannot add a contact force trajectory for effector "+eeName+" as it is not in contact for the current phase.");
    bool alreadyExist(m_contact_forces.count(eeName));
    if(m_contact_forces.count(eeName))
      m_contact_forces.erase(eeName);
    m_contact_forces.insert(std::pair<std::string,curve_ptr>(eeName,trajectory));
    return !alreadyExist;
  }
  /**
   * @brief addContactNormalForceTrajectory add a trajectory to the map of contact normal forces.
   * If a trajectory already exist for this effector, it is overwritted.
   * @param eeName the name of the effector (key of the map)
   * @param trajectory the trajectory to add
   * @throw invalid_argument if eeName is not defined in contact for this phase
   * @throw invalid_argument if trajectory is not of dimension 1
   * @return false if a trajectory already existed (and have been overwrited) true otherwise
   */
  bool addContactNormalForceTrajectory(const std::string& eeName, const curve_ptr trajectory){
    if(m_effector_in_contact.count(eeName)==0)
      throw std::invalid_argument("Cannot add a contact normal trajectory for effector "+eeName+" as it is not in contact for the current phase.");
    if(trajectory->dim() != 1)
      throw std::invalid_argument("Contact normal force trajectory must be of dimension 1");
    bool alreadyExist(m_contact_normal_force.count(eeName));
    if(m_contact_normal_force.count(eeName))
      m_contact_normal_force.erase(eeName);
    m_contact_normal_force.insert(std::pair<std::string,curve_ptr>(eeName,trajectory));
    return !alreadyExist;
  }
  /**
   * @brief adEffectorTrajectory add a trajectory to the map of contact forces.
   * If a trajectory already exist for this effector, it is overwritted.
   * @param eeName the name of the effector (key of the map)
   * @param trajectory the trajectory to add
   * @throw invalid_argument if eeName is defined in contact for this phase
   * @return false if a trajectory already existed (and have been overwrited) true otherwise
   */
  bool adEffectorTrajectory(const std::string& eeName, const curve_SE3_ptr trajectory){
    if(m_effector_in_contact.count(eeName) > 0)
      throw std::invalid_argument("Cannot add an effector trajectory for effector "+eeName+" as it is in contact for the current phase.");
    bool alreadyExist(m_effector_trajectories.count(eeName));
    if(m_effector_trajectories.count(eeName))
      m_effector_trajectories.erase(eeName);
    m_effector_trajectories.insert(std::pair<std::string,curve_SE3_ptr>(eeName,trajectory));
    return !alreadyExist;
  }

  ContactPatchMap contactPatches() const{return m_contact_patches;}
  ContactPatch contactPatch(const std::string& eeName) {
    if(m_contact_patches.count(eeName) == 0){
      throw std::invalid_argument("This contact phase do not contain any contact patch for the effector "+eeName);
    }else{
      return m_contact_patches.at(eeName);
    }
  }
  /**
   * @brief addContact add a new contact patch to this contact phase
   * If a contact phase already exist for this effector, it is overwritted.
   * If an end effector trajectory exist for this contact, it is removed.
   * @param eeName the name of the effector in contact
   * @param patch the contact patch
   * @return false if a contact for this effector already existed (and have been overwrited) true otherwise
   */
  bool addContact(const std::string& eeName, const ContactPatch& patch){
    bool alreadyExist(m_contact_patches.count(eeName));
    if(m_contact_patches.count(eeName))
      m_contact_patches.erase(eeName);
    m_contact_patches.insert(std::pair<std::string,ContactPatch>(eeName,patch));
    m_effector_in_contact.insert(eeName);
    m_effector_trajectories.erase(eeName);
    return !alreadyExist;
  }

  /**
   * @brief removeContact remove the given contact
   * This will also remove the contact_patch all the contact_forces and contact_normal_forces related to this contact
   * @param eeName the name of the effector to remove
   * @return true if the effector was in contact, false otherwise
   */
  bool removeContact(const std::string& eeName){
    bool existed(m_effector_in_contact.count(eeName));
    m_effector_in_contact.erase(eeName);
    m_contact_patches.erase(eeName);
    m_contact_forces.erase(eeName);
    m_contact_normal_force.erase(eeName);
    return existed;
  }

  /**
   * @brief isConsistent check if all the members of the phase are consistent together:
   * - There is a contact patch defined for all effector in contact
   * - There is only contact forces for the effector defined in contact
   * - If a trajectory is defined, it is defined between t_begin and t_final
   * - If init/end values are defined and a trajectory for this values is also defined, check that they match
   * - The times are positives and tbegin <= tmax
   * @param throw_if_inconsistent if true, throw an runtime_error if not consistent
   * @return true if consistent, false otherwise
   */
  bool isConsistent(const bool throw_if_inconsistent = false){
    //TODO
    return true;
  }


protected:
  // private members
  /// \brief map with keys : effector name containing the contact forces
  CurveMap m_contact_forces;
  /// \brief map with keys : effector name containing the contact normal force
  CurveMap m_contact_normal_force;
  /// \brief map with keys : effector name containing the end effector trajectory
  CurveSE3Map m_effector_trajectories;
  /// \brief set of the name of all effector in contact for this phase
  std::set<std::string> m_effector_in_contact;
  /// \brief map effector name : contact patches. All the patches are actives
  ContactPatchMap m_contact_patches;
  /// \brief time at the beginning of the contact phase
  Scalar m_t_init;
  /// \brief time at the end of the contact phase
  Scalar m_t_final;



}; //struct contact phase
} //scenario
}// multicontact-api


#endif // CONTACTPHASE_HPP
