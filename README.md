# Multicontact API

[![Pipeline status](https://gepgitlab.laas.fr/loco-3d/multicontact-api/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/loco-3d/multicontact-api/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/loco-3d/multicontact-api/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/loco-3d/multicontact-api/master/coverage/)


This package is extracted from an original work of Justin Carpentier (jcarpent@laas.fr),
with the goal to simplify the library, make it more generic and remove old dependencies.

This package provide C++ structures with python bindings used to define and store contact phases and contact sequences.

# Dependencies

* Eigen
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
* [NDCurves](https://github.com/loco-3d/ndcurves)
* [Eigenpy](https://github.com/stack-of-tasks/eigenpy) (Only for python bindings)

# Installation procedure

## From binary
This package is available as binary in [robotpkg/wip](http://robotpkg.openrobots.org/robotpkg-wip.html)

## From sources
Install the required dependencies, eg. (choose for python version):

```
sudo apt-get install robotpkg-py35-pinocchio robotpkg-py3\*-curves
```

Clone the repository and build the package:

```
git clone --recursive git@gepgitlab.laas.fr:loco-3d/multicontact-api.git
cd multicontact-api && mkdir build && cd build
cmake .. && make
make test
```

# Usage

## Main classes

This package define 4 main classes representing physical concepts of legged locomotion. Contact Model, Contact Patch, Contact Phase and Contact Sequence.

A Contact Model define the physical properties of a contact. A Contact Patch define completely a contact between a part of the robot and the environment, it contain a Contact Model. A Contact Phase is defined by a constant set of contacts, it contains one or more Contact Patches. Finally, a Contact Sequence is a sequence of Contact Phases.

### Contact Patch

A contact patch define the placement (in SE(3), in the world frame) of a contact between a part of the robot and the environment. It contains a ContactModel.

### Contact Phase

A contact phase is defined by a constant set of contact points.
In the context of bipedal walking, two examples of contact phases are the single and double support phases.
A contact phase store several data, many of which are optional.

It store the set of active contacts as a map<String, ContactPatch> with the effector name as Keys:

```python
cp = ContactPhase()
p = SE3()
p.setRandom()
patchRF = ContactPatch(p,0.5) # create a new contact patch at the placement p with a friction coefficient of 0.5
cp.addContact("right-feet",patchRF)
# check if an effector is in contact:
cp.isEffectorInContact("right-feet")
# access to the contact patch from the effector name:
cp.contactPatch("right-feet")
```

A contact phase can be defined in a specific time interval:

```python
cp = ContactPhase()
cp.timeInitial = 1.
cp.timeFinal =3.5
```

**Centroidal dynamic data**

Optionnaly, a Contact Phase can store data related to the centroidal dynamic. It store the following initial and final values as public member:

```c
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
```

It also store centroidal trajectories, represented with objects from the [NDCurves](https://github.com/loco-3d/ndcurves) library:


```c
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
  /// \brief SE3 trajectory of the root of the robot
  curve_SE3_ptr m_root;
```

**Wholebody data:**

A Contact Phase can also store data related to the wholebody motion, it store the following initial and final wholebody configuration as public member:

```c
  /// \brief Initial whole body configuration of this phase
  pointX_t m_q_init;
  /// \brief Final whole body configuration of this phase
  pointX_t m_q_final;
```

And the following trajectories:

```c
/// \brief trajectory for the joint positions
  curve_ptr m_q;
  /// \brief trajectory for the joint velocities
  curve_ptr m_dq;
  /// \brief trajectory for the joint accelerations
  curve_ptr m_ddq;
  /// \brief trajectory for the joint torques
  curve_ptr m_tau;
```

It can also store the contact forces and contact normal forces, in a map<String,curve_ptr> with the effector name as Key:


```python
fR = createRandomPiecewisePolynomial(12)
cp.addContactForceTrajectory("right-feet",fR)
# access the trajectory :
cp.contactForce("right-feet")
# contact normal force :
fnR = createRandomPiecewisePolynomial(1)
cp.addContactNormalForceTrajectory("right-feet",fnR)
# access the trajectory :
cp.contactNormalForce("right-feet")
```

And the effector trajectory for the swinging limbs, also in a map<String,curve_ptr> with the effector name as Key:

```python
# create a SE3 trajectory:
init_pose = SE3.Identity()
end_pose = SE3.Identity()
init_pose.translation = array([0.2, -0.7, 0.6])
end_pose.translation = array([3.6, -2.2, -0.9])
init_pose.rotation = Quaternion.Identity().matrix()
end_pose.rotation = Quaternion(sqrt(2.) / 2., sqrt(2.) / 2., 0, 0).normalized().matrix()
effL = SE3Curve(init_pose, end_pose, cp.timeInitial,cp.timeFinal)
# add it to the contact phase:
cp.addEffectorTrajectory("left-feet",effL)
# access the trajectory :
cp.effectorTrajectory("left-feet")
```

### Contact Sequence

As soon as a creation or a rupture of contact point occurs, the contact set is modified, defining a new contact phase.
The concatenation of contact phases describes what we name a contact sequence, inside which all the contact phases have
their own duration.

A contact sequence is basically a Vector of Contact Phase, with several helper method which can be used to ease the creation of a Contact Sequence.

One can either create a Contact sequence with a know number of contact Phase and correctly set the members of the Contact Phases with:

```c
ContactSequence cs = ContactSequence(3);
ContactPhase cp0;
/* set the contact phase members ... */
cs.contactPhase(0) = cp0;
// or:
cs.contactPhase(1).m_c_init = Point3_t(1,0,0.7);
cs.contactPhase(1).timeFinal(3.);
// or :
ContactPhase& cp2 = cs.contactPhase(2);
/* set the contact phase members ... */

```

Or simply append new Contact Phase at the end of the current Contact Sequence;

```c
ContactSequence cs; // create empty contact sequence
ContactPhase cp0;
/* set the contact phase members ... */
cs.append(cp0);
```

**Helper methods to create contact sequence**

Several helper methods have been added to the ContactSequence class to ease the contact creation process:

* `breakContact(eeName, phaseDuration)` Add a new contactPhase at the end of the current ContactSequence, the new ContactPhase have the same ContactPatchs as the last phase of the sequence, with the exeption of the given contact removed.

* `createContact(eeName, contactPatch, phaseDuration)` Add a new contactPhase at the end of the current ContactSequence, the new ContactPhase have the same ContactPatchs as the last phase of the sequence, with the exeption of the given contact added.

* `moveEffectorToPlacement(eeName, placement, durationBreak, durationCreate)`  Add two new phases at the end of the current ContactSequence:
  * it break the contact with eeName
  * it create the contact with eeName at the given placement.


* `moveEffectorOf(eeName, transform, durationBreak, durationCreate)` Similar to moveEffectorToPlacement but use a transform from the previous contact placement instead of a new placement.



**Helper methods to check a contact sequence**

The ContactSequence class contains several methods to check if the sequence contains some of the optional data, and if they are consistents across all the contact phases.
This methods should be used in order to check if a ContactSequence object given as input to your algorithm have been correctly initialized with all the data that you are going to use in your algorithm.
It may also be used to check if your algorithm output consistent data.

Examples of such methods are `haveConsistentContacts` or `haveCentroidalTrajectories` which check that the (c, dc, ddc, L, dL) have been initialized, have the correct duration,
and that each trajectories of one phase correctly end at the same value as it begin in the next phase.


**Helper methods to access Data**

The ContactSequence class also contains methods for easier access to the data contained in the ContactPhase vector. For example, `phaseAtTime` or `phaseIdAtTime` can be used to access a specific ContactPhase at a given time.
`getAllEffectorsInContact` output all the effector used to create contact during the sequence.

Finally, methods exists to return the complete trajectory along the contact sequence, concatenating the trajectories of each phases (eg. `concatenateCtrajectories` return the complete c(t) trajectory for all the contact sequence).

## Serialization

All classes have Boost Serialization features. This is intended for data transfert between processes, and not long-term storage.
