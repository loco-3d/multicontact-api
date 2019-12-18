# Copyright (c) 2019, CNRS
# Authors: Pierre Fernbach <pfernbac@laas.fr>
import unittest

import numpy as np
from numpy import array,array_equal
from random import uniform
from math import sqrt,sin,cos
import multicontact_api
multicontact_api.switchToNumpyArray()

from multicontact_api import ContactModelPlanar,ContactPatch,ContactPhase
from pinocchio import SE3,Quaternion
import curves
from curves import SE3Curve,polynomial,bezier,piecewise,piecewise_SE3

def randomQuaternion():
  u1 = uniform(0.,1.)
  u2 = uniform(0.,2.*np.pi)
  u3 = uniform(0.,2.*np.pi)
  a = sqrt(1-u1)
  b = sqrt(u1)
  q = Quaternion(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3))
  q.normalize()
  return q



# build random piecewise polynomial with 2 polynomial of degree 3
# between 0;1 and 1;2
def createRandomPiecewisePolynomial(dim,t_min = 0,t_max = 2):
  t_mid = (t_min+t_max)/2.
  coefs0 = np.random.rand(dim,4) # degree 3
  pol0 = polynomial(coefs0,t_min,t_mid)
  pc = piecewise(pol0)
  coefs1 = np.random.rand(dim,4) # degree 3
  pc.append(polynomial(coefs1,t_mid,t_max))
  return pc

def createRandomSE3Traj(t_min = 0,t_max = 2):
  p0 = SE3()
  p0.setRandom()
  p1 = SE3()
  p1.setRandom()
  curve = SE3Curve(p0,p1,t_min,t_max)
  return curve

class ContactModelTest(unittest.TestCase):

  def test_contact_model_planar(self):
    mu = 0.3
    zmp_radius = 0.01
    # constructor with both values
    mp1 = ContactModelPlanar(mu,zmp_radius)
    # test getter bindings
    self.assertEqual(mp1.mu,mu)
    self.assertEqual(mp1.ZMP_radius,zmp_radius)

    # copy constructor :
    mp2 = ContactModelPlanar(mp1)
    self.assertEqual(mp2.mu,mu)
    self.assertEqual(mp2.ZMP_radius,zmp_radius)

    # test operator ==
    self.assertTrue(mp1 == mp2)
    mp1.mu = 0.5
    self.assertTrue(mp1 != mp2)


class ContactPatchTest(unittest.TestCase):

  def test_default_constructor(self):
    cp = ContactPatch()
    self.assertEqual(cp.friction,-1.0)
    self.assertTrue(cp.placement == SE3.Identity())

  def test_setter_getter(self):
    cp = ContactPatch()
    p = SE3()
    p.setRandom()
    cp.placement = p
    self.assertTrue(cp.placement == p)
    cp.friction = 0.7
    self.assertTrue(cp.friction == 0.7)
    self.assertTrue(cp.placement == p)


  def test_constructor_with_arguments(self):
    p = SE3()
    p.setRandom()
    cp = ContactPatch(p)
    self.assertTrue(cp.friction == -1.0)
    self.assertTrue(cp.placement == p)
    # check that the value have been copied and it's not the same pointer anymore :
    p.setRandom()
    self.assertTrue(cp.placement != p)

    p = SE3()
    p.setRandom()
    cp = ContactPatch(p,0.9)
    self.assertTrue(cp.friction == 0.9)
    self.assertTrue(cp.placement == p)


  def test_operator_equal(self):
    cp1 = ContactPatch()
    cp2 = ContactPatch()
    self.assertTrue(cp1 == cp2)
    self.assertFalse(cp1 != cp2)
    cp1.friction = 0.5
    self.assertTrue(cp1 != cp2)
    self.assertFalse(cp1 == cp2)
    cp2.friction = 0.5
    self.assertTrue(cp1 == cp2)

    p = SE3()
    p.setRandom()
    cp1 = ContactPatch(p,0.9)
    cp2 = ContactPatch(p,0.9)
    self.assertTrue(cp1 == cp2)
    cp1.placement.setRandom()
    self.assertTrue(cp1 != cp2)


  def test_copy_constructor(self):
    p = SE3()
    p.setRandom()
    cp1 = ContactPatch(p,0.9)
    cp2 = ContactPatch(cp1)
    self.assertTrue(cp1 == cp2)
    cp1.placement.setRandom()
    self.assertTrue(cp1 != cp2)

  def test_serialization(self):
    #TODO
    pass

class ContactPhaseTest(unittest.TestCase):

  def test_default_constructor(self):
    cp = ContactPhase()
    self.assertEqual(cp.timeInitial,-1)
    self.assertEqual(cp.timeFinal,-1)
    self.assertEqual(cp.duration,0)
    self.assertEqual(cp.numContacts(),0)
    self.assertEqual(len(cp.effectorsInContact()),0)

  def test_constructor_with_arguments(self):
    cp = ContactPhase(1,5)
    self.assertEqual(cp.timeInitial,1)
    self.assertEqual(cp.timeFinal,5)
    self.assertEqual(cp.duration,4)
    self.assertEqual(cp.numContacts(),0)
    self.assertEqual(len(cp.effectorsInContact()),0)
    with self.assertRaises(ValueError):
      cp = ContactPhase(1,0.5)


  def test_timings_setter(self):
    cp = ContactPhase()
    cp.timeInitial = 1.5
    cp.timeFinal = 3.
    self.assertEqual(cp.timeInitial,1.5)
    self.assertEqual(cp.timeFinal,3.)
    self.assertEqual(cp.duration,1.5)
    cp.duration = 2.
    self.assertEqual(cp.timeInitial,1.5)
    self.assertEqual(cp.timeFinal,3.5)
    self.assertEqual(cp.duration,2.)
    with self.assertRaises(ValueError):
      cp.timeFinal = 1.
    with self.assertRaises(ValueError):
      cp.duration = -0.5

  def test_contact_methods(self):
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    new = cp.addContact("right-leg",patchRF)
    self.assertTrue(new)
    self.assertTrue(cp.isEffectorInContact("right-leg"))
    self.assertTrue("right-leg" in cp.effectorsInContact())
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))
    self.assertEqual(cp.numContacts(),1)

    # add another contact :
    p = SE3()
    p.setRandom()
    patchLF = ContactPatch(p,0.5)
    new = cp.addContact("left-leg",patchLF)
    self.assertTrue(new)
    self.assertTrue(cp.isEffectorInContact("right-leg"))
    self.assertTrue("right-leg" in cp.effectorsInContact())
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))
    self.assertTrue(cp.isEffectorInContact("left-leg"))
    self.assertTrue("left-leg" in cp.effectorsInContact())
    self.assertEqual(patchLF,cp.contactPatch("left-leg"))
    self.assertEqual(cp.numContacts(),2)
    # check that the patch can be overwritten:
    p = SE3()
    p.setRandom()
    patchRF2 = ContactPatch(p,0.5)
    new = cp.addContact("right-leg",patchRF2)
    self.assertFalse(new)

    # check deletion of contacts :
    exist = cp.removeContact("right-leg")
    self.assertTrue(exist)
    self.assertTrue(cp.isEffectorInContact("left-leg"))
    self.assertTrue("left-leg" in cp.effectorsInContact())
    self.assertEqual(patchLF,cp.contactPatch("left-leg"))
    self.assertEqual(cp.numContacts(),1)
    self.assertFalse(cp.isEffectorInContact("right-leg"))
    self.assertFalse("right-leg" in cp.effectorsInContact())
    exist = cp.removeContact("right-leg")
    self.assertFalse(exist)

    exist = cp.removeContact("left-leg")
    self.assertTrue(exist)
    self.assertFalse(cp.isEffectorInContact("left-leg"))
    self.assertFalse("left-leg" in cp.effectorsInContact())
    self.assertFalse(cp.isEffectorInContact("right-leg"))
    self.assertFalse("right-leg" in cp.effectorsInContact())
    self.assertEqual(cp.numContacts(),0)

  def test_contact_patch_access(self):
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    cp.addContact("right-leg",patchRF)
    # check that the contactPatch have been copied and it's not a pointer :
    patchRF.placement.setRandom()
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF = ContactPatch(cp.contactPatch("right-leg"))
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF.placement.translation += np.array([0,0.1,0])
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF =  ContactPatch(cp.contactPatch("right-leg"))
    # check that the getter of contactPatch is a non const reference:
    cp.contactPatch('right-leg').placement.setRandom()
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF =  ContactPatch(cp.contactPatch("right-leg"))
    cp.contactPatch('right-leg').friction = 0.7
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF =  ContactPatch(cp.contactPatch("right-leg"))
    cp.contactPatch("right-leg").placement.translation += np.array([0,0.1,0])
    self.assertNotEqual(patchRF,cp.contactPatch("right-leg"))

    patchRF = cp.contactPatch("right-leg")
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))
    patchRF.placement.translation += np.array([0,0.1,0])
    self.assertEqual(patchRF,cp.contactPatch("right-leg"))
    # check errors :
    with self.assertRaises(ValueError):
      cp.contactPatch("left-leg")


  def test_contact_patch_dict(self):
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    cp.addContact("right-leg",patchRF)
    dict = cp.contactPatches()
    self.assertTrue("right-leg" in dict.keys())
    self.assertEqual(dict["right-leg"],patchRF)
    self.assertEqual(len(dict.keys()),1)

    # add another contact :
    p = SE3()
    p.setRandom()
    patchLF = ContactPatch(p,0.5)
    cp.addContact("left-leg",patchLF)
    #check that it's not a pointer :
    self.assertEqual(len(dict.keys()),1)
    self.assertFalse("left-leg" in dict.keys())
    #check that the contact have been added
    dict = cp.contactPatches()
    self.assertTrue("right-leg" in dict.keys())
    self.assertTrue("left-leg" in dict.keys())
    self.assertEqual(dict["right-leg"],patchRF)
    self.assertEqual(dict["left-leg"],patchLF)
    self.assertEqual(len(dict.keys()),2)

    # check that changing the dict doesn't change the contact phase:
    p = SE3()
    p.setRandom()
    patch2 = ContactPatch(p,0.5)
    dict.update({"test":patch2})
    self.assertFalse("test" in cp.contactPatches().keys())
    # check that the map is const
    cp.contactPatches().update({"test":patch2}) # should not have any effect
    self.assertFalse("test" in cp.contactPatches().keys())

    # check deletion :
    cp.removeContact("right-leg")
    dict = cp.contactPatches()
    self.assertFalse("right-leg" in dict.keys())
    self.assertTrue("left-leg" in dict.keys())
    self.assertEqual(dict["left-leg"],patchLF)
    self.assertEqual(len(dict.keys()),1)

  def test_effector_trajectory(self):
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    cp.addContact("right-leg",patchRF)
    # create a SE3 trajectory :
    init_pose = SE3.Identity()
    end_pose = SE3.Identity()
    init_pose.translation = array([0.2, -0.7, 0.6])
    end_pose.translation = array([3.6, -2.2, -0.9])
    init_pose.rotation = Quaternion.Identity().normalized().matrix()
    end_pose.rotation = Quaternion(sqrt(2.) / 2., sqrt(2.) / 2., 0, 0).normalized().matrix()
    effL = SE3Curve(init_pose, end_pose, 0.5, 2.5)
    # add the trajectory to the contact phase :
    new = cp.addEffectorTrajectory("left-leg",effL)
    self.assertTrue(new)
    self.assertTrue(cp.effectorHaveAtrajectory("left-leg"))
    self.assertTrue("left-leg" in cp.effectorsWithTrajectory())
    self.assertEqual(cp.effectorTrajectory("left-leg"),effL)
    self.assertEqual(cp.effectorTrajectory("left-leg").min(),0.5)
    self.assertEqual(cp.effectorTrajectory("left-leg").max(),2.5)
    self.assertTrue(cp.effectorTrajectory("left-leg").evaluateAsSE3(0.5).isApprox(init_pose))
    self.assertTrue(cp.effectorTrajectory("left-leg").evaluateAsSE3(2.5).isApprox(end_pose))

    # check with piecewise SE3
    effH = piecewise_SE3(effL)
    end_pose2 = SE3.Identity()
    end_pose2.translation = array([-4.9, 0.8, 0.9])
    end_pose2.rotation = Quaternion(sqrt(2.) / 2., 0., sqrt(2.) / 2., 0).normalized().matrix()
    effH.append(end_pose2,4.)
    new = cp.addEffectorTrajectory("hand",effH)
    self.assertTrue(new)
    self.assertTrue(cp.effectorHaveAtrajectory("left-leg"))
    self.assertTrue("left-leg" in cp.effectorsWithTrajectory())
    self.assertTrue(cp.effectorHaveAtrajectory("hand"))
    self.assertTrue("hand" in cp.effectorsWithTrajectory())
    self.assertEqual(cp.effectorTrajectory("left-leg"),effL)
    self.assertEqual(cp.effectorTrajectory("hand"),effH)
    self.assertEqual(cp.effectorTrajectory("hand").min(),0.5)
    self.assertEqual(cp.effectorTrajectory("hand").max(),4.)
    self.assertTrue(cp.effectorTrajectory("hand").evaluateAsSE3(0.5).isApprox(init_pose))
    self.assertTrue(cp.effectorTrajectory("hand").evaluateAsSE3(4).isApprox(end_pose2))

    # check that the getter return a pointer to a non const object :
    end_pose3 = SE3.Identity()
    end_pose3.setRandom()
    cp.effectorTrajectory("hand").append(end_pose3,6.5)
    self.assertEqual(cp.effectorTrajectory("hand").max(),6.5)
    self.assertTrue(cp.effectorTrajectory("hand").evaluateAsSE3(6.5).isApprox(end_pose3))

    effH = cp.effectorTrajectory("hand")
    end_pose4 = SE3.Identity()
    end_pose4.setRandom()
    effH.append(end_pose4,10.)
    self.assertEqual(cp.effectorTrajectory("hand").max(),10.)
    self.assertTrue(cp.effectorTrajectory("hand").evaluateAsSE3(10.).isApprox(end_pose4))

    # check errors :
    with self.assertRaises(ValueError):
      cp.addEffectorTrajectory("right-leg",effL)

    # check that we cannot add other kind of trajectories than SE3 :
    waypoints = array([[1., 2., 3.], [4., 5., 6.]]).transpose()
    a = bezier(waypoints, 0., 1.)
    with self.assertRaises(BaseException):
      cp.addEffectorTrajectory("other-leg",a)

  def test_effector_trajectory_dict(self):
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    cp.addContact("right-leg",patchRF)
    # create a SE3 trajectory :
    init_pose = SE3.Identity()
    end_pose = SE3.Identity()
    init_pose.translation = array([0.2, -0.7, 0.6])
    end_pose.translation = array([3.6, -2.2, -0.9])
    init_pose.rotation = Quaternion.Identity().normalized().matrix()
    end_pose.rotation = Quaternion(sqrt(2.) / 2., sqrt(2.) / 2., 0, 0).normalized().matrix()
    effL = SE3Curve(init_pose, end_pose, 0.5, 2.5)
    # add the trajectory to the contact phase :
    cp.addEffectorTrajectory("left-leg",effL)
    dict = cp.effectorTrajectories()
    self.assertEqual(len(dict.keys()),1)
    self.assertTrue("left-leg" in dict.keys())
    self.assertEqual(dict["left-leg"],effL)
    self.assertEqual(dict["left-leg"].min(),0.5)
    self.assertEqual(dict["left-leg"].max(),2.5)
    self.assertTrue(dict["left-leg"].evaluateAsSE3(0.5).isApprox(init_pose))
    self.assertTrue(dict["left-leg"].evaluateAsSE3(2.5).isApprox(end_pose))

    # check that changing the dict doesn't change the contact phase:
    effH = piecewise_SE3(effL)
    end_pose2 = SE3.Identity()
    end_pose2.translation = array([-4.9, 0.8, 0.9])
    end_pose2.rotation = Quaternion(sqrt(2.) / 2., 0., sqrt(2.) / 2., 0).normalized().matrix()
    effH.append(end_pose2,4.)
    dict.update({"hand":effH})
    self.assertFalse("hand" in cp.effectorTrajectories().keys())
    # check that the map is const
    cp.effectorTrajectories().update({"hand":effH}) # should not have any effect
    self.assertFalse("hand" in cp.effectorTrajectories().keys())




  def test_contact_force_trajectory(self):
    # create phase and add two contacts
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    cp.addContact("right-leg",ContactPatch(p,0.5))
    p = SE3()
    p.setRandom()
    cp.addContact("left-leg",ContactPatch(p,0.5))
    # create a polynomial 12D trajectory
    fR = createRandomPiecewisePolynomial(12)
    fL = createRandomPiecewisePolynomial(12)
    new = cp.addContactForceTrajectory("right-leg",fR)
    self.assertTrue(new)
    self.assertEqual(cp.contactForce("right-leg"),fR)
    self.assertEqual(cp.contactForce("right-leg").min(),0)
    self.assertEqual(cp.contactForce("right-leg").max(),2.)
    self.assertTrue(array_equal(cp.contactForce("right-leg")(0.5),fR(0.5)))
    self.assertTrue(array_equal(cp.contactForce("right-leg")(1.5),fR(1.5)))

    new = cp.addContactForceTrajectory("left-leg",fL)
    self.assertTrue(new)
    self.assertEqual(cp.contactForce("left-leg"),fL)
    self.assertEqual(cp.contactForce("left-leg").min(),0)
    self.assertEqual(cp.contactForce("left-leg").max(),2.)
    self.assertTrue(array_equal(cp.contactForce("left-leg")(0.5),fL(0.5)))
    self.assertTrue(array_equal(cp.contactForce("left-leg")(1.5),fL(1.5)))

    new = cp.addContactForceTrajectory("left-leg",fL)
    self.assertFalse(new)

    # check that the getter return a pointer to a non const object :
    cp.contactForce("left-leg").append(np.random.rand(12,1),3.5)
    self.assertEqual(cp.contactForce("left-leg").max(),3.5)

    pc = cp.contactForce("left-leg")
    pc.append(np.random.rand(12,1),6.)
    self.assertEqual(cp.contactForce("left-leg").max(),6.)
    self.assertTrue(array_equal(cp.contactForce("left-leg")(6.),pc(6.)))

    # check errors :
    with self.assertRaises(ValueError):
      cp.addContactForceTrajectory("hand",fL)



  def test_contact_force_trajectory_dict(self):
    # create phase and add two contacts
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    cp.addContact("right-leg",ContactPatch(p,0.5))
    p = SE3()
    p.setRandom()
    cp.addContact("left-leg",ContactPatch(p,0.5))
    # create a polynomial 12D trajectory
    fR = createRandomPiecewisePolynomial(12)
    fL = createRandomPiecewisePolynomial(12)
    cp.addContactForceTrajectory("right-leg",fR)
    dict = cp.contactForces()
    self.assertEqual(len(dict.keys()),1)
    self.assertTrue("right-leg" in dict.keys())
    self.assertEqual(dict["right-leg"],fR)
    self.assertEqual(dict["right-leg"].min(),0)
    self.assertEqual(dict["right-leg"].max(),2.)
    self.assertTrue(array_equal(dict["right-leg"](0.5),fR(0.5)))
    self.assertTrue(array_equal(dict["right-leg"](1.5),fR(1.5)))

    cp.addContactForceTrajectory("left-leg",fL)
    self.assertEqual(len(dict.keys()),1)
    self.assertTrue("right-leg" in dict.keys())
    self.assertFalse("left-leg" in dict.keys())
    dict = cp.contactForces()
    self.assertEqual(len(dict.keys()),2)
    self.assertTrue("right-leg" in dict.keys())
    self.assertTrue("left-leg" in dict.keys())

    #check that changing the dict doesn"t change the contact phase
    f2 = createRandomPiecewisePolynomial(12)
    dict.update({"hand":f2})
    self.assertFalse("hand" in cp.contactForces().keys())
    # check that the map is const
    cp.contactForces().update({"hand":f2}) # should not have any effect
    self.assertFalse("hand" in cp.contactForces().keys())



  def test_contact_normal_force_trajectory(self):
    # create phase and add two contacts
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    cp.addContact("right-leg",ContactPatch(p,0.5))
    p = SE3()
    p.setRandom()
    cp.addContact("left-leg",ContactPatch(p,0.5))
    # create a polynomial 12D trajectory
    fR = createRandomPiecewisePolynomial(1)
    fL = createRandomPiecewisePolynomial(1)
    new = cp.addContactNormalForceTrajectory("right-leg",fR)
    self.assertTrue(new)
    self.assertEqual(cp.contactNormalForce("right-leg"),fR)
    self.assertEqual(cp.contactNormalForce("right-leg").min(),0)
    self.assertEqual(cp.contactNormalForce("right-leg").max(),2.)
    self.assertTrue(array_equal(cp.contactNormalForce("right-leg")(0.5),fR(0.5)))
    self.assertTrue(array_equal(cp.contactNormalForce("right-leg")(1.5),fR(1.5)))

    new = cp.addContactNormalForceTrajectory("left-leg",fL)
    self.assertTrue(new)
    self.assertEqual(cp.contactNormalForce("left-leg"),fL)
    self.assertEqual(cp.contactNormalForce("left-leg").min(),0)
    self.assertEqual(cp.contactNormalForce("left-leg").max(),2.)
    self.assertTrue(array_equal(cp.contactNormalForce("left-leg")(0.5),fL(0.5)))
    self.assertTrue(array_equal(cp.contactNormalForce("left-leg")(1.5),fL(1.5)))

    new = cp.addContactNormalForceTrajectory("left-leg",fL)
    self.assertFalse(new)

    # check that the getter return a pointer to a non const object :
    cp.contactNormalForce("left-leg").append(np.random.rand(1,1),3.5)
    self.assertEqual(cp.contactNormalForce("left-leg").max(),3.5)

    pc = cp.contactNormalForce("left-leg")
    pc.append(np.random.rand(1,1),6.)
    self.assertEqual(cp.contactNormalForce("left-leg").max(),6.)
    self.assertTrue(array_equal(cp.contactNormalForce("left-leg")(6.),pc(6.)))


    # check errors :
    with self.assertRaises(ValueError):
      cp.addContactNormalForceTrajectory("hand",fL)

    fL = createRandomPiecewisePolynomial(3)
    with self.assertRaises(ValueError):
      cp.addContactNormalForceTrajectory("left-leg",fL)

  def test_contact_normal_force_trajectory_dict(self):
    # create phase and add two contacts
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    cp.addContact("right-leg",ContactPatch(p,0.5))
    p = SE3()
    p.setRandom()
    cp.addContact("left-leg",ContactPatch(p,0.5))
    # create a polynomial 12D trajectory
    fR = createRandomPiecewisePolynomial(1)
    fL = createRandomPiecewisePolynomial(1)
    cp.addContactNormalForceTrajectory("right-leg",fR)
    dict = cp.contactNormalForces()
    self.assertEqual(len(dict.keys()),1)
    self.assertTrue("right-leg" in dict.keys())
    self.assertEqual(dict["right-leg"],fR)
    self.assertEqual(dict["right-leg"].min(),0)
    self.assertEqual(dict["right-leg"].max(),2.)
    self.assertTrue(array_equal(dict["right-leg"](0.5),fR(0.5)))
    self.assertTrue(array_equal(dict["right-leg"](1.5),fR(1.5)))

    cp.addContactNormalForceTrajectory("left-leg",fL)
    self.assertEqual(len(dict.keys()),1)
    self.assertTrue("right-leg" in dict.keys())
    self.assertFalse("left-leg" in dict.keys())
    dict = cp.contactNormalForces()
    self.assertEqual(len(dict.keys()),2)
    self.assertTrue("right-leg" in dict.keys())
    self.assertTrue("left-leg" in dict.keys())

    #check that changing the dict doesn"t change the contact phase
    f2 = createRandomPiecewisePolynomial(1)
    dict.update({"hand":f2})
    self.assertFalse("hand" in cp.contactNormalForces().keys())
    # check that the map is const
    cp.contactNormalForces().update({"hand":f2}) # should not have any effect
    self.assertFalse("hand" in cp.contactNormalForces().keys())

  def test_members_points(self):
    cp = ContactPhase()
    # check default values :
    self.assertTrue(array_equal(np.zeros(3),cp.c_init))
    self.assertTrue(array_equal(np.zeros(3),cp.dc_init))
    self.assertTrue(array_equal(np.zeros(3),cp.ddc_init))
    self.assertTrue(array_equal(np.zeros(3),cp.L_init))
    self.assertTrue(array_equal(np.zeros(3),cp.dL_init))
    self.assertTrue(array_equal(np.zeros(3),cp.c_final))
    self.assertTrue(array_equal(np.zeros(3),cp.dc_final))
    self.assertTrue(array_equal(np.zeros(3),cp.ddc_final))
    self.assertTrue(array_equal(np.zeros(3),cp.L_final))
    self.assertTrue(array_equal(np.zeros(3),cp.dL_final))
    # set random values :
    c_init= np.random.rand(3)
    dc_init= np.random.rand(3)
    ddc_init= np.random.rand(3)
    L_init= np.random.rand(3)
    dL_init= np.random.rand(3)
    q_init= np.random.rand(35)
    c_final= np.random.rand(3)
    dc_final= np.random.rand(3)
    ddc_final= np.random.rand(3)
    L_final= np.random.rand(3)
    dL_final= np.random.rand(3)
    q_final= np.random.rand(35)
    cp.c_init = c_init
    cp.dc_init = dc_init
    cp.ddc_init = ddc_init
    cp.L_init = L_init
    cp.dL_init = dL_init
    cp.q_init = q_init
    cp.c_final = c_final
    cp.dc_final = dc_final
    cp.ddc_final = ddc_final
    cp.L_final = L_final
    cp.dL_final = dL_final
    cp.q_final = q_final
    self.assertTrue(array_equal(cp.c_init,c_init))
    self.assertTrue(array_equal(cp.dc_init,dc_init))
    self.assertTrue(array_equal(cp.ddc_init,ddc_init))
    self.assertTrue(array_equal(cp.L_init,L_init))
    self.assertTrue(array_equal(cp.dL_init,dL_init))
    self.assertTrue(array_equal(cp.q_init,q_init))
    self.assertTrue(array_equal(cp.c_final,c_final))
    self.assertTrue(array_equal(cp.dc_final,dc_final))
    self.assertTrue(array_equal(cp.ddc_final,ddc_final))
    self.assertTrue(array_equal(cp.L_final,L_final))
    self.assertTrue(array_equal(cp.dL_final,dL_final))
    self.assertTrue(array_equal(cp.q_final,q_final))
    #check that it's not a pointer :
    ci = cp.c_init
    ci = np.random.rand(3)
    self.assertFalse(array_equal(cp.c_init,ci))
    # it's a copy (limitation from eigenpy ...) :
    dc_init = cp.dc_init.copy()
    cp.dc_init += np.array([0.1,0.,-2.]) # this work as += call the setter
    self.assertFalse(array_equal(cp.dc_init,dc_init))
    dc_init = cp.dc_init.copy()
    cp.dc_init[2] = 0. # this line have no effect
    self.assertTrue(array_equal(cp.dc_init,dc_init))


    # check error due to incorrect dimensions :
    print("# Expected warning messages about dimension / column vector : ")
    with self.assertRaises(BaseException):
      cp.c_init = np.random.rand(4)
    with self.assertRaises(BaseException):
      cp.dc_init = np.random.rand(2)
    with self.assertRaises(BaseException):
      cp.ddc_init = np.random.rand(10)
    with self.assertRaises(BaseException):
      cp.L_init = np.random.rand(1)
    with self.assertRaises(BaseException):
      cp.dL_init = np.random.rand(4)
    with self.assertRaises(BaseException):
      cp.c_final = np.random.rand(5)
    with self.assertRaises(BaseException):
      cp.dc_final = np.random.rand(3,2)
    with self.assertRaises(BaseException):
      cp.ddc_final = np.random.rand(3,3)
    with self.assertRaises(BaseException):
      cp.L_final = np.random.rand(1,2)
    with self.assertRaises(BaseException):
      cp.dL_final = np.random.rand(1,3)
    print("# End of Expected warning messages.")

  def test_member_curves(self):
    cp = ContactPhase()
    # check default values :
    self.assertIsNone(cp.q_t)
    self.assertIsNone(cp.dq_t)
    self.assertIsNone(cp.ddq_t)
    self.assertIsNone(cp.tau_t)
    self.assertIsNone(cp.c_t)
    self.assertIsNone(cp.dc_t)
    self.assertIsNone(cp.ddc_t)
    self.assertIsNone(cp.L_t)
    self.assertIsNone(cp.dL_t)
    self.assertIsNone(cp.wrench_t)
    self.assertIsNone(cp.zmp_t)
    self.assertIsNone(cp.root_t)
    # build random trajectories :
    q = createRandomPiecewisePolynomial(31)
    dq = createRandomPiecewisePolynomial(30)
    ddq = createRandomPiecewisePolynomial(30)
    tau = createRandomPiecewisePolynomial(30)
    c = createRandomPiecewisePolynomial(3)
    dc = createRandomPiecewisePolynomial(3)
    ddc = createRandomPiecewisePolynomial(3)
    L = createRandomPiecewisePolynomial(3)
    dL = createRandomPiecewisePolynomial(3)
    wrench = createRandomPiecewisePolynomial(6)
    zmp = createRandomPiecewisePolynomial(3)
    root = createRandomSE3Traj()
    # assign trajectories :
    cp.q_t = q
    cp.dq_t = dq
    cp.ddq_t = ddq
    cp.tau_t = tau
    cp.c_t = c
    cp.dc_t = dc
    cp.ddc_t = ddc
    cp.L_t = L
    cp.dL_t = dL
    cp.wrench_t = wrench
    cp.zmp_t = zmp
    cp.root_t = root
    # check getter :
    self.assertEqual(cp.q_t , q)
    self.assertEqual(cp.dq_t , dq)
    self.assertEqual(cp.ddq_t , ddq)
    self.assertEqual(cp.tau_t , tau)
    self.assertEqual(cp.c_t , c)
    self.assertEqual(cp.dc_t , dc)
    self.assertEqual(cp.ddc_t , ddc)
    self.assertEqual(cp.L_t , L)
    self.assertEqual(cp.dL_t , dL)
    self.assertEqual(cp.wrench_t , wrench)
    self.assertEqual(cp.zmp_t , zmp)
    self.assertEqual(cp.root_t , root)
    for t in np.linspace(0.,2.,10):
      self.assertTrue(array_equal(cp.q_t(t) , q(t)))
      self.assertTrue(array_equal(cp.dq_t(t) , dq(t)))
      self.assertTrue(array_equal(cp.ddq_t(t) , ddq(t)))
      self.assertTrue(array_equal(cp.tau_t(t) , tau(t)))
      self.assertTrue(array_equal(cp.c_t(t) , c(t)))
      self.assertTrue(array_equal(cp.dc_t(t) , dc(t)))
      self.assertTrue(array_equal(cp.ddc_t(t) , ddc(t)))
      self.assertTrue(array_equal(cp.L_t(t) , L(t)))
      self.assertTrue(array_equal(cp.dL_t(t) , dL(t)))
      self.assertTrue(array_equal(cp.wrench_t(t) , wrench(t)))
      self.assertTrue(array_equal(cp.zmp_t(t) , zmp(t)))
      self.assertTrue(array_equal(cp.root_t(t) , root(t)))
      self.assertEqual(cp.root_t.evaluateAsSE3(t) , root.evaluateAsSE3(t))

    # check that deleting python variables doesn't delete members after assignement:
    del q
    self.assertIsNotNone(cp.q_t)
    self.assertEqual(cp.q_t.min(),0)
    self.assertEqual(cp.q_t.max(),2)
    self.assertIsNotNone(cp.q_t(1.))
    c = None
    self.assertIsNotNone(cp.c_t)
    self.assertEqual(cp.c_t.min(),0)
    self.assertEqual(cp.c_t.max(),2)
    self.assertIsNotNone(cp.c_t(1.))
    # check that curve have not been copied and that it's the same pointer
    dc.append(np.random.rand(3,1),3.5)
    self.assertEqual(cp.dc_t.min(),0)
    self.assertEqual(cp.dc_t.max(),3.5)
    self.assertEqual(cp.dc_t,dc)
    # check that the return of the getter is not const :
    cp.dq_t.append(np.random.rand(30,1),4.)
    self.assertEqual(cp.dq_t.min(),0)
    self.assertEqual(cp.dq_t.max(),4.)
    self.assertEqual(cp.dq_t,dq)



  def test_operator_equal(self):
    cp1 = ContactPhase()
    cp2 = ContactPhase()
    #check timings
    self.assertTrue(cp1 == cp2)
    cp1.timeInitial = 1.
    self.assertTrue(cp1 != cp2)
    cp2.timeInitial = 1.
    self.assertTrue(cp1 == cp2)
    cp1.timeFinal =3.5
    self.assertTrue(cp1 != cp2)
    cp2.duration =2.5
    self.assertTrue(cp1 == cp2)
    #check public members :
    # points :
    c_init= np.random.rand(3)
    dc_init= np.random.rand(3)
    ddc_init= np.random.rand(3)
    L_init= np.random.rand(3)
    dL_init= np.random.rand(3)
    q_init= np.random.rand(35)
    c_final= np.random.rand(3)
    dc_final= np.random.rand(3)
    ddc_final= np.random.rand(3)
    L_final= np.random.rand(3)
    dL_final= np.random.rand(3)
    q_final= np.random.rand(35)
    cp1.c_init = c_init
    self.assertTrue(cp1 != cp2)
    cp2.c_init = c_init
    self.assertTrue(cp1 == cp2)
    cp1.dc_init = dc_init
    self.assertTrue(cp1 != cp2)
    cp2.dc_init = dc_init
    self.assertTrue(cp1 == cp2)
    cp1.ddc_init = ddc_init
    self.assertTrue(cp1 != cp2)
    cp2.ddc_init = ddc_init
    self.assertTrue(cp1 == cp2)
    cp1.L_init = L_init
    self.assertTrue(cp1 != cp2)
    cp2.L_init = L_init
    self.assertTrue(cp1 == cp2)
    cp1.dL_init = dL_init
    self.assertTrue(cp1 != cp2)
    cp2.dL_init = dL_init
    self.assertTrue(cp1 == cp2)
    cp1.q_init = q_init
    self.assertTrue(cp1 != cp2)
    cp2.q_init = q_init
    self.assertTrue(cp1 == cp2)
    cp1.c_final = c_final
    self.assertTrue(cp1 != cp2)
    cp2.c_final = c_final.copy()
    self.assertTrue(cp1 == cp2)
    cp1.dc_final = dc_final
    self.assertTrue(cp1 != cp2)
    cp2.dc_final = dc_final.copy()
    self.assertTrue(cp1 == cp2)
    cp1.ddc_final = ddc_final
    self.assertTrue(cp1 != cp2)
    cp2.ddc_final = ddc_final.copy()
    self.assertTrue(cp1 == cp2)
    cp1.L_final = L_final
    self.assertTrue(cp1 != cp2)
    L_final2 = np.array(L_final)
    cp2.L_final = L_final2
    self.assertTrue(cp1 == cp2)
    cp1.dL_final = dL_final
    self.assertTrue(cp1 != cp2)
    dL_final2 = np.array(dL_final)
    cp2.dL_final = dL_final2
    self.assertTrue(cp1 == cp2)
    cp1.q_final = q_final
    self.assertTrue(cp1 != cp2)
    cp2.q_final = q_final
    self.assertTrue(cp1 == cp2)
    # curves :
    q = createRandomPiecewisePolynomial(31)
    dq = createRandomPiecewisePolynomial(30)
    ddq = createRandomPiecewisePolynomial(30)
    tau = createRandomPiecewisePolynomial(30)
    c = createRandomPiecewisePolynomial(3)
    dc = createRandomPiecewisePolynomial(3)
    ddc = createRandomPiecewisePolynomial(3)
    L = createRandomPiecewisePolynomial(3)
    dL = createRandomPiecewisePolynomial(3)
    wrench = createRandomPiecewisePolynomial(6)
    zmp = createRandomPiecewisePolynomial(3)
    root = createRandomSE3Traj()
    # assign trajectories :
    cp1.q_t = q
    self.assertTrue(cp1 != cp2)
    cp2.q_t = q
    self.assertTrue(cp1 == cp2)
    cp1.dq_t = dq
    self.assertTrue(cp1 != cp2)
    cp2.dq_t = piecewise(dq)
    self.assertTrue(cp1 == cp2)
    cp1.ddq_t = ddq
    self.assertTrue(cp1 != cp2)
    cp2.ddq_t = piecewise(ddq)
    self.assertTrue(cp1 == cp2)
    cp1.tau_t = tau
    self.assertTrue(cp1 != cp2)
    cp2.tau_t = tau
    self.assertTrue(cp1 == cp2)
    cp1.c_t = c
    self.assertTrue(cp1 != cp2)
    cp2.c_t = piecewise(c)
    self.assertTrue(cp1 == cp2)
    cp1.dc_t = dc
    self.assertTrue(cp1 != cp2)
    dc2 = piecewise(dc)
    cp2.dc_t = dc2
    self.assertTrue(cp1 == cp2)
    cp1.ddc_t = ddc
    self.assertTrue(cp1 != cp2)
    cp2.ddc_t = ddc
    self.assertTrue(cp1 == cp2)
    cp1.L_t = L
    self.assertTrue(cp1 != cp2)
    cp2.L_t = L
    self.assertTrue(cp1 == cp2)
    cp1.dL_t = dL
    self.assertTrue(cp1 != cp2)
    cp2.dL_t = dL
    self.assertTrue(cp1 == cp2)
    cp1.wrench_t = wrench
    self.assertTrue(cp1 != cp2)
    cp2.wrench_t = wrench
    self.assertTrue(cp1 == cp2)
    cp1.zmp_t = zmp
    self.assertTrue(cp1 != cp2)
    cp2.zmp_t = zmp
    self.assertTrue(cp1 == cp2)
    cp1.root_t = root
    self.assertTrue(cp1 != cp2)
    cp2.root_t = piecewise_SE3(root)
    self.assertTrue(cp1 == cp2)

    # test contacts
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    cp1.addContact("right-leg",patchRF)
    self.assertTrue(cp1 != cp2)
    cp2.addContact("right-leg2",patchRF)
    self.assertTrue(cp1 != cp2)
    cp2.addContact("right-leg",patchRF)
    self.assertTrue(cp1 != cp2)
    cp2.removeContact("right-leg2")
    self.assertTrue(cp1 == cp2)
    p = SE3()
    p.setRandom()
    patchLF = ContactPatch(p,0.5)
    patchLF2 = ContactPatch(p)
    cp1.addContact("left-leg",patchLF)
    self.assertFalse(cp1 == cp2)
    cp2.addContact("left-leg",patchLF2)
    self.assertFalse(cp1 == cp2)
    cp2.removeContact("left-leg")
    cp2.addContact("left-leg", patchLF.copy())
    self.assertFalse(cp1 != cp2)

    # test force trajectories :
    fR = createRandomPiecewisePolynomial(12)
    fL = createRandomPiecewisePolynomial(12)
    fL2 = createRandomPiecewisePolynomial(12)
    cp1.addContactForceTrajectory("right-leg",fR)
    self.assertTrue(cp1 != cp2)
    cp2.addContactForceTrajectory("right-leg",fR)
    self.assertTrue(cp1 == cp2)
    cp1.addContactForceTrajectory("left-leg",fL)
    self.assertTrue(cp1 != cp2)
    cp2.addContactForceTrajectory("left-leg",fL2)
    self.assertTrue(cp1 != cp2)
    cp2.addContactForceTrajectory("left-leg",fL)
    self.assertTrue(cp1 == cp2)
    fR = createRandomPiecewisePolynomial(1)
    fL = createRandomPiecewisePolynomial(1)
    fL2 = createRandomPiecewisePolynomial(1)
    cp1.addContactNormalForceTrajectory("right-leg",fR)
    self.assertTrue(cp1 != cp2)
    cp2.addContactNormalForceTrajectory("right-leg",fR)
    self.assertTrue(cp1 == cp2)
    cp1.addContactNormalForceTrajectory("left-leg",fL)
    self.assertTrue(cp1 != cp2)
    cp2.addContactNormalForceTrajectory("left-leg",fL2)
    self.assertTrue(cp1 != cp2)
    cp2.addContactNormalForceTrajectory("left-leg",fL)
    self.assertTrue(cp1 == cp2)
    # test effector trajectories :
    fR = createRandomSE3Traj()
    fL = createRandomSE3Traj()
    fL2 = createRandomSE3Traj()
    cp1.addEffectorTrajectory("right-hand",fR)
    self.assertTrue(cp1 != cp2)
    cp2.addEffectorTrajectory("right-hand",fR)
    self.assertTrue(cp1 == cp2)
    cp1.addEffectorTrajectory("left-hand",fL)
    self.assertTrue(cp1 != cp2)
    cp2.addEffectorTrajectory("left-hand",fL2)
    self.assertTrue(cp1 != cp2)
    cp2.addEffectorTrajectory("left-hand",fL)
    self.assertTrue(cp1 == cp2)

  def test_copy_constructor(self):
    pass

  def test_contact_phase_serialization(self):
    pass    #TODO

if __name__ == '__main__':
  unittest.main()

