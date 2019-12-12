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
def createRandomPiecewisePolynomial(dim):
  coefs0 = np.random.rand(dim,4) # degree 3
  pol0 = polynomial(coefs0)
  pc = piecewise(pol0)
  coefs1 = np.random.rand(dim,4) # degree 3
  pc.append(polynomial(coefs1,1,2))
  return pc

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


  def test_effector_trajectory(self):
    cp = ContactPhase(1.5,3)
    p = SE3()
    p.setRandom()
    patchRF = ContactPatch(p,0.5)
    cp.addContact("right-leg",patchRF)
    # create a SE3 trajectory :
    init_pose = SE3.Identity()
    end_pose = SE3.Identity()
    init_pose.translation = array([0.2, -0.7, 0.6]).reshape(-1,1)
    end_pose.translation = array([3.6, -2.2, -0.9]).reshape(-1,1)
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
    end_pose2.translation = array([-4.9, 0.8, 0.9]).reshape(-1,1)
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





if __name__ == '__main__':
  unittest.main()

