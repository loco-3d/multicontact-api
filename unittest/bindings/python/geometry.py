# Copyright (c) 2015-2018, CNRS
# Authors: Justin Carpentier <jcarpent@laas.fr>

import multicontact_api
import numpy as np

A = np.matrix(np.identity(3))
c = np.matrix([1.,1.,1.]).T

e = multicontact_api.Ellipsoid3d(A,c)
print e.lhsValue(c)

Q = np.matrix(np.identity(6))
direction = np.matrix(np.ones((6,1)))

C6D = multicontact_api.SOC6d(Q,direction)
