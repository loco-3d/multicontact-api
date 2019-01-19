# Copyright (c) 2015-2018, CNRS
# Authors: Justin Carpentier <jcarpent@laas.fr>
# Simplified BSD license :
#Redistribution and use in source and binary forms, with or without modification,
#are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice,
#this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice,
#this list of conditions and the following disclaimer in the documentation
#and/or other materials provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
#OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import locomote
import numpy as np

size = 6
dim = 3
gmm = locomote.GMMDiag3(size)
covs = np.matrix(np.ones((3,size)))
means = np.matrix(np.ones((3,size)))
mixt_coeffs = np.matrix(np.ones(size))

gmm.setDistributionParameters(covs,means,mixt_coeffs)

point = np.matrix(np.zeros(3)).T

pdf_value = gmm.pdf(point)


import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from sklearn import mixture

n_samples = 300

# generate random sample, two components
np.random.seed(0)

# generate spherical data centered on (20, 20)
shifted_gaussian = np.random.randn(n_samples, dim) + np.array([20, 20, 20])

# generate zero centered stretched Gaussian data
C = np.array([[0., -0.7, 0.], [3.5, .7, 0], [0., 0., 1.]])
stretched_gaussian = np.dot(np.random.randn(n_samples, dim), C)

# concatenate the two datasets into the final training set
X_train = np.vstack([shifted_gaussian, stretched_gaussian])

# fit a Gaussian Mixture Model with two components
clf = mixture.GMM(n_components=size, covariance_type='diag')
clf.fit(X_train)


covs2 = np.matrix(clf.covars_).T
means2 = np.matrix(clf.means_).T
mixt_coeffs2 = np.matrix(clf.weights_).T

gmm2 = locomote.GMMDiag3(size)
gmm2.setDistributionParameters(covs2,means2,mixt_coeffs2)

logpdf = gmm2.logpdf(point)
logpdf_ref = clf.score_samples(point.T)

#filename = "gmm2.txt"
#gmm2.saveAsText(filename)

#gmm3 = locomote.GMMDiag3d(1)
#gmm3.loadFromText(filename)
#gmm3.isApprox(gmm2)

# fit a Gaussian Mixture Model with two components
clf2 = mixture.GMM(n_components=size, covariance_type='full')
clf2.fit(X_train)

covs_full = locomote.StdVec_CovarianceMatrix3()
for k in range(clf2.covars_.shape[0]):
  covs_full.append(np.matrix(clf2.covars_[k]))

means_full = np.matrix(clf2.means_).T
mixt_coeffs_full = np.matrix(clf2.weights_).T

gmm_full = locomote.GMMFull3(size)
gmm_full.setDistributionParameters(covs_full,means_full,mixt_coeffs_full)

logpdf = gmm_full.logpdf(point)
logpdf_ref = clf2.score_samples(point.T)
