#!/usr/bin/env python

from AR_week4_test.srv import compute_cubic_traj, compute_cubic_trajResponse
import numpy as np
from numpy.linalg import inv
import rospy

def handle_compute_coeffs(req):
    # compute the coeffs vector (A) such that A = inverse(M) * c where M*A = c makes a system of linear equations for p0,pf, v0 and vf
    M = np.array([[1, req.t0, req.t0**2, req.t0**3], [0, 1, 2*req.t0, 3*(req.t0**2)], [1, req.tf, req.tf**2, req.tf**3], [0, 1, 2*req.tf, 3*(req.tf**2)]])
    c = np.array([req.p0, req.v0, req.pf, req.vf])
    A = np.matmul(inv(M), c)
    #return the coeffs a0, a1, a2 and a3
    return compute_cubic_trajResponse(A[0],A[1],A[2],A[3])

def compute_cubic_coeffs_server():
    rospy.init_node('compute_cubic_coeffs_server')
    s = rospy.Service('compute_cubic_coeffs', compute_cubic_traj, handle_compute_coeffs)
    print "compute_cubic_coeff server is ready"
    rospy.spin()

if __name__ == "__main__":
    compute_cubic_coeffs_server()
