#!/usr/bin/env python

import rospy
from AR_week4_test.msg import cubic_traj_params, cubic_traj_coeffs
from AR_week4_test.srv import compute_cubic_traj, compute_cubic_trajResponse

def callback(data):
    res = compute_traj_coeffs_client(data)
    # declare the node is publishing 'trajectory_coeffs'
    pub = rospy.Publisher('trajectory_coeffs', cubic_traj_coeffs, queue_size=10)
    msg = cubic_traj_coeffs()
    msg.a0 = res.a0
    msg.a1 = res.a1
    msg.a2 = res.a2
    msg.a3 = res.a3
    msg.t0 = data.t0
    msg.tf = data.tf
    print "Computed Coeffs: "
    rospy.loginfo(msg)
    pub.publish(msg)

# client for calling the service
def compute_traj_coeffs_client(cubic_traj_params):
    # wait for the service
    rospy.wait_for_service('compute_cubic_coeffs')
    try:
        # created handle for service
        compute_cubic_coeffs = rospy.ServiceProxy('compute_cubic_coeffs', compute_cubic_traj)
        # call the service
        response = compute_cubic_coeffs(cubic_traj_params.p0, cubic_traj_params.pf, cubic_traj_params.v0, cubic_traj_params.vf, cubic_traj_params.t0, cubic_traj_params.tf)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def listener():
    # initialise node
    rospy.init_node('cubic_traj_planner', anonymous=True)
    # declare that the node is subscribing to 'generated_points' topic
    rospy.Subscriber('generated_points', cubic_traj_params, callback)
    # keep python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
