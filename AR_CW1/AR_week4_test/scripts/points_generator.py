#!/usr/bin/env python

import rospy
from AR_week4_test.msg import cubic_traj_params
import random

def talker():
    # node is publishing 'generated_points' topic
    pub = rospy.Publisher('generated_points', cubic_traj_params, queue_size=10)
    msg = cubic_traj_params()
    # initialise the node
    rospy.init_node('points_generator', anonymous=True)
    # publish random points at every 20s
    rate = rospy.Rate(0.05)
    # set the range for the points
    P_MAX = 10
    P_MIN = -10
    V_MAX = 10
    V_MIN = -10
    T_MAX = 10
    T_MIN = 5
    while not rospy.is_shutdown():
	msg.p0 = random.uniform(P_MIN, P_MAX)
    	msg.pf = random.uniform(P_MIN, P_MAX)
    	msg.v0 = random.uniform(V_MIN, V_MAX)
    	msg.vf = random.uniform(V_MIN, V_MAX)
    	msg.t0 = 0
    	msg.tf = msg.t0 + random.uniform(T_MIN, T_MAX)
	print "Generated Points: "
        rospy.loginfo(msg)
        # publish the message on the topic
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass2
