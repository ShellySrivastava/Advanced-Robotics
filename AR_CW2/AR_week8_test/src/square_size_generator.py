#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float64

def talker():
    # node is publishing the length of the side of the square to the topic called '/square_side'
    pub = rospy.Publisher('/square_side', Float64, queue_size=10)
    # initialise the node
    rospy.init_node('square_size_generator', anonymous=True)
    # publish random points at every 20s
    rate = rospy.Rate(0.05)
    # set the range for the points
    A_MIN = 0.05
    A_MAX = 0.20
    while not rospy.is_shutdown():
    	side = random.uniform(A_MIN, A_MAX)
    	print "Generated length of the square: {}".format(side)
        # publish the message on the topic
        pub.publish(side)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass2