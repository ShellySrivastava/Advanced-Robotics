#!/usr/bin/env python
import rospy
from AR_week4_test.msg import cubic_traj_coeffs
from std_msgs.msg import Float64

def callback(data):
    # the node is publishing three topics
    pub_pos = rospy.Publisher('/position_trajectory', Float64, queue_size=10)
    pub_vel = rospy.Publisher('/velocity_trajectory', Float64, queue_size=10)
    pub_acc = rospy.Publisher('/acceleration_trajectory', Float64, queue_size=10)
    # to publish the topics for tf duration:
    # store the time at the beginning 
    begin_time = rospy.Time.now()
    # calculate the time at which the node should stop publishing the topic i.e. begin_time + tf
    end_time = begin_time + rospy.Duration(data.tf)
    now = rospy.Time.now()
    # the loop executes till the certain time elapses
    while (now < end_time):
          t = now.to_sec() - begin_time.to_sec()
	  # plot the trajectory at time t i.e. the time elapsed since the begin_time
          pub_pos.publish(position_trajectory(data, t))
          pub_vel.publish(velocity_trajectory(data, t))
          pub_acc.publish(acceleration_trajectory(data, t))
          now = rospy.Time.now()

# method to calculate position trajectory                      
def position_trajectory(coeffs, t):
    return coeffs.a0 + coeffs.a1*t + coeffs.a2*(t**2) + coeffs.a3*(t**3)

# method to calculate velocity trajectory  
def velocity_trajectory(coeffs, t):
    return coeffs.a1 + coeffs.a2*2*t + coeffs.a3*3*(t**2)

# method to calculate acceleration trajectory  
def acceleration_trajectory(coeffs, t):
    return 2*coeffs.a2 + 6*coeffs.a3*t

def listener():
    rospy.init_node('plot_cubic_traj', anonymous=True)
    rospy.Subscriber("trajectory_coeffs", cubic_traj_coeffs, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
