#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float64
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## Initializing `moveit_commander`:
    moveit_commander.roscpp_initialize(sys.argv)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints). This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## A `DisplayTrajectory`_ ROS publisher is created which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.robot = robot
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):

    # Defining joint space based on the given configuration
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # move the panda robot to the starting configuration
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    # For testing:
    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def plan_cartesian_path(self, side, scale=1):

    ## define the cartesian path
    waypoints = []

    wpose = self.move_group.get_current_pose().pose
    # increment along the x axis
    wpose.position.x += scale * side
    waypoints.append(copy.deepcopy(wpose))
    # increment along the y axis
    wpose.position.y += scale * side
    waypoints.append(copy.deepcopy(wpose))
    # decrement along the x axis
    wpose.position.x -= scale * side
    waypoints.append(copy.deepcopy(wpose))
    # decrement along the y axis
    wpose.position.y -= scale * side
    waypoints.append(copy.deepcopy(wpose))


    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    
    return plan, fraction

  def display_trajectory(self, plan):

    ## Displaying the planned path
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):

    # Execute the planned path
    self.move_group.execute(plan, wait=True)


def callback(length, move_panda):
    try:
        # define the sleep time between planning and displaying path
        # sleep time is directly proportional to the length of the side of the square
        # the sleep time is set in relation to the State Display Time = 0.05s
        sleep_time = (length.data * (5 / 0.2)) + 1

        # receive the length of the side of the square
        print ""
        print "========================================================"
        print "Message Received"
        print "Side: {}".format(length.data)
        print ""

        print "============ MOVING TO STARTING CONFIGURATION =========="
        move_panda.go_to_joint_state()

        print "============ PLANNING A CARTESIAN PATH ================="
        cartesian_plan, fraction = move_panda.plan_cartesian_path(length.data)
        rospy.sleep(sleep_time)

        print "============ SHOWING THE PLANNED TRAJECTORY ============"
        move_panda.display_trajectory(cartesian_plan)
        rospy.sleep(sleep_time)

        print "============ EXECUTING THE PLANNED PATH ================"
        move_panda.execute_plan(cartesian_plan)
        print ""
        print "Waiting for the next message"
        print "========================================================"
        print ""

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

def main():

    move_panda = MoveGroupPythonInteface()
    # initialise the node
    rospy.init_node('move_panda_square', anonymous=True)
    # this node subscribes to the topic /square_side
    rospy.Subscriber('square_side', Float64, callback, move_panda)
    rospy.spin()

if __name__ == '__main__':
  main()