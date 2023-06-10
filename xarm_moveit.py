#!/usr/bin/env python3
"""
Moveit Client: The client that provides goal for the robotic arms in this case being xarm5.
"""
import sys
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
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

class ManipulatorMoveit(object):

  def __init__(self, manipulator_group_name):
    super(ManipulatorMoveit, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = manipulator_group_name
    group = moveit_commander.MoveGroupCommander(group_name)

    # group.allow_replanning(True)
    group.set_max_velocity_scaling_factor(1)
    group.set_max_acceleration_scaling_factor(1)
    # group.set_planner_id("PRMkConfigDefault")
    group.set_planner_id("T-RRT")
    group.set_num_planning_attempts(5)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    # print(eef_link)
    # We can get a list of all the groups in the robot:
    joint_names = robot.get_joint_names ()

    # Sometimes for debugging it is useful to print the entire state of the robot:
    # print(robot.get_current_state())
    # print(group.get_named_targets())

    # Misc variables
    self.moveit_group_name = manipulator_group_name
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = group.get_planning_frame()
    self.eef_link = eef_link
    self.group_names = robot.get_group_names()

  def go_to_joint_state(self, j0=0, j1= -pi/4, j2= 0, j3= -pi/2, j4= 0, j5= pi/3):
    group = self.group

    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = j0
    joint_goal[1] = j1
    joint_goal[2] = j2
    joint_goal[3] = j3
    joint_goal[4] = j4
    # joint_goal[5] = j5

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_manipulator_joint_state(self, joints):
    """Moves gripper from open to close. 0 is open 0.848 is closed

    Args:
        joints (float): Gripper Postion

    Returns:
        Bool: Verifies whether joint angle is the same as given angle.
    """    
    group = self.group

    joint_goal = group.get_current_joint_values()

    joint_goal[0] = joints
    joint_goal[1] = joints
    joint_goal[2] = joints
    joint_goal[3] = joints
    joint_goal[4] = joints
    joint_goal[5] = joints

    group.go(joint_goal, wait=True)
    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self, x =0.6, y =0.05, z =0.400, R=0 , P=0, Y=0, frame='link_tcp'):

    quaternion = quaternion_from_euler(R, P, Y)

    group = self.group
    group.clear_pose_targets()
    # group.allow_replanning(True)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    group.set_pose_target(pose_goal, frame) # original
    # group.set_random_target()

    plan = group.go(wait=True)

    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def set_state(self, state):

    group = self.group
    group.set_named_target(state)
    plan = group.go(wait=True)
    group.stop()

  def plan_cartesian_path(self, scale=1):
    group = self.group

    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)


  def execute_plan(self, plan):

    group = self.group
    group.execute(plan, wait=True)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


  def add_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene

    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "xarm5_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):

    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object(box_name)

    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def main():

    xarm5 = ManipulatorMoveit('xarm5')
    # xarm5_gripper = ManipulatorMoveit('xarm_gripper')
    
    # Arm control
    for i in range(1):

      # Planner execution
      result = xarm5.go_to_pose_goal(x =0.500, y =0.10,  z =0.20, R=0 , P=180, Y=0)
      # result = xarm5.go_to_pose_goal(x =0.500, y =0.10,  z =0.100, R=0 , P=180, Y=0)
      # result = xarm5.go_to_pose_goal(x =0.500, y =-0.40, z =0.100, R=0 , P=180, Y=0)
      # result = xarm5.go_to_pose_goal(x =0.500, y =-0.50, z =0.05, R=0 , P=180, Y=0)

      # Joint state execution
      # print(xarm5.group.get_current_joint_values())
      # results = xarm5.go_to_joint_state(0.19732783745028915, 0.5545522696969947, -1.0790973475430228, 0.5249336401887801, -2.944199885397924)
      # results = xarm5.go_to_joint_state(-0.6748302475740608, 0.7491247801516803, -1.7612577052731577, 1.0127911059163779, 2.467154691713624)

    # Gripper movement
    # xarm5_gripper.set_state(state='close')
    # xarm5_gripper.go_to_manipulator_joint_state(0.8)

    # Additional Controls
    # xarm5.set_state('home') 
    # xarm5.set_state('hold-up')
    # reult = xarm5.plan_cartesian_path()
    # result = xarm5.go_to_joint_state()

if __name__ == '__main__':
  main()