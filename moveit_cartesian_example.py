#!/usr/bin/env python3

import tf
import math
import geometry_msgs.msg

from fc_xarm_sim.robot_controllers.xarm_moveit import *


def go_to_joint_state(moveit_group_class=None, j0=None, j1=None, j2=None, j3=None,
                      j4=None, j5=None, check=False):
    """
    Utilize for changing manipulator's position. Provide angles in degrees.

    Args:
        moveit_group_class: The MoveGroupCommander instance for the robot arm.
        j0, j1, j2, j3, j4, j5: Joint angles in degrees for the manipulator.
        check: Boolean flag to log current and target joint angles for verification.

    Returns:
        True if the manipulator successfully reaches the target joint state, False otherwise.
    """    
    group = moveit_group_class.group

    joint_goal = group.get_current_joint_values()

    if check:
        rospy.loginfo('Current Joint Angles')
        rospy.loginfo(f'j0 {str(round(math.degrees(joint_goal[0]), 2))}') 
        rospy.loginfo(f'j1 {str(round(math.degrees(joint_goal[1]), 2))}') 
        rospy.loginfo(f'j2 {str(round(math.degrees(joint_goal[2]), 2))}') 
        rospy.loginfo(f'j3 {str(round(math.degrees(joint_goal[3]), 2))}') 
        rospy.loginfo(f'j4 {str(round(math.degrees(joint_goal[4]), 2))}') 

    if j0:
        joint_goal[0] = math.radians(j0)
    if j1:
        joint_goal[1] = math.radians(j1)
    if j2:
        joint_goal[2] = math.radians(j2)
    if j3:
        joint_goal[3] = math.radians(j3)
    if j4:
        joint_goal[4] = math.radians(j4)
    if j5:
        joint_goal[5] = math.radians(j5)

    if check:
        rospy.loginfo('Target Joint Angles')
        rospy.loginfo(f'j0 {str(round(math.degrees(joint_goal[0]), 2))}') 
        rospy.loginfo(f'j1 {str(round(math.degrees(joint_goal[1]), 2))}') 
        rospy.loginfo(f'j2 {str(round(math.degrees(joint_goal[2]), 2))}') 
        rospy.loginfo(f'j3 {str(round(math.degrees(joint_goal[3]), 2))}') 
        rospy.loginfo(f'j4 {str(round(math.degrees(joint_goal[4]), 2))}') 

    group.go(joint_goal, wait=True)
    group.stop()

    current_joints = moveit_group_class.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


def print_current_cartesian():
    """
    Print the current Cartesian coordinates and Euler angles of the manipulator's end effector.
    """
    current_pose = xarm5.group.get_current_pose().pose
    euler = tf.transformations.euler_from_quaternion([current_pose.orientation.x,
                                                      current_pose.orientation.y,
                                                      current_pose.orientation.z,
                                                      current_pose.orientation.w])
    rospy.logerr(f"Current Cartesian Coordinates:")
    rospy.loginfo(f"X: {str(round(current_pose.position.x, 3))}")
    rospy.loginfo(f"Y: {str(round(current_pose.position.y, 3))}")
    rospy.loginfo(f"Z: {str(round(current_pose.position.z, 3))}")

    rospy.loginfo(f"Roll: {str(round(math.degrees(euler[0])))}")
    rospy.loginfo(f"Pitch: {str(round(math.degrees(euler[1])))}")
    rospy.loginfo(f"Yaw: {str(round(math.degrees(euler[2])))}")


def go_to_pose_goal(moveit_group_class=None, waypoints=None, base_frame='link_base', end_frame='link_tcp'):
    """
    Move the manipulator's end effector to a series of target poses defined by waypoints.
    Does not work well when behind the robot. Use the print_cartesian to ensure valid
    workspace. Moving end-of-effector in this mode not recommended. Use Jointstates instead.

    Args:
        moveit_group_class: The MoveGroupCommander instance for the robot arm.
        waypoints: List of geometry_msgs.Pose waypoints defining the target poses.
        base_frame: The reference frame for the target poses.
        end_frame: The end effector link of the manipulator.

    Returns:
        True if the manipulator successfully reaches the last waypoint, False otherwise.
    """
    moveit_group_class.group.set_pose_reference_frame(base_frame)
    group = moveit_group_class.group
    group.clear_pose_targets()

    rospy.loginfo(f"Planning frame: {str(moveit_group_class.group.get_planning_frame())}")
    rospy.loginfo(f"End effector link: {str(moveit_group_class.group.get_end_effector_link())}]")
    rospy.loginfo(f"Available Planning Groups: {str(moveit_group_class.robot.get_group_names())}")
    print_current_cartesian()

    waypoints_list = []
    for item_number, waypoint in enumerate(waypoints):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = waypoint.position.x
        pose_goal.position.y = waypoint.position.y
        pose_goal.position.z = waypoint.position.z
        pose_goal.orientation.x = waypoint.orientation.x
        pose_goal.orientation.y = waypoint.orientation.y
        pose_goal.orientation.z = waypoint.orientation.z
        pose_goal.orientation.w = waypoint.orientation.w
        waypoints_list.append(pose_goal)

        euler = tf.transformations.euler_from_quaternion([pose_goal.orientation.x,
                                                          pose_goal.orientation.y,
                                                          pose_goal.orientation.z,
                                                          pose_goal.orientation.w])

        rospy.logwarn(f"Waypoint {str(item_number)} Coordinates:")
        rospy.loginfo(f"X: {str(round(waypoint.position.x, 3))}")
        rospy.loginfo(f"Y: {str(round(waypoint.position.y, 3))}")
        rospy.loginfo(f"Z: {str(round(waypoint.position.z, 3))}")

        rospy.loginfo(f"Roll: {str(round(math.degrees(euler[0])))}")
        rospy.loginfo(f"Pitch: {str(round(math.degrees(euler[1])))}")
        rospy.loginfo(f"Yaw: {str(round(math.degrees(euler[2])))}")

    group.set_pose_targets(waypoints_list, end_frame)
    (plan, fraction) = group.compute_cartesian_path(waypoints_list, 0.01, 0.0)

    group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()

    current_pose = moveit_group_class.group.get_current_pose().pose
    return all_close(waypoints_list[-1], current_pose, 0.01)


def create_pose_msg(x, y, z, roll, pitch, yaw):
    """
    Create a geometry_msgs.Pose message based on the given Cartesian coordinates and Euler angles.

    Args:
        x, y, z: Cartesian coordinates of the position.
        roll, pitch, yaw: Euler angles in degrees.

    Returns:
        geometry_msgs.Pose message with the specified position and orientation.
    """
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    q1, q2, q3, q4 = tf.transformations.quaternion_from_euler(math.radians(roll),
                                                               math.radians(pitch),
                                                               math.radians(yaw))
    pose.orientation.x = q1
    pose.orientation.y = q2
    pose.orientation.z = q3
    pose.orientation.w = q4

    return pose


def get_current_joint_state(moveit_group_class):
    """
    Get the current joint state of the manipulator.

    Args:
        moveit_group_class: The MoveGroupCommander instance for the robot arm.

    Returns:
        List of current joint angles.
    """
    joint_state = moveit_group_class.group.get_current_joint_values()
    return joint_state


if __name__ == '__main__':
    xarm5 = ManipulatorMoveit('xarm5')
    xarm5.set_state('home')

    print_current_cartesian()
    waypoints1 = []
    # waypoints1.append(create_pose_msg(0.3,0.0,0.4,180,270,0))
    waypoints1.append(create_pose_msg(0.1, 0.0, 0.5, 180, 0, 0))
    waypoints1.append(create_pose_msg(0.2, 0.0, 0.5, 180, 0, 0))
    waypoints1.append(create_pose_msg(0.3, 0.0, 0.5, 180, 0, 0))
    waypoints1.append(create_pose_msg(0.4, 0.0, 0.5, 180, 0, 0))
    waypoints1.append(create_pose_msg(0.5, 0.0, 0.5, 180, 0, 0))
    waypoints1.append(create_pose_msg(0.0, -0.5, 0.5, 180, 0, 0))
    r = go_to_pose_goal(xarm5, waypoints=waypoints1, base_frame='link_base', end_frame='link_tcp')

    r = go_to_joint_state(xarm5, j3=45, check=True)

    waypoints2 = []
    waypoints2.append(create_pose_msg(0.0, -0.5, 0.5, 180, 0, 0))
    waypoints2.append(create_pose_msg(0.5, 0.0, 0.5, 180, 0, 0))
    waypoints2.append(create_pose_msg(0.0, 0.5, 0.5, 180, 0, 0))
    waypoints2.append(create_pose_msg(0.5, 0.0, 0.5, 180, 0, 0))
    r = go_to_pose_goal(xarm5, waypoints=waypoints2, base_frame='link_base', end_frame='link_tcp')
