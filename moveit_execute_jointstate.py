#!/usr/bin/env python3

import rospy
import yaml
from fc_xarm_sim.robot_controllers.xarm_moveit import *


def go_to_joint_state(moveit_group_class=None, joint_angles=None, check=False):
    """
    Utilize for changing manipulator's position. Provide angles in radians.

    Args:
        moveit_group_class: The MoveGroupCommander instance for the robot arm.
        joint_angles: List of joint angles in radians for the manipulator.
        check: Boolean flag to log current and target joint angles for verification.

    Returns:
        True if the manipulator successfully reaches the target joint state, False otherwise.
    """
    group = moveit_group_class.group

    joint_goal = group.get_current_joint_values()
    joint_goal = joint_angles 

    group.go(joint_goal, wait=True)
    group.stop()

    current_joints = moveit_group_class.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


if __name__ == '__main__':
    xarm5 = ManipulatorMoveit('xarm6')
    xarm5.set_state('home')

    with open('joint_states.yaml', 'r') as file:
        yaml_data = yaml.load(file, Loader=yaml.FullLoader)

    skip_count = 0  # Counter to skip waypoints
    for waypoint, joint_angles in yaml_data['robotic_arm'].items():
        if skip_count > 0:
            skip_count -= 1
            continue  # Skip this waypoint
        else:
            result = go_to_joint_state(xarm5, joint_angles)
            if result:
                rospy.loginfo(f"Reached waypoint: {waypoint}")
            else:
                rospy.logwarn(f"Failed to reach waypoint: {waypoint}")

        skip_count = 0  
