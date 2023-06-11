#!/usr/bin/env python3
import rospy
import yaml
from sensor_msgs.msg import JointState

def auto_save():
    rospy.init_node('joint_state_saver')

    joint_states = {'robotic_arm': {}}  # Dictionary to store joint states

    waypoint_counter = 1

    def joint_state_callback(msg):
        global waypoint_counter
        waypoint_name = 'waypoint_{}'.format(waypoint_counter)
        joint_states['robotic_arm'][waypoint_name] = list(msg.position)
        waypoint_counter += 1

    rospy.Subscriber('/xarm/joint_states', JointState, joint_state_callback)

    while not rospy.is_shutdown():
        key = input("q to quit): ")
        if key.lower() == 'q':
            break

    with open('joint_states.yaml', 'w') as file:
        yaml.dump(joint_states, file, sort_keys=False)

    rospy.logerr("Joint states saved to joint_states.yaml")

class ManualSave():
    
    def __init__(self) -> None:    
        rospy.init_node('joint_state_saver')
        rospy.Subscriber('/xarm/joint_states', JointState, self.joint_state_callback)

        self.joint_states = {'robotic_arm': {}}     
        self.waypoint_counter = 1

    def update_waypoint(self):

        waypoint_name = 'waypoint_{}'.format(self.waypoint_counter)
        self.joint_states['robotic_arm'][waypoint_name] = list(self.pose)
        self.waypoint_counter += 1

    def joint_state_callback(self,msg):
        self.pose = msg.position

    def execute(self):

        while not rospy.is_shutdown():
            key = input("Press s to save and q to quit: ")

            if key.lower() == 's':
                self.update_waypoint()
                continue
            if key.lower() == 'q':
                break

        with open('joint_states.yaml', 'w') as file:
            yaml.dump(self.joint_states, file, sort_keys=False)

        rospy.logerr("Joint states saved to joint_states.yaml")

if __name__=='__main__':
    ms = ManualSave()
    ms.execute()
