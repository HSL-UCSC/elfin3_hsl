#!/usr/bin/env python

from __future__ import print_function

import rospy
import math
import moveit_commander
from elfin_robot_msgs.srv import SetInt16, SetInt16Request, SetJointTarget, SetJointTargetResponse
from std_srvs.srv import SetBool, SetBoolRequest

class LowLevel:

    def __init__(self):
        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.group=moveit_commander.MoveGroupCommander('elfin_arm')
        
        self.elfin_basic_api_ns='elfin_basic_api/'
        self.call_teleop_joint = rospy.ServiceProxy(self.elfin_basic_api_ns + 'joint_teleop', SetInt16)
        self.call_teleop_joint_req = SetInt16Request()

        self.call_teleop_stop = rospy.ServiceProxy(self.elfin_basic_api_ns + 'stop_teleop', SetBool)
        self.call_teleop_stop_req = SetBoolRequest()

    def move_handler(self, req):
        joint_num = req.joint
        joint_index = joint_num - 1
        joint_target = req.target

        # current_joint_values = self.group.get_current_joint_values()
        current_pos = self.group.get_current_joint_values()[joint_index] * 180 / math.pi

        direction = 1 if joint_target > current_pos else -1
        threshold = 1

        self.call_teleop_joint_req.data = joint_num * direction
        self.call_teleop_joint.call(self.call_teleop_joint_req)

        if direction > 0:
            while self.group.get_current_joint_values()[joint_index] * 180 / math.pi < joint_target:
                pass
        else:
            while self.group.get_current_joint_values()[joint_index] * 180 / math.pi > joint_target:
                pass

        # while abs(joint_target - self.group.get_current_joint_values()[joint_index] * 180 / math.pi) > threshold:
        #     pass
        # while abs(joint_target - current_pos) > threshold:
        #     current_joint_values = self.group.get_current_joint_values()
        #     current_pos = current_joint_values[joint_index] * 180 / math.pi
            # rospy.loginfo(current_joint_values)
            # rospy.loginfo("Current position: " + str(current_pos) + " Joint motion: " + str(joint_num * direction))

        self.call_teleop_stop_req.data = True
        self.call_teleop_stop.call(self.call_teleop_stop_req)
        self.call_teleop_joint_req.data = 0

        return SetJointTargetResponse(self.group.get_current_joint_values()[joint_index] * 180 / math.pi)

if __name__ == "__main__":
    rospy.init_node('low_level')
    ll = LowLevel()
    rospy.Service('move_handler', SetJointTarget, ll.move_handler)
    rospy.spin()