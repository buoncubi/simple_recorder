#!/usr/bin/env python
from numpy import *
import rospy
from baxter_core_msgs.msg import JointCommand
pub_joints = rospy.Publisher('/robot/limb/right/joint_command', JointCommand)
rospy.init_node('write_to_baxr', anonymous=False)
rate = rospy.Rate(20)
cmd_msg = JointCommand()
cmd_msg.mode = JointCommand.POSITION_MODE
cmd_msg.names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

qini= [0.14035924209151535, -0.9794467330648366,1.588437105855346,0.39154859610775183,-0.8233641878974959, 0.7113835903818606, 0.8348690438066364]
while not rospy.is_shutdown():
    cmd_msg.command = qini
    pub_joints.publish(cmd_msg)
    rate.sleep()
