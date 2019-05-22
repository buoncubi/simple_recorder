#!/usr/bin/env python
from numpy import *
import rospy
from baxter_core_msgs.msg import JointCommand
pub_joints = rospy.Publisher('/robot/limb/left/joint_command', JointCommand)
rospy.init_node('write_to_baxr', anonymous=False)
rate = rospy.Rate(20)
cmd_msg = JointCommand()
cmd_msg.mode = JointCommand.POSITION_MODE
cmd_msg.names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

qini = [-0.9522185740798705,-0.5553010452145197,-1.9167089944628244,0.2381505173192105,0.5483981316690354,0.018791264651596317, -1.6451943950071062]
while not rospy.is_shutdown():
    cmd_msg.command = qini
    pub_joints.publish(cmd_msg)
    rate.sleep()



#name ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
#Position:  (0.0, -0.02914563496982286, -1.9167089944628244, 0.2381505173192105, -0.9522185740798705, -0.5553010452145197, 0.5483981316690354, 0.018791264651596317, -1.6451943950071062, 1.588437105855346, #0.39154859610775183, 0.14035924209151535, -0.9794467330648366, -0.8233641878974959, 0.7113835903818606, 0.8348690438066364, -12.565987119160338)

