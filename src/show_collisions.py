#!/usr/bin/env python

import rospy
import numpy
import argparse
import struct
import sys
import math

from scipy import spatial
from collision_checker.srv import CheckCollision
from std_msgs.msg import String

import baxter_interface
#import baxter_pykdl as bk

from baxter_pykdl import baxter_kinematics as bk

from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF

from group7_proj2.srv import *
from group7_proj2.msg import ExistsCollision

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import(
    SolvePositionIK,
    SolvePositionIKRequest,
)

def get_Hand():
    hands= {'right','left'}
    if rospy.get_param('/hand_mode') == 'both':
        if rospy.get_param('/hand_switch') == 1:
            return 'right'
        else:
            return 'left'
    elif rospy.get_param('/hand_mode') in hands:
            return rospy.get_param('/hand_mode')
    else:
        return 0
        
def FindCollision():
	hand = get_Hand()
	limb=baxter_interface.Limb(hand)
	limb_joints=limb.joint_angles()
	check_point = [limb_joints[hand+'_s0'],limb_joints[hand+'_s1'],limb_joints[hand+'_e0'],limb_joints[hand+'_e1'],limb_joints[hand+'_w0'],limb_joints[hand+'_w1'],limb_joints[hand+'_w2']]
	res = s_cc(String(hand),check_point)
	if res.collision:
		return "!!!!!!!!!!!!!!!!!!!!!!!!!! COLLISION !!!!!!!!!!!!!!!!!!!!!!!!!!!"
	else:
		return ":)"

    
def show_collisions():
	rospy.init_node('show_collisions')
	arm = FindCollision()
	hello_str = ExistsCollision(arm)
	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()
		
		
if __name__ == "__main__":
	pub = rospy.Publisher('exists_collision',ExistsCollision,queue_size=10)
	s_cc = rospy.ServiceProxy('/check_collision', CheckCollision)
	rospy.wait_for_service('/check_collision')
	show_collisions()
	
	
