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
from group7_proj2.msg import State

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
	
def DrawLine(negx,a,negy,b,c): #a and b are offests to current position in the "whiteboard plane" coords
    print("Starting line...")
    hand = get_Hand()
    limb=baxter_interface.Limb(hand)
    rospy.sleep(0.1)
    j_vels=limb.joint_velocities()
    start_pose=limb.endpoint_pose()
    start_pos=start_pose['position']
    x_unit = numpy.array(rospy.get_param('/plane_x_vector'))
    y_unit = numpy.array(rospy.get_param('/plane_y_vector'))
    z_unit = numpy.cross(x_unit,y_unit)
    z_abs=numpy.array([0,0,c])

    abs_dist=numpy.linalg.norm([a,b,c])

    if negx:	
        a=-a

    if negy:
        b=-b

    dir_vect =(a*x_unit+b*y_unit+c*z_unit)/(numpy.linalg.norm(a*x_unit+b*y_unit+c*z_unit))

	#MOVING IN ABSOLUTE Z 
	#dir_vect =(a*x_unit+b*y_unit+z_abs)/(numpy.linalg.norm(a*x_unit+b*y_unit+z_abs)) rostopic pub /command group7_proj2/Command DrawLine 1 2 9 0 8 8 -1
    v_0=.02
    start_xyz=numpy.array([start_pos.x, start_pos.y, start_pos.z])
    goal=numpy.array(start_xyz+a*x_unit+b*y_unit+c*z_unit)

    v=v_0*dir_vect
    w = numpy.array([0, 0, 0])
    xi=numpy.concatenate((v, w),axis=0)	

    joint_values = limb.joint_names()
    kin=bk(hand)

    q_dot=numpy.dot(kin.jacobian_pseudo_inverse(),xi)

    j_vels[hand+'_s0']=q_dot[0,0]
    j_vels[hand+'_s1']=q_dot[0,1]
    j_vels[hand+'_e0']=q_dot[0,2]
    j_vels[hand+'_e1']=q_dot[0,3]
    j_vels[hand+'_w0']=q_dot[0,4]
    j_vels[hand+'_w1']=q_dot[0,5]
    j_vels[hand+'_w2']=q_dot[0,6]

    frq=.01
    limb.set_command_timeout(.01)
    limb.set_joint_velocities(j_vels)
    vel_dict=limb.endpoint_velocity()['linear']
    #	print(vel_dict)
    rospy.sleep(frq)


    #eps=.01 	#threshold for how close is good enough to goal
    time_at_start = rospy.get_time() # in seconds
    isMoving = True # get this from topic
    while isMoving:
        t = rospy.get_time() - time_at_start
        pos_dict=limb.endpoint_pose()['position']
        or_dict=limb.endpoint_pose()['orientation']
        current_position = numpy.array([pos_dict.x,pos_dict.y,pos_dict.z,or_dict.x,or_dict.y,or_dict.z])

        #		print("current pos:")
        #		print(current_position)
        expected_position = numpy.array([start_pos.x, start_pos.y, start_pos.z,0,0,0]) + numpy.array([xi[0]*t, xi[1]*t, xi[2]*t,0,0,0])
        joint_angles=limb.joint_angles()
        #		print("expected pos:")
        #		print(expected_position)		
        position_error = current_position - expected_position

        #		print("pos error:")
        #		print(position_error)
        vel_dict=limb.endpoint_velocity()['linear']
        vel_ang=limb.endpoint_velocity()['angular']
        #		print(vel_dict)
        current_v = numpy.array([vel_dict.x,vel_dict.y,vel_dict.z,0,0,0])
        #		print("current v:")
        #		print(current_v)

        k = .25
        dir_vect = dir_vect-k*numpy.array([position_error[0],position_error[1],position_error[2]])
        dir_vect = dir_vect/(numpy.linalg.norm(dir_vect))
        new_vel = v_0*numpy.array([dir_vect[0],dir_vect[1],dir_vect[2],0,0,0])

        I=numpy.identity(7)

        b_vect=numpy.array([0,0,0,0,0,0,0])

        #if near_limits(joint_angles,hand):
        #    b_vect=move_away(joint_angles,hand)



        #q_dot = numpy.dot(kin.jacobian_pseudo_inverse(),(new_vel))+numpy.dot((I-numpy.dot(kin.jacobian_pseudo_inverse(),kin.jacobian())),b_vect)
	
        q_dot = numpy.dot(kin.jacobian_pseudo_inverse(),(new_vel))

        j_vels[hand+'_s0']=q_dot[0,0]
        j_vels[hand+'_s1']=q_dot[0,1]
        j_vels[hand+'_e0']=q_dot[0,2]
        j_vels[hand+'_e1']=q_dot[0,3]
        j_vels[hand+'_w0']=q_dot[0,4]
        j_vels[hand+'_w1']=q_dot[0,5]
        j_vels[hand+'_w2']=q_dot[0,6]
        #		print("new j_vels:")
        #		print(j_vels)
        rospy.sleep(frq)
        #		if AreWeThereYet(limb,goal,eps):
        if AreWeThereYet(limb,start_xyz,abs_dist):
        #limb.exit_control_mode()
            j_vels=stop_moving(hand)
            isMoving = False
            print("We're there!!!!!!!!!!!!!")

        limb.set_joint_velocities(j_vels)

    return True 
	
def near_limits(ang,hand):
    angle_ranges = {'S0':[-2.461,0.890],
				    'S1':[-2.147,1.047],
				    'E0':[-3.028,3.028],
				    'E1':[-0.052,2.618],
				    'W0':[-3.059,3.059],
				    'W1':[-1.571,2.094],
				    'W2':[-3.059,3.059]}

    if((ang[hand+'_s0']-angle_ranges['S0'][0]<.1) | (ang[hand+'_s0']-angle_ranges['S0'][1]>-.1)):
        return True

    if((ang[hand+'_s1']-angle_ranges['S1'][0]<.1) | (ang[hand+'_s1']-angle_ranges['S1'][1]>-.1)):
        return True

    if((ang[hand+'_e0']-angle_ranges['E0'][0]<.1) | (ang[hand+'_e0']-angle_ranges['E0'][1]>-.1)):
        return True

    if((ang[hand+'_e1']-angle_ranges['E1'][0]<.1) | (ang[hand+'_e1']-angle_ranges['E1'][1]>-.1)):
        return True

    if((ang[hand+'_w0']-angle_ranges['W0'][0]<.1) | (ang[hand+'_w0']-angle_ranges['W0'][1]>-.1)):
        return True

    if((ang[hand+'_w1']-angle_ranges['W1'][0]<.1 | ang[hand+'_w1']-angle_ranges['W1'][1]>-.1)):
        return True

    if((ang[hand+'_w2']-angle_ranges['W2'][0]<.1 | ang[hand+'_w2']-angle_ranges['W2'][1]>-.1)):
        return True

    return False

def move_away(ang,hand):
    angle_ranges = {'S0':[-2.461,0.890],
			    'S1':[-2.147,1.047],
			    'E0':[-3.028,3.028],
			    'E1':[-0.052,2.618],
			    'W0':[-3.059,3.059],
			    'W1':[-1.571,2.094],
			    'W2':[-3.059,3.059]}

    mid_angs=numpy.array([-.8755,-.55,0,1.283,0,.2615,0])

    b0=mid_angs[0]-ang[hand+'_s0']
    b1=mid_angs[1]-ang[hand+'_s1']
    b2=mid_angs[2]-ang[hand+'_e0']
    b3=mid_angs[3]-ang[hand+'_e1']
    b4=mid_angs[4]-ang[hand+'_w0']
    b5=mid_angs[5]-ang[hand+'_w1']
    b6=mid_angs[6]-ang[hand+'_w2']

    b=numpy.array([b0,b1,b2,b3,b4,b5,b6])/numpy.linalg.norm([b0,b1,b2,b3,b4,b5,b6])

    speed=0.3

    return speed*b

def stop_moving(hand):
	limb=baxter_interface.Limb(hand)
	rospy.sleep(0.1)
	j_vels=limb.joint_velocities()
	j_vels=limb.joint_velocities()
	j_vels[hand+'_s0']=0
	j_vels[hand+'_s1']=0
	j_vels[hand+'_e0']=0
	j_vels[hand+'_e1']=0
	j_vels[hand+'_w0']=0
	j_vels[hand+'_w1']=0
	j_vels[hand+'_w2']=0
	
	return j_vels

	
def DrawSpline(cw,theta,ni,i,nj,j):
	# f an array of functions (for each piecewise component)
	hand = get_Hand()
	limb=baxter_interface.Limb(hand)
	rospy.sleep(0.1)
	j_vels=limb.joint_velocities()
	start_pose=limb.endpoint_pose()
	start_pos=start_pose['position']
	start_xyz=numpy.array([start_pos.x, start_pos.y, start_pos.z])
	x_unit = numpy.array(rospy.get_param('/plane_x_vector'))
	y_unit = numpy.array(rospy.get_param('/plane_y_vector'))

#	x_unit = numpy.array([1, 0 ,0])
#	y_unit = numpy.array([0, 1, 0])

	if ni:
		i=-i

	if nj:
		j=-j

	
	print('start xyz: ', start_xyz)
	
	center_arc=start_xyz+i*x_unit+j*y_unit
	
	print('center arc: ', center_arc)

	start_vect=start_xyz-center_arc

	radius=numpy.linalg.norm(start_vect)
	
	print('radius :', radius)
	
	print('start vector: ', start_vect)
	
	start_vect_unit=start_vect/(numpy.linalg.norm(start_vect))
	
	start_theta=numpy.arctan2([-j],[-i])

	if cw:
		end_theta=start_theta-theta
	else:
		end_theta=start_theta+theta

	n_theta=theta*(20/numpy.pi)

	theta_arr=numpy.array(numpy.linspace(start_theta,end_theta,n_theta))
	
	print('start theta: ', start_theta)

	print('end theta: ',end_theta)
	
	x_positions=numpy.concatenate([numpy.array([-i]),radius*numpy.cos(theta_arr)])
	y_positions=numpy.concatenate([numpy.array([-j]),radius*numpy.sin(theta_arr)])

	print('x_pos: ',x_positions)
	print('y_pos: ',y_positions)

	for k in range(0,numpy.size(x_positions)-1):
		delta_x=x_positions[k+1]-x_positions[k]
		delta_y=y_positions[k+1]-y_positions[k]
		cur_pose=limb.endpoint_pose()
		cur_pos=start_pose['position']
		cur_xyz=numpy.array([cur_pos.x, cur_pos.y, cur_pos.z])
		
		print('dx: ',delta_x)
		print('dy: ',delta_y)
		
		temp=DrawLine(0,delta_x,0,delta_y,0)#start_xyz[2]-cur_xyz[2])

	return True
	
def AreWeThereYet(limb,start,dist):
	cur_pos=limb.endpoint_pose()['position']
	cur_pos_arr=numpy.array([cur_pos.x, cur_pos.y, cur_pos.z])
	cur_dist_vect=cur_pos_arr-start
	cur_dist=numpy.linalg.norm(cur_dist_vect)
	
	if(cur_dist<dist):
		return False

	return True    
	
def Alphabet():
    small_r=.075

    big_r=2*small_r

    clear=.05

    keep_typing=True
    while keep_typing:
        letter=raw_input("Input a letter:")
        if letter=='.':
            keep_typing= False

        elif letter.upper()=='A':
            DrawLine(0,small_r/2,0,big_r,0)
            DrawLine(0,small_r/2,0,-big_r,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,-small_r/4,0,small_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,-small_r/2,0,0,0)

        elif letter.upper()=='B':
            DrawLine(0,small_r,0,0,0)
            DrawSpline(0,numpy.pi,0,0,0,small_r)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawSpline(0,numpy.pi,0,0,0,small_r)
            DrawLine(1,small_r,0,0,0)
            DrawLine(0,0,1,2*big_r,0)

        elif letter.upper()=='C':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawSpline(1,numpy.pi,0,0,0,small_r)

        elif letter.upper()=='D':
            DrawLine(0,0,0,big_r,0)
            DrawSpline(1,numpy.pi,0,0,0,-small_r)

        elif letter.upper()=='E':
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,1,small_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(1,small_r,0,0,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,1,small_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,small_r,0,0,0)

        elif letter.upper()=='F':
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,1,small_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(1,small_r,0,0,0)

        elif letter.upper()=='G':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,0,0,small_r,0)
            DrawLine(1,small_r,0,0,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r,1,small_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawSpline(1,numpy.pi,0,0,0,small_r)

        elif letter.upper()=='H':
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,0,1,big_r,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,0,small_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(1,small_r,0,0,0)

        elif letter.upper()=='I':
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(1,small_r/2,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,small_r/2,0,0,0)
            DrawLine(1,small_r,0,0,0)

        elif letter.upper()=='J':
            DrawSpline(0,numpy.pi/2,0,0,0,small_r)
            DrawLine(0,0,0,small_r,0)
            DrawLine(1,small_r,0,0,0)

        elif letter.upper()=='K':
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(1,small_r,1,small_r,0)
            DrawLine(0,small_r,1,small_r,0)

        elif letter.upper()=='L':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(1,small_r,0,0,0)
            DrawLine(0,0,0,big_r,0)

        elif letter.upper()=='M':
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,small_r,1,small_r,0)
            DrawLine(0,small_r,0,small_r,0)
            DrawLine(0,0,0,big_r,0)

        elif letter.upper()=='N':
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,small_r,1,small_r,0)
            DrawLine(0,0,0,big_r,0)

        elif letter.upper()=='O':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawSpline(1,numpy.pi,0,0,0,small_r)
            DrawSpline(1,numpy.pi,0,0,1,small_r)

        elif letter.upper()=='P':
            DrawLine(0,0,0,big_r,0)
            DrawSpline(1,numpy.pi,0,0,1,0.5*small_r)

        elif letter.upper()=='Q':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawSpline(1,numpy.pi,0,0,0,small_r)
            DrawSpline(1,numpy.pi,0,0,1,small_r)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,small_r/2,0,small_r/2,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,small_r/2,1,small_r/2,0)

        elif letter.upper()=='R':
            DrawLine(0,0,0,big_r,0)
            DrawSpline(1,numpy.pi,0,0,1,small_r/2)
            DrawLine(0,small_r,1,small_r,0)

        elif letter.upper()=='S':
            DrawSpline(0,numpy.pi,0,small_r,0,0)
            DrawSpline(1,numpy.pi,0,small_r,0,0)

        elif letter.upper()=='T':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,big_r,0,0,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(1,small_r,0,0,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,0,1,big_r,0)

        elif letter.upper()=='U':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,0,1,small_r,0)
            DrawSpline(0,numpy.pi,0,small_r,0,0)
            DrawLine(0,0,0,small_r,0)

        elif letter.upper()=='V':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,small_r,1,big_r,0)
            DrawLine(0,small_r,0,big_r,0)

        elif letter.upper()=='W':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,0,1,big_r,0)
            DrawLine(0,small_r,0,small_r,0)
            DrawLine(0,small_r,1,small_r,0)
            DrawLine(0,0,0,big_r,0)

        elif letter.upper()=='X':
            DrawLine(0,small_r,0,big_r,0)
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,1,big_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(1,small_r,0,big_r,0)

        elif letter.upper()=='Y':
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,small_r,1,small_r,0)
            DrawLine(0,small_r,0,small_r,0)
            DrawLine(0,big_r,1,big_r,0)

        elif letter.upper()=='Z':
            print ("Writing letter...")
            DrawLine(0,0,0,0,clear)
            DrawLine(0,0,0,big_r,0)
            DrawLine(0,0,0,0,-clear)
            DrawLine(0,small_r,0,0,0)
            DrawLine(1,small_r,1,big_r,0)
            DrawLine(0,small_r,0,0,0)
            print ("Done.")
            
        else:
            print ("This is not a letter, try again!!!")
            
    return True


def handle_move_robot(req):
    if req.Action=="DrawLine":
        isPossible = DrawLine(req.nx,req.x,req.ny,req.y,0)
    elif req.Action=="DrawSpline":
        isPossible = DrawSpline(req.cw,req.theta,req.nx,req.x,req.ny,req.y)
    elif req.Action=="Typewriter":
        isPossible=Alphabet()
    return isPossible

def define_plane():
	hand = get_Hand()
	limb=baxter_interface.Limb(hand)
	rospy.sleep(0.1)
	
	#limb.move_to_neutral()

	limb_joints=limb.joint_angles()
	raw_input("Place the arm at the origin of your plane. Then press 'Enter'.")	
	pose=limb.endpoint_pose()
	opos=pose['position']
	rospy.set_param('/plane_origin',[opos.x, opos.y, opos.z])

	limb_joints[hand+'_s0']=limb_joints[hand+'_s0']+.65
	#limb.set_joint_positions(limb_joints)

	raw_input("Place the arm at the tip of your x unit vector. Then press 'Enter'.")
	pose=limb.endpoint_pose()
	xpos=pose['position']
	print([xpos.x, xpos.y, xpos.z])

	limb_joints[hand+'_s0']=limb_joints[hand+'_s0']-numpy.pi/2
	#limb.set_joint_positions(limb_joints)
	
	raw_input("Place the arm at the tip of your y unit vector. Then press 'Enter'.")
	pose=limb.endpoint_pose()
	ypos=pose['position']
	print([ypos.x, ypos.y, ypos.z])

	#limb.move_to_neutral()

	#make these unit vectors:
	mag_x=math.sqrt(math.pow(xpos.x-opos.x,2)+math.pow(xpos.y-opos.y,2)+math.pow(xpos.z-opos.z,2))
	mag_y=math.sqrt(math.pow(ypos.x-opos.x,2)+math.pow(ypos.y-opos.y,2)+math.pow(ypos.z-opos.z,2))
	norm_x=[(xpos.x-opos.x)/mag_x, (xpos.y-opos.y)/mag_x, (xpos.z-opos.z)/mag_x]
	norm_y=[(ypos.x-opos.x)/mag_y, (ypos.y-opos.y)/mag_y, (ypos.z-opos.z)/mag_y]
	rospy.set_param('/plane_x_vector',norm_y)	#this makes it so our x and y are similar to baxters x and 
	rospy.set_param('/plane_y_vector',norm_x)
#	rospy.set_param('/plane_y_vector',[0,1,0])
#	rospy.set_param('/plane_x_vector',[1,0,0])
	print "Plane Origin is: %s"%(rospy.get_param('/plane_origin'))
	print "x vect is: %s"%(rospy.get_param('/plane_x_vector'))
	print "y vect is: %s"%(rospy.get_param('/plane_y_vector'))

	a = (xpos.y-opos.y)*(ypos.z-opos.z)-(ypos.y-opos.y)*(xpos.z-opos.z)
	b = (xpos.z-opos.z)*(ypos.x-opos.x)-(ypos.z-opos.z)*(xpos.x-opos.x)
	c = (xpos.x-opos.x)*(ypos.y-opos.y)-(ypos.x-opos.x)*(xpos.y-opos.y)
	d = -(a*opos.x+b*opos.y+c*opos.z)
	rospy.set_param('/plane_eq_params',[a,b,c,d])

def robot_interface_drawer():
	hand = rospy.get_param('/hand_mode')
	if hand=="right":
		arm = "Right"
	else:
		arm = "Left"
	hello_str = State(arm)
	rate = rospy.Rate(.1)
	while not rospy.is_shutdown():
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == "__main__":
	rospy.init_node('robot_interface_drawer')
	pub = rospy.Publisher('current_state',State,queue_size=10)
	s = rospy.Service('move_robot', MoveRobot, handle_move_robot)
	define_plane()
	robot_interface_drawer()



