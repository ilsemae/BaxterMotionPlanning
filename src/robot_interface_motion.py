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

def AreWeThereYetAngles(limb,goal,thresh):
    hand = get_Hand()
    limb_joints = limb.joint_angles()
    current_angles = [limb_joints[hand+'_s0'],limb_joints[hand+'_s1'],limb_joints[hand+'_e0'],limb_joints[hand+'_e1'],limb_joints[hand+'_w0'],limb_joints[hand+'_w1'],limb_joints[hand+'_w2']]
    diff = numpy.linalg.norm(current_angles-goal)
    if diff>thresh:
        return False
    else:
        return True

def draw_random_sample(end,sample_it):
	# true robot joint limits
	#angle_ranges = {'S0':[-2.461,0.890],
	#			    'S1':[-2.147,1.047],
	#			    'E0':[-3.028,3.028],
	#			    'E1':[-0.052,2.618],
	#			    'W0':[-3.059,3.059],
	#			    'W1':[-1.571,2.094],
	#			    'W2':[-3.059,3.059]}
	angle_ranges = {'S0':[-0.961,1.590],
					'S1':[-0.847,0.847],
					'E0':[-2.828,2.828],
					'E1':[ 0.748,2.218],
					'W0':[-2.859,2.859],
					'W1':[-1.371,1.894],
					'W2':[-2.859,2.859]}
	arm = get_Hand()
	if arm == "left":
		angle_ranges['S0'] = [-1.561,0.490]
	goal_or_rand = numpy.random.randint(1,6)
	isend = False
	if goal_or_rand == 3 and sample_it:
		sample = end
		isend = True
	else:
		sample = numpy.array([numpy.random.uniform(angle_ranges['S0'][0],angle_ranges['S0'][1],1),
								numpy.random.uniform(angle_ranges['S1'][0],angle_ranges['S1'][1],1),
								numpy.random.uniform(angle_ranges['E0'][0],angle_ranges['E0'][1],1),
								numpy.random.uniform(angle_ranges['E1'][0],angle_ranges['E1'][1],1),
								numpy.random.uniform(angle_ranges['W0'][0],angle_ranges['W0'][1],1),
								numpy.random.uniform(angle_ranges['W1'][0],angle_ranges['W1'][1],1),
								numpy.random.uniform(angle_ranges['W2'][0],angle_ranges['W2'][1],1)])
	sample = numpy.reshape(sample,(1,7))[0]
	return (sample,isend)

def find_1nn(sample,RRT_nodes):
	nodes_list = []
	nodes_indxs = []
	for key in RRT_nodes:
		nodes_list.append(RRT_nodes[key])
		nodes_indxs.append(key)
	if len(nodes_list) == 1:
		nearest_neighbor = nodes_list[0]
		knn_index = nodes_indxs[0]
	else:
		tree = spatial.KDTree(nodes_list)
		knn_index = tree.query(sample,1,0,2)[1] # args: 1=single closest neighbor, 0, p=2
		nearest_neighbor = nodes_list[knn_index]
	return (nearest_neighbor,knn_index)
    
def find_1nn_old(sample,RRT_nodes):
    nodes_list = []
    nodes_indxs = []
    for key in RRT_nodes:
        if key != 9999999999:
            nodes_list.append(RRT_nodes[key])
    tree = spatial.KDTree(nodes_list)
    knn_index = tree.query(sample,1,0,2)[1] # args = single closest neighbor, 0, p=2
    nearest_neighbor = nodes_list[knn_index]
    return (nearest_neighbor,knn_index)

def get_RRT():
	hand = get_Hand()
	limb=baxter_interface.Limb(hand)
	rospy.sleep(0.1)
	raw_input("Place the arm at the desired end position. Then press 'Enter'.")	
	limb_joints=limb.joint_angles()
	end = [limb_joints[hand+'_s0'],limb_joints[hand+'_s1'],limb_joints[hand+'_e0'],limb_joints[hand+'_e1'],limb_joints[hand+'_w0'],limb_joints[hand+'_w1'],limb_joints[hand+'_w2']]
	res = s_cc(String(hand),end)
	print("end has a collision:")
	raw_input(res.collision)
	raw_input("Place the arm at the desired start position. When you press 'Enter' this time, the arm will move back to the previous postion while avoiding obstacles.")
	limb_joints=limb.joint_angles()
	start = [limb_joints[hand+'_s0'],limb_joints[hand+'_s1'],limb_joints[hand+'_e0'],limb_joints[hand+'_e1'],limb_joints[hand+'_w0'],limb_joints[hand+'_w1'],limb_joints[hand+'_w2']]
	res = s_cc(String(hand),start)
	print("start has a collision:")
	raw_input(res.collision)
	tree_has_collided = False
	stepsize = 0.25
	RRT_nodes = {0:numpy.array(start),9999999999:numpy.array(end)}
	RRT_node_parents = {0:[],9999999999:[]}
	nodes_in_tree = 1
	have_complete_path = False
	current_node = 0
	current_node_angles = numpy.array(start)
	current_goal = 9999999999
	current_goal_angles = numpy.array(end)
	sampled_end_goal = True
	building_from_root = True # other than the very first round of node adding, this will be False
	while not have_complete_path:
		while not tree_has_collided:
			# check to see if we are within the stepsize of the goal
			#print("current goal:")
			#print(current_goal_angles)
			raw_direction = current_goal_angles-current_node_angles
			#print("Let's check to see if we are close to our goal...")
			if numpy.linalg.norm(raw_direction) < stepsize :
				if building_from_root:
					RRT_node_parents[current_goal] = RRT_node_parents[current_goal]+[current_node]
					have_complete_path = True
					tree_has_collided = True
					#print("We've reached our final goal!")
				else:
					RRT_node_parents[current_node] = RRT_node_parents[current_node]+[current_goal]
				if sampled_end_goal:
					have_complete_path = True
					tree_has_collided = True
					#print("We've reached our final goal!")
				else:
					#print("We've reached the main tree! Let's draw a new sample.")
					tree_has_collided = True
			else:
				print("We're not near our goal. Let's step toward it!")
				new_node_angles = stepsize*raw_direction/numpy.linalg.norm(raw_direction) + numpy.array(current_node_angles)
				# if statement here only needed while collision checker is not ready
				res = s_cc(String(hand),new_node_angles)
				no_collisions = not res.collision
				if no_collisions:
					no_path_collisions = True
					path_length = numpy.linalg.norm(new_node_angles - current_node_angles)
					increment = 0.01*(new_node_angles - current_node_angles)/path_length
					check_point = current_node_angles
					i=0
					while i in range(0,int(numpy.floor(path_length/0.01))) and no_path_collisions:
						check_point = check_point + increment
						res = s_cc(String(hand),check_point)
						no_path_collisions = not res.collision
						i = i+1
					if no_path_collisions:
						RRT_nodes.update({nodes_in_tree:new_node_angles})
						if building_from_root:
							RRT_node_parents.update({nodes_in_tree:[current_node]})
						else:
							RRT_node_parents.update({nodes_in_tree:[]})
							RRT_node_parents[current_node] = RRT_node_parents[current_node]+[nodes_in_tree]
						current_node = nodes_in_tree
						nodes_in_tree = nodes_in_tree + 1
						current_node_angles = new_node_angles
						print('added node!:')
						#print(current_node)
						#print(current_node_angles)
						#print("node_parents:")
						#print(RRT_node_parents)
					else:
						#print('New path to new point causes a collision. Tree growing stopped. A random sample will be drawn now.')
						tree_has_collided = True
				else:
					#print('New point collides with obstacle. Tree growing stopped. A random sample will be drawn now.')
					tree_has_collided = True
		building_from_root = False	
		if have_complete_path == False:
			no_collisions = False
			no_path_collisions = False
			while not no_collisions or not no_path_collisions:
				tree_has_collided = False
				(sample,isend) = draw_random_sample(end,True)
				res = s_cc(String(hand),sample)
				no_collisions = not res.collision
				if no_collisions or isend:
					print("finding its nearest neighbor on the tree...")
					(nearest_neighbor,nn_index) = find_1nn_old(sample,RRT_nodes)
					#print("nearest neighbor:")
					#print(nearest_neighbor)
					#print("nearest neighbor index:")
					#print(nn_index)
					no_path_collisions = True
					path_length = numpy.linalg.norm(nearest_neighbor - sample)
					increment = 0.01*(nearest_neighbor - sample)/path_length
					check_point = sample
					i=0
					while i in range(0,int(numpy.floor(path_length/0.01))) and no_path_collisions:
						check_point = check_point + increment
						res = s_cc(String(hand),check_point)
						no_path_collisions = not res.collision
						i = i+1
					if no_path_collisions:
						current_goal_angles = nearest_neighbor
						current_goal = nn_index
						current_node_angles = sample
						if isend:
							current_node = 9999999999
							raw_input("You sampled the end goal!")
							sampled_end_goal = True
						else:
							sampled_end_goal = False
							RRT_nodes.update({nodes_in_tree:sample})
							RRT_node_parents.update({nodes_in_tree:[]})
							current_node = nodes_in_tree
							nodes_in_tree = nodes_in_tree + 1
							print('added node!:')
							#print(current_node)
							#print(current_node_angles)
							#print("node_parents:")
							#print(RRT_node_parents)
					#else:
						#print("there's a path collision. press enter")
				#else:
					#print("there's a point collision. press enter")
	return (RRT_nodes,RRT_node_parents)

def get_biRRT():
	hand = get_Hand()
	limb=baxter_interface.Limb(hand)
	rospy.sleep(0.1)
	raw_input("Place the arm at the desired end position. Then press 'Enter'.")	
	limb_joints=limb.joint_angles()
	end = [limb_joints[hand+'_s0'],limb_joints[hand+'_s1'],limb_joints[hand+'_e0'],limb_joints[hand+'_e1'],limb_joints[hand+'_w0'],limb_joints[hand+'_w1'],limb_joints[hand+'_w2']]
	res = s_cc(String(hand),end)
	print("end has a collision:")
	raw_input(res.collision)
	raw_input("Place the arm at the desired start position. When you press 'Enter' this time, the arm will move back to the previous postion while avoiding obstacles.")
	limb_joints=limb.joint_angles()
	start = [limb_joints[hand+'_s0'],limb_joints[hand+'_s1'],limb_joints[hand+'_e0'],limb_joints[hand+'_e1'],limb_joints[hand+'_w0'],limb_joints[hand+'_w1'],limb_joints[hand+'_w2']]
	res = s_cc(String(hand),start)
	print("start has a collision:")
	raw_input(res.collision)
	no_path_collisions_other_tree = False
	tree_has_collided = False
	stepsize = 0.2
	tree_switch = 1 # 1 means start tree, -1 means goal tree
	RRT_start_nodes = {0:numpy.array(start)}
	RRT_goal_nodes  = {9999999999:numpy.array(end)}
	RRT_node_parents = {0:[],9999999999:[]}
	nodes_in_tree = 1
	have_complete_path = False
	current_node = 0
	current_node_angles = numpy.array(start)
	current_goal = 9999999999
	current_goal_angles = numpy.array(end)
	built_goal_tree = False
	begun_sampling = False
	building_from_root = True
	parent_to_child = True
	last_round = False
	while not have_complete_path:
		tree_has_collided = False
		while not tree_has_collided:
			# check to see if we are within the stepsize of the goal
			# print("current goal:"+str(current_goal))
			raw_direction = numpy.array(current_goal_angles)-numpy.array(current_node_angles)
			# print("Let's check to see if we are close to our goal...")
			if numpy.linalg.norm(raw_direction) < stepsize :
				if building_from_root:
					RRT_node_parents[current_goal] = [current_node]
					have_complete_path = True
					tree_has_collided = True
				else:
					if not parent_to_child:
						RRT_node_parents[current_node] = [current_goal]
					else:
						RRT_node_parents[current_goal] = [current_node]
					if last_round:
						have_complete_path = True
						tree_has_collided = True
					elif not begun_sampling:
						have_complete_path = True
						tree_has_collided = True
					elif not no_path_collisions_other_tree:
						tree_has_collided = True
						tree_switch = -1 * tree_switch
						#raw_input(len(RRT_goal_nodes))
						if len(RRT_goal_nodes) == 1:
							#current_goal_angles = start
							#current_goal = 0
							#current_node_angles = end
							#current_node = 9999999999
							tree_has_collided = True
					else:
						current_goal = nn_index_o
						current_goal_angles = nearest_neighbor_o
						current_node = sample_index
						current_node_angles = sample
						# print(current_node)
						# print(current_goal)
						# raw_input("Our sample can reach both trees! Let's connect them.")
						no_path_collisions_other_tree = False
						last_round = True
				# print("node_parents:")
				# print(RRT_node_parents)
				# print("start tree:")
				# print(RRT_start_nodes.keys())
				# print("goal tree:")
				# print(RRT_goal_nodes.keys())
				# raw_input("We've reached some goal!")
			else:
				# print("We're not near our goal. Let's step toward it!")
				new_node_angles = stepsize*raw_direction/numpy.linalg.norm(raw_direction) + numpy.array(current_node_angles)
				res = s_cc(String(hand),new_node_angles)
				no_collisions = not res.collision
				if no_collisions:
					no_path_collisions = True
					path_length = numpy.linalg.norm(new_node_angles - current_node_angles)
					increment = 0.01*(new_node_angles - current_node_angles)/path_length
					check_point = current_node_angles
					i=0
					while i in range(0,int(numpy.floor(path_length/0.01))) and no_path_collisions:
						check_point = check_point + increment
						res = s_cc(String(hand),check_point)
						no_path_collisions = not res.collision
						i = i+1
					if no_path_collisions:
						RRT_node_parents.update({nodes_in_tree:[]})
						if last_round:
							if tree_switch == 1:
								parent_to_child = True
								RRT_start_nodes.update({nodes_in_tree:new_node_angles})
							else:
								parent_to_child = False
								RRT_goal_nodes.update({nodes_in_tree:new_node_angles})
						else:
							if tree_switch == 1:
								print("updating start tree")
								parent_to_child = False
								RRT_start_nodes.update({nodes_in_tree:new_node_angles})
							else:
								print("updating goal tree")
								parent_to_child = True
								RRT_goal_nodes.update({nodes_in_tree:new_node_angles})
						
						if building_from_root or parent_to_child:
							RRT_node_parents[nodes_in_tree] = [current_node]
						else:						
							RRT_node_parents[current_node] = [nodes_in_tree]

						current_node = nodes_in_tree
						nodes_in_tree = nodes_in_tree + 1
						current_node_angles = new_node_angles
						#print('added node!:')
						#print(current_node)
						#print(current_node_angles)
						#print("node_parents:")
						#print(RRT_node_parents)
						#print("start tree:")
						#print(RRT_start_nodes.keys())
						#print("goal tree:")
						#raw_input(RRT_goal_nodes.keys())
					else:
						tree_has_collided = True
						#print('New path collides with obstacle. Tree growing stopped.')
				else:
					#print('New point collides with obstacle. Tree growing stopped.')
					tree_has_collided = True
		building_from_root = False
		if not have_complete_path:
			begun_sampling = True
			tree_has_collided = False
			no_collisions = False
			no_path_collisions = True
			#print("drawing a random sample...")
			while not (no_collisions and no_path_collisions):
				(sample,isend) = draw_random_sample(end,False)
				res = s_cc(String(hand),sample)
				no_collisions = not res.collision
				if tree_switch == 1:
					#raw_input("finding its nearest neighbor on the START tree...")
					(nearest_neighbor,nn_index) = find_1nn(sample,RRT_start_nodes)
					(nearest_neighbor_o,nn_index_o) = find_1nn(sample,RRT_goal_nodes)
				else:
					#raw_input("finding its nearest neighbor on the GOAL tree...")
					(nearest_neighbor,nn_index) = find_1nn(sample,RRT_goal_nodes)
					(nearest_neighbor_o,nn_index_o) = find_1nn(sample,RRT_start_nodes)
				i=0
				path_length = numpy.linalg.norm(nearest_neighbor - sample)
				increment = 0.01*(nearest_neighbor - sample)/path_length
				check_point = sample
				no_path_collisions = True
				while i in range(0,int(numpy.floor(path_length/0.01))) and no_path_collisions:
					check_point = check_point + increment
					res = s_cc(String(hand),check_point)
					no_path_collisions = not res.collision
					i = i+1
					#print(i)
			#print("nearest neighbor:")
			#print(nearest_neighbor)
			#print("nearest neighbor index: "+str(nn_index))
			#print("nearest neighbor other tree index: "+str(nn_index_o))
			no_path_collisions_other_tree = True
			path_length = numpy.linalg.norm(nearest_neighbor_o - sample)
			increment = 0.01*(nearest_neighbor_o - sample)/path_length
			check_point = sample
			i=0
			while i in range(0,int(numpy.floor(path_length/0.01))) and no_path_collisions_other_tree:
				check_point = check_point + increment
				res = s_cc(String(hand),check_point)
				no_path_collisions_other_tree = not res.collision
				i = i+1
			current_goal_angles = nearest_neighbor
			current_goal = nn_index
			current_node_angles = sample
			if tree_switch == 1:
				RRT_start_nodes.update({nodes_in_tree:sample})
			else:
				RRT_goal_nodes.update({nodes_in_tree:sample})
			RRT_node_parents.update({nodes_in_tree:[]})
			current_node = nodes_in_tree
			sample_index = current_node
			nodes_in_tree = nodes_in_tree + 1
			#print('added node!:')
			#print(current_node)
			#print(current_node_angles)
			#print("node_parents:")
			#print(RRT_node_parents)
			#print("start tree:")
			#print(RRT_start_nodes.keys())
			#print("goal tree:")
			#raw_input(RRT_goal_nodes.keys())
	RRT_start_nodes.update(RRT_goal_nodes)
	return (RRT_start_nodes,RRT_node_parents)
	
def Smoother(path):
    hand = get_Hand()
    w = 0  # used to check how many iterations we've had without a tree update
    keep_smoothing = len(path)>3 # no need to smooth if path has less than 4 points.
    while keep_smoothing:
        RRT_segments = {0:[0,1]}
        current_segment = 0
        i=0
        for i in range(0,len(path)-2):
            if numpy.linalg.norm((path[i+2]-path[i+1])/numpy.linalg.norm(path[i+2]-path[i+1])-(path[i+1]-path[i])/numpy.linalg.norm(path[i+1]-path[i])) < .001:
                RRT_segments[current_segment].append(i+2)
            else:
                current_segment = current_segment+1
                RRT_segments.update({current_segment:[i+2]})
        #print("still smoothing...")
        print(RRT_segments)
        print
        # pick two different segments
        keep_going = True
        q = 0
        while keep_going:
            s1 = numpy.random.randint(0,len(RRT_segments.keys()))
            s2 = numpy.random.randint(0,len(RRT_segments.keys()))
            while s1 == s2 or len(RRT_segments[s1])==1 or len(RRT_segments[s2])==1:
                s1 = numpy.random.randint(0,len(RRT_segments.keys()))
                s2 = numpy.random.randint(0,len(RRT_segments.keys()))
            print(str(s1)+"<-s1  s2->"+str(s2))
            p1 = numpy.random.randint(0,len(RRT_segments[s1]))
            p2 = numpy.random.randint(0,len(RRT_segments[s2]))
            h = 0
            while (p1 == p2 or p1 in RRT_segments[s2] or p2 in RRT_segments[s1] or abs(p1-p2)==1) and h<10:
                p1 = numpy.random.randint(0,len(RRT_segments[s1]))
                p2 = numpy.random.randint(0,len(RRT_segments[s2]))
                print(str(p1)+"<-p1  p2->"+str(p2))
                print("h: "+str(h))
                h = h+1
            q = q + 1
            if h==10 or q==10:
                keep_going = False
        no_collision = True
        path_length = numpy.linalg.norm(path[RRT_segments[s1][p1]] - path[RRT_segments[s2][p2]])
        increment = 0.01*(path[RRT_segments[s2][p2]] - path[RRT_segments[s1][p1]])/path_length
        check_point = path[RRT_segments[s1][p1]]
        i=0
        while i in range(0,int(numpy.floor(path_length/0.01))) and no_collision:
            check_point = check_point + increment
            res = s_cc(String(hand),check_point)
            no_collision = not res.collision
            i = i+1
        if no_collision:
            if RRT_segments[s1][p1] > RRT_segments[s2][p2]:
                b = RRT_segments[s1][p1]
                a = RRT_segments[s2][p2]
                newpath = []
                i = 0
                #add points up until the first checkpoint
                for i in range(0,s2+1):
                    j = 0
                    if i==s2:
                        for j in range(0,p2+1):
                            newpath.append(path[RRT_segments[i][j]])
                    else:
                        for j in range(0,len(RRT_segments[i])):
                            newpath.append(path[RRT_segments[i][j]])
                #create new points between these
                #joining_vector = path[RRT_segments[s1][p1]] - path[RRT_segments[s2][p2]]
                #path_length = numpy.linalg.norm(joining_vector)
                #increment = 0.01*joining_vector/path_length
                #check_point = path[RRT_segments[s2][p2]]
                #i=0
                #while i in range(0,int(numpy.floor(path_length/0.01))):
                #    check_point = check_point + increment
                #    newpath.append(check_point)
                #    i = i+1
                #raw_input(i)
                #add points after the checkpoint
                i = 0
                for i in range(s1,len(RRT_segments.keys())):
                    j = 0
                    if i==s1:
                        for j in range(p1,len(RRT_segments[i])):
                            newpath.append(path[RRT_segments[i][j]])
                    else:
                        for j in range(0,len(RRT_segments[i])):
                            newpath.append(path[RRT_segments[i][j]])
            else:
                a = RRT_segments[s1][p1]
                b = RRT_segments[s2][p2]
                newpath = []
                i = 0
                for i in range(0,s1+1):
                    j = 0
                    if i==s1:
                        for j in range(0,p1+1):
                            newpath.append(path[RRT_segments[i][j]])
                    else:
                        for j in range(0,len(RRT_segments[i])):
                            newpath.append(path[RRT_segments[i][j]])
                #create new points between these
                #joining_vector = path[RRT_segments[s2][p2]] - path[RRT_segments[s1][p1]]
                #path_length = numpy.linalg.norm(joining_vector)
                #increment = 0.01*joining_vector/path_length
                #check_point = path[RRT_segments[s1][p1]]
                #i=0
                #while i in range(0,int(numpy.floor(path_length/0.01))):
                #    check_point = check_point + increment
                #    newpath.append(check_point)
                #    i = i+1
                #raw_input(i)
                i = 0
                for i in range(s2,len(RRT_segments.keys())):
                    j = 0
                    if i==s2:
                        for j in range(p2,len(RRT_segments[i])):
                            newpath.append(path[RRT_segments[i][j]])
                    else:
                        for j in range(0,len(RRT_segments[i])):
                            newpath.append(path[RRT_segments[i][j]])
            path = newpath
            print("there is NOT a collision between points "+str(a)+" and "+str(b)+".")
            w = 0
        else: 
            #print("there is a collision between points "+str(a)+" and "+str(b)+".")
            w = w+1
        if w > 100 or len(path)<4:
            keep_smoothing = False
        print("w= "+str(w))
    print(path)
    return path
	
def biRRT():
	(tree_nodes,tree_node_parents) = get_biRRT()
	return go_RRT(tree_nodes,tree_node_parents)

def RRT():
	(tree_nodes,tree_node_parents) = get_RRT()
	return go_RRT(tree_nodes,tree_node_parents)

def execute_path(path):
    for i in range(1,len(path)):  		# can skip 0th element since the arm is already there
        hand = get_Hand()
        limb=baxter_interface.Limb(hand)
        rospy.sleep(0.1)
        limb_joints=limb.joint_angles()
        current_angles = [limb_joints[hand+'_s0'],limb_joints[hand+'_s1'],limb_joints[hand+'_e0'],limb_joints[hand+'_e1'],limb_joints[hand+'_w0'],limb_joints[hand+'_w1'],limb_joints[hand+'_w2']]
        speed = 0.2
        goal = path[i]
        desired_joint_velocities = speed * numpy.array(goal - current_angles)/numpy.linalg.norm(numpy.array(goal - current_angles))
        j_vels=limb.joint_velocities()
        j_vels[hand+'_s0']=desired_joint_velocities[0]
        j_vels[hand+'_s1']=desired_joint_velocities[1]
        j_vels[hand+'_e0']=desired_joint_velocities[2]
        j_vels[hand+'_e1']=desired_joint_velocities[3]
        j_vels[hand+'_w0']=desired_joint_velocities[4]
        j_vels[hand+'_w1']=desired_joint_velocities[5]
        j_vels[hand+'_w2']=desired_joint_velocities[6]
        frq=0.5
        eps=0.15 	#threshold for how close is good enough to goal
        while not AreWeThereYetAngles(limb,goal,eps):
            limb.set_command_timeout(frq)
            limb.set_joint_velocities(j_vels)
            rospy.sleep(frq)
            limb_joints = limb.joint_angles()
            current_angles = [limb_joints[hand+'_s0'],limb_joints[hand+'_s1'],limb_joints[hand+'_e0'],limb_joints[hand+'_e1'],limb_joints[hand+'_w0'],limb_joints[hand+'_w1'],limb_joints[hand+'_w2']]
            print(limb.joint_velocities()) # if this is zero or something weird, or very diff from j_vels, quit and return False
        if i == len(path)-1:        
            limb.exit_control_mode()
            print("We're there!!!!!!!!!!!!!")
            return True
    return False

def go_RRT(tree_nodes,tree_node_parents):
    wedidit = False
    print("Our tree:")
    print(tree_nodes)
    print("The tree's parents:")
    print(tree_node_parents)
    current_node = 9999999999
    path = numpy.array([tree_nodes[current_node]])
    while True:
        if tree_node_parents[current_node]:
            parent_node = tree_node_parents[current_node][0]
            path = numpy.concatenate(([tree_nodes[parent_node]],path),axis=0)
            current_node = parent_node
        else:
            break
    print("final path:")
    raw_input(path)
    wedidit = execute_path(path)
    hand = get_Hand()
    limb=baxter_interface.Limb(hand)
    limb_joints=limb.joint_angles()
    limb_joints[hand+'_s0'] = path[0][0]
    limb_joints[hand+'_s1'] = path[0][1]
    limb_joints[hand+'_e0'] = path[0][2]
    limb_joints[hand+'_e1'] = path[0][3]
    limb_joints[hand+'_w0'] = path[0][4]
    limb_joints[hand+'_w1'] = path[0][5]
    limb_joints[hand+'_w2'] = path[0][6]
    raw_input("Press enter to return to start position and run the smoothed version of this path.")
    limb.move_to_joint_positions(limb_joints)
    path = Smoother(path)
    print("smoothed path:")
    raw_input(path)
    wedidit_s = execute_path(path)
    wedidit = wedidit or wedidit_s
    return wedidit

def handle_plan_robot(req):
    if req.Action=="RRT":
        isPossible = RRT()
    elif req.Action=="biRRT":
        isPossible = biRRT()
    return isPossible

def robot_interface_motion():
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
	rospy.init_node('robot_interface_motion')
	pub = rospy.Publisher('current_state',State,queue_size=10)
	s = rospy.Service('plan_robot', PlanRobot, handle_plan_robot)
	s_cc = rospy.ServiceProxy('/check_collision', CheckCollision)
	rospy.wait_for_service('/check_collision')
	robot_interface_motion()



