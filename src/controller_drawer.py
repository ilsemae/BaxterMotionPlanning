#!/usr/bin/env python

import sys
import rospy
import numpy
from group7_proj2.srv import *
from group7_proj2.msg import Command


def move_robot_client(act,nx,x,ny,y,cw,theta):

   rospy.wait_for_service('move_robot')
   try:
      move_robot = rospy.ServiceProxy('move_robot', MoveRobot)
      resp1 = move_robot(act,nx,x,ny,y,cw,theta)
      if resp1.Move:
         return "Action successfully performed."
      else:
         return "Action not possible. State of system unchanged."
   except rospy.ServiceException, e:
      print "Service call failed: %s"%e

def doCommand(c,nx,x,ny,y,cw,theta):
    move_commands = {'DrawLine','DrawSpline','Typewriter'}
    if c in move_commands:
        print "%s"%(move_robot_client(c,nx,x,ny,y,cw,theta)) # could instead call functions for each command in this file
    if c not in move_commands:
        print "Please enter a valid command: DrawLine, DrawSpline, or Typewriter"
   
def callback(data):
    doCommand(data.command,data.nx,data.x,data.ny,data.y,data.cw,data.theta)
    
def controller_drawer():
    rospy.init_node('controller_drawer',anonymous=True)
    rospy.Subscriber("command",Command,callback)
    rospy.spin()

if __name__ == "__main__":
   controller_drawer()
