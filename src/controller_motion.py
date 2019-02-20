#!/usr/bin/env python

import sys
import rospy
import numpy
from group7_proj2.srv import PlanRobot
from group7_proj2.msg import MP_Command

def plan_robot_client(act):
   rospy.wait_for_service('plan_robot')
   try:
      plan_robot = rospy.ServiceProxy('plan_robot', PlanRobot)
      resp1 = plan_robot(act)
      if resp1.Plan:
         return "Action successfully performed."
      else:
         return "Action not possible. State of system unchanged."
   except rospy.ServiceException, e:
      print "Service call failed: %s"%e

def doCommand(c):
    plan_commands = {'RRT','biRRT'}
    if c in plan_commands:
        print "%s"%(plan_robot_client(c))
    if c not in plan_commands:
        print "Please enter a valid command: RRT or biRRT"
   
def callback(data):
    doCommand(data.command)
    
def controller_motion():
	rospy.init_node('controller_motion',anonymous=True)
	rospy.Subscriber("mp_command",MP_Command,callback)
	rospy.spin()

if __name__ == "__main__":
	controller_motion()
