In your first terminal:

	cd ~/ros_ws
	catkin_make
	. ~/ros_ws/devel/setup.bash
	. baxter.sh (sim)

If you are in the simulator, run the line below and then open a new terminal

	roslaunch baxter_gazebo baxter_world.launch

Enable the robot (real or simulator):

	rosrun baxter_tools enable_robot.py -e
	

-------------------------------------------------------------------------

To launch the master node for the drawer:

	roslaunch group7_proj2 drawer.launch hand:=left
	
To send a command to drawer master node:

	rostopic pub /command group7_proj2/Command DrawLine 1 2 9 0 8 8 -1

-------------------------------------------------------------------------

OR

To launch the master node for the motion planner:

	roslaunch group7_proj2 motion.launch hand:=left
	
To send a command to motion planner master node:

	rostopic pub /mp_command group7_proj2/MP_Command biRRT -1
	
To run a separate collision checker window that continuously checks for collisions:

	roslaunch group7_proj2 collision.launch
	



