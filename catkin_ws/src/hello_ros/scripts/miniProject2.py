#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
import math
import numpy as np
 
from std_msgs.msg import String
from gazebo_msgs import msg
from sensor_msgs.msg import JointState

"""
roslaunch jaco_on_table jaco_on_table_gazebo_controlled.launch load_grasp_fix:=true
roslaunch jaco_on_table_moveit jaco_on_table_moveit.launch
roslaunch jaco_on_table_moveit jaco_on_table_rviz.launch
rosrun hello_ros cube_spawn.py
"""

## Initializing the cube positions

objects_name = ['cube1', 'cube1', 'cube2', 'cube3', 'cube4', 'cube6', 'bucket']
cube_pose = []

currentJointState = JointState()

def jointStatesCallback(msg):
	global currentJointState
	currentJointState = msg

"""
		Obtaining the data from gazebo 
"""

def callback(data):
	global cube_pose
	#print(len(objects_name))

	for i in range(len(objects_name)):
		try:
			object_index = data.name.index(objects_name[i])
			cube_pose.append(data.pose[object_index])
		except:
			continue

"""
		Initialization function;	 
"""

def initialization():

	print "============ Starting tutorial "
  ## We can get the name of the reference frame for this robot
	#print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
	#print "============ End effector frame: %s" % group.get_end_effector_link()
	## We can get a list of all the groups in the robot
	#print "============ Robot Groups:"
	#print robot.get_group_names()
	## Sometimes for debugging it is useful to print the entire state of the robot.
	#print "============ Printing robot state"
	#print robot.get_current_state()
	print "============"

	## Let's setup the planner
	#group.set_planning_time(0.0)
	group.set_goal_orientation_tolerance(0.01)
	group.set_goal_tolerance(0.01)
	group.set_goal_joint_tolerance(0.01)
	group.set_num_planning_attempts(100)
	group.set_max_velocity_scaling_factor(1.0)
	group.set_max_acceleration_scaling_factor(1.0)

"""
		Gripper functions;
"""


## Function to close the hand (Taken from lecture_5_4_close_gripper)
def close_grapper():
	global currentJointState

	currentJointState = rospy.wait_for_message("/joint_states",JointState)
	tmp = 0.8
	currentJointState.header.stamp = rospy.get_rostime()
	currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])

	# Doing it multiple times, to make sure it works
	gripper_pub.publish(currentJointState)
	for i in range(4):
		gripper_pub.publish(currentJointState)
		rate.sleep()

	rospy.sleep(1.5)

## Function to open the hand (taken from lecture_5_4_open_gripper)
def open_grapper():
	global currentJointState
	currentJointState = rospy.wait_for_message("/joint_states",JointState)
	tmp = 0.005
	currentJointState.header.stamp = rospy.get_rostime()
	currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])

	# Making the gripp
	gripper_pub.publish(currentJointState)
	for i in range(4):
		gripper_pub.publish(currentJointState)
		rate.sleep()
	rospy.sleep(1.5)

"""
		RIVZ function
"""

## Define the objects in RIVZ 
## Inspiration from moveit_fake_objects.py from lecture 4

def RViz_obstacles():
	## RViz cubes
	p = geometry_msgs.msg.PoseStamped()
	p.header.frame_id = robot.get_planning_frame()
	#print "############## Printing length of cube_pose #############"
	#print(len(cube_pose))
	print(len(objects_name))
	for i in range(len(objects_name)):
		p.pose.position.x = cube_pose[i].position.x
		p.pose.position.y = cube_pose[i].position.y
		p.pose.position.z = cube_pose[i].position.z
		p.pose.orientation.x = cube_pose[i].orientation.x
		p.pose.orientation.y = cube_pose[i].orientation.y
		p.pose.orientation.z = cube_pose[i].orientation.z
		p.pose.orientation.w = cube_pose[i].orientation.w

		# Adding the sizes of the cubes
		if i != (len(objects_name)-1):
			scene.add_box('cube'+str(i), p, (0.05, 0.05, 0.05+0.02))
			print('adding cube;', i)
		# Adding the sizes of the bucket
		else:
			p.pose.position.z = cube_pose[i].position.z + 0.11
			scene.add_box('bucket', p, (0.20, 0.20, 0.20))
			print('Adding the bucket')


def resets_RVIZ():
	for i in objects_name:
		if i in scene.get_known_object_names():
			scene.remove_world_object(i)


"""
		Robot manipulation functions	 
"""

## Moving the robot (not cartesian)
## Inspired from lecture_5_2_pose_commands.py 
def moving_the_robot(cube_number, object_height):
	 #We add an object hight so that we are able to avoid hitting the cubes as well as the buckst. 
	# Planning to a Pose goal
	pose_goal = group.get_current_pose().pose

	pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, cube_pose[cube_number].orientation.w ))
	pose_goal.position.x = cube_pose[cube_number].position.x
	pose_goal.position.y = cube_pose[cube_number].position.y
	pose_goal.position.z = cube_pose[cube_number].position.z + 0.13 + object_height 
	
	group.set_pose_target(pose_goal)

	# Now, we call the planner to compute the plan
	plan = group.plan()
	rospy.sleep(.5)

	# If a plan is created
	if plan.joint_trajectory.joint_names != []:
		# Make RViz visualize a plan (aka the trajectory)
		print('============ Visualizing plan')
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = robot.get_current_state()
		display_trajectory.trajectory.append(plan)
		display_trajectory_publisher.publish(display_trajectory)
		print('============ Waiting while plan is visualized (again)...')
		rospy.sleep(.5)
        
		## Moving to a pose goal
		group.go(wait=True)
		rospy.sleep(.5)
		return True

	else:
		return False


## Taken from lecture_5_3_cartesian.py
def cartesian_maipulation_of_the_robot(cube_number, object_height):
	## Planning to a Pose goal
	## Going to the position of a cube. 
	print "============ Generating plans"

	pose_goal = group.get_current_pose().pose
	waypoints = []
    
	waypoints.append(pose_goal)
	pose_goal.position.x = cube_pose[cube_number].position.x
	pose_goal.position.y = cube_pose[cube_number].position.y
	pose_goal.position.z = cube_pose[cube_number].position.z + 0.13 + object_height
	print pose_goal
    
	#Create waypoints
	waypoints.append(pose_goal)
    
	#createcartesian  plan
	(plan1, fraction) = group.compute_cartesian_path(
																			waypoints,   # waypoints to follow
																			0.01,        # eef_step
																			0.0)         # jump_threshold
    
	print "============ Waiting while RVIZ displays Plan..."
	rospy.sleep(0.5)
    
    
	## You can ask RVIZ to visualize a plan (aka trajectory) for you.
	print "============ Visualizing Plan"
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan1)
	display_trajectory_publisher.publish(display_trajectory)
	print "============ Waiting while is visualized (again)..."
	rospy.sleep(1.)
    
	## Moving to a pose goal
	group.execute(plan1,wait=True)
	rospy.sleep(1.)

def collect_cubes_in_bucket():

	for i in reversed(range(len(objects_name)-1)):
		RViz_obstacles()

		# Planning to a Pose goal abit above the target
		plan_developed = moving_the_robot(i, 0.05 + 0.05)


		if plan_developed:
			# Going straight down to the target
			# plan_developed = moving_the_robot(i, 0.05)
			cartesian_maipulation_of_the_robot(i, 0.05)

			# Grapping the cube
			print "Closing the gripper"
			close_grapper()
			print "The gripper has closed"

			rospy.sleep(1)

			# Move to basket
			moving_the_robot(-1, 0.38)
			
			# Letting go of the cube
			print "Opening the gripper"
			open_grapper()
			print "The gripper has opened"

			# Set up Velocity JointState to 0
			currentJointState = rospy.wait_for_message("/joint_states",JointState)
			currentJointState.header.stamp = rospy.get_rostime()
			currentJointState.velocity = tuple(list(np.zeros(9)))
			gripper_pub.publish(currentJointState)

			# Upodate the position of the cubes
			rospy.sleep(1)
			resets_RVIZ()
			##RViz_obstacles()
			rospy.sleep(0.5)
		else:
			print('Cant access cube ' + str(i))


	rospy.sleep(1.)




if __name__=='__main__':

	## To be determined
	rospy.init_node('collect_cubes_in_bucket',anonymous=True)

	rate = rospy.Rate(10)

	moveit_commander.roscpp_initialize(sys.argv) 

  ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
	robot = moveit_commander.RobotCommander()
 
  ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
	scene = moveit_commander.PlanningSceneInterface()
 
  ## Instantiate a MoveGroupCommander object.  This object is an interface to one group of joints.  
  ## In this case the group is the joints in the left Arm.  
  ## This interface can be used to plan and execute motions on the left Arm.
	group = moveit_commander.MoveGroupCommander("Arm")

	rospy.Subscriber("gazebo/model_states", msg.ModelStates, callback, queue_size=5)

  ## trajectories for RVIZ to visualize.
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=5)

	gripper_pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=5)
	rospy.sleep(1)

	##RViz_obstacles()
	
	try:
		initialization()

		start_pose = currentJointState.position

		collect_cubes_in_bucket()

	except rospy.ROSInterruptException:
		pass
