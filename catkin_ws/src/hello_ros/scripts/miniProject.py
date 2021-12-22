#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import os
import rospy
import random
import tf_conversions
import moveit_commander
import moveit_msgs.msg
from gazebo_msgs.srv import GetModelState,GetWorldProperties
from gazebo_msgs.srv import DeleteModel, SpawnModel
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
import shape_msgs.msg as shape_msgs
import math
 
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

"""
roslaunch jaco_on_table jaco_on_table_gazebo_controlled.launch load_grasp_fix:=true
roslaunch jaco_on_table_moveit jaco_on_table_moveit.launch
roslaunch jaco_on_table_moveit jaco_on_table_rviz.launch
rosrun hello_ros cube_spawn.py
"""

class Robot:
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("Arm")
		self.gripper_pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
		self.display_trajectory_publisher = rospy.Publisher(
											'/move_group/display_planned_path',
											moveit_msgs.msg.DisplayTrajectory, queue_size=10)
		self.rate = rospy.Rate(10)


	def recognize_enviroment(self):
		'''Useful commands:
		--> to get all models in the enciroment
		rosservice call /gazebo/get_world_properties "{}"
		--> to get the model properties (link, joint...)
		rosservice call /gazebo/get_model_properties jaco_on_table #to get the model properties
		--> to get the model states (position and orientation) of a specific link
		rosservice call /gazebo/get_model_state "model_name: 'ground_plane',relative_entity_name: 'link'" 
		'''
		# get the pose of all the models
		self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
		# get the name of the models
		self.world_state = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
		self.object_in_world = self.world_state().model_names
		
		# look for cubes
		self.cubes_coordinates = []
		i = 1
		for el in self.object_in_world:
			if 'cube' in el:
				self.cubes_coordinates.append(self.model_coordinates("cube"+str(i),'link'))
				i = i+1
			elif 'bucket' in el:
				self.bucket_coordinates = self.model_coordinates("bucket",'link')
		
		# add cubes to the scene
		for i in range(len(self.cubes_coordinates)):
			p = geometry_msgs.msg.PoseStamped()
			p.header.frame_id = self.robot.get_planning_frame()
			p.pose.position = self.cubes_coordinates[i].pose.position
			p.pose.orientation = self.cubes_coordinates[i].pose.orientation
			self.scene.add_box("cube"+str(i+1), p, (0.05, 0.05, 0.05))
			# print("Cube%2d added to the scene" %(i))
		print('============ Cubes added as obstacles')
		# add bucket as obstacle
		p = geometry_msgs.msg.PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = self.bucket_coordinates.pose.position.x
		p.pose.position.y = self.bucket_coordinates.pose.position.y
		p.pose.position.z = self.bucket_coordinates.pose.position.z + 0.1
		p.pose.orientation = self.bucket_coordinates.pose.orientation
		self.scene.add_box('bucket', p, (0.2, 0.2, 0.2))
		print("============ Bucket added as scene")
		

	def move(self,x,y,z):
		# name of the reference frame for this robot:
		self.group.get_planning_frame()
		# end-effector link for this group:
		self.group.get_end_effector_link()
		# a list of all the groups in the robot:
		self.robot.get_group_names()
		# current entire state
		self.robot.get_current_state()

		# self.group.set_planning_time(0.0)
		self.group.set_goal_orientation_tolerance(0.01)
		self.group.set_goal_tolerance(0.01)
		self.group.set_goal_joint_tolerance(0.01)
		self.group.set_num_planning_attempts(100)

		# self.group.set_max_velocity_scaling_factor(1.0)
		# self.group.set_max_acceleration_scaling_factor(1.0)

		pose_goal = self.group.get_current_pose().pose
		waypoints = []
		waypoints.append(copy.deepcopy(pose_goal))
		
		pose_goal.position.x = x
		pose_goal.position.y = y
		pose_goal.position.z = z
		# pose_goal.orientation = default_orientation
		# pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(1.820457, -1.524760, 0.429910))
		pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(1.224876, -1.525105, 0.433846))
		
		# pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
		#Create waypoints
		waypoints.append(copy.deepcopy(pose_goal))

		#createcartesian  plan
		(plan1, fraction) = self.group.compute_cartesian_path(
											waypoints,   # waypoints to follow
											0.01,        # eef_step
											0.0)         # jump_threshold
		# while (fraction != 1.0):
		# 	(plan1, fraction) = self.group.compute_cartesian_path(
		# 										waypoints,   # waypoints to follow
		# 										0.01,        # eef_step
		# 										0.0)         # jump_threshold
		rospy.sleep(.5)

		## Moving to a pose goal
		print "============ Executing"
		print("fraction: ", fraction)
		self.group.execute(plan1,wait=True)
		rospy.sleep(1)
	
	def gripper_controller(self,command):
		currentJointState = JointState()
		currentJointState = rospy.wait_for_message("/joint_states",JointState)
		currentJointState.header.stamp = rospy.get_rostime()
		if (command == 'open'):
			tmp = 0.005
		else:
			tmp = 0.7
		currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
		currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		for i in range(3):
			self.gripper_pub.publish(currentJointState)
			self.rate.sleep()
		rospy.sleep(1.)
		
	def collect(self):
		offset_z = 0.35
		# offset_z_buck = 0.3
		# offset_grab = 0.92
		offset_grab = 0.935
		for cube in self.cubes_coordinates:
			position = cube.pose.position
			self.move(position.x,position.y,position.z + offset_z)
			self.move(position.x,position.y,offset_grab)
			self.gripper_controller('close')
			self.move(position.x,position.y,position.z + offset_z)
			self.move(self.bucket_coordinates.pose.position.x+0.03,
						self.bucket_coordinates.pose.position.y-0.06,
						self.bucket_coordinates.pose.position.z+offset_z)
			self.gripper_controller('open')
	
	def taskCompleted(self):
		# print('============ Number of attempts: ' + str(self.attempts))
		# bucket_p = self.bucket_coordinates.pose.position
		bucket_x = 0.517079
		bucket_y = -0.208795
		delta = 0.02
		for cube in self.cubes_coordinates:
			cube_p = cube.pose.position
			if (cube_p.x >= bucket_x - delta and cube_p.x <= bucket_x + delta
				and cube_p.y >= bucket_y - delta and cube_p.y <= bucket_y + delta):
				print ("============ Cube already in bucket -> remove from goals")
				self.cubes_coordinates.remove(cube)
		
		print("============ Number of cubes to collect: " + str(len(self.cubes_coordinates)))
		if len(self.cubes_coordinates) > 0:
			return False
		else:
			print('============ Task completed')
			return True

	def save_position(self):
		robot.move(0.5,0.5,1.1)
		print('============ Save position reached')

class Object_spawner():
	def __init__(self):
		#rospy.wait_for_service("/jaco/joint_control")
		rospy.wait_for_service("gazebo/delete_model")
		rospy.wait_for_service("gazebo/spawn_sdf_model")
		print('============ Object spawner: services registered')
		self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
		self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
		self.orient = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., 0.0, 0.785398))

	def cleanEnviroment(self):
		self.world_state = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
		self.models_in_world = self.world_state().model_names
		self.prev_num_of_cubes = len(self.models_in_world)-2
		for i in range(self.prev_num_of_cubes):
			self.delete_model("cube"+str(i))
		if 'bucket' in self.models_in_world :
			self.delete_model("bucket")
		print('============ Enviroment cleaned')

	def spawnObjects(self):
		dirname = os.path.dirname(__file__)
		cubefile = os.path.join(dirname, '../urdf/cube.urdf')
		with open(cubefile, "r") as f:
			product_xml = f.read()
		self.num_of_cubes = random.randint(1,3)
		for num in xrange(0,self.num_of_cubes):
			bin_x   =   random.uniform(0.1,0.5)
			bin_y   =   random.uniform(0.1,0.5)
			item_name   =   "cube{}".format(num)
			print("============ Spawning model: "+ str(item_name))
			item_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=1),   self.orient)
			self.spawn_model(item_name, product_xml, "", item_pose, "world")

		dirname = os.path.dirname(__file__)
		bucketfile = os.path.join(dirname, '../urdf/bucket.urdf')
		with open(bucketfile, "r") as f:
			product_xml = f.read()
		item_pose   =   Pose(Point(x=0.53, y=-0.23,    z=0.78),   self.orient)
		print("============ Spawning model: bucket")
		self.spawn_model("bucket", product_xml, "", item_pose, "world")



if __name__ == '__main__':
	rospy.init_node('collect_cubes_in_bucket')

	#initialization of the enviroment
	object_spawner = Object_spawner()
	object_spawner.cleanEnviroment()
	object_spawner.spawnObjects()

	robot = Robot()
	robot.recognize_enviroment()
	robot.save_position()
	while not(robot.taskCompleted()):
		robot.collect()
		robot.recognize_enviroment()
	print("============ All cubes have been collected ")
	



def cubeInsideSquare(self,x,y):
	print"here"
	x_lim = [0.138,0.5,0.29,-0.15]
	y_lim = [-0.06,0.31,0.54,0.22]
	coeff = []
	for i in range(3):
		m = (y_lim[i] - y_lim[i+1]) / (x_lim[i] + x_lim[i+1])
		q = (x_lim[i]*y_lim[i+1] - x_lim[i+1]*y_lim[i]) / (x_lim[i] - x_lim[i+1])
		coeff.appen([m,q])

	m = (y_lim[3] - y_lim[0]) / (x_lim[3] + x_lim[0])
	q = (x_lim[3]*y_lim[0] - x_lim[0]*y_lim[3]) / (x_lim[3] - x_lim[0])
	coeff.appen([m,q])

	if (y >= coeff[0][0]*x + coeff[0][1]
		and y <= coeff[1][0]*x + coeff[1][1]
		and y <= coeff[2][0]*x + coeff[2][1]
		and y >= coeff[3][0]*x + coeff[3][1]):
		pass
	print("x:",x,"\n","y:",y)