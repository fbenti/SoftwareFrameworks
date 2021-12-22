#!/usr/bin/env python

import rospy, tf, random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from gazebo_msgs.srv import GetModelState,GetWorldProperties
import roslib
import sys
import os
import copy
from sensor_msgs.msg import JointState
import moveit_commander
import moveit_msgs.msg
import shape_msgs.msg as shape_msgs
import math

class Robot:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_counter = 0
        self.attempts = 0
        self.done = False
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("Arm")
        self.joint_control_pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
        self.display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.rate = rospy.Rate(10)

    def recognize_enviroment(self):
        self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.world_state = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        self.models_in_world = self.world_state().model_names
        self.num_of_cubes =len(self.models_in_world)-3
        self.cubes_coordinates =[]
        rospy.sleep(3.)
        for i in range(self.num_of_cubes):  #to be changed by subscribing to a specific topic
            self.cubes_coordinates.append(self.model_coordinates("cube"+str(i), "link"))
        self.bucket_coordinates = self.model_coordinates("bucket", "link")

        for i in range(self.num_of_cubes):
            p = geometry_msgs.msg.PoseStamped()
            p.header.frame_id = self.robot.get_planning_frame()
            p.pose.position = self.cubes_coordinates[i].pose.position
            p.pose.orientation = self.cubes_coordinates[i].pose.orientation
            self.scene.add_box("cube"+str(i), p, (0.05, 0.05, 0.05))
        print('============ Cubes added as an obstacles')
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = self.bucket_coordinates.pose.position.x
        p.pose.position.y = self.bucket_coordinates.pose.position.y
        p.pose.position.z = self.bucket_coordinates.pose.position.z + 0.1
        p.pose.orientation = self.bucket_coordinates.pose.orientation
        self.scene.add_box("bucket", p, (0.2, 0.2, 0.2))
        print('============ Bucket added as an obstacles')

    def move(self,x,y,z):

        self.group.get_planning_frame()
        self.group.get_end_effector_link()
        self.robot.get_group_names()
        self.robot.get_current_state()

        self.group.set_goal_orientation_tolerance(0.1)
        self.group.set_goal_tolerance(0.1)
        self.group.set_goal_joint_tolerance(0.1)
        self.group.set_num_planning_attempts(100)#not supported anymoe
        ## Planning to a Pose goal
        print ("============ Generating move: "+str(self.move_counter))
        self.move_counter += 1

        pose_goal = self.group.get_current_pose().pose
        waypoints = []

        #pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  0.  , 0.))
        waypoints.append(pose_goal)
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(2.330175, -1.524318, 0.419453))
        #pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,0,0))

        #Create waypoints
        waypoints.append(pose_goal)

        #createcartesian  plan
        (plan1, fraction) = self.group.compute_cartesian_path(
                                            waypoints,   # waypoints to follow
                                            0.01,        # eef_step
                                            0.0)         # jump_threshold
        rospy.sleep(0.5)

        ## Moving to a pose goal
        print "============ Executing"
        print("fraction: ", fraction)
        self.group.execute(plan1,wait=True)
        rospy.sleep(1.)

    def gripper_open(self,state): #if state == true then open the gripper
        currentJointState = rospy.wait_for_message("/joint_states",JointState)
        currentJointState.header.stamp = rospy.get_rostime()
        if (state):
            tmp = 0.005
        else:
            tmp = 0.7
        currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
        for i in range(3):
            self.joint_control_pub.publish(currentJointState)
            self.rate.sleep()
        rospy.sleep(1.)
    def grab_cubes(self):
        safe_above = 0.307
        save_grab = 0.935
        self.gripper_open(True)
        for cube in self.cubes_coordinates:  #to be changed by subscribing to a specific topic
            coord = cube.pose.position
            self.move(coord.x,coord.y,coord.z + safe_above)
            self.move(coord.x,coord.y,save_grab)
            self.gripper_open(False)
            self.move(coord.x,coord.y,coord.z + safe_above)
            self.move(self.bucket_coordinates.pose.position.x+0.02,self.bucket_coordinates.pose.position.y+0.05,self.bucket_coordinates.pose.position.z+0.5)
            self.gripper_open(True)
        self.attempts+=1
    def save_position(self):
        robot.move(0.5,0.5,1.1)
        print('============ Save position reached')
        #there should be a calibration
    def check_the_goal(self):
        print('============ Number of attempts: ' +str(self.attempts))
        #to be changed
        bucket_p = self.bucket_coordinates.pose.position
        bucket_p.x = 0.53
        bucket_p.y = -0.23

        #bucket_p = self.bucket_coordinates.pose.position # thats not a  center of this f.. bucket(why?)
        bucket_size = 0.3 #to change-> dynamically reading bucket dimensions
        for cube in self.cubes_coordinates:
            cube_p = cube.pose.position
            if (( ((cube_p.x-bucket_p.x) ** 2) + ((cube_p.y-bucket_p.y) ** 2) ) <= ((bucket_size ** 2)/2) ):
                print('============ Removing cube as a goal')
                self.cubes_coordinates.remove(cube)
        #checking the last cube -> to be changed in code
        if len(self.cubes_coordinates) == 1:
            cube_p = self.cubes_coordinates[0].pose.position
            if (( ((cube_p.x-bucket_p.x) ** 2) + ((cube_p.y-bucket_p.y) ** 2) ) <= ((bucket_size ** 2)/2) ):
                print('============ Removing cube as a goal')
                self.cubes_coordinates.pop(0)

        print('============ Number of cubes to grab: ' + str(len(self.cubes_coordinates)))
        if len(self.cubes_coordinates) > 0:
            self.done = False
            return False
        else:
            print('============ Task completed')
            self.done = True
            return True


class Object_spawner():
    def __init__(self):
        #rospy.wait_for_service("/jaco/joint_control")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        print('============ Object spawner: services registered')
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.orient = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., 0.0, 0.785398))
    
    def clean_enviroment(self):
        self.world_state = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        self.models_in_world = self.world_state().model_names
        self.prev_num_of_cubes = len(self.models_in_world)-2
        for i in range(self.prev_num_of_cubes):
            self.delete_model("cube"+str(i))
        if 'bucket' in self.models_in_world :
            self.delete_model("bucket")
        print('============ Enviroment cleaned')

    def spawn_cubes(self):
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
    
    def spawn_bucket(self):
        dirname = os.path.dirname(__file__)
        bucketfile = os.path.join(dirname, '../urdf/bucket.urdf')
        with open(bucketfile, "r") as f:
            product_xml = f.read()
        item_pose   =   Pose(Point(x=0.53, y=-0.23,    z=0.78),   self.orient)
        print("============ Spawning model: bucket")
        self.spawn_model("bucket", product_xml, "", item_pose, "world")

if __name__ == '__main__':

    rospy.init_node("spawn_products_in_bins")

    #initialization of the enviroment
    object_spawner = Object_spawner()
    object_spawner.clean_enviroment()
    object_spawner.spawn_cubes()
    object_spawner.spawn_bucket()

    #initialization of a robot
    robot = Robot()
    robot.save_position()
    robot.recognize_enviroment()

    while ((not robot.check_the_goal()) and robot.attempts < 3):
        robot.grab_cubes()
        robot.recognize_enviroment()
    if robot.done == True :
        print('=========== Mission complete')
    else:
        print('============ Mission fail')

    robot.move(robot.bucket_coordinates.pose.position.x+0.02,robot.bucket_coordinates.pose.position.y+0.05,robot.bucket_coordinates.pose.position.z+0.5)
