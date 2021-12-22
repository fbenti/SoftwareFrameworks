#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
import math
import copy

markerPose = None

def MarkerPoseCallback(msg):
    global markerPose
    markerPose = msg.pose


def markerPoseInCamera():
    """Return  marker's pose related the CameraLink. """
    global markerPose
    # markerPose = None
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, MarkerPoseCallback)
    while markerPose == None:
        rospy.sleep(1)
    return markerPose

def markerPoseInOdom(robotPose,mPoseInCamera):
    ''' Return marker pose with respect to /map frame '''
    # global qrPoseInOdom
    # robotPose wrt the /odom frame
    # markerPoseInCamera wrt to optical_camera_link
    mPoseInOdom = copy.deepcopy(robotPose)
    quaternion = (robotPose.orientation.x,robotPose.orientation.y,robotPose.orientation.z,robotPose.orientation.w)
    angle = tf.transformations.euler_from_quaternion(quaternion)
    # roll = angle[0] --- pitch = angle[1] --- yaw = angle[2]
    mPoseInOdom.position.x += mPoseInCamera.position.z * math.cos(angle[2])
    mPoseInOdom.position.y += mPoseInCamera.position.z * math.sin(angle[2])
    # Save position
    # qrPoseInOdom[lastId] = (markerPose.position.x, markerPose.position.y)
    return mPoseInOdom


def robotCurrentPose(tfBuffer):
    ''' Return the current position of the robot in odom frame'''
    # Tf listener
    rate = rospy.Rate(10.0)
    trans = None
    while trans == None:
        try:
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    pose = Pose()
    pose.position = trans.transform.translation
    pose.orientation = trans.transform.rotation
    return pose

rospy.init_node("object_pos")

# Define tf listener
tfBuffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tfBuffer)

print("--- Pose object in camera---")
mPose = markerPoseInCamera()
print(mPose)
print("\n\n")
print("--- Pose Robot in Map ---")
robotPose = robotCurrentPose(tfBuffer)
print(robotPose)
print("\n\n")
print("--- Pose object in Map ---")
print(markerPoseInOdom(robotPose, mPose))