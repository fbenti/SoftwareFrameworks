#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Pose

def robotCurrentPose(tfBuffer):
    ''' Return the current position of the robot in odom frame'''
    # Tf listener
    rate = rospy.Rate(10.0)
    trans = None
    while trans == None:
        try:
            trans = tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    pose = Pose()
    pose.position = trans.transform.translation
    pose.orientation = trans.transform.rotation
    return pose

rospy.init_node("navigation")

# Define tf listener
tfBuffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tfBuffer)

print("--- Current Robot Pose ---")
print(robotCurrentPose(tfBuffer))