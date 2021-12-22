#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
 
# Connect to data streams in ROS

# The scan_callback() function is called each time a new message arrives on the scan topic
def scan_callback(msg):
  # Prints the range measured to the object directly in front of the robot 
  # by picking the middle element of the ranges field of the LaserScan message

  range_ahead = msg.ranges[0]
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  # Prints the range(in meters) from the bot to the nearest obstacles 
  # directly in front of him (min(tmp))
  print("range ahead: %0.1f" % min(tmp))
 

rospy.init_node('range_ahead')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()