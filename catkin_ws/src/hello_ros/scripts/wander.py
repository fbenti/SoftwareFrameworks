#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

def scan_callback(msg):
    global g_range_ahead
    tmp=[msg.ranges[0]]
    for i in range(1,21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
        tmp.append(msg.ranges[i])
    g_range_ahead = min(tmp)
 
 
g_range_ahead = 1 # anything to start

# Create a subscriber to scan and a publisher to cmd_vel
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')

# Set up 2 variables for the controller logic
state_change_time = rospy.Time.now()
driving_forward = True
# the rate variable ihelps create loops that run at a fixed frequency
rate = rospy.Rate(60)
# We then call rate.sleep() at the end of the loop,
# rospy will adjust the amount of actual sleeping
 
# while not rospy.is_shutdown():
#     if driving_forward:
#         # keep driving until obstacle within 0.8 meters or times out after 30
#         if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
#             driving_forward = False
#             state_change_time = rospy.Time.now() + rospy.Duration(5)
#         else: # we're not driving_forward
#             if rospy.Time.now() > state_change_time:
#                 driving_forward = True # we're done spinning, time to go forward!
#                 state_change_time = rospy.Time.now() + rospy.Duration(30)

#     twist = Twist()
#     if driving_forward:
#         twist.linear.x = 0.4
#         twist.angular.z = 0.0
#     else:
#         twist.linear.x = 0.0
#         twist.angular.z = 0.4

#     cmd_vel_pub.publish(twist)

#     rate.sleep()
# # END ALL
change = False
while not rospy.is_shutdown():
    print g_range_ahead
    if g_range_ahead < 0.8:
        # TURN
        driving_forward = False
        print "Turn"
    
    else: # we're not driving_forward
        driving_forward = True # we're done spinning, time to go forward!
        #DRIVE
        print "Drive"
    
    twist = Twist()
    if driving_forward:
        twist.linear.x = 0.4
        twist.angular.z = 0.0
    else:
        twist.linear.x = 0.0
        twist.angular.z = -0.4
    cmd_vel_pub.publish(twist)
    
    rate.sleep()