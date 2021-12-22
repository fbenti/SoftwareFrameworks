#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

# Send a stream of motion commands 10 times per second, alternating
# every 3 seconds between driving and stopping. When driving, the program will send
# forward velocity commands of 0.5 meters per second. When stopped, it will send
# commands of 0 meters per second.

# the queue size=1 tells rospy to only buffer a single outbond message.In
# case the node sending the messages is transmitting at a higher rate than the receiving
# node(s) can receive them, rospy will simply drop any messages beyond the
# queue_size.

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('red_light_green_light')

# The message constructors set all fields to zero. Therefore, the red_light_twist
# message tells a robot to stop, since all of its velocity subcomponents are zero.
red_light_twist = Twist()
green_light_twist = Twist()
# The x component of the linear velocity in a Twist message is, by convention, aligned
# in the direction the robot is facing, so this line means “drive straight ahead at 0.5
# meters per second.
green_light_twist.linear.x = 0.5

driving_forward = False
light_change_time = rospy.Time.now()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # We need to continually publish a stream of velocity command messages, since most
    # mobile base drivers will time out and stop the robot if they don’t receive at least
    # several messages per second.
    if driving_forward:
        cmd_vel_pub.publish(green_light_twist)
    else:
        cmd_vel_pub.publish(red_light_twist)
    # This if checks the system time and toggles the red/green light periodically.
    if rospy.Time.now() > light_change_time:
        driving_forward = not driving_forward
        light_change_time = rospy.Time.now() + rospy.Duration(3)
    # Without this call to rospy.sleep() the code would still run, but it would send far too
    # many messages, and take up an entire CPU core!
    rate.sleep()