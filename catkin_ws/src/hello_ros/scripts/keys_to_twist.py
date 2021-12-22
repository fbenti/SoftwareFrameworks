#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# ROS node that output a Twist message every time it receives a std_msgs/String message 
# that start with a character it understands

# We will use 
# w : go forward
# x : go backword # not possible
# a : turn left
# d : turn right
# s : stop

# BEGIN KEYMAP
key_mapping = { 'w': [ 0, 0.2],
                'd': [-1, 0], 'a': [1,  0],
                's': [ 0, 0] }
# END KEYMAP

# Incoming keys are looked up in the dict, if found the target velocities are extracted from
# the dictionary
def keys_cb(msg, twist_pub):
    # BEGIN CB
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return # unknown key.
    vels = key_mapping[msg.data[0]]
    # END CB
    t = Twist()
    t.angular.z = vels[0]
    t.linear.x  = vels[1]
    twist_pub.publish(t)
 
if __name__ == '__main__':
    rospy.init_node('keys_to_twist')
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, keys_cb, twist_pub)
    rospy.spin()