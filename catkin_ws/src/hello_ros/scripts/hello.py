#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
 
def hello_ros():
    rospy.init_node('hello_ros', anonymous=True) # initialize a node with a unique name
    rate = rospy.Rate(5) # set the frequency of sending data
    hello_str = "hello ros!!! %s" % rospy.get_time()
    rospy.loginfo(hello_str) # print the message data
    while not rospy.is_shutdown(): # infinte while loop
        rate.sleep()
 
if __name__ == '__main__':
    try:    
        hello_ros()
    except rospy.ROSInterruptException:
        pass