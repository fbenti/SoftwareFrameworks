#!/usr/bin/env python2.7
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
 
if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')
 
    listener = tf.TransformListener() # create a tf.TransformListener object, once it's created it starts 
    # receveing tf transformations over the wire and buffers them for up to 10 sec
 
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')
 
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    
    # we query the listener for a specific transformation by lookupTransform 
    # - we want the transform from the /turtle1 frame ..
    # - to the /turtle2 frame
    # - the time at which we want to ransform. Providing rospy.Time(0) will just get us the latest available transform
    # this function returns two list: the first is the (x,y,z) linear transformation of the child frame relative to the parent, 
    # and the second is the (x,y,z,w) quaternion required to rotate from the parent orientation to the child orientation
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
 
        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)
 
        rate.sleep()