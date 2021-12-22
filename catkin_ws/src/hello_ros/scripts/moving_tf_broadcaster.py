#!/usr/bin/env python2.7
import rospy
import tf
from math import sin, cos

if __name__ == '__main__':
    rospy.init_node("moving_tf_broadcaster")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        br.sendTransform(
                        (2*cos(rospy.Time.now().to_sec() - start_time), 2*sin(rospy.Time.now().to_sec() - start_time), 0.0),
                        # (2*cos(rospy.Time.now().to_sec()), 2*sin(rospy.Time.now().to_sec()), 0.0),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "carrot1",
                        "turtle1"
                        )
        rate.sleep()


