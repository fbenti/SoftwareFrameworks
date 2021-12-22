#!/usr/bin/env python2.7
import rospy
 
import tf
import turtlesim.msg
 
 # the handler function for the turtle pose message broadcasts this turtle's translation and rotation
 # and publishes it as a transform from frame "world" to frame "turtleX"
def handle_turtle_pose (msg, turtlename):
    br = tf.TransformBroadcaster()
    # send out the pose of the turtle in the form of a transform
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0 , 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")
 
if __name__ == '__main__':
    rospy.init_node( 'turtle1_tf_broadcaster')

    turtlename1 = rospy.get_param('~turtle', default='turtle1') # specifies turtle name
    # subscribe to the topic pose of the turtle and runs the function handle_turtle_pose on every incoming message
    rospy.Subscriber( '/%s/pose' %  turtlename1,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename1)
    rospy.spin()