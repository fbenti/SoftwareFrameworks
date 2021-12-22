#!/usr/bin/env python
 
import sys
import rospy
from turtlesim.srv import *
 
def call_service_rel(x, theta):
    rospy.wait_for_service('/turtle1/teleport_relative')
    try:
        teleport_relative = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)
        resp1 = teleport_relative(x, theta)
        return True
    except (rospy.ServiceException, e):
        print("Service call failed: %s"%e)
        return False

def call_service_abs(x, y, theta):
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        teleport_absolute = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        resp1 = teleport_absolute(x, y, theta)
        return True
    except (rospy.ServiceException, e):
        print("Service call failed: %s"%e)
        return False
 
def usage():
    return "%s [x y theta]"%sys.argv[0]
 
if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        theta = float(sys.argv[2])
        print("Requesting a relative telepot linear:%s, angular:%s"%(x, theta))
        print("The execution is %s"%call_service_rel(x, theta))
    elif len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3])
        print("Requesting an absolute teleport x:%s, y: %s, theta:%s"%(x, y, theta))
        print("The execution is %s"%call_service_abs(x, y, theta))
    else:
        print(usage())
        sys.exit(1)
    # print("Requesting an absolute teleport x:%s, y: %s, theta:%s"%(x, y, theta))
    # print("The execution is %s"%call_service(x, y, theta))