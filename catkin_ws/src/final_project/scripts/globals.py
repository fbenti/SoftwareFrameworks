import rospy
import tf
import tf2_ros
import copy, math, random as rnd, numpy as np
from geometry_msgs.msg import Pose, Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def init():
    """  Global variables  """
    # Velocity publisher
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


    # Wandering phase:
    turn_right = 0 # randomly turn left or right when wandering
    g_range_ahead = 1 # laser_scan

    readingMarker = False
    currMarkerHiddenPos = None



    IDs = []
    nMarkers = 5
    sentence = [None]*nMarkers
    qrCurrPosHidden = [None]*nMarkers
    qrNextPosHidden = [None]*nMarkers
    lastId = 0


    qrPoseInOdom = [None]*nMarkers
    countMarker = 0
    markers_letter = [None] * nMarkers

    markerPose = None
    targetMsg = None

    p1 = [-4.5406, 0.30975]
    p2 = [-6.01, 1.62]
    p3 = [-4.5875, -2.00109]

    p = [p3,p2,p1]