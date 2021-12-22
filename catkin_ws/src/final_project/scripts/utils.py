# -*- coding: utf-8 -*-
from codecs import CodecInfo
import rospy
import tf
import tf2_ros
import copy, math, random as rnd, numpy as np
from geometry_msgs.msg import Pose, Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import random as rnd

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
currID = 0
currIdx = 0

qrPoseInOdom = [None]*nMarkers
countMarker = 0
markers_letter = [None] * nMarkers

markerPose = None
targetMsg = None

p1 = [-4.406, 0.30975]
p2 = [-6.01, 1.62]
p3 = [-4.4246, -1.92109]
p4 = [4.9831, 0.8198]
p5 = [5.9842, -1.5888]

p = [p1,p2,p1,p4,p5]



def LaserScanCallback(msg):
    global g_range_ahead
    tmp=[msg.ranges[0]]
    for i in range(1,21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges)-21,len(msg.ranges)):
        tmp.append(msg.ranges[i])
    g_range_ahead = min(tmp)

def patrol():
    global turn_right, g_range_ahead
    driving_forward = True
    # Create a subscriber to scan and a publisher to cmd_vel
    rospy.Subscriber('scan', LaserScan, LaserScanCallback)
    if g_range_ahead < 0.8:
        driving_forward = False
    else: 
        driving_forward = True # we're done spinning, time to go forward!
        turn_right = rnd.randrange(2) # randomly turn right or left
        
    twist = Twist()
    if driving_forward:
        twist.linear.x = 0.4
        twist.angular.z = 0.0
    else:
        if turn_right:
            twist.linear.x = 0.0
            twist.angular.z = -0.4
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.4
    cmd_vel_pub.publish(twist)

def stop():
    cmd_vel_pub.publish(Twist())



def markerIsScanned():
    global targetMsg
    # targetMsg = None
    # rospy.Subscriber('/visp_auto_tracker/code_message', String, readCodeCallback)
    # when statu == 3 -> marker decoded
    # rospy.Subscriber('/visp_auto_tracker/code_message', String, readCodeCallback)

    # counter = 0
    # while targetMsg == None and counter < 100:
    #     counter += 1
    # if targetMsg == "" or targetMsg == None:
    #     return False
    # else:
    #     return True
    rospy.Subscriber('/visp_auto_tracker/status', Int8, readStatusCallback)
    if readingMarker:
        return True
    else: return False

def markerAlreadySeen(string,poseHidden):
    global IDs, currID, currIdx
    msg = string.split()
    id = int(msg[4][2:])
    x = float(msg[0][2:])
    y = float(msg[1][2:])
    deltaErr = 0.2
    # print("already seen")
    # print(id)
    # print(IDs)
    # print(poseHidden)
    # print(x,y)
    if id in IDs:
        if (abs(x - poseHidden[0]) <= deltaErr):
            if (abs(y - poseHidden[1]) <= deltaErr):
                currID = id
                currIdx = id - 1
                # print("okk")
                # print(currID)
                # print(currIdx)
                return True
    return False

def decodeMessage(string):
    """ Return TRUE if the marker read is new """
    global IDs, sentence, qrCurrPosHidden, qrNextPosHidden, countMarker,currID, currIdx
    msg = string.split()
    print(msg)
    # Current ID 
    id = int(msg[4][2:])
    if id not in IDs:
        countMarker += 1
        currID = id
        currIdx = id - 1
        IDs.append(currID)
        sentence[currIdx] = msg[5][2]
         # Position current qr
        x = float(msg[0][2:])
        y = float(msg[1][2:])
        qrCurrPosHidden[id-1] = (x,y)
        # Position next qr
        x = float(msg[2][7:])
        y = float(msg[3][7:])
        qrNextPosHidden[id-1] = (x,y)
        # qrNextPosHidden.append((x,y))
        return True
    else:
        return False

def MarkerPoseCallback(msg):
    global markerPose
    markerPose = msg.pose


def markerPoseInCamera():
    """Return  marker's pose related Map. """
    global markerPose
    # markerPose = None
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, MarkerPoseCallback)
    while markerPose == None:
        rospy.sleep(1)
    return markerPose


def markerPoseInOdom(robotPose,mPoseInCamera):
    ''' Return marker pose with respeact to /odom frame '''
    global qrPoseInOdom, currIdx
    # robotPose wrt the /odom frame
    # markerPoseInCamera wrt to optical_camera_link
    mPoseInOdom = copy.deepcopy(robotPose)
    quaternion = (robotPose.orientation.x,robotPose.orientation.y,robotPose.orientation.z,robotPose.orientation.w)
    angle = tf.transformations.euler_from_quaternion(quaternion)
    # roll = angle[0] --- pitch = angle[1] --- yaw = angle[2]
    mPoseInOdom.position.x += mPoseInCamera.position.z * math.cos(angle[2])
    mPoseInOdom.position.y += mPoseInCamera.position.z * math.sin(angle[2])
    # Save position
    qrPoseInOdom[currIdx] = (mPoseInOdom.position.x, mPoseInOdom.position.y)
    return mPoseInOdom


def robotCurrentPose(tfBuffer):
    ''' Return the current position of the robot in odom frame'''
    # Tf listener
    rate = rospy.Rate(10.0)
    trans = None
    while trans == None:
        try:
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    pose = Pose()
    pose.position = trans.transform.translation
    pose.orientation = trans.transform.rotation
    return pose


def smallTurn(vel_angular):
    """
    smallTurn(vel_angular)
    Works like move() but sets the speed for limited amount of time.
    """
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = vel_angular
    cmd_vel_pub.publish(twist)
    rospy.sleep(1)
    stop()


def wander(tfBuffer):
    global targetMsg, qrPoseInOdom, countMarker, qrCurrPosHidden, IDs,currID
    # rospy.init_node('wander')
    # Define deconstructor
    rospy.on_shutdown(stop)

    
    # tf_listener = tf.TransformListener()
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown() and countMarker < 2:
        # print("--- Current robot pose ---")
        # robotPoseInMap = robotCurrentPose(tfBuffer)
        # print(robotPoseInMap)
        # countMarker = 5

        print("Wandering.. looking for 2 QR-codes")

        # No marker is seen
        if not markerIsScanned():
            patrol()
        else:
            print("\n --- Stopping ---")
            stop()
            rospy.sleep(2)
            print("\n --- Patrolling after stop ---")
            break

        # elif decodeMessage(targetMsg):
        #     currentID = targetMsg
        #     stop()
        #     print("--- Decoding new marker ---")
        #     # markerPoseInOdom()
        #     centerCamera()
        #     rospy.sleep(2)
        #     print(IDs)
        #     print(qrCurrPosHidden)
        #     print(qrNextPosHidden)

        #     # print("--- Current robot pose ---")
        #     robotPoseInMap = robotCurrentPose(tfBuffer)
        #     # print(robotPoseInMap)

        #     # print("--- Marker pose in camera ---")
        #     mPoseInCamera = markerPoseInCamera()
        #     # print(mPoseInCamera)
            
        #     # print("--- Marker pose in map ---")
        #     mPoseInOdom = markerPoseInOdom(robotPoseInMap, mPoseInCamera)
        #     # print(mPoseInOdom)
        #     # qrPoseInOdom[currID] = [mPoseInOdom]
        # else:
        #     patrol()
        # rate.sleep()


def kabsch():
    global qrCurrPosHidden, qrPoseInOdom, IDs

    # First 2 QR in hidden frame
    Q = np.empty((2,3))
    Q[0,0] = qrCurrPosHidden[IDs[0]-1][0]
    Q[0,1] = qrCurrPosHidden[IDs[0]-1][1]
    Q[1,0] = qrCurrPosHidden[IDs[1]-1][0]
    Q[1,1] = qrCurrPosHidden[IDs[1]-1][1]
    Q[0,2] = 0
    Q[1,2] = 0
    # First 2 QR in odom frame
    P = np.empty((2,3))
    P[0,0] = qrPoseInOdom[IDs[0]-1][0] #.position.x
    P[0,1] = qrPoseInOdom[IDs[0]-1][1] #.position.y
    P[1,0] = qrPoseInOdom[IDs[1]-1][0] #.position.x
    P[1,1] = qrPoseInOdom[IDs[1]-1][1] #.position.y
    P[0,2] = 0
    P[1,2] = 0

    # Centroid and subtract
    centQ = np.mean(Q,axis=0)
    centP = np.mean(P,axis=0)
    Q = np.subtract(Q,centQ)
    P = np.subtract(P,centP)

    # Compute covariance matrix
    H = np.dot(Q.T,P)

    # Singular Value Decomposition
    U,S,Vt = np.linalg.svd(H)
    V = Vt.T

    # Decide whether we need to correct our rotation matrix to ensure a right-handed coordinate system
    d = np.sign(np.linalg.det(np.dot(V,U.T)))
    D = np.diag((1,1,d))

    # Rotation Matrix
    R = np.dot(V,(np.dot(D,U.T)))
    
    # Translation vector
    t = - np.dot(R,centQ) + centP
    t = t.reshape((3,1))

    return t,R

def transformNextQr(t,R,q):
    Q = np.empty((1,3))
    Q[0,0] = q[0]
    Q[0,1] = q[1]
    Q[0,2] = 0
    Q = Q.reshape((3,1))
    return np.dot(R,Q) + t

def go2marker(pos,tfBuffer):

    # rospy.init_node('smart_navigation')

    currPos = robotCurrentPose(tfBuffer)
    print("\n--- Curr Pose ---")
    print(currPos)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose = copy.deepcopy(currPos)
    # Original Pose Goal
    goal_pose.target_pose.pose.position.x = pos[0][0]
    goal_pose.target_pose.pose.position.y = pos[1][0]
    print("\n--- Original Goal Pose ---")
    print(goal_pose.target_pose.pose)   
    # Actual Pose Goal
    goal_pose.target_pose.pose.position.x = pos[0][0] * 0.85
    goal_pose.target_pose.pose.position.y = pos[1][0] * 0.75
    print("\n--- Actual Goal Pose: 80{} of the Original ---".format('%'))
    print(goal_pose.target_pose.pose) 
    # Remove past goals
    # client.cancel_all_goals()
    # client.stop_tracking_goal()
    # Set new goal
    client.send_goal(goal_pose)
    # print("\n--- Goal Pose ---")
    # print(goal_pose.target_pose.pose)   
    client.wait_for_result()

def goToKeyPoint(tfBuffer, i):
    currPos = robotCurrentPose(tfBuffer)
    print("\n--- Curr Pose ---")
    print(currPos)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose = copy.deepcopy(currPos)
    goal_pose.target_pose.pose.position.x = p[i][0]
    goal_pose.target_pose.pose.position.y = p[i][1]
    client.send_goal(goal_pose)
    print("\n--- Goal Pose ---")
    print(goal_pose.target_pose.pose)
    client.wait_for_result()


def returnMarkerHiddenPos(msg):
    mmsg = msg.split()
    print(mmsg)
    x = float(mmsg[0][2:])
    y = float(mmsg[1][2:])
    return (x,y)

def readCodeCallback(msg, arg):
    global targetMsg
    counter = 0
    while msg.data == None:# and counter < 100:
        rospy.sleep(0.1)
    if arg == 'decode':
        print('decode')
        targetMsg = msg.data
    elif arg == 'justPose':
        print(arg)
        returnMarkerHiddenPos(msg.data)

readingMarkerList = []

def readStatusCallback(msg):
    global readingMarker,readingMarkerList
    readingMarker=msg.data
    
def do360(tfBuffer, ):
    global readingMarker
    deltaErr = 0.1
    print("--- Doing 360 ---")
    nTurns = 0
    while nTurns < 12 and countMarker < 2:
        nTurns += 1
        smallTurn(np.pi/6)
        print("Done {}° out of 360, nTurns = {}".format(nTurns*30, nTurns))
        rospy.sleep(3)
        # rospy.Subscriber('/visp_auto_tracker/status', Int8, readStatusCallback)
        status = rospy.wait_for_message('visp_auto_tracker/status', Int8).data
        rospy.sleep(1)
        # print("--- Current Status : {} ---".format(readingMarker))
        print("--- Current Status : {} ---".format(status))
        if status == 3:
            nOccurences = 0
            mPose = markerPoseInCamera()
            while nOccurences < 3:
                temp = markerPoseInCamera()
                if abs(temp.position.x - mPose.position.x) <= deltaErr and abs(temp.position.y - mPose.position.y) <= deltaErr:
                    print("--equalss--")
                    nOccurences +=1
            # Read hidden message
            arg = 'decode'
            # rospy.Subscriber('/visp_auto_tracker/code_message', String, readCodeCallback, arg)
            msg2 = rospy.wait_for_message('/visp_auto_tracker/code_message', String).data
            rospy.sleep(1)
            # print("\nMessage Target")
            # print(targetMsg)
            decodeMessage(msg2)
            rPose = robotCurrentPose(tfBuffer)
            # print("\n--- Robot Current Pose ---")
            # print(rPose)
            mPoseInOdom = markerPoseInOdom(rPose, mPose)
            # print("\n--- Marker Pose In Odom ---")
            # print(mPoseInOdom)
            # print("\n--- Curr Marker in Hidden ---")
            # print(qrCurrPosHidden)
            # print("\n--- Curr Marker in Odom ---")
            # print(qrPoseInOdom)
            # print("\n--- Next Marker in Hidden ---")
            # print(qrNextPosHidden)
            rospy.sleep(3)
    return countMarker


def detectNextMarker(poseHidden):
    # pos: [(x,y)] of the marker to detect in hidden frame
    global currMarkerHiddenPos, targetMsg, countMarker, currID, currIdx
    deltaErr = 0.2
    print("\n--- Detecting New Marker ---")
    # print(currID)
    # print(currIdx)
    nTurns = 0
    while nTurns < 12:
        # rospy.Subscriber('/visp_auto_tracker/status', Int8, readStatusCallback, queue_size=1)
        status = rospy.wait_for_message('visp_auto_tracker/status', Int8).data
        rospy.sleep(1)
        # readingMarkerList = []
        print("--- Current Status : {} ---".format(status))
        # if readingMarker == 3:
        if status == 3:
            stable = False
            occurences  = 0
            while occurences < 3:
                status = rospy.wait_for_message('visp_auto_tracker/status', Int8).data
                if status == 3:
                    stable = True
                else:
                    stable = False
                    break
                occurences += 1
            
            if stable:
                # rospy.Subscriber('/visp_auto_tracker/code_message', String, readCodeCallback, callback_args=arg)
                msg = None
                while msg == None:
                    msg = rospy.wait_for_message('/visp_auto_tracker/code_message', String).data
                    rospy.sleep(0.2)
                    try:
                        mPose = returnMarkerHiddenPos(msg)
                    except:
                        msg = None
                print("nowPose : ",mPose)
                print("prevPose : ",poseHidden)
                print("abs(x): ", abs(mPose[0] - poseHidden[0]))
                print("abs(y): ", abs(mPose[1] - poseHidden[1]))

                # if (abs(mPose[0] - poseHidden[0]) <= deltaErr and abs(mPose[1] == poseHidden[1]) <= deltaErr):
                if (abs(mPose[0] - poseHidden[0]) <= deltaErr):
                    if (abs(mPose[1] - poseHidden[1]) <= deltaErr):
                        print("\n--- Found match !! ---")
                        # rospy.Subscriber('/visp_auto_tracker/code_message', String, readCodeCallback, callback_args=arg)
                        msg2 = rospy.wait_for_message('/visp_auto_tracker/code_message', String).data
                        rospy.sleep(1)
                        if markerAlreadySeen(msg2,poseHidden):
                            print("already seen")
                            # print(currID)
                            # print(currIdx)
                            pass
                        else:
                            decodeMessage(msg2)
                            print("decoding")
                            # print(currID)
                            # print(currIdx)
                        return [countMarker, currID, currIdx]
        nTurns += 1
        smallTurn(np.pi/6)
        print("Done {}° out of 360, nTurns = {}".format(nTurns*30, nTurns))
        rospy.sleep(3)
    # print("last")
    # print(currID)
    # print(currIdx)
    return [countMarker, currID, currIdx]





