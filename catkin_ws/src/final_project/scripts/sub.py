#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

IDs = []
sentence = []
known_pos = []
qrPose = []
recordPose = False


def decode_message(string):
    global IDs, sentence, known_pos,recordPose
    msg = string.split()
    # Position current qr
    x = float(msg[0][2:])
    y = float(msg[1][2:])
    curr_pos = (x,y)
    if curr_pos not in known_pos:
        recordPose = True
        known_pos.append(curr_pos)
    # Position next qr
    x = float(msg[2][7:])
    y = float(msg[3][7:])
    next_pos = (x,y)
    if next_pos not in known_pos:
        known_pos.append(next_pos)
    # Current ID 
    id = int(msg[4][2:])
    if id not in IDs:
        IDs.append(id)
        word = msg[5]
        sentence.append(word)
    return


def message_callback(msg):
    if not msg.data:
        return False
    else:
        decode_message(msg.data)
        return True

def pose_callback(msg):
    # print("here")
    # print(msg.pose)
    global recordPose, qrPose
    if recordPose:
        qrPose.append(msg.pose)
        for i in range(len(qrPose)):
            print("\n{0}\n".format(i))
            print(qrPose[i])
        recordPose = False

if __name__ == '__main__':
    rospy.init_node( 'subscriberCODE')

    rospy.Subscriber('/visp_auto_tracker/code_message', String, message_callback)
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, pose_callback)
    rospy.spin()
    # print(qrPose)
    # rospy.sleep(2)