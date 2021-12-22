#!/usr/bin/env python
import rospy
from  utils import *



'''
roslaunch final_project turtlebot3_world.launch layout:=1
roslaunch final_project navigation.launch
roslaunch final_project qr_visp.launch
rostopic echo /visp_auto_tracker/object_position
rostopic echo /visp_auto_tracker/code_message
'''


if __name__ == '__main__':


    rospy.init_node("navigation")

    # Define tf listener
    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)
    
    # [countMarker, nextID] = detectNextMarker([-3.08,1.95])
    # print(countMarker)
    # print(nextID)
    keyPoint = 0
    while countMarker < 2 and keyPoint < len(p):
        print("--- countMarker: ",countMarker)
        print("\n--- Reaching key point {} ---".format(keyPoint+1))
        goToKeyPoint(tfBuffer, keyPoint)
        keyPoint += 1
        countMarker = do360(tfBuffer)

    print("\n--- Two Markers Found --")

    print(IDs)
    print("\n--- Curr Marker in Hidden ---")
    print(qrCurrPosHidden)
    print("\n--- Curr Marker in Odom ---")
    print(qrPoseInOdom)
    print("\n--- Next Marker in Hidden ---")
    print(qrNextPosHidden)

    # Find transformation
    t,R = kabsch()

    # In the first cicle, go to the next hidden marker given by the highest ID
    nextID = max(IDs)
    position = transformNextQr(t,R,qrNextPosHidden[nextID-1])
    print(position)
    while countMarker < 5:
        idx = nextID - 1
        prevID = copy.deepcopy(nextID)
        if nextID == 5:
            currID = 1
            nextID = 1            
            print("\n--- Going towards marker {} at location ({},{}) ---".format(nextID,qrNextPosHidden[idx][0],qrNextPosHidden[idx][1]))
        else:
            print("\n--- Going towards marker {} at location ({},{}) ---".format(nextID+1,qrNextPosHidden[idx][0],qrNextPosHidden[idx][1]))
        print(position)
        go2marker(position,tfBuffer)
        # print(".....")
        # print(currID)
        [countMarker, currID, currIdx] = detectNextMarker(qrNextPosHidden[idx])
        # print("----")
        # print(prevID)
        # print(currID)
        # print(currIdx)
        # print(nextID)
        keyPoint = 0
        while currID == prevID and keyPoint < len(p):
            print("\n--- Going back to previous keyPoint {} ---".format(keyPoint))
            goToKeyPoint(tfBuffer, keyPoint)
            print("\n--- Detecting marker from keyPoint {} ---".format(keyPoint))
            [countMarker, currID, currIdx] = detectNextMarker(qrNextPosHidden[idx])
            if currID == nextID:
                print("\n--- Going again to previous position {} ---".format(position))
                go2marker(position, tfBuffer)
                print("\n--- Detecting again from position ---")
                [countMarker, currID, currIdx] = detectNextMarker(qrNextPosHidden[idx])
            keyPoint += 1
        print("\nCountMarker :", countMarker)
        print("\nTarget found ID: :", currID)
        position = transformNextQr(t,R,qrNextPosHidden[currIdx])
        nextID = copy.deepcopy(currID)

    print("\n--- TERMINATED: FOUND ALL TARGETS ---")
    s = sentence[0]
    for i in range(1,len(sentence)):
        s += sentence[i]
    print("\n--- {} ---".format(s))