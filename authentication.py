# -*- coding: utf-8 -*-
"""
Created on Wed Jun 07 22:05:52 2017

@author: Kyle
"""

#!/usr/bin/env python

import pickle
import Leap, sys, thread, time
import matplotlib.pyplot as plt
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

starttime = time.time()

class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']
    palm = []
    index = []
    mid = []
    ring = []
    pinky = []
    thumb = []
    roll = []
    handlist = [thumb, index, mid, ring, pinky]
    vectlist = [[], [], [], [], []]

    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        # Get hands
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"

            print "  %s, id %d, position: %s" % (
                handType, hand.id, hand.palm_position)

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            self.palm.append(normal)
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            print "  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG)
            self.roll.append(normal.roll)
            
            # Get arm bone
            # arm = hand.arm
            # print "  Arm direction: %s, wrist position: %s, elbow position: %s" % (
            #     arm.direction,
            #     arm.wrist_position,
            #     arm.elbow_position)

            # Get fingers
            for finger in hand.fingers:
                self.handlist[finger.type].append(finger.direction)
                finger_pos = finger.tip_position
                hand_pos = hand.palm_position
                self.vectlist[finger.type].append(subtract_vectors(finger_pos, hand_pos))

#                print "    %s finger, id: %d, length: %fmm, width: %fmm" % (
#                     self.finger_names[finger.type],
#                     finger.id,
#                     finger.lengthwqwq,
#                     finger.width)

        # Get tools
        for tool in frame.tools:

            print "  Tool id: %d, position: %s, direction: %s" % (
                tool.id, tool.tip_position, tool.direction)


    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

def main(stage):
    global starttime
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()
    
    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until 2 seconds have passed
    while(time.time() - starttime < 2):
        pass
    else:
        controller.remove_listener(listener)
     # Compute average finger directional vectors
    avgNorms = []
    for vectorList in listener.handlist:
        avgNorms.append(compute_avg_vector(vectorList))
    avgPalm = compute_avg_vector(listener.palm)
    print('palm',avgPalm)
    print([vector for vector in avgNorms])
    # Compute absolute dot product of palm to finger directional vectors
    avgDotProds = []
    for vector in avgNorms:
        avgDotProds.append(abs(sum(p*q for p,q in zip(avgPalm, vector))))
    # Compute roll
    avgRoll = sum(listener.roll)/len(listener.roll)
    
    avgFingerVectors = []
    for vectorList in listener.vectlist:
        avgFingerVectors.append(compute_avg_vector(vectorList))
    
        # Getting back the objects:
    with open('objs.pickle') as f:  # Python 3: open(..., 'rb')
        fInt, dInt, rInt = pickle.load(f)
    for i in range(len(fInt)):
        print('new finger')
        for j in range(3):
            print('actual finger', avgFingerVectors[i][j])
            print('coordinate bound',fInt[i][j])
    for i in range(len(dInt)):
        print('actual dot', avgDotProds[i])
        print('dot bound',dInt[i])
    print('Average roll', avgRoll)
    print('roll interval', rInt)
        
def compute_avg_vector(vectorList):
    # Computes average vector from a list of vectors at different times
    x = 0
    y = 0
    z = 0
    numVecs = len(vectorList)
    for vector in vectorList:
        x += vector.x
        y += vector.y
        z += vector.z
    return [x/numVecs,y/numVecs,z/numVecs]

def get_dim_across_vecs(vectorList, dimension):
    # Returns every element of a dimension across all vectors
    # e.g. get_across_list(someList,1) returns a list of X values
    # for every vector in someList
    values = []
    if dimension == 0:
        values = [vector.x for vector in vectorList]
    if dimension == 1:
        values = [vector.y for vector in vectorList]
    else:
        values = [vector.z for vector in vectorList]
    return values

def subtract_vectors(vector1, vector2):
    # Subtract two leap vectors, where we subtract vector2 from vector1
    x = vector1.x - vector2.x
    y = vector1.y - vector2.y
    z = vector1.z - vector2.z
    return Leap.Vector(x,y,z)

if __name__ == "__main__":
    main(2)
