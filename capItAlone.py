#!/usr/bin/env python


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

                # print "    %s finger, id: %d, length: %fmm, width: %fmm" % (
                #     self.finger_names[finger.type],
                #     finger.id,
                #     finger.length,
                #     finger.width)

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

def main():
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
    for vectorData in listener.handlist:
         x = 0
         y = 0
         z = 0
         numVecs = len(vectorData)
         for vector in vectorData:
             x += vector.x
             y += vector.y
             z += vector.z
         avgNorms.append((x/numVecs,y/numVecs,z/numVecs))
    # Compute average palm directional vector
    x = 0
    y = 0
    z = 0
    numVecs = len(vectorData)
    for vector in listener.palm:
          x += vector.x
          y += vector.y
          z += vector.z
    avgPalm = (x/numVecs,y/numVecs,z/numVecs)
    print('palm',avgPalm)
    print([vector for vector in avgNorms])
    # Compute absolute dot product of palm to finger directional vectors
    dotProdSum = 0
    for vector in avgNorms:
        print(abs(sum(p*q for p,q in zip(avgPalm, vector))))
        dotProdSum += abs(sum(p*q for p,q in zip(avgPalm, vector)))
    if dotProdSum == 0:
        print("Division by zero - hand might not have been detected")
    else:
        avgDot = dotProdSum/5
        print('Average dot product',avgDot)
    # Compute roll
    avgRoll = sum([abs(num) for num in listener.roll])/len(listener.roll)
    print('Average roll', avgRoll)
    successful = check_tolerances(avgRoll,avgDot)
    print("Password success: ", successful)
    
def check_tolerances(roll, dot):
    # Check whether or not roll and dot match our binary password
    # This position assumes a flat hand with fingers orthogonal to palm
    ROLL_TOLERANCE = 0.2
    DOT_TOLERANCE = 0.15
    if roll <= ROLL_TOLERANCE and dot <= DOT_TOLERANCE:
        return True
    else:
        return False
    
if __name__ == "__main__":
    main()
