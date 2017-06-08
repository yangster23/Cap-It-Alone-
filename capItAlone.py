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
    for vectorList in listener.handlist:
        avgNorms.append(compute_avg_vector(vectorList))
    avgPalm = compute_avg_vector(listener.palm)
    print('palm',avgPalm)
    print([vector for vector in avgNorms])
    # Compute absolute dot product of palm to finger directional vectors
    avgDotProds = []
    stdDotProds = []
    for vector in avgNorms:
        avgDotProds.append(abs(sum(p*q for p,q in zip(avgPalm, vector))))
    ####
    for vectorList in listener.handlist:
        stdDotProds.append(compute_var_dot_product(listener.palm,vectorList)**0.5)
    ####
    # Compute roll
    avgRoll = sum(listener.roll)/len(listener.roll)
    stdRoll = (sum([(num-avgRoll)**2 for num in listener.roll])/(len(listener.roll)-1))**0.5
    
    print('Average roll', avgRoll)
    print('Std Roll', stdRoll)
    print('Average dot prods', [item for item in avgDotProds])
    print('Average dot prod stds', [item for item in stdDotProds])

    avgFingerVectors = []
    for vectorList in listener.vectlist:
        avgFingerVectors.append([compute_avg_vector(vectorList), compute_std_vector(vectorList)])
    for vector in avgFingerVectors:
        print('Avg. Finger Vector: ', vector[0])
        print('Avg. Finger Std: ', vector[1])
    fingerIntervals = []
    for i in range(len(avgFingerVectors)):
        finger = []
        for j in range(3):
            finger.append([avgFingerVectors[i][0][j]-avgFingerVectors[i][1][j],
                           avgFingerVectors[i][0][j]+avgFingerVectors[i][1][j]])
        fingerIntervals.append(finger)
    print('Finger Confidence Intervals', fingerIntervals)
    dotProdIntervals = []
    for i in range(len(avgDotProds)):
        dotProdIntervals.append([avgDotProds[i]-stdDotProds[i],
                           avgDotProds[i]+stdDotProds[i]])
    print('Dot Prod Confidence Intervals', dotProdIntervals)

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

def compute_std_vector(vectorList):
    # Compute std vector from a list of vectors at different times
    var_vector = compute_var_vector(vectorList)
    return [num**0.5 for num in var_vector]

def compute_var_vector(vectorList):
    # Compute variance vector from a list of vectors at different times
    meanX, meanY, meanZ = compute_avg_vector(vectorList)
    x = 0
    y = 0
    z = 0
    numVecs = len(vectorList)    
    for vector in vectorList:
        x += (vector.x - meanX)**2
        y += (vector.y - meanY)**2
        z += (vector.z - meanZ)**2
    return [x/(numVecs-1),y/(numVecs-1),z/(numVecs-1)]

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

def compute_cov(X,Y):
    # Compute cov(X,Y)
    numObs = len(X)
    meanX = sum(X)/numObs
    meanY = sum(Y)/numObs
    covSum = 0
    for i in range(len(X)):
        covSum += (X[i]-meanX)*(Y[i]-meanY)
    return covSum/(numObs-1)

def compute_cov_XiYi_XjYj(vectorList1, vectorList2, dim1, dim2):
    # Compute cov(X_i*Y_i,X_j*Y_j), see compute_var_dot_product
    # for details
    meanFirst = compute_avg_vector(vectorList1)
    meanSecond = compute_avg_vector(vectorList2)
    covSum = (compute_cov(get_dim_across_vecs(vectorList1,dim1),
                          get_dim_across_vecs(vectorList1,dim2))*
        meanSecond[dim1]*meanSecond[dim2] + 
        compute_cov(get_dim_across_vecs(vectorList2,dim1),
                    get_dim_across_vecs(vectorList2,dim2))*
        meanFirst[dim1]*meanFirst[dim2] + 
        compute_cov(get_dim_across_vecs(vectorList1,dim1),
                    get_dim_across_vecs(vectorList1,dim2))*
        compute_cov(get_dim_across_vecs(vectorList2,dim1),
                    get_dim_across_vecs(vectorList2,dim2)))
    return covSum

def compute_var_dot_product(vectorList1,vectorList2):
    # Compute standard deviation of dot product of two vector lists at different times
    # i.e. compute var(X^TY) where dim(X) = nx1, dim(Y) = nx1
    # also see link below to explain formula
    # https://stats.stackexchange.com/questions/76961/variance-of-product-of-2-independent-random-vector
    meanFirst = compute_avg_vector(vectorList1)
    meanSecond = compute_avg_vector(vectorList2)
    varFirst = compute_var_vector(vectorList1)
    varSecond = compute_var_vector(vectorList2)
    indepSum = 0
    for i in range(len(meanFirst)):
        indepSum += (varFirst[i]*meanSecond[i]**2 + 
            varSecond[i]*meanFirst[i]**2 + 
            varFirst[i]*varSecond[i])
    covSum = 0
    for i in range(len(meanFirst)-1):
        for j in range(i+1,len(meanFirst)):
            covSum += compute_cov_XiYi_XjYj(vectorList1,vectorList2,i,j)
    finalVar = indepSum + 2*covSum
    return finalVar

def subtract_vectors(vector1, vector2):
    # Subtract two leap vectors, where we subtract vector2 from vector1
    x = vector1.x - vector2.x
    y = vector1.y - vector2.y
    z = vector1.z - vector2.z
    return Leap.Vector(x,y,z)

def check_finger_tolerance():
    pass

def check_dot_tolerance():
    pass
    
def check_vector_tolerance():
    pass

def check_roll_tolerance():
    pass

def check_tolerances(rollInfo, ):
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
