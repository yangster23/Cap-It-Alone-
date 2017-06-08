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

            # Get the hand's normal vec and direction
            normal = hand.palm_normal
            self.palm.append(normal)
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            print "  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG)
            self.roll.append(normal.roll)
            
            # Get fingers
            for finger in hand.fingers:
                self.handlist[finger.type].append(finger.direction)
                finger_pos = finger.tip_position
                hand_pos = hand.palm_position
                self.vectlist[finger.type].append(subtract_vecs(finger_pos, hand_pos))

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
            
def subtract_vecs(vec1, vec2):
    # Subtract two leap vecs, where we subtract vec2 from vec1
    x = vec1.x - vec2.x
    y = vec1.y - vec2.y
    z = vec1.z - vec2.z
    return Leap.Vector(x,y,z)
    
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
    
    avgPalm, avgNorms = compute_avg_directional_vecs(listener)
    avgDotProds, stdDotProds = compute_dot_products(avgNorms, avgPalm, listener)
    avgRoll, stdRoll = compute_roll(listener)
    avgFingVecs, stdFingVecs = compute_fing_vecs(listener)
    
    fingIntervals, dotIntervals, rollIntervals = construct_conf_intervals(
    avgFingVecs,stdFingVecs,avgDotProds,stdDotProds,avgRoll,stdRoll)
    print_conf_intervals(fingIntervals, dotIntervals, rollIntervals)

    pickle_intervals(fingIntervals, dotIntervals, rollIntervals)


def compute_avg_directional_vecs(listener):
     # Compute Avg finger and palm directional vecs
    avgNorms = []
    for vecList in listener.handlist:
        avgNorms.append(compute_avg_vec(vecList))

    avgPalm = compute_avg_vec(listener.palm)
    return avgPalm, avgNorms
    
def compute_avg_vec(vecList):
    # Computes average vec from a list of vecs at different times
    x = 0
    y = 0
    z = 0
    numVecs = len(vecList)
    for vec in vecList:
        x += vec.x
        y += vec.y
        z += vec.z
    return [x/numVecs,y/numVecs,z/numVecs]

def compute_std_vec(vecList):
    # Compute std vec from a list of vecs at different times
    var_vec = compute_var_vec(vecList)
    return [num**0.5 for num in var_vec]

def compute_var_vec(vecList):
    # Compute variance vec from a list of vecs at different times
    meanX, meanY, meanZ = compute_avg_vec(vecList)
    x = 0
    y = 0
    z = 0
    numVecs = len(vecList)    
    for vec in vecList:
        x += (vec.x - meanX)**2
        y += (vec.y - meanY)**2
        z += (vec.z - meanZ)**2
    return [x/(numVecs-1),y/(numVecs-1),z/(numVecs-1)]

def get_dim_across_vecs(vecList, dimension):
    # Returns every element of a dimension across all vecs
    # e.g. get_across_list(someList,1) returns a list of X values
    # for every vec in someList
    values = []
    if dimension == 0:
        values = [vec.x for vec in vecList]
    if dimension == 1:
        values = [vec.y for vec in vecList]
    else:
        values = [vec.z for vec in vecList]
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

def compute_cov_XiYi_XjYj(vecList1, vecList2, dim1, dim2):
    # Compute cov(X_i*Y_i,X_j*Y_j), see compute_var_dot_product
    # for details
    meanFirst = compute_avg_vec(vecList1)
    meanSecond = compute_avg_vec(vecList2)
    covSum = (compute_cov(get_dim_across_vecs(vecList1,dim1),
                          get_dim_across_vecs(vecList1,dim2))*
        meanSecond[dim1]*meanSecond[dim2] + 
        compute_cov(get_dim_across_vecs(vecList2,dim1),
                    get_dim_across_vecs(vecList2,dim2))*
        meanFirst[dim1]*meanFirst[dim2] + 
        compute_cov(get_dim_across_vecs(vecList1,dim1),
                    get_dim_across_vecs(vecList1,dim2))*
        compute_cov(get_dim_across_vecs(vecList2,dim1),
                    get_dim_across_vecs(vecList2,dim2)))
    return covSum

def compute_var_dot_product(vecList1,vecList2):
    # Compute standard deviation of dot product of two vec lists at different times
    # i.e. compute var(X^TY) where dim(X) = nx1, dim(Y) = nx1
    # also see link below to explain formula
    # https://stats.stackexchange.com/questions/76961/variance-of-product-of-2-independent-random-vectors
    meanFirst = compute_avg_vec(vecList1)
    meanSecond = compute_avg_vec(vecList2)
    varFirst = compute_var_vec(vecList1)
    varSecond = compute_var_vec(vecList2)
    indepSum = 0
    for i in range(len(meanFirst)):
        indepSum += (varFirst[i]*meanSecond[i]**2 + 
            varSecond[i]*meanFirst[i]**2 + 
            varFirst[i]*varSecond[i])
    covSum = 0
    for i in range(len(meanFirst)-1):
        for j in range(i+1,len(meanFirst)):
            covSum += compute_cov_XiYi_XjYj(vecList1,vecList2,i,j)
    finalVar = indepSum + 2*covSum
    return finalVar
    
def compute_dot_products(avgNorms, avgPalm, listener):
    # Computes Avg and Std dot product of palm to finger directional vecs
    avgDotProds = []
    for vec in avgNorms:
        avgDotProds.append(sum(p*q for p,q in zip(avgPalm, vec)))
    stdDotProds = []
    for vecList in listener.handlist:
        stdDotProds.append(compute_var_dot_product(listener.palm,vecList)**0.5)
    return avgDotProds, stdDotProds

def compute_roll(listener):
    # Computes Avg and Std Roll
    avgRoll = sum(listener.roll)/len(listener.roll)
    stdRoll = (sum([(num-avgRoll)**2 for num in listener.roll])/(len(listener.roll)-1))**0.5
    return avgRoll, stdRoll

def compute_fing_vecs(listener):
    # Compute Avg and Std Finger Vecs relative to palm
    avgFingerVecs = []
    stdFingerVecs = []
    for vecList in listener.vectlist:
        avgFingerVecs.append(compute_avg_vec(vecList))
        stdFingerVecs.append(compute_std_vec(vecList))
    return avgFingerVecs, stdFingerVecs

def construct_fing_intervals(avgFingVecs, stdFingVecs):
    # Construct Finger Confidence Intervals for authentication
    fingIntervals = []
    for i in range(len(avgFingVecs)):
        fing = []
        for j in range(3):
            fing.append([avgFingVecs[i][j]-3*stdFingVecs[i][j],
                           avgFingVecs[i][j]+3*stdFingVecs[i][j]])
        fingIntervals.append(fing)
    return fingIntervals

def construct_dot_intervals(avgDotProds, stdDotProds):
    # Construct Dot Product Confidence Intervals for authentication
    dotProdIntervals = []
    for i in range(len(avgDotProds)):
        dotProdIntervals.append([avgDotProds[i]-3*stdDotProds[i],
                                 avgDotProds[i]+3*stdDotProds[i]])
    return dotProdIntervals

def construct_roll_intervals(avgRoll, stdRoll):
    # Construct Roll Confidence Intervals for authentication
    rollInterval = []
    rollInterval.append([avgRoll-3*stdRoll,avgRoll+3*stdRoll])
    return rollInterval

def construct_conf_intervals(avgFingVecs,stdFingVecs,avgDotProds,
                                   stdDotProds,avgRoll,stdRoll):
    # Construct three main intervals for authentication
    fingIntervals = construct_fing_intervals(avgFingVecs, stdFingVecs)
    dotIntervals = construct_dot_intervals(avgDotProds,stdDotProds)
    rollIntervals = construct_roll_intervals(avgRoll, stdRoll)
    return fingIntervals, dotIntervals, rollIntervals

def print_conf_intervals(fingIntervals, dotIntervals, rollIntervals):
    # Print confidence intervals for manual checking
    for fing in fingIntervals:
        print('New Finger')
        for coordinate in fing:
            print('Finger Dimension Bound', coordinate)
    for dotProd in dotIntervals:
        print('Dot Prod Bound', dotProd)
    print('Roll Interval Bound', rollIntervals)

def pickle_intervals(fingIntervals, dotIntervals, rollIntervals):
    # Pickles intervals for authentication
    with open('objs.pickle', 'w') as f:
        pickle.dump([fingIntervals, dotIntervals,rollIntervals], f)  
    
if __name__ == "__main__":
    main()
