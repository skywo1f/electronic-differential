import argparse
import serial
import datetime
import re
import time
import threading
import numpy as np
import zmq
import math
from my_tools import *
from q_tools import *
#meant to work in conjunction with my_tools.py
#script meant to test the parameter space of turning and find optimal turning strategy


#connect to driver-to-gazebo.py interface
context = zmq.Context()
socket = context.socket(zmq.REP) 
socket.bind("tcp://*:5557")
encoding = 'utf-8'


#initialize wheels
leftWheelPwr = 0.0
rightWheelPwr = 0.0


#knobs
angleTolerance = 1                                             #how far off course (angularly) is ok
distanceTolerance = 1                                          #size of radius at which we count as reaching the waypoint
forwardsPwm = 3.0                                               #another strength constant for wheel speed
#globals                                                        # I may want to add "_global" at the end of these
thetaCoord = 999
thisDirection = 0
lastTheta = 999
lastOmega = 0
numWp = 0
turnR = 1.25                                                     #turn radius (should probably be calculated based on w1, w2, and distance between wheels)
#test motors

distance = 0
lastAngle_g = 0
lastTime_g = 0

def getOmega():
    thetaChange = thetaCoord - lastAngle_g
    timeChange = time.time() - lastTime_g
    omega = thetaChange/timeChange

def observe(observation,thetaEnd):
    global lastAngle_g
    global lastTime_g
    thetaNow = thetaCoord
    direction = turnTable(thetaNow,thetaEnd)
    thetaChange = thetaNow - lastAngle_g
    timeChange = time.time() - lastTime_g
    lastTime_g = time.time()
    lastAngle_g = thetaNow
    observation[0] = thetaChange/(timeChange*360)                         #normalizing at a maximum turn rate of 360 degrees per second (will probably need tuning when max turn rate is found)             
    observation[1] = leftWheelPwr/forwardsPwm                           #rightWheelPwr = -leftWheelPwr, forwardsPwm = max power
    observation[2] = deltaTheta(direction, thetaNow, thetaEnd)/180.0    #180 degrees is the max delta theta
    print(observation)
    return observation

def applyAct(act_index):
    stepSize = 0.1
    global leftWheelPwr
    global rightWheelPwr
    if act_index == 0:
        act = 1
    else:
        act = -1    
    leftWheelPwr = leftWheelPwr + act*stepSize
    rightWheelPwr = rightWheelPwr -act*stepSize
    if leftWheelPwr > forwardsPwm - stepSize:
        leftWheelPwr = 3.0 - stepSize
        rightWheelPwr = -3.0 + stepSize
    elif leftWheelPwr < -forwardsPwm + stepSize:
        leftWheelPwr = -3.0 + stepSize
        rightWheelPwr = 3.0 - stepSize
    time.sleep(0.01)                                                    #take the "step" by waiting 

def scorePoints(thetaEnd):
    reward = 0
    direction = turnTable(thetaCoord,thetaEnd)
    lastD = deltaTheta(direction,lastAngle_g,thetaEnd)                    #how far did I have to turn last step
    thisD = deltaTheta(direction,thetaCoord,thetaEnd)                    #how far do I still have to turn now
    if thisD < lastD :
        reward += reward
    else:
        reward -= reward
    return reward

def playOneRound(bins,Q,eps,thetaEnd):
    observation = np.zeros(3)
    GAMMA = 0.9
    ALPHA = 0.01
    observation = observe(observation,thetaEnd)
    state = get_state_as_string(assign_bins(observation, bins))                #grab a state
    total_reward = 0                                                        #initialize reward for this game
    cnt = 0
    stepSize = 0.1
    done = False
    while not done:                                                            #while playing this game
        cnt += 1        
        if np.random.uniform() < eps:                                        #at first pick out actions at random
            if np.random.uniform() < 0.5:
                act_index = 0
            else:
                act_index = 1
        else:            
            score, act_index = max_dict(Q,state)                                    #otherwise find action for maximum future reward
                
        applyAct(act_index)                                                        #apply the action
        reward = scorePoints(thetaEnd)                                                #get points for going the right way
        direction = turnTable(thetaCoord,thetaEnd)
        angleRemain = deltaTheta(direction,thetaCoord,thetaEnd)
        if angleRemain < angleTolerance:                                            #if I get to the right place
            done = True                                                        #flag done
            reward_acceleration = -abs(leftWheelPwr)*100                    #lose points if lap-bars arent zero
            reward_velocity = -abs(getOmega())*10                            #lose points for overshooting
            reward_time = -cnt/2                                                #lose points for every timestep this took
            reward += reward_acceleration + reward_velocity + reward_time   #tally up final score

        observation = observe(observation,thetaEnd)                            #get the new observation
        state_new = get_state_as_string(assign_bins(observation, bins))        #binitize observation into state
        new_score, new_act_index = max_dict(Q,state_new)                                #find if the new state was a good one
        Q[int(state)][act_index] += ALPHA*(reward + GAMMA*new_score - Q[int(state)][act_index])    #average out future state with current one
        state, act_index = state_new, new_act_index                                            #update the current state (not sure why I care about the new action)
        
    return Q, reward, cnt

def resetSim():
    global leftWheelPwr
    global rightWheelPwr 
    leftWheelPwr = 0
    rightWheelPwr = 0
    time.sleep(10)

def driver(numRounds):
    bins = create_bins()                                                    #binitize my problem
    Q = initialize_Q()
    numRounds = 10
    length = []                                                                #store length of each game
    reward = []                                                                #store reward for each game
    print('waiting for gazebo')
    while thetaCoord == 999:                                                #wait til receiving good info before starting
        time.sleep(1)
    for i in range(numRounds):
        thetaEnd = (thetaCoord + 45)%360
        eps = i/numRounds                            #variable for leaning towards random choices or going with the current best (explore/exploit)
        Q, episode_reward, episode_length = playOneRound(bins,Q,eps,thetaEnd)
        length.append(episode_length)                                        #save progress
        reward.append(episode_reward)
        resetSim()        

    print(reward,length)

def rpServer():                                                                         #constantly be communicating with locator
    global latitude                                                                     #whether it is the arduino giving it gps coordinates from the ublox
    global longitude                                                                    #or simulated coordinates from gazebo
    global thetaCoord
    while True:
        response = str(leftWheelPwr) + " " + str(rightWheelPwr)
        message = socket.recv()
        socket.send(response.encode(encoding))
        alphabet = message.decode(encoding)
        data = re.split(' ',alphabet)
        latitude =   float(data[0])   
        longitude = float(data[1])   
        thetaCoord = (-(float(data[2])) + 90)%360

def driveLoop():                                             #starts driving/locator threads and checks for completion
    numRounds = 100
    gpsChecker = threading.Thread(target=rpServer)  
    gpsChecker.daemon = True
    gpsChecker.start()

    driveChecker = threading.Thread(target=driver,args=(numRounds, ))      
    driveChecker.daemon = True
    driveChecker.start()
    while True:                                                          
        time.sleep(5)

driveLoop()


