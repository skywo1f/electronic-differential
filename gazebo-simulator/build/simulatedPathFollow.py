import argparse
import serial
import datetime
import re
import time
import threading
import numpy as np
import zmq

#script to drive a rover based on trying to keep the nose pointed towards the next waypoint

#connect to driver-to-gazebo.py interface
context = zmq.Context()
socket = context.socket(zmq.REP) 
socket.bind("tcp://*:5557")
encoding = 'utf-8'


#initialize wheels
leftWheelPwr = 0.0
rightWheelPwr = 0.0


#knobs
wheelStrength = 0.01                                            #constant to govern general wheel strength (set low because we multiply times delta theta)
angleTolerance = 10                                             #how far off course (angularly) is ok
distanceTolerance = .5                                          #size of radius at which we count as reaching the waypoint
forwardsPwm = 2.0                                               #another strenght constant for wheel speed
backwardsPwm= -2.0                                              #this one matters because it could be pos/neg or 16/14 pwm
#globals
thisLat = 0
thisLong = 0
latitude  = 0
longitude = 0
thetaCoord = 0
thisDirection = 0
lastTheta = 999
lastOmega = 0

#test motors
leftWheelPwr = forwardsPwm                                                      
rightWheelPwr = forwardsPwm
time.sleep(1.0)
leftWheelPwr = backwardsPwm
rightWheelPwr = backwardsPwm
time.sleep(1.0)
leftWheelPwr = 0.0
rightWheelPwr = 0.0


finished = True
distance = 0

def turnTable(g, s):                                                         #goal and start
    f = (int(g + 180))%360                                         #f for opposite
    #left is 1 right is 2                                           #calculate which direction I should be turning based on where I am and where I want to go
    if ( g < s and g < f and f > s):                                #because there is a sudden change at 360 degrees
        direction = 2
    if ( g > s and g > f and f > s):                                #there are 4 scenarios that need to be covered
        direction = 2
    if ( g < s and g < f and f < s):                                #depending on direction
        direction = 1
    if ( g > s and g > f and f < s):                                #and which side either vector is on
        direction = 1
    if ( g < s and g > f):
        direction = 2
    if ( g > s and g < f):
        direction = 1
    return direction

def deltaTheta(direction, thetaNow, thetaEnd) :                         #get absolute angle difference between where I am and
    if direction == 1:                                                  #where I want to go
        if thetaNow > thetaEnd:
            delta = thetaEnd + 360 - thetaNow
        else:
            delta = thetaEnd - thetaNow
    else:
        if thetaNow < thetaEnd:
            delta = 360 + thetaNow - thetaEnd
        else:
            delta = thetaNow - thetaEnd
    return delta

def getCurrentOmega(thisTheta):                                         #save my last angle, use it to find which direction I'm turning
    global lastTheta
    global lastOmega
    if lastTheta != thisTheta:
        diff = thisTheta - lastTheta
        lastTheta = thisTheta
        if diff < 100:
            if diff > 0:
                omega = 1
            else:
                omega = -1
        else :
            if diff > 0:
                omega = -1
            else:
                omega = 1
    else:
        omega = lastOmega
    lastOmega = omega
    return omega

def faceDirection(thetaE):                            # expected direction
    global leftWheelPwr
    global rightWheelPwr
    leftWheelPwr = 0
    rightWheelPwr = 0
    turnRight = 0
    turnLeft = 0
    thetaD = thetaE - thetaCoord                            #how far do we have to turn   
    thetaGStart = thetaCoord                                #where are we now
    reachedTol = 0
    thetaGEnd = thetaE%360                                  #convert to compass angle
    while (reachedTol == 0):                                #keep turning until we are within the angular tolerance
        thetaGNow = thetaCoord
#        print("facing " + str(thetaGNow))
#        print("need to face " + str(thetaGEnd))
        thetaGM = (float(thetaGNow - angleTolerance) )%360                               #theta gyro minus
        thetaGP = (float(thetaGNow + angleTolerance) )%360                               #theta gyro plus
        if ((thetaGNow > thetaGEnd) and (thetaGM > thetaGEnd) and (thetaGNow > thetaGM)) or ((thetaGNow < thetaGEnd) and (thetaGP < thetaGEnd) and (thetaGNow < thetaGP)):  #if angle outside tolerances
            direction = turnTable(thetaGNow,thetaGEnd)                                  #do I turn left or right?
            delta = deltaTheta(direction,thetaGNow,thetaGEnd)                           #calculate how much more turning will be required
#            power = 100*wheelStrength
            omega = getCurrentOmega(thetaGNow)                                          #calculate if I am currently turning clockwise or counterclockwise
            if direction == 1:                                                          #if I want to turn right (on the left side of the goal) 
                if omega < 0:                                                           # and I am turning right       
                    power = delta*wheelStrength/10                                         #start slowing down as I get there
                else :                                                                  #if I am turning left
                    power = 100*wheelStrength                                           #then really crank it cuz I overshot
            else :
                if omega < 0:
                    power = 100*wheelStrength
                else :
                    power = delta*wheelStrength/10

#            power = delta*wheelStrength
#            omega = getCurrentOmega(thetaGNow)
            if (direction == 1):                                                        #turn right
                leftWheelPwr = forwardsPwm*power
                rightWheelPwr = backwardsPwm*power
            else:                                                                               #2 -> turn left
                leftWheelPwr = backwardsPwm*power
                rightWheelPwr = forwardsPwm*power
        else:
            reachedTol = 1                                          
    leftWheelPwr = forwardsPwm                                                          #once we got there, full speed ahead
    rightWheelPwr = forwardsPwm

def vectorToAngle(vecX,vecY):                               #convert a vector to an angle
    if vecX == 0:
        vecX = 1e-15                                        #dont divide by 0
    angle = (180*np.arctan(vecY/vecX)/np.pi)%360            #arctan of ratios will give angle
    return angle                                            


def findBearing(x1,y1, x2,y2):                                                          #take in initial and ideal coordinates
    distance = np.sqrt(np.square(y1-y2) + np.square(x1 - x2))                           #euclidean distance
    diffX = x2 - x1
    diffY = y2 - y1
#    print(diffX,diffY)
    fwd_azimuth = vectorToAngle(diffX,diffY)                                            #euclidean angle
    if (diffX < 0):
        fwd_azimuth = (fwd_azimuth + 180) %360
    return fwd_azimuth, distance

def driver(finalLat,finalLong):                                                         #main driving loop
    global distance
    distance = distanceTolerance + 1
    thisDirection = 0

    global finished
    finished = False
    while distance > distanceTolerance:                                                 #while outside of radius around waypoint                                         
        thisY = longitude                                                               #translate lat/long to x/y for clarity
        thisX = latitude
        floatX = float(thisX)
        floatY = float(thisY)
#        print(str(floatX) + ' ' + str(floatY))
        desiredDirection,distance = findBearing(floatX,floatY,finalLat,finalLong)       #given where you are now and where you want to be, tell me direction in degrees (and distance)
        print("I am facing " + str(thetaCoord))
        print("I need to face " +str(desiredDirection))
        print("distance is " + str(distance))
        faceDirection(desiredDirection)                                                 #once I know the direction, turn vehicle to face it
        time.sleep(1)
    finished = True

def printer():                                                                          #location printing thread
    while True:                                                                         #separated out in case outputting causes lag
        thisLong = longitude
        thisLat = latitude
        floatLat = float(thisLat)/100
        floatLong = -1*float(thisLong)/100
        print(str(floatLat) + ' ' + str(floatLong))
        print(distance)

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
        thetaCoord = (float(data[2]))
	

def driveLoop(finalLat,finalLong,firstRun):                                             #starts driving/locator threads and checks for completion
    
    if (firstRun == True):                                                              #on first run, activate the connection to the locator
        gpsChecker = threading.Thread(target=rpServer)  
        gpsChecker.daemon = True
        gpsChecker.start()

#        printChecker = threading.Thread(target=printer)
#        printChecker.daemon = True
#        printChecker.start()

    if(finished == True):                                                               #if the last run was finished, then start checking to see if I am there yet
        driveChecker = threading.Thread(target=driver,args = (finalLat,finalLong))      
        driveChecker.daemon = True
        driveChecker.start()
    while (finished == False):                                                          #finished is a global variable that will be checked until the driveChecker thread changes it
        time.sleep(0.01)




path = np.loadtxt('finalPath.txt')

dMult = 50000                                                           #convert world coordinates to simulated coordinates
#driveLoop(path[0],path[1],True)    #actual first step
driveLoop(0,0,True)    #for display purposes
print("got to zeroth waypoint")

for i in range(int(len(path)/2)):
#    print(np.sqrt((path[2*i]*100 - path[0]*100)*(path[2*i]*100 - path[0]*100) + (path[2*i + 1] - path[1]*100)*(path[2*i + 1] - path[1]*100))) 
    driveLoop(path[2*i]*dMult - path[0]*dMult,path[2*i + 1]*dMult - path[1]*dMult,False)
    print("got to waypoint")


