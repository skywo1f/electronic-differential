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
#script to simulate path driving, At each waypoint it aims itself at the next waypoint using the compass. 
#meant to work in conjunction with my_tools.py


#current issue is that the critical phi is not being calculated correctly, somewhere near getCriticalAngle

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
angleTolerance = 1                                             #how far off course (angularly) is ok
distanceTolerance = 1                                          #size of radius at which we count as reaching the waypoint
forwardsPwm = 3.0                                               #another strength constant for wheel speed
backwardsPwm= 2.0                                              #this one matters because it could be pos/neg or 16/14 pwm
#globals                                                        # I may want to add "_global" at the end of these
thisLat = 0                                                     #so that they are not confused/mixed up with 
thisLong = 0                                                    #regular variables.
latitude  = 0
longitude = 0
thetaCoord = 0
thisDirection = 0
lastTheta = 999
lastOmega = 0
numWp = 0
turnR = 1.25                                                     #turn radius (should probably be calculated based on w1, w2, and distance between wheels)
#test motors

distance = 0



     

def driver(path):
    while True:
        doubleOrientDrive(path)

def compassOrientation(thetaGoal):                                                       #use compass to rotate
    global leftWheelPwr                                                             #   We will set wheel power and just 
    global rightWheelPwr 
    leftWheelPwr = 0
    rightWheelPwr = 0
    time.sleep(5)
    kP = 0.01
    kD = 0.1
    thetaDiff = abs(thetaCoord - thetaGoal)
    lastTheta = thetaCoord
    while thetaDiff > angleTolerance:                                                   #pid loop
        direction = turnTable(thetaCoord,thetaGoal)                                     #which direction to turn
        delta = deltaTheta(direction,thetaCoord,thetaGoal)                           #calculate how much more turning will be required
        dTheta = thetaCoord - lastTheta
        lastTheta = thetaCoord
        dE = kP*delta - kD*abs(dTheta)
        turnStrength = dE
        if direction > 0:
            leftWheelPwr = turnStrength
            rightWheelPwr = -turnStrength
        else:
            leftWheelPwr =  -turnStrength
            rightWheelPwr = turnStrength
        thetaDiff = abs(thetaCoord - thetaGoal)
        time.sleep(0.01)
    leftWheelPwr = 0
    rightWheelPwr = 0
    time.sleep(5)

def gpsOrientation(thetaGoal):                                                                    #use gps and gyroscope(compass for now)
    global leftWheelPwr
    global rightWheelPwr
    x1 = latitude
    y1 = longitude 

    leftWheelPwr = forwardsPwm 
    rightWheelPwr = forwardsPwm

    time.sleep(2)
    leftWheelPwr = 0
    rightWheelPwr = 0
    time.sleep(2)
    x2 = latitude
    y2 = longitude
    thetaHeading,distance = findBearing(x1,y1,x2,y2)
    direction = turnTable(thetaHeading,thetaGoal)
    compassError = deltaTheta(direction,thetaHeading,thetaGoal)
    if direction > 0:                                                                            #need to check, might be backwards
        compassOrientation(thetaHeading - compassError)
    if direction < 0:
        compassOrientation(thetaHeading + compassError)
    leftWheelPwr = forwardsPwm
    rightWheelPwr = forwardsPwm

def doubleOrientDrive(path):
    global leftWheelPwr
    global rightWheelPwr
    lastWp = numWp
    thisY = longitude                                                               #translate lat/long to x/y for clarity
    thisX = latitude
    wpX,wpY = getPathPoint(path,numWp + 1)
    print(thisX,thisY,wpX,wpY)
    thetaGoal,distance = findBearing(thisX,thisY,wpX,wpY)
    compassOrientation(thetaGoal)                                                   #thetaGoal is the compass angle
 #   leftWheelPwr = forwardsPwm
 #   rightWheelPwr = forwardsPwm
    gpsOrientation(thetaGoal)
    while (lastWp == numWp):                                                        #if the waypoint hasn't changed, keep going
        time.sleep(0.01)



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


def numWpUpdate(path):                                                                  #function which is constantly checking to see if numWp has been reached and needs to be updated
    global numWp                                                                    #important parameter to export
    numWp = 0
    lastD = 1e9                                                                       #initialize
    while(True):                                                                    #check waypoints
        thisY = longitude                                                           #standard long -> Y
        thisX = latitude                                                            #lat -> X (maybe should have been backwards
        nX,nY = getPathPoint(path,numWp + 1)                                                #get position of the next path point
        numD = getEuclidean(thisX,thisY,nX,nY)                                     #get distance to the next path point
        if numD > lastD and numD < distanceTolerance:                                                #if I stopped getting closer to the wp, its time to update
            numWp = numWp + 1
        lastD = numD

def driveLoop(path,finalLat,finalLong,firstRun):                                             #starts driving/locator threads and checks for completion
    
    if (firstRun == True):                                                              #on first run, activate the connection to the locator
        gpsChecker = threading.Thread(target=rpServer)  
        gpsChecker.daemon = True
        gpsChecker.start()

        pointsChecker = threading.Thread(target=numWpUpdate,args=(path, ))
        pointsChecker.daemon = True
        pointsChecker.start()


        driveChecker = threading.Thread(target=driver,args=(path,))      
        driveChecker.daemon = True
        driveChecker.start()
    while True:                                                          
        time.sleep(5)




path = np.loadtxt('finalPath.txt')


#driveLoop(path[0],path[1],True)    #actual first step
driveLoop(path,0,0,True)    #for display purposes



