import argparse
import serial
import datetime
import re
import time
import threading
import numpy as np
import zmq
import math
#used for testing of the gazebo/driver interaction

#current test is to see how well a pid loop works in here
#connect to driver-to-gazebo.py interface
context = zmq.Context()
socket = context.socket(zmq.REP) 
socket.bind("tcp://*:5557")
encoding = 'utf-8'


#initialize wheels
leftWheelPwr = 0.0
rightWheelPwr = 0.0
LWP = 0.0
RWP = 0.0


#knobs
wheelStrength = 0.01                                            #constant to govern general wheel strength (set low because we multiply times delta theta)
angleTolerance = 10                                             #how far off course (angularly) is ok
distanceTolerance = 2                                          #size of radius at which we count as reaching the waypoint
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
    f = (int(g + 180))%360                                         #f for opposite, int-ing this might cause accuracy issues
    #left is 1 right is 2                                           #   Calculate which direction I should be turning based on where I am and where I want to go
    if ( g < s and g < f and f > s):                                #because there is a sudden change at 360 degrees
        direction = 1
    if ( g > s and g > f and f > s):                                #there are 4 scenarios that need to be covered
        direction = 1
    if ( g < s and g < f and f < s):                                #depending on direction
        direction = -1
    if ( g > s and g > f and f < s):                                #and which side either vector is on.
        direction = -1
    if ( g < s and g > f):
        direction = 1
    if ( g > s and g < f):
        direction = -1
    if(g == s):
        print('headed right direction')
        direction = 1
    return direction

#def vectorToAngle(vecX,vecY):                               #convert a vector to an angle
#    if vecX == 0:
#        vecX = 1e-15                                        #dont divide by 0
#    angle = (180*np.arctan(vecY/vecX)/np.pi)%360            #arctan of ratios will give angle
#    return angle                                            

def vectorToAngle(vecX,vecY):
    degrees = 180*math.atan2(vecX,vecY)/math.pi
    if degrees < 0:
        angle = 360+degrees
    else:
        angle = degrees
#    angle = (angle + 90)%360                                #reference frame starts at north instead of east
    angle = (angle)%360                                #reference frame starts at north instead of east

    return angle
        

def driver():
    global leftWheelPwr                                                             #   We will set wheel power and just 
    global rightWheelPwr                                                            #throw it into the wind.
    while True:
        thisY = longitude                                                               #translate lat/long to x/y for clarity
        thisX = latitude                                                                #just pull them down from global
        turnDir,turnStrength = turnDirection(numWp,thisX,thisY)
        turnStrength = turnStrength/10000000                                                #damper based on distance to line and angle
#        if turnDir > 0:
#            leftWheelPwr = forwardsPwm
#            rightWheelPwr = backwardsPwm
#        else:
#            leftWheelPwr = backwardsPwm
#            rightWheelPwr = forwardsPwm
        if turnDir > 0:
            leftWheelPwr = forwardsPwm + turnStrength
            rightWheelPwr = backwardsPwm - turnStrength
        else:
            leftWheelPwr = backwardsPwm -  turnStrength
            rightWheelPwr = forwardsPwm + turnStrength


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
        response = str(LWP) + " " + str(RWP)
        message = socket.recv()
        socket.send(response.encode(encoding))
        alphabet = message.decode(encoding)
        data = re.split(' ',alphabet)
        latitude =   float(data[0])   
        longitude = float(data[1])   
        thetaCoord = (-(float(data[2])) + 90)%360

def getEuclidean(x1,y1,x2,y2):                                                           #I should probably use this function more in this code                                                           
    dx = x1 - x2
    dy = y1 - y2
    D = math.sqrt(dx*dx + dy*dy)
    return D
    
def getPathPoint(i):
    global finished
    try:
        x = path[2*i]*dMult - path[0]*dMult
        y = path[2*i + 1]*dMult - path[1]*dMult                                     #currently based around simulated coordinates
    except:
        print("probably ran out of points")
        x = path[0]*dMult
        y = path[1]*dMult
        finished = true
    return x,y

def compassToRadian(angle):                                                         #   Radians like to be in the scale of -pi/2 to pi/2
    if angle < 180:                                                                 #instead of 0 to 360 like compass units, 
        theta = -math.pi*angle/180                                                  #also they go counter-clockwise instead of clockwise.
    else:
        theta = math.pi*(360 - angle)/180
    return theta

def radianToCompass(theta):
    if theta < 0:
        angle = -180*theta/math.pi
    else:
        angle = 180*(2*math.pi - theta)/math.pi
    return angle

def returnToCompassFrame(theta):
    degreesOffLine = radianToCompass(theta)                                     #convert radians to degrees
#    print("degrees off the line ",str(degreesOffLine))
    vX,vY = getWpVector(numWp)                                                  #vector form
    lineAngle = vectorToAngle(vX,vY)                                            #euclidean angle in compass units
#    print("angle of line  ", str(lineAngle))
#    print("angle of rover ", str(thetaCoord))
    compassFrameDegrees = (lineAngle + degreesOffLine)%360                      #add line angle to line difference
    return compassFrameDegrees


def getCriticalAngle(numWp,roverX,roverY):
    phi0 = angleFromLine(numWp)                                                         #how far away am I looking from the right direction (compass units)
    phi0Radian = compassToRadian(phi0)                                                  #convert compass units to radians (line reference frame)    
    print("radians from line ",str(phi0Radian))
    d0 = distanceToLine(numWp,roverX,roverY)                                            #distance from line
#    print("d0",str(d0))
    x0 = d0 - turnR*math.sin(phi0Radian + math.pi/2)                                    #distance of the center of the turning circle from the line
#    print("x0 ",str(x0))
    ratio = 0.5-x0/(2*turnR)
    if phi0Radian > math.pi/2:                                                                       #if things go very wrong, just come back in a straight line
        criticalPhi = math.pi/2
        print('outside of angular range')
    elif phi0Radian < -math.pi/2:
        criticalPhi = -math.pi/2
        print('outside of angular range')
    else:
        criticalPhi = math.asin(ratio) - math.pi/2                                       #so now I have the critical angle with respect to the direction of ideal travel
    if d0 > 2*turnR:
        print('too far from line')
#    print(criticalPhi)
    criticalPhiCompass = returnToCompassFrame(criticalPhi)                              #convert to compass frame from line frame
    return criticalPhiCompass

def turnDirection(numWp,roverX,roverY):
    criticalPhi = getCriticalAngle(numWp,roverX,roverY)
    currentPhi = thetaCoord
    time.sleep(0.1)
#    print(roverX,roverY)
    print(criticalPhi)
    print(numWp)
    diffPhi = criticalPhi - currentPhi
    lineSide = findSide(numWp,roverX,roverY)
    lineX,lineY = getWpVector(numWp)
    lineDistance = distanceToLine(numWp,roverX,roverY)
    lineDirection = vectorToAngle(lineX,lineY)                                               #line direction in compass units
    diffDirection = abs(lineDirection - currentPhi)
#    direction = turnTable(currentPhi,lineDirection)                                          #which direction is the angle (accounting for disconnect at theta = 0)
    turnStrength = lineDistance
#    diffPhi =  -1.0*diffPhi    
#    direction = -1*direction                                                                #default 
#    lineSide = -lineSide
#    print("criticalPhi - currentPhi ", str(diffPhi))
    if lineSide > 0 :
        if (diffDirection < 90) or (diffDirection > 270):
            if diffPhi > 0:
                turnDir = 1
            else:
                turnDir = -1
        else:
            if diffPhi > 0:
                turnDir = -1
            else:
                turnDir = 1
    else:
        if (diffDirection < 90) or (diffDirection > 270):
            if diffPhi > 0:
                turnDir = -1
            else :
                turnDir = 1
        else :
            if diffPhi > 0:
                turnDir = 1
            else :
                turnDir = -1
    return turnDir, turnStrength
    
def getWpVector(numWp):                             #get the direction from this wp to next
    x1,y1 = getPathPoint(numWp)
    x2,y2 = getPathPoint(numWp+1)
    xComp = x2 - x1
    yComp = y2 - y1
    length = np.sqrt(np.square(xComp) + np.square(yComp))
    return xComp/length,yComp/length                            #normalize


def angleFromLine(numWp):
    vX,vY = getWpVector(numWp)                                                  #vector form
    lineAngle = vectorToAngle(vX,vY)                                            #euclidean angle in compass units
    roverAngle = thetaCoord                                                             #pull down from global
    theta = (lineAngle - roverAngle)%360                                              #get difference in compass units
    return theta                                                                #return difference in compass units
    
def distanceToLine(numWp,roverX,roverY):            #get distance from a point to a line (how far I am off the track)
    x1,y1 = getPathPoint(numWp)
    x2,y2 = getPathPoint(numWp+1)
    dLine = np.abs((y2 - y1)*roverX - (x2 - x1)*roverY + x2*y1 - y2*x1)/np.sqrt(np.square(y2-y1) + np.square(x2-x1))
    return dLine

def findSide(numWp,roverX,roverY):              #what side of the line am I on (which direction will I have to turn to get back)
    x1,y1 = getPathPoint(numWp)
    x2,y2 = getPathPoint(numWp+1)
    side = np.sign((x2 - x1) * (roverY - y1) - (y2 - y1) * (roverX - x1))
    return side

def updateClosestWp():                                                           #the next point becomes this point when it is closer 
    global numWp
    lastI = 0                                                                    #last index
    nextI = lastI + 1                                                         #next index
    lastD = 9999                                                                 #distance to last point
    NextD = 9999                                                                 #distance to next point
    global finished
    finished = False
    while(True):                                                                    #constantly be checking to see if the next point is closer than the current one
        thisY = longitude                                                               #translate lat/long to x/y for clarity
        thisX = latitude
        lX,lY = getPathPoint(lastI)
        lastD = getEuclidean(thisX,thisY,lX,lY)
        nX,nY = getPathPoint(nextI)
        nextD = getEuclidean(thisX,thisY,nX,nY)
#        if nextD < lastD :                                                         #back of vector line is whoever is closest
        if nextD < distanceTolerance :                                                          #back of vector line doesnt change until I am a turn radius away
            lastI = lastI + 1
            nextI = nextI + 1
        numWp = lastI
#        print(numWp) 
#        print("at       ",str(thisX),str(thisY))
#        print("going to ",str(nX),str(nY))
#        print(finished)
def driveLoop(finalLat,finalLong,firstRun):                                             #starts driving/locator threads and checks for completion
    
    if (firstRun == True):                                                              #on first run, activate the connection to the locator
        gpsChecker = threading.Thread(target=rpServer)  
        gpsChecker.daemon = True
        gpsChecker.start()

    if(finished == True):                                                               #if the last run was finished, then start checking to see if I am there yet
        pointsChecker = threading.Thread(target=updateClosestWp)
        pointsChecker.daemon = True
        pointsChecker.start()

#        printChecker = threading.Thread(target=printer)
#        printChecker.daemon = True
#        printChecker.start()

        driveChecker = threading.Thread(target=driver)      
        driveChecker.daemon = True
        driveChecker.start()
    while (finished == False):                                                          #finished is a global variable that will be checked until the driveChecker thread changes it
        time.sleep(0.01)


def simpleDrive():
    global rightWheelPwr
    global leftWheelPwr
    while True:
        rightWheelPwr = 2
        leftWheelPwr = 2
        time.sleep(5)
        rightWheelPwr = 0
        leftWheelPwr = 0
        time.sleep(5)


def makeTurns(t1,t2,t3):      
    global RWP,LWP
    FF = 2                                          #forward fast
    FS = 1.5                                          #forward slow
    time.sleep(10)
    RWP = FS
    LWP = FF
    time.sleep(t1)
    RWP = 0
    LWP = -FF
    time.sleep(0.5)
    RWP = 0
    LWP = 0
    time.sleep(10)
    RWP = -FF
    LWP = -FS
    time.sleep(t2)
    RWP = FF
    LWP = 0
    time.sleep(0.5)    
    RWP = 0
    LWP = 0
    time.sleep(10)
    LWP = FF
    RWP = FS
    time.sleep(t3)
    LWP = -FF
    RWP = 
    time.sleep(0.5)
    WP = 0
    RWP = 0
    time.sleep(10)

def threePointTurn():
    t1 = 5
    t2 = 10
    diff=0.1
    nIter = 100
    global RWP
    global LWP
    for i in range(nIter):
        tm = t1 - diff
        x1 = latitude 
        y1 = latitude
        makeTurns(tm,t2,tm)
        makeTurns(tm,t2,tm)
        x2 = latitude
        y2 = latitude
        distance1m = getEuclidean(x1,y1,x2,y2)
    
        tp = t1 + diff
        x1 = latitude
        y1 = latitude
        makeTurns(tp,t2,tp)
        makeTurns(tp,t2,tp)
        x2 = latitude
        y2 = latitude
        distance1p = getEuclidean(x1,y1,x2,y2)
    
        tp = t1 
        x1 = latitude
        y1 = latitude
        makeTurns(tp,t2,tp)
        makeTurns(tp,t2,tp)
        x2 = latitude
        y2 = latitude
        distance = getEuclidean(x1,y1,x2,y2)

        if distance1p < distance:
            t1 = tp
            print("min distance is ",str(distance1p))
        elif distance1m < distance:
            t1 = tm
            print("min distance is ",str(distance1m))
        else: 
            print("min distance is ",str(distance))
            diff = diff/2
        print("t1 is ",str(t1))
        print("diff is ",str(diff))
        



global path
path = np.loadtxt('finalPath.txt')

dMult = 30000                                                           #convert world coordinates to simulated coordinates
#driveLoop(0,0,True)    #for display purposes




gpsChecker = threading.Thread(target=rpServer)
gpsChecker.daemon = True
gpsChecker.start()


simpleDriveChecker = threading.Thread(target=threePointTurn)
simpleDriveChecker.daemon = True
simpleDriveChecker.start()

while True:
    time.sleep(5)
