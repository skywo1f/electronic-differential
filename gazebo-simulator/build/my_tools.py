


import argparse
import serial
import datetime
import re
import time
import threading
import numpy as np
import zmq
import math




def findBearing(x1,y1, x2,y2):                                                          #take in initial and ideal coordinates
    distance = np.sqrt(np.square(y1-y2) + np.square(x1 - x2))                           #euclidean distance
    diffX = x2 - x1
    diffY = y2 - y1
#    print(diffX,diffY)
    fwd_azimuth = vectorToAngle(diffX,diffY)                                            #euclidean angle
    if (diffX < 0):
        fwd_azimuth = (fwd_azimuth + 180) %360
    return fwd_azimuth, distance



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



def turnTable(g, s):                                                         #goal and start (backwards at the moment)
    f = (int(g + 180))%360                                         #f for opposite, int-ing this might cause accuracy issues
    #left is 1 right is -1                                           #   Calculate which direction I should be turning based on where I am and where I want to go
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
    if (abs(g - s) == 180):                                         #if facing complete opposite direction
        direction = -1                                               #then just pick a direction

    return direction

def vectorToAngle(vecX,vecY):
    degrees = 180*math.atan2(vecX,vecY)/math.pi
    if degrees < 0:
        angle = 360+degrees
    else:
        angle = degrees
#    angle = (angle + 90)%360                                #reference frame starts at north instead of east
    angle = (angle)%360                                #reference frame starts at north instead of east

    return angle
   

def getEuclidean(x1,y1,x2,y2):                                                           #I should probably use this function more in this code
    dx = x1 - x2
    dy = y1 - y2
    D = math.sqrt(dx*dx + dy*dy)
    return D

def getPathPoint(path,i):
    dMult = 30000                                                           #convert world coordinates to simulated coordinates
    try:
        x = path[2*i]*dMult - path[0]*dMult
        y = path[2*i + 1]*dMult - path[1]*dMult                                     #currently based around simulated coordinates
    except:
        print("probably ran out of points")
        x = path[0]*dMult
        y = path[1]*dMult
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

def getWpVector(path,numWp):                             #get the direction from this wp to next
    x1,y1 = getPathPoint(path,numWp)
    x2,y2 = getPathPoint(path,numWp+1)
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

def distanceToLine(path,numWp,roverX,roverY):            #get distance from a point to a line (how far I am off the track)
    x1,y1 = getPathPoint(path,numWp)
    x2,y2 = getPathPoint(path,numWp+1)
    dLine = np.abs((y2 - y1)*roverX - (x2 - x1)*roverY + x2*y1 - y2*x1)/np.sqrt(np.square(y2-y1) + np.square(x2-x1))
    return dLine

def findSide(path,numWp,roverX,roverY):              #what side of the line am I on (which direction will I have to turn to get back)
    x1,y1 = getPathPoint(path,numWp)
    x2,y2 = getPathPoint(path,numWp+1)
    side = np.sign((x2 - x1) * (roverY - y1) - (y2 - y1) * (roverX - x1))
    return side

