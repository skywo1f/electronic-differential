import argparse
import serial
#import pynmea2

import datetime
import re
import time
import threading
import numpy as np
import zmq
import math
from my_tools import *
from simulate_model import create_model_parameters
from kalman_filter import KalmanFilter
from sensor import *

#script to simulate path driving, At each waypoint it aims itself at the next waypoint using the compass. 
#meant to work in conjunction with my_tools.py
#2 step driving process.
#when at a waypoint, it aims itself at the next waypoint by slowly turning in the direction of the goal until the compass tells it to stop. 
#once it has turned that much, it should obstensively be within 5-10 degrees of where it needs to aim. 
#it then starts driving in that direction, making small changes to the wheel speeds in order to keep aiming in the right direction.
#wheel speed changes are proportional to how far off the mower is and how far from the goal it is. Long distance means only small changes are required, wide angles means larger changes are required. 
#the first main change from sp8 are that instead of using driver-to-sp8 to communicate with the wheels and run the pid code, it uses the ST device.
#the second main change is that it combines gps, gyro, and compass together with a simple model (theta = omega*time + theta_0) to estimate its orientation


#small additions to test sensor class and simulator

#known bugs:
#turn-table sometimes turns the wrong way (>90 degrees, when crossing the 0 line)
#num_wp seems to be skipping the first waypoint

#globals
leftWheelPwr_g = 0.0                            #angle to set left wheel to
rightWheelPwr_g = 0.0                           #angle to set right wheel to
thetaCoord_g = 0                                #heading data from compass
latitude_g  = 0                                 #latitude data from gps
longitude_g = 0                                 #longitude data from gps
omega_g = 0                                     #turning rate data from gyro
numWp_g = 0                                     #what waypoint am I heading towards
kAngle_g = 0                                    #kalman smoothed heading
gpsTheta_g = 0                                  #omegaheading data from gps history
gpsDistance_g = 1.0

#knobs
angleTolerance = 20.0                                            #how far off course (angularly) is ok (degrees)
distanceTolerance = 1.0                                          #size of radius at which we count as reaching the waypoint (meters)

#define
encoding = 'utf-8'

def convertToDecimal(coordinate):
    coord = coordinate/100
    decimal = coord - math.floor(coord)
    decimal = 100*decimal/60
    return math.floor(coord) + decimal

def getCompass():
    global thetaCoord_g
    gyroContext = zmq.Context()
    gyroSocket = gyroContext.socket(zmq.REP) 
    gyroSocket.bind("tcp://*:5582")
    lastAngle = 0.1
    histNum = 10
    gpsCounter = 0
    lonHist = np.zeros(histNum)
    latHist = np.zeros(histNum)
    times = np.zeros(histNum)
    times = startTime(times,histNum)
    while True:
        message = b"0"
        message = gyroSocket.recv()
        gyroSocket.send(b"thanks")
#        print(message)
        angle = float(message.decode(encoding))
        if (angle == 0.0):
            angle = lastAngle
        deviation = angle_interpolate(angle)
#        thetaCoord_g = angle + deviation 
        thetaCoord_g = angle 
        lastAngle = angle 

def getImu():
    global omega_g
    imuContext = zmq.Context()
    imuSocket = imuContext.socket(zmq.REP)
    imuSocket.bind("tcp://*:5583")
    while True:
        message = b"0"
        message = imuSocket.recv()
        imuSocket.send(b"thanks")
        omega_g = -float(message.decode(encoding))

def angle_interpolate(angle):
#    deviations = [0.95, 6.75, 8.7, 9.3, 5.35, -6.1, -15.5, -9.5, 0.95]                    #deviations
    deviations = [0.95, 6.75, 13, 9.3, 5.35, -6.1, -15.5, -9.5, 0.95]                    #deviations
    angles     = [   0,   45,  90, 135,  180,  225,   270,  315,  360]                    #corresponding angles
    counter = -1
    for value in angles:                                                          #find where my value lies and linearly interpolate
        counter = counter + 1
        if value > angle :
            counter = counter - 1    
            break

    a = angles[counter + 1] - angles[counter]                           #get the spacing between reference values
    b = angle - angles[counter]                                              #get the difference between
    c = b/a                                                                                 #is the value closer to ffV[counter] or ffV[counter + 1]
    deviation = deviations[counter + 1]*(c) + deviations[counter]*(1-c)                           #use that fraction to weight the averages
    return deviation

def kalman_angle():
    global kAngle_g
    T = 0.01
    lamC = 1.0                                          #variance for compass
    lamGPS = 1.0                                          #variance for gps bearing
    lamGyro = 1.0					#variance for gyroscope
    sig = 1.0						#variance for model
    sig2 = sig**2
    lamC2 = lamC**2
    lamGPS2 = lamGPS**2
    lamGyro2 = lamGyro**2
    (A, H, Q, R) = create_model_parameters(T,  sig2, lamC2)                                    #initialize evolution, measurement, model error, and measurement error
# initial state
    x = np.array([0, 0.1])                                              #starting velocity and position
    P = 0 * np.eye(2)                                                           #starting probability 100%
    kalman_filter = KalmanFilter(A, H, Q, R, x, P)                              #create the weights filter
    lastTime = time.time()
    startTime = lastTime
    lastOmega = 0
    lastAngle = 0
    thetaCoord_l = 0                                                  #local variable to prevent race conditions
    lastBearing = 0
    gpsTrust = 1.0
    while True:
        while (omega_g == lastOmega or thetaCoord_g == lastAngle):               #check if there were updates to the compass
            time.sleep(0.01)                                            #if no updates then wait a bit
        thetaCoord_l = thetaCoord_g
        lastAngle = thetaCoord_l
        if (kalman_filter._x[0] - thetaCoord_l) > 180:                      #make sure the filter knows that we crossed the pole
           kalman_filter._x[0] = kalman_filter._x[0] - 360
        elif (thetaCoord_l - kalman_filter._x[0]) > 180:
           kalman_filter._x[0] = kalman_filter._x[0] + 360
        kalman_filter.predict()                                                 #evolve the state and the error
        kalman_filter.update((thetaCoord_l,omega_g),(lamC2,lamGyro2))                         #load in 2 measurement values
        (x, P) = kalman_filter.get_state()                                      #return state
        dt = time.time() - lastTime                                             #calculate dt
        lastTime = time.time()                                              
        kalman_filter.update_time(dt,sig2)                                      #Make sure the filter knows how much time occured in between steps
        kAngle_g = x[0]%360                                                     #because the model portion is estimating based on that.

#        while (omega_g == lastOmega or gpsTheta_g == lastBearing):               #check if there were updates to the gps bearing
#            time.sleep(0.01)                                            #if no updates then wait a bit
        gpsTheta_l = gpsTheta_g
        lastBearing = gpsTheta_l
        if (kalman_filter._x[0] - gpsTheta_l) > 180:                      #make sure the filter knows that we crossed the pole
           kalman_filter._x[0] = kalman_filter._x[0] - 360
        elif (gpsTheta_l - kalman_filter._x[0]) > 180:
           kalman_filter._x[0] = kalman_filter._x[0] + 360
        lamGPS2 = gpsTrust/gpsDistance_g
        kalman_filter.predict()                                                 #evolve the state and the error
        kalman_filter.update((gpsTheta_l,omega_g),(lamGPS2,lamGyro2))                         #load in 2 measurement values
        (x, P) = kalman_filter.get_state()                                      #return state
        dt = time.time() - lastTime                                             #calculate dt
        lastTime = time.time()
        kalman_filter.update_time(dt,sig2)                                      #Make sure the filter knows how much time occured in between steps
        kAngle_g = x[0]%360                                                     #because the model portion is estimating based on that.
#        print(thetaCoord_l, gpsTheta_g,kAngle_g,latitude_g,longitude_g)

def calcGpsHeading():
    global gpsTheta_g
    global gpsDistance_g
    lastLon = longitude_g
    lastLat = latitude_g
    while True:
        if lastLon != longitude_g or lastLat != latitude_g:
            gpsTheta_g,gpsDistance_g = findBearing(lastLat,lastLon,latitude_g,longitude_g)
            lastLon = longitude_g
            lastLat = latitude_g
            time.sleep(0.001)
            

def getGPS():
	gpsContext = zmq.Context()
	gpsSocket = gpsContext.socket(zmq.REP)
	gpsSocket.bind("tcp://*:5580")
	global latitude_g
	global longitude_g
	histNum = 10
	gpsCounter = 0
	lonHist = np.zeros(histNum)
	latHist = np.zeros(histNum)
	times = np.zeros(histNum)
	times = startTime(times,histNum)
	while True:
		message = b"0"
		message = gpsSocket.recv()
		gpsSocket.send(b"thanks")
		decoded = message.decode(encoding)		
		data = re.split(' ',decoded)
		preLat = float(data[0])
		preLong = float(data[1])
		latitude_g = preLat
		longitude_g = preLong


#		thisLon = -convertToDecimal(preLong)
#		thisLat  = convertToDecimal(preLat)
#		print(thisLat,thisLon)
#		latitude_g = thisLat
#		longitude_g = thisLon




def driver(path):
    time.sleep(2)
    while True:
        lastWp = numWp_g
        thisY = longitude_g                                                              #translate lat/long to x/y for clarity
        thisX = latitude_g
        wpX,wpY = getPathPoint(path,numWp_g + 1)
        thetaGoal,distance = findBearing(thisX,thisY,wpX,wpY)
#    print(thetaCoord,thetaGoal,distance)
        compassOrientation(thetaGoal)                                                   #thetaGoal is the compass angle
#    print("finished turning")
        time.sleep(0.5)
        gpsDrive(wpX,wpY)
        time.sleep(0.5)


def compassOrientation(thetaGoal):
	global leftWheelPwr_g                                                            
	global rightWheelPwr_g 
	lastE = 0
	sumE = 0
	currentE = 0
	changeV = 0
	turnRate = 5
#	turnRate = 2
	delay = 0.5
	turnTime = 2				#twice as long as it takes to get to max turn speed
	time.sleep(delay)
	start_time = time.time()
	run_time = time.time() - start_time
	turnDone = 0
	while(turnDone == 0):
#		print(thetaCoord_g,thetaGoal)
		direction = turnTable(thetaCoord_g,thetaGoal)                                           #get direction to turn	
		delta = deltaTheta(direction,thetaCoord_g,thetaGoal)					#get distance left to turn
		if (delta < angleTolerance):
			turnDone = 1
		run_time = time.time() - start_time
		if run_time > 0 and run_time < turnTime/2:
			desiredVal = turnRate*(run_time)/(turnTime/2)	
		if run_time > turnTime/2:
			desiredVal = turnRate
		if direction > 0:
			leftWheelPwr_g = -desiredVal 
			rightWheelPwr_g = desiredVal 
		else:
			leftWheelPwr_g =  desiredVal 
			rightWheelPwr_g = -desiredVal 
	leftWheelPwr_g =  0
	rightWheelPwr_g = 0
	time.sleep(delay)
#	print(delta)
	turnDone = 0
	turnRate = turnRate/2							#this seems odd
	while(turnDone == 0):
		direction = turnTable(thetaCoord_g,thetaGoal)							#get direction to turn	
		delta = deltaTheta(direction,thetaCoord_g,thetaGoal)					#get distance left to turn
		if (delta < angleTolerance/2):
			turnDone = 1
		run_time = time.time() - start_time
		if run_time > 0 and run_time < turnTime/2:
			desiredVal = turnRate*(run_time)/(turnTime/2)	
		if run_time > turnTime/2:
			desiredVal = turnRate
		if direction > 0:
			leftWheelPwr_g = -desiredVal + 2
			rightWheelPwr_g = desiredVal  + 2
		else:
			leftWheelPwr_g =  desiredVal + 2
			rightWheelPwr_g = -desiredVal + 2
	leftWheelPwr_g =  0
	rightWheelPwr_g = 0
	time.sleep(delay)
#	print(delta)


def gpsDrive(wpX,wpY):
	delay = 2	
	global leftWheelPwr_g
	global rightWheelPwr_g
	fwdSpeed = 60								#tested ok up to 60
	fullSpeed = fwdSpeed
#	AEK = 0.01							#default angle error constant
	AEK = 0.025
#	DEK = 0.01							#default distance aggresiveness constant
	DEK = 0.001
#	SEK = 0.0002							#summed error constant
	SEK = 0.0003
	thisY = longitude_g                                                               #translate lat/long to x/y for clarity
	thisX = latitude_g
	thetaGoal,distance = findBearing(thisX,thisY,wpX,wpY) 
	leftWheelPwr_g = fwdSpeed
	rightWheelPwr_g = fwdSpeed
	lastDistance = distance 
	thetaLast = thetaCoord_g
	offset = 0.0
	sumE = 0
	thetaSmooth = 0
	omegaSmooth = 0
	startT = time.time()
#	omegaStrength = 2
	omegaStrength = 0.4
	K = 5
	C = fullSpeed - K*(5-fullSpeed)/(1-K)
	while distance > distanceTolerance:
		thetaCoord_l = kAngle_g
		fwdSpeed = fullSpeed
		if distance < distanceTolerance*K:			#K was 2
			fwdSpeed = distance*(5 - fullSpeed)/(distanceTolerance-K*distanceTolerance) + C 
#			fwdSpeed = distance*(fullSpeed-5)/distanceTolerance + 10 - fullSpeed
		if ((time.time() - startT) < 5):
			startSpeed = (time.time() - startT)*fullSpeed/5
			if startSpeed < fwdSpeed:
				fwdSpeed = startSpeed
		thisY = longitude_g
		thisX = latitude_g
		thetaGoal,distance = findBearing(thisX,thisY,wpX,wpY)
		direction = turnTable(thetaCoord_l,thetaGoal) 	#get direction to turn	
		thetaSmooth = 0.0001*thetaSmooth + 0.9999*thetaCoord_g
		thetaLast = thetaSmooth		
		delta = deltaTheta(direction,thetaCoord_l,thetaGoal)					#get distance left to turn		
		overshot = 0
		sumE += abs(delta)
		if abs(delta) < 1:					#default 4
			sumE = 0
		if sumE*SEK > 20/4:					#15 worked for slow speeds
			sumE = 20/(SEK*4)
		omegaSmooth = 0.5*omegaSmooth + 0.5*omega_g
		if abs(delta) < 10 and abs(omegaSmooth) > 0:
			overshot = abs(omegaSmooth)*omegaStrength

		turnStrength = DEK/distance + AEK*abs(delta) + sumE*SEK
#		print(latitude_g,longitude_g)
#		print(thetaCoord_g,kAngle_g)
		print(distance,delta,direction,overshot,sumE*SEK)
		if omegaSmooth > 0:
			if direction > 0:
				leftWheelPwr_g = (fwdSpeed - turnStrength + overshot)
				rightWheelPwr_g = (fwdSpeed + turnStrength - overshot)
			else:
				leftWheelPwr_g =  (fwdSpeed + turnStrength + overshot)
				rightWheelPwr_g = (fwdSpeed - turnStrength - overshot)
		else:
			if direction > 0:
				leftWheelPwr_g = (fwdSpeed - turnStrength - overshot)
				rightWheelPwr_g = (fwdSpeed + turnStrength + overshot)
			else:
				leftWheelPwr_g =  (fwdSpeed + turnStrength - overshot)
				rightWheelPwr_g = (fwdSpeed - turnStrength + overshot)


		lastDistance = distance
		
#		print(rightWheelPwr_g,leftWheelPwr_g,thetaCoord_g)
#		print(distance)
	print('reached the waypoint, pausing')	
	leftWheelPwr_g = 0
	rightWheelPwr_g = 0
	time.sleep(2)


'''
def pidServer1():                                                                         #send desired wheel speeds to pid
#connect to driver-to-pid.py interface
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5557")
    

    while True:
        response = str(leftWheelPwr_g)
        message = socket.recv()
        socket.send(response.encode(encoding))

def pidServer2():                                                                         #send desired wheel speeds to pid
#second wheel
    context2 = zmq.Context()
    socket2 = context2.socket(zmq.REP)
    socket2.bind("tcp://*:5558")

    while True:
        response = str(rightWheelPwr_g)
        message = socket2.recv()
        socket2.send(response.encode(encoding))
'''

def bothPid():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5581")
    while True:
        response = str(leftWheelPwr_g) + " " + str(rightWheelPwr_g)
        message = socket.recv()
        socket.send(response.encode(encoding))

def numWpUpdate(path):                                                                  #function which is constantly checking to see if numWp has been reached and needs to be updated
    global numWp_g                                                                    #important parameter to export
    numWp_g = 0
    lastD = 1e9                                                                       #initialize
    while(True):                                                                    #check waypoints
        thisY = longitude_g                                                           #standard long -> Y
        thisX = latitude_g                                                           #lat -> X (maybe should have been backwards
        nX,nY = getPathPoint(path,numWp_g + 1)                                                #get position of the next path point
        thetaHeading,distance = findBearing(thisX,thisY,nX,nY)
        numD = distance                                     #get distance to the next path point
#        print(thisX,thisY,nX,nY)
#        print(thetaHeading,distance)
        if numD < distanceTolerance:                                                #if I stopped getting closer to the wp, its time to update
            numWp_g = numWp_g + 1
        lastD = numD


def save_and_print():
#print leftwheelpwr,rightwheelpwr, latitude, longitude, theta, omega,numwp, time
    startTime = int(time.time())
    outfile = "output-" + str(startTime) + ".dat"
    fullData = [0,0,0,0,0,0,0,0]
    try:
        while True:
            dataRow = [leftWheelPwr_g,rightWheelPwr_g,latitude_g,longitude_g,thetaCoord_g,omega_g,numWp_g,kAngle_g,time.time()]
            fullData = np.vstack([fullData,dataRow])
            time.sleep(0.01)
    except:
        np.savetxt(outfile,fullData)



def driveLoop(path,finalLat,finalLong,firstRun):                                             #starts driving/locator threads and checks for completion
    
    if (firstRun == True):                                                              #on first run, activate the connection to the locator

        '''
        pidDriver1 = threading.Thread(target=pidServer1)  
        pidDriver1.daemon = True
        pidDriver1.start()

        pidDriver2 = threading.Thread(target=pidServer2)  
        pidDriver2.daemon = True
        pidDriver2.start()
        '''
        bothDriver = threading.Thread(target=bothPid)
        bothDriver.daemon = True
        bothDriver.start()

        pointsChecker = threading.Thread(target=numWpUpdate,args=(path, ))
        pointsChecker.daemon = True
        pointsChecker.start()

        gpsChecker = threading.Thread(target=getGPS)
        gpsChecker.daemon = True
        gpsChecker.start()

        gyroChecker = threading.Thread(target=getCompass)
        gyroChecker.daemon = True
        gyroChecker.start()

        imuChecker = threading.Thread(target=getImu)
        imuChecker.daemon = True
        imuChecker.start()

        driveChecker = threading.Thread(target=driver,args=(path,))      
        driveChecker.daemon = True
        driveChecker.start()

        gpsBearingChecker = threading.Thread(target=calcGpsHeading)
        gpsBearingChecker.daemon = True
        gpsBearingChecker.start()

        kalmanRunner = threading.Thread(target=kalman_angle)
        kalmanRunner.daemon = True
        kalmanRunner.start()

        simToCode = simulatedThrow("simulator",5580,5581,5582,5583,5584)                #initialize bridging algorithm for simulator
        simToCode.grabSpeeds()                                                      #grab speeds from driver and prepare to hand to simulator
        simToCode.sendValues()                                                      #take values from simulator and give them to driver
        simToCode.grabValues()                                                      #grab location data from simulator and prepare to give them to driver
        simToCode.convertValues()                                                   #convert from simulated coordinates to gps coordinates
    while True:                                                          
        time.sleep(5)




path = np.loadtxt('coords.txt')


#driveLoop(path[0],path[1],True)    #actual first step
driveLoop(path,0,0,True)    #for display purposes



