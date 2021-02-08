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
from simulate_model import create_model_parameters
from kalman_filter import KalmanFilter
from sensor import *
from runAndManage import *

encoding = 'utf-8'
'''
navigation class. Provides handles for user to easily write and test driving algorithm.
Example:
nav = navigation("sim")                                                             #initialize navigation to simulated universe
thisY = nav.longitude_g                                                             #read location values from sensor
nav.leftWheelPwr_g =  5                                                             #set wheel speed to some arbitrary value
'''

class navigation:

        @threaded
        def __init__(self,sim_or_real):
#globals
                self.leftWheelPwr_g = 0.0                            #angle to set left wheel to
                self.rightWheelPwr_g = 0.0                           #angle to set right wheel to
                self.thetaCoord_g = 0                                #heading data from compass
                self.latitude_g  = 0                                 #latitude data from gps
                self.longitude_g = 0                                 #longitude data from gps
                self.omega_g = 0                                     #turning rate data from gyro
                self.numWp_g = 0                                     #what waypoint am I heading towards
                self.kAngle_g = 0                                    #kalman smoothed heading
                self.gpsTheta_g = 0                                  #omegaheading data from gps history
                self.gpsDistance_g = 1.0                             #distance to next waypoint
                self.getImu()                           
                self.getCompass(sim_or_real)
                self.getGPS()
                self.kalman_angle()
                self.calcGpsHeading()
                self.bothPid()
                if sim_or_real == "sim":
                        simToCode = simulatedThrow("simulator",5580,5581,5582,5583,5584)                #initialize bridging algorithm for simulator
                        simToCode.grabSpeeds()                                                      #grab desired speeds from driver and prepare to hand to simulator
                        simToCode.sendValues()                                                      #take values from simulator and give them to driver
                        simToCode.grabValues()                                                      #grab location data from simulator and prepare to give them to driver
                        simToCode.convertValues()                                                   #convert from simulated coordinates to gps coordinates

                if sim_or_real == "real":                        
                        runner = jobs()                                                                         #initiate job scheduler
                        gpsData = ["python3","sensor.py","gps","5580","ACM0","115200","2"]
                        runner.launchJob(gpsData)                                                               #start reading gps sensor
                        compassData = ["python3","sensor.py","compass","5582","USB2","9600","1"]
                        runner.launchJob(compassData)                                                           #start reading compass sensor
                        imuData = ["python3","sensor.py","imu","5583","USB3","38400","1"]
                        runner.launchJob(imuData)                                                               #start reading imu sensor
                                                
                        try:
                                while True:
                                        time.sleep(5)
                        except:
                                runner.endThis()

                        

        @threaded
        def getImu(self):
                imuContext = zmq.Context()
                imuSocket = imuContext.socket(zmq.REP)
                imuSocket.bind("tcp://*:5583")
                while True:
                        message = b"0"
                        message = imuSocket.recv()
                        imuSocket.send(b"thanks")
                        self.omega_g = -float(message.decode(encoding))

        @threaded
        def getCompass(self,sim_or_real):
            gyroContext = zmq.Context()
            gyroSocket = gyroContext.socket(zmq.REP) 
            gyroSocket.bind("tcp://*:5582")
            lastAngle = 0.1
            while True:
                message = b"0"
                message = gyroSocket.recv()
                gyroSocket.send(b"thanks")
                angle = float(message.decode(encoding))
                if (angle == 0.0):                                                                  #sometimes compass (or usb interface) gives 0's. just ignore
                    angle = lastAngle
                if sim_or_real == "real":
                    deviation = self.angle_interpolate(angle)
                    thetaCoord_g = angle + deviation 
                else:
                    self.thetaCoord_g = angle 
                lastAngle = angle 

        @threaded
        def getGPS(self):
                gpsContext = zmq.Context()
                gpsSocket = gpsContext.socket(zmq.REP)
                gpsSocket.bind("tcp://*:5580")
        
                while True:
                        message = b"0"
                        message = gpsSocket.recv()
                        gpsSocket.send(b"thanks")
                        decoded = message.decode(encoding)              
                        data = re.split(' ',decoded)
                        if(len(data) > 1):
                            preLat = float(data[0])
                            preLong = float(data[1])
                            self.latitude_g = preLat
                            self.longitude_g = preLong
        

        def angle_interpolate(self,angle):
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


        @threaded
        def kalman_angle(self):
            T = 0.01
            lamC = 1.0                                          #variance for compass
            lamGPS = 1.0                                          #variance for gps bearing
            lamGyro = 1.0                                   #variance for gyroscope
            sig = 1.0                                               #variance for model
            sig2 = sig**2
            lamC2 = lamC**2
            lamGPS2 = lamGPS**2
            lamGyro2 = lamGyro**2
            (A, H, Q, R) = create_model_parameters(T,  sig2, lamC2)                                    #initialize evolution, measurement, model error, and measurement error
# in        itial state
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
                while (self.omega_g == lastOmega or self.thetaCoord_g == lastAngle):               #check if there were updates to the compass
                    time.sleep(0.01)                                            #if no updates then wait a bit
                thetaCoord_l = self.thetaCoord_g
                lastAngle = thetaCoord_l
                if (kalman_filter._x[0] - thetaCoord_l) > 180:                      #make sure the filter knows that we crossed the pole
                   kalman_filter._x[0] = kalman_filter._x[0] - 360
                elif (thetaCoord_l - kalman_filter._x[0]) > 180:
                   kalman_filter._x[0] = kalman_filter._x[0] + 360
                kalman_filter.predict()                                                 #evolve the state and the error
                kalman_filter.update((thetaCoord_l,self.omega_g),(lamC2,lamGyro2))                         #load in 2 measurement values
                (x, P) = kalman_filter.get_state()                                      #return state
                dt = time.time() - lastTime                                             #calculate dt
                lastTime = time.time()                                              
                kalman_filter.update_time(dt,sig2)                                      #Make sure the filter knows how much time occured in between steps
                self.kAngle_g = x[0]%360                                                     #because the model portion is estimating based on that.
            
#                while (omega_g == lastOmega or gpsTheta_g == lastBearing):               #check if there were updates to the gps bearing
#                time.sleep(0.01)                                            #if no updates then wait a bit
                gpsTheta_l = self.gpsTheta_g
                lastBearing = gpsTheta_l
                if (kalman_filter._x[0] - gpsTheta_l) > 180:                      #make sure the filter knows that we crossed the pole
                   kalman_filter._x[0] = kalman_filter._x[0] - 360
                elif (gpsTheta_l - kalman_filter._x[0]) > 180:
                   kalman_filter._x[0] = kalman_filter._x[0] + 360
                lamGPS2 = gpsTrust/self.gpsDistance_g
                kalman_filter.predict()                                                 #evolve the state and the error
                kalman_filter.update((gpsTheta_l,self.omega_g),(lamGPS2,lamGyro2))                         #load in 2 measurement values
                (x, P) = kalman_filter.get_state()                                      #return state
                dt = time.time() - lastTime                                             #calculate dt
                lastTime = time.time()
                kalman_filter.update_time(dt,sig2)                                      #Make sure the filter knows how much time occured in between steps
                self.kAngle_g = x[0]%360                                                     #because the model portion is estimating based on that.
#                print(thetaCoord_l, gpsTheta_g,kAngle_g,latitude_g,longitude_g)
            
            
            
        @threaded
        def calcGpsHeading(self):
            lastLon = self.longitude_g
            lastLat = self.latitude_g
            while True:
                if lastLon != self.longitude_g or lastLat != self.latitude_g:
                    self.gpsTheta_g, self.gpsDistance_g = findBearing(lastLat,lastLon,self.latitude_g,self.longitude_g)
                    lastLon = self.longitude_g
                    lastLat = self.latitude_g
                    time.sleep(0.001)
                
        
        @threaded
        def bothPid(self):
            context = zmq.Context()
            socket = context.socket(zmq.REP)
            socket.bind("tcp://*:5581")
            while True:
                response = str(self.leftWheelPwr_g) + " " + str(self.rightWheelPwr_g)
                message = socket.recv()
                socket.send(response.encode(encoding))
        











                
