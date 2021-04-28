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
from navigation import *
'''
This driving algorithm consists of two steps. The first step aligns the mower with the next waypoint using the compass. The second step drives the mower towards the waypoint using a modified pid loop. Acceleration and decelleration are dampened for a smoother ride.
'''


class twoStepDriving:
        def __init__(self,path,simOrReal):
                self.path = path
                self.numWp_g = 0
                self.angleTolerance = 20.0                                            #how far off course (angularly) is ok (degrees)
                self.distanceTolerance = 1.0                                          #size of radius at which we count as reaching the waypoint (meters)
                if simOrReal == "sim":
                        nav = navigation("sim")                                               #call navigation class to talk to outside world/sim
                else:
                        nav = navigation("real")
                        import enacter
                        mover = self.enacter.enact()
                        mover.runSpeedRead()
                self.numWpUpdate(nav,self.path)
                self.driver(nav)


        def returnSimDone():
                if self.numWp_g == len(self.path):
#                        print(self.numWp_g,len(self.path))
                        return True
                else:
                        return False
        
        @threaded
        def driver(self,nav):
            time.sleep(2)
            while True:
                lastWp = self.numWp_g
                thisY = nav.longitude_g                                                              #translate lat/long to x/y for clarity
                thisX = nav.latitude_g
                wpX,wpY = getPathPoint(self.path,self.numWp_g + 1)
                thetaGoal,distance = findBearing(thisX,thisY,wpX,wpY)
#    print(thetaCoord,thetaGoal,distance)
                self.compassOrientation(nav,thetaGoal)                                                   #thetaGoal is the compass angle
#    print("finished turning")
                time.sleep(0.5)
                self.gpsDrive(nav,wpX,wpY)
                time.sleep(0.5)


        def compassOrientation(self,nav,thetaGoal):
                lastE = 0
                sumE = 0
                currentE = 0
                changeV = 0
                turnRate = 5
#               turnRate = 2
                delay = 0.5
                turnTime = 2                            #twice as long as it takes to get to max turn speed
                time.sleep(delay)
                start_time = time.time()
                run_time = time.time() - start_time
                turnDone = 0
                while(turnDone == 0):
#                        print(nav.thetaCoord_g,thetaGoal)
                        direction = turnTable(nav.thetaCoord_g,thetaGoal)                                       #get direction to turn      
                        delta = deltaTheta(direction,nav.thetaCoord_g,thetaGoal)                                        #get distance left to turn
                        if (delta < self.angleTolerance):
                                turnDone = 1
                        run_time = time.time() - start_time
                        if run_time > 0 and run_time < turnTime/2:
                                desiredVal = turnRate*(run_time)/(turnTime/2)   
                        if run_time > turnTime/2:
                                desiredVal = turnRate
                        if direction > 0:
                                nav.leftWheelPwr_g = -desiredVal 
                                nav.rightWheelPwr_g = desiredVal 
                        else:
                                nav.leftWheelPwr_g =  desiredVal 
                                nav.rightWheelPwr_g = -desiredVal 
                nav.leftWheelPwr_g =  0
                nav.rightWheelPwr_g = 0
                time.sleep(delay)
#               print(delta)
                turnDone = 0
                turnRate = turnRate/2                                                   #this seems odd
                while(turnDone == 0):
                        direction = turnTable(nav.thetaCoord_g,thetaGoal)                                                       #get direction to turn  
                        delta = deltaTheta(direction,nav.thetaCoord_g,thetaGoal)                                        #get distance left to turn
                        if (delta < self.angleTolerance/2):
                                turnDone = 1
                        run_time = time.time() - start_time
                        if run_time > 0 and run_time < turnTime/2:
                                desiredVal = turnRate*(run_time)/(turnTime/2)   
                        if run_time > turnTime/2:
                                desiredVal = turnRate
                        if direction > 0:
                                nav.leftWheelPwr_g = -desiredVal + 2
                                nav.rightWheelPwr_g = desiredVal  + 2
                        else:
                                nav.leftWheelPwr_g =  desiredVal + 2
                                nav.rightWheelPwr_g = -desiredVal + 2
                nav.leftWheelPwr_g =  0
                nav.rightWheelPwr_g = 0
                time.sleep(delay)               



        def gpsDrive(self,nav,wpX,wpY):
                delay = 2       
                fwdSpeed = 60                                                           #tested ok up to 60
                fullSpeed = fwdSpeed
#               AEK = 0.01                                                      #default angle error constant
                AEK = 0.025
#               DEK = 0.01                                                      #default distance aggresiveness constant
                DEK = 0.001
#               SEK = 0.0002                                                    #summed error constant
                SEK = 0.0003
                thisY = nav.longitude_g                                                               #translate lat/long to x/y for clarity
                thisX = nav.latitude_g
                thetaGoal,distance = findBearing(thisX,thisY,wpX,wpY) 
                nav.leftWheelPwr_g = fwdSpeed
                nav.rightWheelPwr_g = fwdSpeed
                lastDistance = distance 
                thetaLast = nav.thetaCoord_g
                offset = 0.0
                sumE = 0
                thetaSmooth = 0
                omegaSmooth = 0
                startT = time.time()
#               omegaStrength = 2
                omegaStrength = 0.4
                K = 5
                C = fullSpeed - K*(5-fullSpeed)/(1-K)
                while distance > self.distanceTolerance:
                        thetaCoord_l = nav.kAngle_g
                        fwdSpeed = fullSpeed
                        if distance < self.distanceTolerance*K:                 #K was 2
                                fwdSpeed = distance*(5 - fullSpeed)/(self.distanceTolerance-K*self.distanceTolerance) + C 
#                               fwdSpeed = distance*(fullSpeed-5)/distanceTolerance + 10 - fullSpeed
                        if ((time.time() - startT) < 5):
                                startSpeed = (time.time() - startT)*fullSpeed/5
                                if startSpeed < fwdSpeed:
                                        fwdSpeed = startSpeed
                        thisY = nav.longitude_g
                        thisX = nav.latitude_g
                        thetaGoal,distance = findBearing(thisX,thisY,wpX,wpY)
                        direction = turnTable(thetaCoord_l,thetaGoal)   #get direction to turn  
                        thetaSmooth = 0.0001*thetaSmooth + 0.9999*nav.thetaCoord_g
                        thetaLast = thetaSmooth         
                        delta = deltaTheta(direction,thetaCoord_l,thetaGoal)                                    #get distance left to turn              
                        overshot = 0
                        sumE += abs(delta)
                        if abs(delta) < 1:                                      #default 4
                                sumE = 0
                        if sumE*SEK > 20/4:                                     #15 worked for slow speeds
                                sumE = 20/(SEK*4)
                        omegaSmooth = 0.5*omegaSmooth + 0.5*nav.omega_g
                        if abs(delta) < 10 and abs(omegaSmooth) > 0:
                                overshot = abs(omegaSmooth)*omegaStrength
        
                        turnStrength = DEK/distance + AEK*abs(delta) + sumE*SEK
#                        print(nav.phiCoord_g)
#                        print(nav.rollCoord_g)
#                       print(latitude_g,longitude_g)
#                        print(thetaCoord_g,kAngle_g)
#                        print(distance,delta,direction,overshot,sumE*SEK)
                        if omegaSmooth > 0:
                                if direction > 0:
                                        nav.leftWheelPwr_g = (fwdSpeed - turnStrength + overshot)
                                        nav.rightWheelPwr_g = (fwdSpeed + turnStrength - overshot)
                                else:
                                        nav.leftWheelPwr_g =  (fwdSpeed + turnStrength + overshot)
                                        nav.rightWheelPwr_g = (fwdSpeed - turnStrength - overshot)
                        else:
                                if direction > 0:
                                        nav.leftWheelPwr_g = (fwdSpeed - turnStrength - overshot)
                                        nav.rightWheelPwr_g = (fwdSpeed + turnStrength + overshot)
                                else:
                                        nav.leftWheelPwr_g =  (fwdSpeed + turnStrength - overshot)
                                        nav.rightWheelPwr_g = (fwdSpeed - turnStrength + overshot)
        
                        lastDistance = distance 
                        time.sleep(0.001)       
#                       print(rightWheelPwr_g,leftWheelPwr_g,thetaCoord_g)
#                       print(distance)
                print('reached the waypoint, pausing')  
                nav.leftWheelPwr_g = 0
                nav.rightWheelPwr_g = 0
                time.sleep(2)
        
        @threaded
        def numWpUpdate(self,nav,path):                                                                  #function which is constantly checking to see if numWp has been reached and needs to be updated
                self.numWp_g = 0
                lastD = 1e9                                                                       #initialize
                while(True):                                                                    #check waypoints
                        thisY = nav.longitude_g                                                           #standard long -> Y
                        thisX = nav.latitude_g                                                           #lat -> X (maybe should have been backwards
                        nX,nY = getPathPoint(path,self.numWp_g + 1)                                                #get position of the next path point
                        thetaHeading,distance = findBearing(thisX,thisY,nX,nY)
                        numD = distance                                     #get distance to the next path point
#            print(thisX,thisY,nX,nY)
#            print(thetaHeading,distance)
                        if numD < self.distanceTolerance:                                                #if I stopped getting closer to the wp, its time to update
                                self.numWp_g = self.numWp_g + 1
                        lastD = numD
                        time.sleep(0.001)
        
        
        
        
















