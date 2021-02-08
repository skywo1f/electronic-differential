from sensor import *
from runAndManage import *
import threading
from my_tools import *


#use cameras,radar,lidar, sonar to track 
class obstacleTrack:
	def __init__(self,sim_or_real):
		self.distances = []
		self.types = []
		self.arrayUS_g = [0]*7
		self.arrayRadar_g = [0]*4
		self.arrayLidar_g = [0]*3
#		self.values = [0]*(7+4+3+3+3+3)							#sonar + radar + lidar + camL + camM + camR
		if sim_or_real == "real":
			runner = jobs()
			sonarData = ["python3","sensor.py","sonar",5585,"ACM0",115200,8]
			runner.launchJob(sonarData)
			radarData = ["python3","sensor.py","radar",5586,"ACM1-4",9600,4]
			runner.launchJob(radarData)
			lidarData = ["python3","sensor.py","lidar",5587,"rs-pipe",0,3] 
			runner.launchJob(lidarData)
			self.getSonar = sensorCatch("sonar",5585,7)
			self.getSonar.catchValues()			
			self.getLidar = sensorCatch("lidar",5586,3)
			self.getLidar.catchValues()
			self.getRadar = sensorCatch("radar",5587,4)
			self.getRadar.catchValues()
			self.getLeftCam = cameraCatch("left",5551,3)
			self.getLeftCam.catchValues()
			self.getMidCam = cameraCatch("mid",5552,3)
			self.getMidCam.catchValues()
			self.getRightCam = cameraCatch("right",5553,3)
			self.getRightCam.catchValues()
			
'''	
	def getValues(self)
		for i in range(7):
			self.values[i] =  getSonar.values[i]
		for i in range(4):
			self.values[i + 7] = getRadar.values[i]
		for i in range(3):
			self.values[i + 11] = getLidar.values[i]
		for i in range(3):
			self.values[i + 14] = getLeftCam.values[i]
		for i in range(3):
			self.values[i + 17] = getMidLidar.values[i]
		for i in range(3):
			self.values[i + 21] = getRightLidar.values[i]
'''
	
