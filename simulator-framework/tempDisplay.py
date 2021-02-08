import time
import zmq
import threading
import numpy as np
import re
import serial
from tkinter import *
import math
import pyrealsense2 as rs
from lightSimple import *

encoding = 'utf-8'                                                          #set up encoding for transfering bytes

print("connecting to democritus (left)")
contextL = zmq.Context()
socketL = contextL.socket(zmq.REP)
socketL.bind("tcp://*:5551")

print("connecting to xenocrates (mid)")
contextM = zmq.Context()
socketM = contextM.socket(zmq.REP)
socketM.bind("tcp://*:5552")

print("connecting to theocritus (right)")
contextR = zmq.Context()
socketR = contextR.socket(zmq.REP)
socketR.bind("tcp://*:5553")

print("connecting to radar")
contextRadar = zmq.Context()
socketRadar = contextRadar.socket(zmq.REP)
socketRadar.bind("tcp://*:5565")

global yPos
yPos = 0
global xPos
xPos = 0
global lastTheta
lastTheta = 0
camHeight = 39                          #height of camera off the ground
screenHeight = 480                      #number of vertical pixels
global area
area = 0
radar1_g = 0
radar2_g = 0
radar3_g = 0 
radar4_g = 0
radarT1_g = 1
radarT2_g = 1
radarT3_g = 1
radarT4_g = 1
arrayUS_g = [0,0,0,0,0,0,0]                                                 #create array to transfer ultrasound distances
boolUS_g =  [0,0,0,0,0,0,0]                                                 #create array to transfer re-ordered sonar triggers
leftDetect = 0
midDetect = 0
rightDetect = 0
lidarLeft = True
lidarMid = True
lidarRight = True
gyroAngle = 0
secondScreenX = 640
leftTrigger = True
midTrigger = True
rightTrigger = True

try:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    align_to = rs.stream.color
    align = rs.align(align_to)
    pipeline.start(config)
except:
    print("could not connect to lidar")

def recvData(socket):				#catch data from socket and put into variables
	message = socket.recv()
	socket.send(b"confirmed")
	alphabet = message.decode('utf-8')
	data = re.split(' ',alphabet)
	theta = float(data[0])
	phi = float(data[1])
	area = float(data[2])		
	return theta,phi,area

def relaxTheta(theta, lastTheta):		#reduce noise by allowing the approximation to get closer but not farther
	if lastTheta < theta:			#if it went down the screen, good, let it go
		approxTheta = 0.001*lastTheta + 0.999*theta
	else:					#if it went up the screen, bad, slow it down
		approxTheta = 0.999*lastTheta + 0.001*theta
	approxTheta = theta
	lastTheta = approxTheta
	theta = approxTheta
	return theta, lastTheta

def readLidar():
#try to connect to lidar
    try:
#if True:
        pipeline = rs.pipeline()                                                #initialize the pipeline
        config = rs.config()                                                    #configure pipeline with defaults
        config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)     #start a depth query stream
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)    #start a color query stream
        align_to = rs.stream.color                                              #align streams together
        align = rs.align(align_to)
        pipeline.start(config)                                                  #start query
    except:
        print("could not connect to lidar")

    w = 10
    h = 10
    global lidarRight
    global lidarMid
    global lidarLeft
    global lidarRange 
    lastLidar = 0
    try:
        while True:
        # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            if not depth_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            toggleLeft = False
            toggleMid = False
            toggleRight = False
            d_l = depth_image[400,:]				#depth_line
            d_l = np.where(d_l == 0,13000,d_l)
            if np.any(d_l[:200] < 10000):
                lidarLeft = True
                toggleLeft = True	
            if np.any(d_l[200:1000] < 10000):
                lidarMid = True
                toggleMid = True
            if np.any(d_l[1000:] < 10000):
                lidarRight = True
                toggleRight = True
                print('right seen')
            if toggleLeft == False:
                lidarLeft = False
            if toggleMid == False:
                lidarMid = False
            if toggleRight == False:
                lidarRight = False


    finally:
    # Stop streaming
        pipeline.stop()

def catchRadar():
	global radar1_g,radar2_g,radar3_g,radar4_g
	lastR1 = 0
	lastR2 = 0
	lastR3 = 0
	lastR4 = 0
	while True:	
		message = socketRadar.recv()
		socketRadar.send(b"confirmed")
		alphabet = message.decode('utf-8')
		data = re.split(' ',alphabet)
		if float(data[0]) != 0:
			radar1_g = float(data[0])
			lastR1 = radar1_g
		else:
			radar1_g = lastR1

		if float(data[1]) != 0:
			radar2_g = float(data[1])
			lastR2 = radar2_g
		else:
			radar2_g = lastR2

		if float(data[2]) != 0:
			radar3_g = float(data[2])
			lastR3 = radar3_g
		else:
			radar3_g = lastR3

		if float(data[3]) != 0:
			radar4_g = float(data[3])
			lastR4 = radar4_g
		else:
			radar4_g = lastR4


def radarTrigger():
	global radarT1_g,radarT2_g,radarT3_g,radarT4_g
	minRange = 0.5
	while True:

		if radar1_g > minRange:
			radarT1_g = False
		else:
			radarT1_g = True
		if radar2_g > minRange:
			radarT2_g = False
		else:
			radarT2_g = True
		if radar3_g > minRange:
			radarT3_g = False
		else:
			radarT3_g = True
		if radar4_g > minRange:
			radarT4_g = False
		else:
			radarT4_g = True
	

def checkSocket(socket,camLabel):						#cam label not exactly working, instead using "area"
	
	lastTheta = 0
	verticalPixels = 480
	degreesToRadians = np.pi/180					#convert degrees to radians
	verticalScreenAngle = 60					#vertical angle of view
	angleToScreen = 90 - verticalScreenAngle/2				#assuming camera is mounted looking at horizon
	pixelsToDegrees = (verticalScreenAngle/2)/(verticalPixels/2)					#convert pixels to degrees
	whichCam = "not picked yet"
	while True:
		global yPos
		global xPos
		global area
		xPos = yPos = 0
#  Wait for next request from client
		theta,phi,area = recvData(socket)
		
		print(theta,phi,area)
		fFd = 0.785398					#radians to degrees (45?)

		theta, lastTheta = relaxTheta(theta, lastTheta)
		yPos = camHeight*np.tan(degreesToRadians*(angleToScreen - gyroAngle + pixelsToDegrees*(verticalPixels - theta)))
		xPos = -yPos*np.tan((degreesToRadians)*(phi-secondScreenX/2)*30/(secondScreenX/2))
		if(yPos > 0):	
			if int(area) == 3:
				realX = xPos*np.cos(-fFd) - yPos*np.sin(-fFd)
				realY = xPos*np.sin(-fFd) + yPos*np.cos(-fFd)
				xPos = realX
				yPos = realY
				whichCam = "left"
			if int(area) == 1:
				realX = xPos*np.cos(fFd) - yPos*np.sin(fFd)
				realY = xPos*np.sin(fFd) + yPos*np.cos(fFd)
				xPos = realX
				yPos = realY
				whichCam = "right"
			if int(area) == 2:
				whichCam = "middle"
			time.sleep(0.01)
#			print(xPos,yPos)
			print("from camera ", whichCam)

def checkUltrasound() :
#try to connect to the ultrasoundArray
	try:
		ser = serial.Serial('/dev/ttyACM0',115200)                          #connect to the sonar usb hub
		foundUltra = True
	except:
		print("sonar array not found")
		foundUltra = False
	global arrayUS_g
	if foundUltra == True:
		while True:
			line = ser.readline()
			string = line.decode(encoding)
			data = re.split(' ',string)
			arrayUS_g = data[:-1]

#test if within range and also rearrange clockwise
def rangeUltrasound():
	order = [6,0,2,3,5,4,1]
	global boolUS_g
	rangeUS = 50	
	while True:
		for i in range(7):
			rn = order[i]			#renumber according to order vector
			if float(arrayUS_g[i]) < rangeUS:
				boolUS_g[rn] = True
			else:
				boolUS_g[rn] = False
		time.sleep(0.001)

def circleTable(i):
	offSetK = 4
	offSetL = -2
	if i == 0:
		k = 5
		l = 5
	if i == 1:
		k = 5
		l = 6
	if i == 2:
		k = 5
		l = 7
	if i == 3:
		k = 4
		l = 7
	if i == 4:
		k = 3
		l = 7
	if i == 5:
		k = 3
		l = 6
	if i == 6:
		k = 3
		l = 5
	ko = offSetK + k
	lo = offSetL + l
	return ko,lo




def loopLight():
	strip = startColor()
	for i in range(24):
		setColor(strip,i)
	time.sleep(4)
	for i in range(24):
		offColor(strip,i)
	while True:
#trigger camera	
		if rightTrigger==True:
			setColor(strip,15)
		else:
			offColor(strip,15)
		if midTrigger==True:
			setColor(strip,12)		
		else:
			offColor(strip,12)
		if leftTrigger==True:
			setColor(strip,9)
		else:
			offColor(strip,9)
#trigger lidar
		if lidarRight==True:
			setColor(strip,14)		
		else:
			offColor(strip,14)
		if lidarMid==True:
			setColor(strip,12)
		else:
			offColor(strip,12)
		if lidarLeft==True:
			setColor(strip,10)
		else:
			offColor(strip,10)
#trigger radar
		if radarT1_g==True:
			setColor(strip,17)
		else:
			offColor(strip,17)		
		if radarT2_g==True:
			setColor(strip,20)		
		else:
			offColor(strip,20)
		if radarT3_g==True:
			setColor(strip,22)	
		else:
			offColor(strip,22)
		if radarT4_g==True:
			setColor(strip,1)
		else:
			offColor(strip,1)
#trigger ultrasound
		for i in range(7):
			shifted = ((i*3)+12)%24 
			if boolUS_g[i] == True:
				setColor(strip,shifted)
			else:
				offColor(strip,shifted)

def loopDisplay():
	window = Tk()
	window.title("detection display")
	window.geometry('190x160')
	lblUS = [1]*7
	while True:
#trigger camera	
		if rightTrigger==True:
			lblR = Label(window, text="R", bg='red')
			lblR.grid(column=12, row=1)
		else:
			lblR = Label(window, text="R", bg='green')
			lblR.grid(column=12, row=1)			
		if midTrigger==True:
			lblM = Label(window, text="M", bg='red')
			lblM.grid(column=8, row=1)
		else:
			lblM = Label(window, text="M", bg='green')
			lblM.grid(column=8, row=1)	
		if leftTrigger==True:
			lblL = Label(window, text="L", bg='red')
			lblL.grid(column=6, row=1)
		else:
			lblL = Label(window, text="L", bg='green')
			lblL.grid(column=6, row=1)
#trigger lidar
		if lidarRight==True:
			lblRL = Label(window, text="R", bg='red')
			lblRL.grid(column=12, row=2)
		else:
			lblRL = Label(window, text="R", bg='green')
			lblRL.grid(column=12, row=2)			
		if lidarMid==True:
			lblML = Label(window, text="M", bg='red')
			lblML.grid(column=8, row=2)
		else:
			lblML = Label(window, text="M", bg='green')
			lblML.grid(column=8, row=2)	
		if lidarLeft==True:
			lblLL = Label(window, text="L", bg='red')
			lblLL.grid(column=6, row=2)
		else:
			lblLL = Label(window, text="L", bg='green')
			lblLL.grid(column=6, row=2)
#trigger radar
		if radarT1_g==True:
			lbl1Rad = Label(window, text="RadRF", bg='red')
			lbl1Rad.grid(column=13, row=3)
		else:
			lbl1Rad = Label(window, text="RadRF", bg='green')
			lbl1Rad.grid(column=13, row=3)	
		if radarT2_g==True:
			lbl2Rad = Label(window, text="RadRB", bg='red')
			lbl2Rad.grid(column=13, row=4)
		else:
			lbl2Rad = Label(window, text="RadRB", bg='green')
			lbl2Rad.grid(column=13, row=4)	
		if radarT3_g==True:
			lbl3Rad = Label(window, text="RadBR", bg='red')
			lbl3Rad.grid(column=12, row=7)
		else:
			lbl3Rad = Label(window, text="RadBR", bg='green')
			lbl3Rad.grid(column=12, row=7)	
		if radarT4_g==True:
			lbl4Rad = Label(window, text="RadBL", bg='red')
			lbl4Rad.grid(column=6, row=7)
		else:
			lbl4Rad = Label(window, text="RadBL", bg='green')
			lbl4Rad.grid(column=6, row=7)				
#trigger ultrasound
		for i in range(7):
			k,l = circleTable(i)
			if boolUS_g[i] == True:
				lblUS[i] = Label(window, text=str(i), bg='red')
				lblUS[i].grid(column=k, row=l)
			else:
				lblUS[i] = Label(window, text=str(i), bg='green')
				lblUS[i].grid(column=k, row=l)
		window.update()
	window.mainloop()

def talkToBT():
	try:
		serbt = serial.Serial('/dev/ttyUSB0',115200)
		sendNum = 0000
		while True:
			sendNum = 0000
			if radarT1_g==True:
				sendNum = sendNum + 1000
			if radarT2_g==True:
				sendNum = sendNum + 100
			if radarT3_g==True:
				sendNum = sendNum + 10
			if radarT4_g==True:
				sendNum = sendNum + 1
			sendByte = str(sendNum).encode('utf-8')
			serbt.write(sendByte)
			serbt.flush()
			print(sendByte)
	
			time.sleep(1.0)
	except:	
		print("unable to connect to bluetooth hub (heltec)")


def locatePerson():
    histNum = 75						#interpolate position based on last histNum points
    lonHist = np.zeros(histNum)
    latHist = np.zeros(histNum)
    times = np.zeros(histNum)
    times = startTime(times,histNum)
    gpsCounter = 0
    global futureY
    while True:

            thisLon = xPos
            thisLat = yPos
            if thisLat > 0:
                if (gpsCounter == 0):
                    lonHist = startArray(lonHist,thisLon,histNum)
                    latHist = startArray(latHist,thisLat,histNum)
                gpsCounter += 1
                lonHist = pushBack(lonHist,thisLon,histNum)
                latHist = pushBack(latHist,thisLat,histNum)           
                pLon = np.polyfit(times,lonHist,2) #default 5
                pLat = np.polyfit(times,latHist,2) # 5
                lonPoly = np.polyval(pLon,times);
                latPoly = np.polyval(pLat,times);
                longitude = lonPoly[histNum-1]
                latitude = latPoly[histNum-1]
 #               print(latHist)
                futureY = np.polyval(pLat,85)
                time.sleep(.0001)

def cameraTrigger():
	global leftTrigger,midTrigger,rightTrigger
	piOver3	= 1.0471975512								#60 degrees
	while True:
		leftTrigger = False						#reset triggers to false
		midTrigger = False
		rightTrigger = False
		if(yPos > 0):
			distance = np.sqrt(xPos*xPos + yPos*yPos)			#calculate euclidean distance
			objectAngle = math.acos(xPos/distance)				#get horizontal angle of object in radians
#			print(objectAngle)
#			print(distance)
#			print(xPos)
			
			if objectAngle < piOver3:
				leftTrigger = True
				print('left triggered')
				time.sleep(0.5)	
			if objectAngle > piOver3*2:
				rightTrigger = True
				print('right triggered')
				time.sleep(0.5)			
			if objectAngle > piOver3 and objectAngle < piOver3*2:
				midTrigger = True
				print('middle triggered')
				time.sleep(0.5)
			
if __name__ == "__main__":
    firstThread = threading.Thread(target=checkSocket,args=(socketL,"left"))
    firstThread.start()
    secondThread = threading.Thread(target=checkSocket,args=(socketM,"middle"))
    secondThread.start()
    thirdThread = threading.Thread(target=checkSocket,args=(socketR,"right"))
    thirdThread.start()
    
    fourthThread = threading.Thread(target=checkUltrasound)
    fourthThread.start()
    
    fourPointFive = threading.Thread(target=rangeUltrasound)
    fourPointFive.start()
    
    fifthThread = threading.Thread(target=loopDisplay)
    fifthThread.start()
    
    seventhThread = threading.Thread(target=cameraTrigger)
    seventhThread.start()
    
    eigthThread = threading.Thread(target=readLidar)
    eigthThread.start()
    
    ninthThread = threading.Thread(target=catchRadar)
    ninthThread.start()
    
    tenthThread = threading.Thread(target=radarTrigger)
    tenthThread.start()
    
    eleventhThread = threading.Thread(target=talkToBT)
    eleventhThread.start()
