import threading
import numpy as np
import serial
import re
import zmq
import time
from pyproj import Proj
from my_tools import *
import sys


encoding = 'utf-8'

#catch sensor values from sensor reading subroutine and give them to the driving algorithm
class sensorCatch:
    def __init__(self,name,port,nValues):
        self.name = name
        tcpPart = "tcp://*:"
        fullPort = tcpPart + str(port)
        self.port = fullPort
        self.nValues = nValues
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(fullPort)
        self.values = [0]*nValues

        @threaded
        def catchValues():
                while True:
                        message = b"0"
                        message = self.socket.recv()
                        self.socket.send(b"thanks")
                        data = message.decode(encoding)
                        for i in range(nValues):                        
                                self.values[i] = data[i]                        

class cameraCatch(sensorCatch):
        def __init__(self,name,port,nValues):
                super().__init__(name,port,nValues)
                self.yPos = 0
                self.xPos = 0
                self.area = 0


        def relaxTheta(theta, lastTheta):               #reduce noise by allowing the approximation to get closer but not farther
                if lastTheta < theta:                   #if it went down the screen, good, let it go
                        approxTheta = 0.001*lastTheta + 0.999*theta
                else:                                   #if it went up the screen, bad, slow it down
                        approxTheta = 0.999*lastTheta + 0.001*theta
                approxTheta = theta
                lastTheta = approxTheta
                theta = approxTheta
                return theta, lastTheta

        def catchValues(socket,name):
                lastTheta = 0
                verticalPixels = 480
                degreesToRadians = np.pi/180                                    #convert degrees to radians
                verticalScreenAngle = 60                                        #vertical angle of view
                angleToScreen = 90 - verticalScreenAngle/2                              #assuming camera is mounted looking at horizon
                pixelsToDegrees = (verticalScreenAngle/2)/(verticalPixels/2)                                    #convert pixels to degrees
                whichCam = "not picked yet"
                while True:
                        self.xPos = self.yPos = 0
        #  Wait for next request from client
                        theta,phi,area = recvData(socket)
                        
                        print(theta,phi,area)
                        fFd = 0.785398                                  #radians to degrees (45?)

                        theta, lastTheta = self.relaxTheta(theta, lastTheta)
                        yPosL = camHeight*np.tan(degreesToRadians*(angleToScreen - gyroAngle + pixelsToDegrees*(verticalPixels - theta)))                               #differentiate to avoid race condition
                        xPosL = -yPos*np.tan((degreesToRadians)*(phi-secondScreenX/2)*30/(secondScreenX/2))                                                                                             #L for local
                        if(yPos > 0):   
                                if int(area) == 3:
                                        realX = xPosL*np.cos(-fFd) - yPosL*np.sin(-fFd)
                                        realY = xPosL*np.sin(-fFd) + yPosL*np.cos(-fFd)
                                        self.xPos = realX
                                        self.yPos = realY
                                        whichCam = "left"
                                if int(area) == 1:
                                        realX = xPosL*np.cos(fFd) - yPosL*np.sin(fFd)
                                        realY = xPosL*np.sin(fFd) + yPosL*np.cos(fFd)
                                        self.xPos = realX
                                        self.yPos = realY
                                        whichCam = "right"
                                if int(area) == 2:
                                        whichCam = "middle"
                                        self.xPos = xPosL
                                        self.yPos = yPosL
                                time.sleep(0.01)
        #                       print(xPos,yPos)
                                print("from camera ", whichCam)


#grab sensor values from sensor and throw them to the catcher
class sensorThrow:
    def __init__(self,name,port,usb,baudrate,nValues,deviations = []):
        self.name = name
        tcpPart = "tcp://localhost:"
        fullPort = tcpPart + str(port)
        self.port = fullPort
        self.nValues = int(nValues)
        if len(str(deviations)) == 1:
            self.deviations = [0]*self.nValues
        else:
            self.deviations = deviations
        self.values = [0] * self.nValues
        self.usb = usb
        self.baud = baudrate
    
    def connectToSensor(self):
        devTty = "/dev/tty" + self.usb
        self.ser = serial.Serial(devTty,self.baud)



    @threaded                
    def grabValues(self):
        while True:
            line = self.ser.readline()
            string = line.decode(encoding)
            data = re.split(' ',string)
            for i in range(self.nValues):
                self.values[i] = float(data[i])

    @threaded
    def sendValues(self):
        context = zmq.Context()
        socketZmq = context.socket(zmq.REQ)
        socketZmq.connect(self.port)
        deviated = [0]*int(self.nValues) 
        while True:
            returnString = ""
            for i in range(self.nValues):
                if len(self.deviations) > 0:
                    deviated[i] = int(self.values[i]) + int(self.deviations[i])
                else:
                    deviated[i] = self.values[i]
                returnString = returnString.join(str(deviated[i])) 
            socketZmq.send(returnString.encode(encoding))
            message = socketZmq.recv()


#port1 gives gps data to the driving algorithm
#port2 takes in wheel speeds from driving algorithm
#port3 gives compass data to driving algorithm
#port4 gives gyro data to driving algorithm
#port5 talks to d2g
class simulatedThrow(sensorThrow):
    encoding = 'utf-8'
    def __init__(self,name,port,port2,port3,port4,port5,port6,port7,port8,port9,port10,port11,port12,port13,usb = 0,baudrate = 0,nValues = 0,deviations = 0):
        super().__init__(name,port,usb,baudrate,nValues,deviations)
        tcpPart = "tcp://localhost:"
        fullPort2 = tcpPart + str(port2)
        self.port2 = fullPort2
        fullPort3 = tcpPart + str(port3)
        self.port3 = fullPort3
        fullPort4 = tcpPart + str(port4)
        self.port4 = fullPort4
        fullPort6 = tcpPart + str(port6)
        self.port6 = fullPort6
        fullPort7 = tcpPart + str(port7)
        self.port7 = fullPort7

        fullPort8 = tcpPart + str(port8)
        self.port8 = fullPort8
        fullPort9 = tcpPart + str(port9)
        self.port9 = fullPort9
        fullPort10 = tcpPart + str(port10)
        self.port10 = fullPort10
        fullPort11 = tcpPart + str(port11)
        self.port11 = fullPort11
        fullPort12 = tcpPart + str(port12)
        self.port12 = fullPort12
        fullPort13 = tcpPart + str(port13)
        self.port13 = fullPort13



        tcpPartStar = "tcp://*:"
        fullPort5 = tcpPartStar + str(port5)
        self.port5 = fullPort5

        self.northward = 0 
        self.eastward = 0
        self.altitude = 0
        self.thetaCoord = 0
        self.phiCoord = 0
        self.rollCoord = 0
        self.northward2 = 0
        self.eastward2 = 0
        self.altitude2 = 0
        self.thetaCoord2 = 0
        self.phiCoord2 = 0
        self.rollCoord2 = 0

        self.lon = 0
        self.lat = 0
        self.alt = 0
        self.omega = 0

        self.lon2 = 0
        self.lat2 = 0
        self.alt2 = 0
        self.omega2 = 0

        self.lwp = 0
        self.lwp2 = 0
        self.rwp = 0
        self.rwp2 = 0
        self.dampConstant = -0.1
#also returns values to simulator. overwrite superclass
    @threaded
    def grabValues(self):
        print("connecting to simulation")
        context5 = zmq.Context()
        socket5 = context5.socket(zmq.REP)
        socket5.bind("tcp://*:5584")
        message = socket5.recv()
#        response = str(self.lwp*self.dampConstant) + " " + str(self.rwp*self.dampConstant) + " " + str(self.lwp2*self.dampConstant) + " " + str(self.rwp2*self.dampConstant)
        response = str(self.lwp*self.dampConstant) + " " + str(self.rwp*self.dampConstant) + " " + str(self.lwp2*self.dampConstant) + " " + str(self.rwp2*self.dampConstant)
        socket5.send(response.encode(encoding))
        print("connected")
        myProj = Proj("+proj=utm +zone=16, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
        lastTime = time.time()
        lastTheta = self.thetaCoord
        lastTheta2 = self.thetaCoord2

        while True:
            response = str(self.lwp*self.dampConstant) + " " + str(self.rwp*self.dampConstant) + " " + str(self.lwp2*self.dampConstant) + " " + str(self.rwp2*self.dampConstant)
            message = socket5.recv()
            socket5.send(response.encode(encoding))
            alphabet = message.decode(encoding)
            data = re.split(' ',alphabet)
            self.northward =   -float(data[2])
            self.eastward = -float(data[0])
            self.altitude = float(data[1])

            self.thetaCoord = (-(float(data[3])) + 90)%360
            self.phiCoord = float(data[4])
            self.rollCoord = float(data[5])

            self.northward2 =   -float(data[8])
            self.eastward2 = -float(data[6])
            self.altitude2 = float(data[7])

            self.thetaCoord2 = (-(float(data[9])) + 90)%360
            self.phiCoord2 = float(data[10])
            self.rollCoord2 = float(data[11])

#            print(self.thetaCoord)
            self.omegaCalm = 0.01

            self.alt = self.altitude
            self.lon, self.lat = myProj(self.eastward +562543.955,self.northward + 4415393.656 ,inverse=True)
            self.alt2 = self.altitude2

            self.lon2, self.lat2 = myProj(self.eastward2 +562543.955,self.northward2 + 4415393.656 ,inverse=True)
            theta = self.thetaCoord                                                                     #avoid race conditions
            theta2 = self.thetaCoord2                                                                     #avoid race conditions
            dt = lastTime - time.time()
            lastTime = time.time()
            dTheta = self.crossPole(theta,lastTheta)
            dTheta2 = self.crossPole(theta2,lastTheta2)
            if(theta != lastTheta):
                lastTheta = theta
                self.omega = self.omegaCalm*(dTheta / dt)
            if(theta2 != lastTheta2):
                lastTheta2 = theta2
                self.omega2 = self.omegaCalm*(dTheta2 / dt)





    def crossPole(self,theta,lastTheta):
        if theta - lastTheta > 180:
            dTheta = -(360 - (theta - lastTheta))
        elif theta - lastTheta < -180:
            dTheta = (theta - lastTheta) + 360
        else:
            dTheta = theta - lastTheta
        return dTheta

#overwrite sendValues as well
    @threaded
    def sendValues(self):
#gps
        context = zmq.Context()
        socket1 = context.socket(zmq.REQ)
        socket1.connect(self.port)
#compass
        context3 = zmq.Context()
        socket3 = context3.socket(zmq.REQ)
        socket3.connect(self.port3)
#gyro
        context4 = zmq.Context()
        socket4 = context4.socket(zmq.REQ)
        socket4.connect(self.port4)

#imu
        context6 = zmq.Context()
        socket6 = context6.socket(zmq.REQ)
        socket6.connect(self.port6)
        context7 = zmq.Context()
        socket7 = context7.socket(zmq.REQ)
        socket7.connect(self.port7)

#gps
        context8 = zmq.Context()
        socket8 = context8.socket(zmq.REQ)
        socket8.connect(self.port8)
#compass
        context9 = zmq.Context()
        socket9 = context9.socket(zmq.REQ)
        socket9.connect(self.port9)
#gyro
        context10 = zmq.Context()
        socket10 = context10.socket(zmq.REQ)
        socket10.connect(self.port10)

#imu
        context11 = zmq.Context()
        socket11 = context11.socket(zmq.REQ)
        socket11.connect(self.port11)
        context12 = zmq.Context()
        socket12 = context12.socket(zmq.REQ)
        socket12.connect(self.port12)


        while True:
            gpsString = str(self.lat) + " " + str(self.lon)
#            print(gpsString)
            socket1.send(gpsString.encode(encoding))
            message = socket1.recv()                            #message doesnt matter here, just an aknowledgement
            compassString = str(self.thetaCoord)
            socket3.send(compassString.encode(encoding))
            message = socket3.recv()

            omegaString = str(self.omega)
            socket4.send(omegaString.encode(encoding))
            message = socket4.recv()

            pitchString = str(self.phiCoord)
            socket6.send(pitchString.encode(encoding))
            message = socket6.recv()

            rollString = str(self.rollCoord)
            socket7.send(rollString.encode(encoding))
            message = socket7.recv()


            gpsString2 = str(self.lat2) + " " + str(self.lon2)
            socket8.send(gpsString2.encode(encoding))
            message = socket8.recv()
            compassString2 = str(self.thetaCoord2)
            socket9.send(compassString2.encode(encoding))
            message = socket9.recv()

            omegaString2 = str(self.omega2)
            socket10.send(omegaString2.encode(encoding))
            message = socket10.recv()

            pitchString2 = str(self.phiCoord2)
            socket11.send(pitchString2.encode(encoding))
            message = socket11.recv()

            rollString2 = str(self.rollCoord2)
            socket12.send(rollString2.encode(encoding))
            message = socket12.recv()






#grab output speed values from driving algorithm to send back to simulator
    @threaded
    def grabSpeeds(self):
        context2 = zmq.Context()
        socket2 = context2.socket(zmq.REQ)
        socket2.connect(self.port2)

        context13 = zmq.Context()
        socket13 = context13.socket(zmq.REQ)
        socket13.connect(self.port13)

        while True:
            response = "thanks"
            socket2.send(response.encode(encoding))
            message = socket2.recv()
            alphabet = message.decode(encoding)
            data = re.split(' ',alphabet)
            self.lwp =   float(data[0])
            self.rwp = float(data[1]) 

            socket13.send(response.encode(encoding))
            message = socket13.recv()
            alphabet = message.decode(encoding)
            data = re.split(' ',alphabet)
            self.lwp2 =   float(data[0])
            self.rwp2 = float(data[1])



class radarSensorThrow(sensorThrow):
        def   __init__(self,name,port,usb,baudrate,nValues,deviations):
                super().__init__(name,port,usb,baudrate,nValues,deviations)
        # variables used in the loop
                self.done = False
                self.prior_output_str = ''
                self.prior_output_clock = 0
                self.speed_available = [True, True, True, True]
                self.speed_clock = [0, 0, 0, 0]
                self.distances_g = [0,0,0,0]
                import radarRead                        #rpi's gpio library is not needed for simulated runs

        def connectToSensor():
                kit = MotorKit()

                encoding = 'utf-8'
        #import re

        # Ops241A module command codes
                Ops241A_Product_Information = '?P'
                Ops241A_BlankLinesOn = 'BL'

                d0_g = 0
                d1_g = 0
                d2_g = 0
                d3_g = 0


                self.radarRead.resetRadar()
                time.sleep(8.0)
                ser_list = get_serial_ports()

                if len(ser_list) == 0:
                        print("Did not find any OPS 241 modules.  Please attach them and restart.")
                        sys.exit()

                print("Initializing Ops241A Modules")
        # Initialize the USB port(s) to read from the OPS-241A module
                ser_1=serial.Serial(
                        port = ser_list[1],
                        baudrate = 9600,
                        parity = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        bytesize = serial.EIGHTBITS,
                        timeout = 0.01,
                        writeTimeout = 2
                        )
                ser_1.flushInput()
                ser_1.flushOutput()
                sendSerCmd(ser_1,"Prep board #1:", Ops241A_BlankLinesOn)

                now = time.clock()
                speed_1_clock = now
                speed_2_clock = now
                speed_3_clock = now
                speed_4_clock = now

                ser_2 = 0
                ser_3 = 0
                ser_4 = 0


                if len(ser_list) > 1:
                        ser_2=serial.Serial(
                            port = ser_list[2],
                            baudrate = 9600,
                            parity = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE,
                            bytesize = serial.EIGHTBITS,
                            timeout = 0.01,
                            writeTimeout = 2
                            )
                        ser_2.flushInput()
                        ser_2.flushOutput()
                        sendSerCmd(ser_2,"Prep board #2:", Ops241A_BlankLinesOn)

                if len(ser_list) > 2:
                        ser_3=serial.Serial(
                            port = ser_list[3],
                            baudrate = 9600,
                            parity = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE,
                            bytesize = serial.EIGHTBITS,
                            timeout = 0.01,
                            writeTimeout = 2
                            )
                        ser_3.flushInput()
                        ser_3.flushOutput()
                        sendSerCmd(ser_3,"Prep board #3:", Ops241A_BlankLinesOn)

                if len(ser_list) > 3:
                        ser_4=serial.Serial(
                            port = ser_list[4],
                            baudrate = 9600,
                            parity = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE,
                            bytesize = serial.EIGHTBITS,
                            timeout = 0.01,
                            writeTimeout = 2
                            )
                        ser_4.flushInput()
                        ser_4.flushOutput()
                        sendSerCmd(ser_4,"Prep board #4:", Ops241A_BlankLinesOn)

                sensor_list = [ser_1, ser_2, ser_3, ser_4]
                time.sleep(1)       




        # really, these should have been put in a List rather than ser_1, ser_2 and ser_2

        # some constants that define behaviour of the script
                low_speed_cutoff = 0   # filter out anything below this threahold
                expired_reading_cutoff = 1000   # if no report for this length of time, must have gone out of range.  
        ## note about 'expired_reading_cutoff': when this was 0.5, some how, things expired that shouldn't have
                repeat_interval = 0.25  # desired repitition time for the same existing activity value



        # sendSerialCommand: function for sending commands to the OPS-241A module
        def sendSerCmd(ser, descrStr, commandStr) :
                data_for_send_str = commandStr
                data_for_send_bytes = str.encode(data_for_send_str)
                print(descrStr, commandStr)
                ser.write(data_for_send_bytes)
                # Initialize message verify checking
                ser_message_start = '{'
                ser_write_verify = False
                # Print out module response to command string
                while not ser_write_verify :
                    data_rx_bytes = ser.readline()
                    data_rx_length = len(data_rx_bytes)
                    if (data_rx_length != 0) :
                        data_rx_str = str(data_rx_bytes)
                        if data_rx_str.find(ser_message_start) :
                            ser_write_verify = True
                            return data_rx_bytes
                        
        def resetRadar():
                kit.motor1.throttle = 0.0
                time.sleep(1.0)
                kit.motor1.throttle = 1.0

        def get_serial_ports():
                """ Lists serial port names

                    :raises EnvironmentError:
                        On unsupported or unknown platforms
                    :returns:
                        A list of the serial ports available on the system
                """
                if sys.platform.startswith('win'):
                    ports = ['COM%s' % (i + 1) for i in range(256)]
                elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
                    # this excludes your current terminal "/dev/tty"
                    ports = glob.glob('/dev/ttyACM*')
                elif sys.platform.startswith('darwin'):
                    ports = glob.glob('/dev/tty.*')
                else:
                    raise EnvironmentError('Unsupported platform')

                result = []
                for port in ports:
                    try:
                        s = serial.Serial(port)
                        # this request-response sometimes hangs, but would be good:
                        # reply = sendSerCmd(s, "Ensure it's an 241 on port:"+port, Ops241A_Product_Information)
                        # if reply.find("241") == -1:
                        result.append(port)
                        print("Active port:"+port)
                        s.close()
                    except (OSError, serial.SerialException):
                        pass
                return result

        def hasNumbers(inputString):
                return any(char.isdigit() for char in inputString)

        @threaded
        def sendDistances():
                context = zmq.Context()
                socket = context.socket(zmq.REQ)
                socket.connect("tcp://localhost:5565")
                distances_l = [0,0,0,0]
                lastDistances = [0,0,0,0]
                while True:
                    for i in range(4):
                        firstParse = re.split(',',str(distances_g[i]))
                        if hasNumbers(str(firstParse)) and firstParse[0] != '0':
        #                print(firstParse)
                            secondParse = firstParse[1][:-5]
        #                print(secondParse)
                            distances_l[i] = secondParse[0:4]
        #                print(distances_l[i])
                        else:
                            distances_l[i] = 0
                    try:
                        response = str(float(distances_l[0])) + " " + str(float(distances_l[1])) + " " + str(float(distances_l[2])) + " " + str(float(distances_l[3])) 
                        socket.send(response.encode(encoding))
                        message = socket.recv()
                        print(response)
                    except:
                        time.sleep(0.001)
                    time.sleep(0.001)

        @threaded
        def readRadar():
        # Loop forever
                global distances_g
                
                while not done:
                    idx = 0
                    now = time.clock()
                    for sensor in sensor_list:
                        if (sensor !=0):
                            Ops241_rx_bytes = sensor.readline()
                            distances_g[idx] = Ops241_rx_bytes
                            # Check for speed information from OPS241-A
                            Ops241_rx_bytes_length = len(Ops241_rx_bytes)
                            if (Ops241_rx_bytes_length != 0) :
                                Ops241_rx_str = str(Ops241_rx_bytes)
        #                    print("%d: "% (sensor_list.index(sensor)+1) +Ops241_rx_str)
                                if Ops241_rx_str.find('{') == -1 :
                                    # Speed data found
                                    try:
                                        Ops241_rx_float = float(Ops241_rx_bytes)
                                        if (Ops241_rx_float > low_speed_cutoff):
                                            speed_available[sensor_list.index(sensor)] = True
                                            speed_clock[sensor_list.index(sensor)] = time.clock()
                                    except ValueError:
                                        speed_available[sensor_list.index(sensor)] = False  
        # check for expired speed reports (and if expired, declare speed not available)
                        idx = idx + 1
                    if (now - speed_clock[0] > expired_reading_cutoff):
                        speed_available[0] = False
                    if (now - speed_clock[1] > expired_reading_cutoff):
                        speed_available[1] = False
                    if (now - speed_clock[2] > expired_reading_cutoff):
                        speed_available[2] = False
                    if (now - speed_clock[3] > expired_reading_cutoff):
                        speed_available[3] = False

class lidarSesorThrow(sensorThrow):
        def __init__(self,name,port,usb,baudrate,nValues,deviations = []):
                super().__init__(name,port,usb,baudrate,nValues,deviations)


        @threaded
        def sendValues(self):
            context = zmq.Context()
            socketZmq = context.socket(zmq.REQ)
            socketZmq.connect(self.port)
            deviated = [0]*int(self.nValues) 
            while True:
                returnString = ""
                for i in range(self.nValues):
                    if len(self.deviations) > 0:
                        deviated[i] = int(self.values[i]) + int(self.deviations[i])
                    else:
                        deviated[i] = self.values[i]
                    returnString = returnString.join(str(deviated[i])) 
                socketZmq.send(returnString.encode(encoding))
                message = socketZmq.recv()

        @threaded
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
                        d_l = depth_image[400,:]                                #depth_line
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

                        self.values[0] = lidarLeft
                        self.values[1] = lidarMid
                        self.values[2] = lidarRight
                finally:
                # Stop streaming
                    pipeline.stop()


class gpsSensorThrow(sensorThrow):
#import superclass methods
    def __init__(self,name,port,usb,baudrate,nValues,deviations = []):
         super().__init__(name,port,usb,baudrate,nValues,deviations)
#override grabValues method
        
    def convertToDecimal(coordinate):
            coord = coordinate/100
            decimal = coord - math.floor(coord)
            decimal = 100*decimal/60
            return math.floor(coord) + decimal

    @threaded
    def grabValues(self):
        while True:
            line = self.ser.readline()
            while (not line.startswith(b'$GNRMC')):
                line = self.ser.readline()
            string = line.decode(encoding)
            if(string.startswith('$GNRMC')):
                msg = pynmea2.parse(string)
                longitude = msg.lon
                latitude = msg.lat
                self.values[0] = latitude
                self.values[1] = longitude

#subclass for the case of compass (%360)
class compassSensorThrow(sensorThrow):
#import superclass methods 
    def __init__(self,name,port,usb,baudrate,nValues,deviations):
         super().__init__(name,port,usb,baudrate,nValues,deviations)

#override sendValues method
    @threaded
    def sendValues(self):
        if not deviations:
            deviations = [0]*nValues
        context = zmq.Context()
        socketZmq = context.socket(zmq.REQ)
        socketZmq.connect(self.port)

        while True:
            for i in range(self.nValues):
                deviated[i] = (self.values[i] + deviations[i])%360
                returnString = returnString.join(deviated[i])
            socketZmq.send(returnString.encode(encoding))
            message = socketZmq.recv()


#the upper section denotes a sensor class which contains handles to make sensor stuff easier and uniform
#the lower section acts to allow the user to launch sensors on independent system threads

if __name__ == "__main__":

    arguments = sys.argv

    if arguments[1] == "gps":
        sensor = gpsSensorThrow(arguments[1],arguments[2],argumets[3],arguments[4],arguments[5])
        sensor.connectToSensor()
        sensor.sendValues()
    elif arguments[1] == "radar":
        sensor = radarSensorThrow(arguments[1],arguments[2],argumets[3],arguments[4],arguments[5])
        sensor.connectToSensor()
        sensor.readRadar()
        sensor.sendDistances()
    elif arguments[1] == "lidar":
        sensor = lidarSensorThrow(arguments[1],arguments[2],argumets[3],arguments[4],arguments[5])
        sensor.readLidar()                                                                                    #could be split out into connect and read
        sensor.sendValues()
                 
    else:
        sensor =  sensorThrow(arguments[1],arguments[2],argumets[3],arguments[4],arguments[5],arguments[6],arguments[7])
        sensor.connectToSensor()
        sensor.sendValues()



