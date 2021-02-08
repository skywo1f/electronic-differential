# multi_ops241.py - interact with 3 OPS-241 modules and represent their activity on the output

# Import modules (glob for linux.  re can probably go away)
#import os
import sys
import time
import glob
import decimal
import serial
import RPi.GPIO as GPIO
import threading
from adafruit_motorkit import MotorKit
import zmq
import re

class radarThrow():
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


		resetRadar()
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

	# variables used in the loop
		done = False
		prior_output_str = ''
		prior_output_clock = 0
		speed_available = [True, True, True, True]
		speed_clock = [0, 0, 0, 0]
		distances_g = [0,0,0,0]

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

	 
	thread1 = threading.Thread(target=readRadar)
	thread1.daemon = True
	thread1.start()

	thread2 = threading.Thread(target=sendDistances)
	thread2.daemon = True
	thread2.start()

	while True:
		time.sleep(100)
	 
