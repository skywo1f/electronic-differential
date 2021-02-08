import os
import re
import RPi.GPIO as GPIO
import time

ledpin = 12				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,100)		#create PWM instance with frequency (1000)
pi_pwm.start(0)				#start PWM of required Duty Cycle 

while True:
	stream = os.popen('vcgencmd measure_temp')
	output = stream.read()
	temp = re.findall(r'\d+', output) 
	T = temp[0]
	duty = T
	pi_pwm.ChangeDutyCycle(int(duty))
	print(duty)
	time.sleep(10)
