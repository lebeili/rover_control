import logging
import io
import json  
from http import server
from threading import Condition
import requests
import RPi.GPIO as GPIO
from time import sleep

ledpin = 12				# PWM pin connected to LED
GPIO.setwarnings(False)			#disable warnings
GPIO.setmode(GPIO.BOARD)		#set pin numbering system
GPIO.setup(ledpin,GPIO.OUT)
pi_pwm = GPIO.PWM(ledpin,1000)		#create PWM instance with frequency
pi_pwm.start(40)				#start PWM of required Duty Cycle 

while True:
	#for duty in range(0,101,1):
	#	pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
	#	sleep(0.01)
	#sleep(0.5)
	#for duty in range(100,-1,-1):
	#	pi_pwm.ChangeDutyCycle(duty)
	#	sleep(0.01)
	#sleep(0.5)
	try:
		json_data = requests.get('http://109.204.233.11/api/getControls').json()
		#print(json_data['x'])
		pi_pwm.ChangeDutyCycle(min(abs(int(json_data['x'])), 100))
		
	except:
		print('error')
	#sleep(0.3)
