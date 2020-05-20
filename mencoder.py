#from_future_importdivision
import time
import RPi.GPIO as GPIO
from time import sleep
import Adafruit_PCA9685
#pins to use
A = 4
B = 17
dir0 = 12 #dir pin

#Define type of GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(A,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(B,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(dir0,GPIO.OUT)

pwm = Adafruit_PCA9685.PCA9685()

def set_servo_pulse(channel,pulse):
	pulse_length = 1000000
	pulse_length//=60
	print('{0}us per biy'.format(pulse_length))
	pulse *= 1000
	pulse //= pulse_length
	pwm.set_pwm(channel,0,pulse)

pwm.set_pwm_freq(60)
GPIO.output(dir0,0)
pwm.set_pwm(6,0,000)

def my_callback(channel):
	global lastA
	global counter
	try:
		while True:
			stateA = GPIO.input(A)
			if stateA != lastA:
				stateB = GPIO.input(B)
				if stateB != stateA:
					counter += 1
				else:
					counter -=1
				print ( counter)
			lastA = stateA

	finally:
		print "Ending"

counter = 0
lastA = GPIO.input(A)
GPIO.add_event_detect(17,GPIO.FALLING,callback=my_callback,bouncetime=10)

raw_input("Enter anything")
pwm.set_pwm(0,0,4000)
while counter < 3000:
	GPIO.output(dir0,0)
	pwm.set_pwm(0,0,4000)

GPIO.cleanup
