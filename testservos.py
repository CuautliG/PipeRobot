import time
import RPi.GPIO as GPIO
import array
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16) #Activate all the channels
GPIO.setmode(GPIO.BCM)
#Maximun range from 550 to 2400
#Recommended library 700 to 2380
#Best for some of them 550 2400
#Suitable for the sixteen 550 to 2300

m1=array.array('i',[0,0,0,0,26,13,12,6,21,20,19,16])

for i in range(0,16): #Declaration for the servomotors and gearmotors
        if(i>3 and i<12): #Motors
                kit.servo[i].set_pulse_width_range(0,19988)
                kit.servo[i].actuation_range = 100
        else: #Servomotors
                kit.servo[i].set_pulse_width_range(550,2300)
                kit.servo[i].actuation_range = 190

for j in range(len(m1)): #Output declaration for the GearMotors
        GPIO.setup(m1[j],GPIO.OUT)

def motor(dir,number,speed):
        if (number>3 and number<12):
                kit.servo[number].angle = speed
        else:
                print('The gearmotor does not exist')
        if number == 7 or number == 9 or number == 10:
                if dir == 1:
        	        GPIO.output(m1[number],0)
                else:
        	        GPIO.output(m1[number],1)
        else :
	        if dir == 1:
		        GPIO.output(m1[number],1)
	        else:
		        GPIO.output(m1[number],0)

def servo(number,angle):
	if (number>3 and number<12):
		print('The servomotor does not exist')
	else:
		kit.servo[number].angle = angle

for m in range (0,4):
	servo(m,0)
	servo(m+12,0)

for l in range (4,12):
    motor(1,l,0)

time.sleep(5)
#Set to the diameter
servo(12,180)
servo(14,180)
servo(1,180)
servo(3,180)
time.sleep(1)
servo(0,180)
servo(2,180)
servo(13,180)
servo(15,180)
time.sleep(1)


#for l in range (4,12):
#    motor(1,l,80)

#time.sleep(3)

for l in range (4,12):
    motor(1,l,0)


"""
#Adjust level
motor(-1,5,80)
motor(1,7,30)
time.sleep(2)

for l in range(1,10):
    servo(12,110-l)
    servo(14,110-l)
    servo(1,70-l)
    servo(3,70-l)

motor(0,5,0)
motor(0,7,0)
motor(0,8,0)
motor(0,10,0)
time.sleep(5)
servo(12,0)
servo(14,0)
servo(1,0)
servo(3,0)
"""

