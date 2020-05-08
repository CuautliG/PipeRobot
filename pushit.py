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

#Adjust level
for k in range(0,4):
    servo(k,0)
    servo(k+12,0)
