import time
import RPi.GPIO as GPIO

#pins to use
CA = 24
CB = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(CA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(CB,GPIO.IN, pull_up_down=GPIO.PUD_UP)

counter = 0
lastA = GPIO.input(CA)
f = open("efd.txt","w+")

def my_callback():
        global lastA
        global counter
        while True:
                        stateA = GPIO.input(CA)
                        if stateA != lastA:
                                stateB = GPIO.input(CB)
                                if stateB != stateA:
                                        counter += 1
                                else:
                                        counter -=1
                                t = time.localtime()
                                ctime = time.strftime("%H:%M:%S",t)
                                print (ctime)
                                print (counter)
                                f.write(ctime)
                                f.write(" %d\n" %counter)
                        lastA = stateA

my_callback()
