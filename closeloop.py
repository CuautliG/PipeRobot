from time import sleep
import RPi.GPIO as GPIO
import servgear
import PID

#pins of the encoders at the back module
BAA = 14
BAB = 15
BBA = 17
BBB = 4
BCA = 18
BCB = 27
BDA = 22
BDB = 23
#pins of the encoder at the front module
FAA = 5
FAB = 7
FBA = 8
FBB = 11
FCA = 9
FCB = 10
FDA = 24
FDB = 25

#Initialize the pins in GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(FBA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FBB,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FDA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FDB,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FAA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FAB,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FCA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FCB,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BBA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BBB,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BDA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BDB,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BAA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BAB,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BCA,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BCB,GPIO.IN, pull_up_down=GPIO.PUD_UP)
#Encoder class
class Encoder(object):
    def __init__(self, pin_a, pin_b):
        self._value = 0
        GPIO.add_event_detect(pin_a,GPIO.BOTH,callback = self._increment)
        if pin_b != 4:
            GPIO.add_event_detect(pin_b,GPIO.BOTH,callback = self._increment)
    def reset(self):
        self._value = 0
    def _increment(self,channel):
        self._value += 1
    @property
    def value(self):
        return self._value

#Initialize encoders
efa = Encoder(FAA,FAB)
efb = Encoder(FBA,FBB)
efc = Encoder(FCA,FCB)
efd = Encoder(FDA,FDB)
eba = Encoder(BAA,BAB)
ebb = Encoder(BBA,BBB)
ebc = Encoder(BCA,BCB)
ebd = Encoder(BDA,BDB)

#Move all the gearmotors with the required pwm
def move(pwm):
#    servgear.motor(1,9,pwm)
    for i in range(4,8):
        servgear.motor(0,i,pwm)
        servgear.motor(0,i+4,pwm)

#Initial variables
#cpr in 100 cm Eq. = (goal_in_cm * cpr_in_a_rev)/(pi*dia)=(100*23673.84)/(pi*3.1)
TARGET = 243084
KP = 0.022
KD = 0.001
KI = 0.000
MAX_SPEED = 100
MIN_SPEED = 0
#Initialize the PD controller
pid = PID.PID(KP, KI, KD)
pid.SetPoint = TARGET
pid.setSampleTime(1)

#Create a txt file to save the encoders readings
cltest = open("cltest.txt","w+")

while True:
    #Encoders average
    prom_encoders=(efa.value+efb.value+efc.value+efd.value)/4
    #prom_encoders = efb.value
    error = TARGET - prom_encoders #Average error
    pid.update(prom_encoders)      #update PD
    PWM = pid.output               #Update the PWM according to the PD 
    PWM = max(min( int(PWM), MAX_SPEED), MIN_SPEED) #Adjust the PWM to be between 0 and 100
    #Modify the speed of the motors
    move(PWM)
    #Print current values of the encoders and error next to the pwm (0 to 100)
    print("eba {} ebb {} ebc {} ebd {}".format((efa.value+efb.value)/2, ebb.value*2, (efb.value+efc.value)/2, (efd.value+efa.value)/2))
    print("efa {} efb {} efc {} efd {}".format(efa.value, efb.value, efc.value, efd.value))
    print("Error {} Speed {}".format(error, PWM))
    cltest.write("%d"%((efa.value+efb.value)/2))
    cltest.write(" , ")
    cltest.write("%d"%(ebb.value*2))
    cltest.write(" , ")
    cltest.write("%d"%((efb.value+efc.value)/2))
    cltest.write(" , ")
    cltest.write("%d"%((efd.value+efa.value)/2))
    cltest.write(" , ")
    cltest.write("%d"%efa.value)
    cltest.write(" , ")
    cltest.write("%d"%efb.value)
    cltest.write(" , ")
    cltest.write("%d"%efc.value)
    cltest.write(" , ")
    cltest.write("%d\n"%efd.value)
