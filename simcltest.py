from time import sleep
import PID

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
cltest = open("simcl.txt","w+")
encoders = 0
while True:

    error = TARGET - encoders #Average error
    pid.update(encoders)      #update PD
    PWM = pid.output               #Update the PWM according to the PD 
    PWM = max(min( int(PWM), MAX_SPEED), MIN_SPEED) #Adjust the PWM to be between 0 and 100
    sleep(0.001)
    print("encoders {} ".format(encoders))
    print("Error {} Speed {}".format(error, PWM))
    if error > 0:
        encoders = encoders + (12.626*PWM/100)
    else:
        emcpders = encoders
    cltest.write("%d\n"%encoders)
