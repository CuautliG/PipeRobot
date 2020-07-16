import time
import gyro

gyro.MPU_Init(0x69)

imuf = open("imuf.txt","w+")

def updateb(address):
        a = gyro.calculate(address)
        t = time.localtime()
        ctime = time.strftime("%H:%M:%S",t)
        imuf.write(ctime)
        imuf.write(" , ")
#        print(round(a.posz,2))
        imuf.write("%f"%round(a.posx,2))
        imuf.write(" , ")
        imuf.write("%f\n"%round(a.posz,2))

while True:
        updateb(0x69)
        time.sleep(0.25)

