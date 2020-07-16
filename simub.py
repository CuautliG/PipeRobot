import time
import gyro

gyro.MPU_Init(0x68)

imub = open("imub.txt","w+")

def updateb(address):
        a = gyro.calculate(address)
        t = time.localtime()
        ctime = time.strftime("%H:%M:%S ",t)
        imub.write(ctime)
        imub.write(" , ")
        print(round( (a.posy*(-1)) ,2))
        imub.write("%f"%round( (a.posy*(-1)) ,2))
        imub.write(" , ")
        imub.write("%f\n"%round(a.posz,2))

while True:
        updateb(0x68)
        time.sleep(0.25)

