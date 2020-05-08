import gyro
import math
from time import sleep          #import

gyro.MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

while True:
	#Read Accelerometer raw value
	acc_x = gyro.read_raw_data(gyro.ACCEL_XOUT_H)
	acc_y = gyro.read_raw_data(gyro.ACCEL_YOUT_H)
	acc_z = gyro.read_raw_data(gyro.ACCEL_ZOUT_H)
	Ax = acc_x*(9.81/16384.0)
	Ay = acc_y*(9.81/16384.0)
	Az = acc_z*(9.81/16384.0)

	angx=math.atan(Ax/math.sqrt((math.pow(Ay,2)+math.pow(Az,2))))*(180.0/math.pi)
	angy=math.atan(Ay/math.sqrt((math.pow(Ax,2)+math.pow(Az,2))))*(180.0/math.pi)
	sleep(1)

	print (angx)
	print (angy)
