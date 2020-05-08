import smbus                    #import SMBus module of I2C
import math
from time import sleep          #import

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards

#class sensor_gyro:
#    def __init__(self,Device_Address,addr):
#        self.Device_Address = Device_Address
#        self.addr = addr
def MPU_Init(Device_Address):
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)
        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(Device_Address,addr):
        #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
        #concatenate higher and lower value
        value = ((high << 8) | low)
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

class calculate:
    def __init__(self,addres):
        acc_x = read_raw_data(addres,ACCEL_XOUT_H)
        acc_y = read_raw_data(addres,ACCEL_YOUT_H)
        acc_z = read_raw_data(addres,ACCEL_ZOUT_H)
        Ax = acc_x*(9.81/16384.0)
        Ay = acc_y*(9.81/16384.0)
        Az = acc_z*(9.81/16384.0)
        self.posx = math.atan(Ax/math.sqrt((math.pow(Ay,2)+math.pow(Az,2))))*(180.0/math.pi)
        self.posy = math.atan(Ay/math.sqrt((math.pow(Ax,2)+math.pow(Az,2))))*(180.0/math.pi)

#bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
#Device_Address = 0x68   # MPU6050 device address

