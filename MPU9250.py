# -*- coding: utf-8 -*-

# import module
import smbus
from time import sleep

#
# define
#
# slave address
MPU9250_ADDRESS = 0x68
AK8963_ADDRESS = 0x0C
# register address(MPU9250)
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40
TEMP_OUT_H = 0x41
TEMP_OUT_L = 0x42
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48
# register address(AK8963)
MAGNET_XOUT_L = 0x03
MAGNET_XOUT_H = 0x04
MAGNET_YOUT_L = 0x05
MAGNET_YOUT_H = 0x06
MAGNET_ZOUT_L = 0x07
MAGNET_ZOUT_H = 0x08


class MPU9250():
    def __init__(self):
        self.bus = smbus.SMBus(1)
    
    def write(self, address, register, value):
        self.bus.write_byte_data(address, register, value)
    
    def read(self, address, register):
        data = self.bus.read_byte_data(address, register)
        return data
    
    def ready(self):
        self.write(MPU9250_ADDRESS, 0x6B, 0x00)
        self.write(MPU9250_ADDRESS, 0x37, 0x02)
    
    def readLine(self, address, high, low):
        hline = self.read(address, high)
        lline = self.read(address, low)
        value = (hline << 8) + lline        
        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def readAccel(self):
        xout = self.readLine(MPU9250_ADDRESS, ACCEL_XOUT_H, ACCEL_XOUT_L)
        yout = self.readLine(MPU9250_ADDRESS, ACCEL_YOUT_H, ACCEL_YOUT_L)
        zout = self.readLine(MPU9250_ADDRESS, ACCEL_ZOUT_H, ACCEL_ZOUT_L)
        x = 2.0 * xout / 32768.0
        y = 2.0 * yout / 32768.0
        z = 2.0 * zout / 32768.0
        return [x, y, z]

    def readGyro(self):
        xout = self.readLine(MPU9250_ADDRESS, GYRO_XOUT_H, GYRO_XOUT_L)
        yout = self.readLine(MPU9250_ADDRESS, GYRO_YOUT_H, GYRO_YOUT_L)
        zout = self.readLine(MPU9250_ADDRESS, GYRO_ZOUT_H, GYRO_ZOUT_L)
        x = 250.0 * xout / 32768.0
        y = 250.0 * yout / 32768.0
        z = 250.0 * zout / 32768.0
        return [x, y, z]

    def readTemp(self):
        temp_out = self.readLine(MPU9250_ADDRESS, TEMP_OUT_H, TEMP_OUT_L)
        temp = temp_out / 340.0 + 36.53
        return temp
    
    def readMagnet(self):
        self.write(0x0c, 0x0A, 0x12)
        sleep(0.15)
        xout = self.readLine(AK8963_ADDRESS, MAGNET_XOUT_H, MAGNET_XOUT_L)
        yout = self.readLine(AK8963_ADDRESS, MAGNET_YOUT_H, MAGNET_YOUT_L)
        zout = self.readLine(AK8963_ADDRESS, MAGNET_ZOUT_H, MAGNET_ZOUT_L)
        x = 1200.0 * xout / 4096.0
        y = 1200.0 * yout / 4096.0
        z = 1200.0 * zout / 4096.0
        return [x, y, z]

