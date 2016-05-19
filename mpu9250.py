# -*- coding: utf-8 -*-

# import module
#import smbus
import math
import numpy as np
from time import sleep
import time

# import my class
from MPU9250 import MPU9250


class RollPitchYaw():
    def calcRoll(self, accel):
        x = accel[0]
        y = accel[1]
        z = accel[2]
        try:
            phi = math.atan(y / z)
        except:
            if y > 0:
                phi = 1.0
            else:
                phi = -1.0
        return phi

    def calcPitch(self, accel):
        x = accel[0]
        y = accel[1]
        z = accel[2]
        try:
            theta = math.atan(-x / math.sqrt(y * y + z * z))
        except:
            if x > 0:
                theta = -1.0
            else:
                theta = 1.0
        return theta

    def calcYaw(self, magnet, roll, pitch):
        magX = magnet[0]
        magY = magnet[1]
        magZ = magnet[2]
        rowX = magX
        rowY = magY
        rowZ = -magZ
        row = np.matrix([[rowX], [rowY], [rowZ]])
        A = np.matrix([\
            [math.cos(roll), math.sin(roll) * math.sin(pitch), math.cos(pitch) * math.sin(roll)]\
            , [0, math.cos(pitch), -math.sin(pitch)]\
            , [-math.sin(roll), math.sin(pitch) * math.cos(roll), math.cos(roll) * math.cos(pitch)]\
            ])
        calib = A * row
        calibX = row[0]
        calibY = row[1]
        calibZ = row[2]
        try:
            yaw = math.atan(-calibY / calibX)
        except:
            if calibY > 0:
                yaw = 1.0
            else:
                yaw = -1.0
        return yaw


def main():
    mpu = MPU9250()
    mpu.ready()
    #mpu.write(MPU9250_ADDRESS, 0x6B, 0x00)
    #mpu.write(MPU9250_ADDRESS, 0x37, 0x02)
    
    rpy = RollPitchYaw()

    startTime = time.time()
    
    while True:
        # get data
        accel = mpu.readAccel()
        gyro = mpu.readGyro()
        temp = mpu.readTemp()
        magnet = mpu.readMagnet()
               
        roll = rpy.calcRoll(accel)
        pitch = rpy.calcPitch(accel)
        yaw = rpy.calcYaw(magnet, roll, pitch)
        
        nowTime = time.time() - startTime
        
        print nowTime,math.degrees(roll),math.degrees(pitch),math.degrees(yaw)
        #print nowTime, math.degrees(yaw), accel[0], accel[1], accel[2]
        #print nowTime,math.degrees(roll),math.degrees(pitch),math.degrees(yaw)
        #print nowTime, accel[0], accel[1], accel[2]

        sleep(0.15)


if __name__ == '__main__':
    main()
