# -*- coding: utf-8 -*-

# import module
from time import sleep
import time
import math

# import my class
from MPU9250 import MPU9250
from RollPitchYaw import RollPitchYaw


def main():
    # class MPU9250
    mpu = MPU9250()
    # ready mpu9250
    mpu.ready()
    
    # class RollPitchYaw
    rpy = RollPitchYaw()

    # get start time
    startTime = time.time()
    
    while True:
        # get data(mpu9250)
        accel = mpu.readAccel()
        gyro = mpu.readGyro()
        temp = mpu.readTemp()
        magnet = mpu.readMagnet()
        
        # calc roll, pitch, yaw
        roll = rpy.calcRoll(accel)
        pitch = rpy.calcPitch(accel)
        yaw = rpy.calcYaw(magnet, roll, pitch)
        
        # get now time
        nowTime = time.time() - startTime
        
        # print
        print nowTime,math.degrees(roll),math.degrees(pitch),math.degrees(yaw)
        
        # sleep
        sleep(0.15)


if __name__ == '__main__':
    main()
