# -*- coding: utf-8 -*-

# import module
from time import sleep
import time
import math

# import my class
from MPU9250 import MPU9250
from RollPitchYaw import RollPitchYaw


def main():
    mpu = MPU9250()
    mpu.ready()

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
        
        sleep(0.15)


if __name__ == '__main__':
    main()
