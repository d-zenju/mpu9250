# -*- coding: utf-8 -*-

# import module
from time import sleep
import time

# import my class
from MPU9250 import MPU9250
from RollPitchYaw import RollPitchYaw


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
