# -*- coding: utf-8 -*-

# import module
import smbus
import math
import numpy as np
from time import sleep
import time


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

    def calcRoll(self, accel):
        x = accel[0]
        y = accel[1]
        z = accel[2]
        phi = math.atan(x / z)
        return phi

    def calcPitch(self, accel):
        x = accel[0]
        y = accel[1]
        z = accel[2]
        theta = math.atan(-y / math.sqrt(x * x + z * z))
        return theta

    def calcYaw(self, magnet, roll, pitch):
        magX = magnet[0]
        magY = magnet[1]
        magZ = magnet[2]
        rowX = magY
        rowY = magX
        rowZ = -magZ
        row = np.matrix([[rowX], [rowY], [rowZ]])
        A = np.matrix([\
            [math.cos(pitch), math.sin(roll) * math.sin(pitch), math.cos(roll) * math.sin(pitch)]\
            , [0, math.cos(roll), -math.sin(pitch)]\
            , [-math.sin(pitch), math.sin(roll) * math.cos(pitch), math.cos(roll) * math.cos(pitch)]\
            ])
        calib = A * row
        calibX = row[0]
        calibY = row[1]
        calibZ = row[2]
        yaw = math.atan(-calibY / calibX)
        return yaw

    def accel2RP(self, accel):
        x = accel[0]
        y = accel[1]
        z = accel[2]
        
        #theta = math.degrees(math.atan(x / math.sqrt(y * y + z * z)))
        #psi = math.degrees(math.atan(y / math.sqrt(x * x + z * z)))
        #phi = math.degrees(math.atan(math.sqrt(x * x + y * y) / z))
        #r = math.degrees(math.asin(x))
        #p = math.degrees(math.asin(y / math.sqrt(1 - (x * x))))
        
        phi = math.degrees(math.asin(x / z))
        theta = math.degrees(math.asin(y / z))
        
        return [phi, theta]
        #return [r, p]

    def magnet2Azimuth(self, magnet):
        x = magnet[0]
        y = magnet[1]
        z = magnet[2]

        l = math.sqrt(x * x + y * y + z * z)
        phi = math.atan(z / l)
        theta = math.degrees(math.atan(x / (l * math.cos(phi))))

        return theta

    def calibAccel(self):
        print 'Accel Calibration'
        flag = 0
        xyz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        while True:
            accel = self.readAccel()
            if flag is 0:
                print '+X Accel:'
                print accel[0]
            if flag is 1:
                print '-X Accel:'
                print accel[0]
            if flag is 2:
                print '+Y Accel:'
                print accel[1]
            if flag is 3:
                print '-Y Accel:'
                print accel[1]
            if flag is 4:
                print '+Z Accel:'
                print accel[2]
            if flag is 5:
                print '-Z Accel:'
                print accel[2]

            print 'OK: y, Bad: any'
            str = raw_input()
            if str is 'y':
                if (flag is 0) or (flag is 1):
                    xyz[flag] = accel[0]
                if (flag is 2) or (flag is 3):
                    xyz[flag] = accel[1]
                if (flag is 4) or (flag is 5):
                    xyz[flag] = accel[2]
                flag = flag + 1
                if flag is 6:
                    break
        
        x_off = 0.5 * (xyz[0] + xyz[1])
        y_off = 0.5 * (xyz[2] + xyz[3])
        z_off = 0.5 * (xyz[4] + xyz[5])
        x_gain = 0.5 * (xyz[0] - xyz[1])
        y_gain = 0.5 * (xyz[2] - xyz[3])
        z_gain = 0.5 * (xyz[4] - xyz[5])

        return [x_off, x_gain, y_off, y_gain, z_off, z_gain]

    def revAccel(self, accel, cab):
        x = (accel[0] - cab[0]) / cab[1]
        y = (accel[1] - cab[2]) / cab[3]
        z = (accel[2] - cab[4]) / cab[5]
        return [x, y, z]

    def revMagnet(self, magnet, pyr):
        return        


def main():
    mpu = MPU9250()
    mpu.write(MPU9250_ADDRESS, 0x6B, 0x00)
    mpu.write(MPU9250_ADDRESS, 0x37, 0x02)

    # calibration Accel(xoff, xgain, yoff, ygain, zoff, zgain)
    #cab_accel = mpu.calibAccel()
    #print cab_accel
    
    startTime = time.time()

    while True:
        # get data
        accel = mpu.readAccel()
        gyro = mpu.readGyro()
        temp = mpu.readTemp()
        magnet = mpu.readMagnet()
        
        # revision data
        #t_accel = mpu.revAccel(accel, cab_accel)
        #print t_accel

        #rp = mpu.accel2RP(t_accel)
        #azi = mpu.magnet2Azimuth(magnet)

        #print rp

        #print accel   ,
        #print gyro    ,
        #print temp    ,
        #print magnet
        
        roll = mpu.calcRoll(accel)
        pitch = mpu.calcPitch(accel)
        yaw = mpu.calcYaw(magnet, roll, pitch)
        
        nowTime = time.time() - startTime
        
        print nowTime,roll,pitch,yaw

        sleep(0.15)


if __name__ == '__main__':
    main()
