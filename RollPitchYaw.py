# -*- coding: utf-8 -*-

# import module
import math
import numpy as np

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
        rowY = -magY
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
            yaw = math.atan(calibY / calibX)
        except:
            if calibY > 0:
                yaw = 1.0
            else:
                yaw = -1.0
        return yaw
