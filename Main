import os
import numpy as np
import readtxt
from readtxt import *
import math
import matplotlib as plt

class Three_Dimensional_Object:
    """
    This class is for drawing the ectodomain of the object of interest as a straight line
    """
    def __init__(self, x1, y1, z1, x2, y2, z2):
        self.x1 = x1
        self.y1 = y1
        self.z1 = z1
        self.x2 = x2
        self.y2 = y2
        self.z2 = z2
        self.point1 = np.array([x1,y1,z1])
        self.point2 = np.array([x2,y2,z2])
        self.Vector = self.point1 - self.point2

    def Info(self):
        return 'point 1 = {}, Point 2 = {}, Vector = {}'.format(self.point1, self.point2, self.Vector)

# function to determine distance between the two ends of the line
    def linelength(self):
        pointdifference = self.point1 - self.point2
        if pointdifference[0] != 0:
            var1 = pointdifference[0]**2
        else:
            var1 = 0
        if pointdifference[1] != 0:
            var2 = pointdifference[1]**2
        else:
            var2 = 0
        if pointdifference[2] != 0:
            var3 = pointdifference[2]**2
        else:
            var3 = 0

        linedistancesquared = var1 + var2 + var3
        linelength = math.sqrt(linedistancesquared)
        return linelength


def RotationMatrix(axis, rotang):
    """
    This uses Euler-Rodrigues formula.
    """
    theta = rotang * 0.01745329252
    axis = np.asarray(axis)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2)
    b, c, d = axis*math.sin(theta/2)
    a2, b2, c2, d2 = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[a2+b2-c2-d2, 2*(bc-ad), 2*(bd+ac)],
                     [2*(bc+ad), a2+c2-b2-d2, 2*(cd-ab)],
                     [2*(bd-ac), 2*(cd+ab), a2+d2-b2-c2]])

def AppleRotationMatrix(vector, rotationmatrix):
    """
    This function take the output from the RotationMatrix function and uses that to apply the rotation to an input vector
    """
    a1 = (vector[0] * rotationmatrix[0, 0]) + (vector[1] * rotationmatrix[0, 1]) + (vector[2] * rotationmatrix[0, 2])
    b1 = (vector[0] * rotationmatrix[1, 0]) + (vector[1] * rotationmatrix[1, 1]) + (vector[2] * rotationmatrix[1, 2])
    c1 = (vector[0] * rotationmatrix[2, 0]) + (vector[1] * rotationmatrix[2, 1]) + (vector[2] * rotationmatrix[2, 2])

    return np.array((a1, b1, c1))

def CalculateAngleBetweenVector(vector, vector2):
    """
    This function was mainly created to double check the angle between the input and outputs of the applyrotationmatrix function
    """
    dp = np.dot(vector, vector2)

    maga = math.sqrt((vector[0] ** 2) + vector[1] ** 2 + vector[2] ** 2)
    magb = math.sqrt((vector2[0] ** 2) + vector2[1] ** 2 + vector2[2] ** 2)
    magc = maga * magb

    dpmag = dp / magc

    angleindeg = ((math.acos(dpmag)) * 180) / math.pi

    return angleindeg

#def RedrawLine(vector, length, origin):



#First we create are line that will be our HA ectodomain

Reference_Cylinder = Three_Dimensional_Object(50,50,68,50,50,25)

Reference_Membrane = Three_Dimensional_Object(56,50,24,66,50,24)

# This just prints the input to ensure each point has the expected x, y and z co-ords
print(Reference_Cylinder.Info())
print(Reference_Membrane.Info())

# This prints the length of the line between the points 1 and 2 of each line using the method linelength
print('Length of ectodomain = {}'.format(Reference_Cylinder.linelength()))
print('Length of membrane = {}'.format(Reference_Membrane.linelength()))

# Now we carry out a rotation and print out the angle between the first line and the rotated line.

RotationM = RotationMatrix((0,0,1), 18)

RotatedRot = AppleRotationMatrix(Reference_Cylinder.Vector, RotationM)

RotationM = RotationMatrix((1,0,0), 163)

RotatedTilt = AppleRotationMatrix(RotatedRot, RotationM)

print RotatedTilt


