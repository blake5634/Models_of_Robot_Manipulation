#!/usr/bin/python
#  Python inverse kinematic equations for Puma

import numpy as np
from math import sqrt
from math import atan2
from math import cos
from math import sin

pi = np.pi

##  define some wrappers for tighter notation
#def atan2(y,x):
    #return np.arctan2(y,x)


# Code to solve the unknowns
def ikin_Puma(T):
    if(T.shape != (4,4)):
        print "bad input to "+funcname
        quit()
#define the input vars
    r_11 = T[0,0]
    r_12 = T[0,1]
    r_13 = T[0,2]
    r_21 = T[1,0]
    r_22 = T[1,1]
    r_23 = T[1,2]
    r_31 = T[2,0]
    r_32 = T[2,1]
    r_33 = T[2,2]
    Px = T[0,3]
    Py = T[1,3]
    Pz = T[2,3]

#    Declare

    d_3 = 0.1245  # actual MKS puma dims.
    d_4 = 0.432
    a_2 = 0.432
    a_3 = 0.0203



#
# Caution:    Generated code is not yet validation

    solvable_pose = True

###########################################################################################
#

#     insert code from CodeGen/Python/IK_equations....py   HERE

#
###########################################################################################



if __name__ == "__main__":
    from ikbtbasics.pykinsym import *
    #from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy


    px = 0.4   # desired EE position
    py = 0.4
    pz = 0.6
    th = np.pi/4  # just a random angle

    # generate a 4x4 pose to test IK

    T1 = RotX4_N(th) * RotY4_N(2*th)  # combine two rotations
    T1[0,3] = px
    T1[1,3] = py
    T1[2,3] = pz

    # try the Puma IK

    list = ikin_Puma(T1)

    for sol in list:
        print '\n ------'
        for i in range(len(sol)):
            x = sol[i]
            sol[i] = x * 57.3 # convert_to_degrees
            
        print sol

