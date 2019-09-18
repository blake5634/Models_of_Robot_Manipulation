#!/usr/bin/python
#  Python inverse kinematic equations for Stanford

import numpy as np
from math import sqrt
from math import atan2
from math import cos
from math import sin

pi = np.pi



#  Declare the parameters

l_1 = 1
l_2 = 2
l_4 = 4
l_6 = 3


# Code to solve the unknowns 
def ikin_Stanford(T):
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

#
# Caution:    Generated code is not yet validated
#

    
    solvable_pose = True


    #Variable:  th_2
    if (solvable_pose and abs ((Py - l_1 - l_6*r_23)/l_4)  > 1):
        solvable_pose = False
    else:
    th_2s2 = -asin((Py - l_1 - l_6*r_23)/l_4) + pi
    if (solvable_pose and abs ((Py - l_1 - l_6*r_23)/l_4)  > 1):
        solvable_pose = False
    else:
    th_2s1 = asin((Py - l_1 - l_6*r_23)/l_4)


    #Variable:  th_4
    if (solvable_pose and abs (r_23/cos(th_2s1))  > 1):
        solvable_pose = False
    else:
    th_4s3 = -asin(r_23/cos(th_2s1))
    if (solvable_pose and abs (r_23/cos(th_2s1))  > 1):
        solvable_pose = False
    else:
    th_4s4 = asin(r_23/cos(th_2s1)) + pi
    if (solvable_pose and abs (r_23/cos(th_2s2))  > 1):
        solvable_pose = False
    else:
    th_4s1 = -asin(r_23/cos(th_2s2))
    if (solvable_pose and abs (r_23/cos(th_2s2))  > 1):
        solvable_pose = False
    else:
    th_4s2 = asin(r_23/cos(th_2s2)) + pi


    #Variable:  th_56
    th_56s2 = atan2(r_21*sin(th_2s1) + r_22*cos(th_2s1)*cos(th_4s4), -r_21*cos(th_2s1)*cos(th_4s4) + r_22*sin(th_2s1))
    th_56s4 = atan2(r_21*sin(th_2s2) + r_22*cos(th_2s2)*cos(th_4s2), -r_21*cos(th_2s2)*cos(th_4s2) + r_22*sin(th_2s2))
    th_56s1 = atan2(r_21*sin(th_2s1) + r_22*cos(th_2s1)*cos(th_4s3), -r_21*cos(th_2s1)*cos(th_4s3) + r_22*sin(th_2s1))
    th_56s3 = atan2(r_21*sin(th_2s2) + r_22*cos(th_2s2)*cos(th_4s1), -r_21*cos(th_2s2)*cos(th_4s1) + r_22*sin(th_2s2))


    #Variable:  th_1
    th_1s3 = atan2(r_13*cos(th_4s1) - r_33*sin(th_2s2)*sin(th_4s1), r_13*sin(th_2s2)*sin(th_4s1) + r_33*cos(th_4s1))
    th_1s1 = atan2(r_13*cos(th_4s3) - r_33*sin(th_2s1)*sin(th_4s3), r_13*sin(th_2s1)*sin(th_4s3) + r_33*cos(th_4s3))
    th_1s4 = atan2(r_13*cos(th_4s2) - r_33*sin(th_2s2)*sin(th_4s2), r_13*sin(th_2s2)*sin(th_4s2) + r_33*cos(th_4s2))
    th_1s2 = atan2(r_13*cos(th_4s4) - r_33*sin(th_2s1)*sin(th_4s4), r_13*sin(th_2s1)*sin(th_4s4) + r_33*cos(th_4s4))


    #Variable:  d_3
    d_3s3 = Px*sin(th_1s4) + Pz*cos(th_1s4) - l_2 - l_6*cos(th_4s2)
    d_3s4 = Px*sin(th_1s2) + Pz*cos(th_1s2) - l_2 - l_6*cos(th_4s4)
    d_3s1 = Px*sin(th_1s3) + Pz*cos(th_1s3) - l_2 - l_6*cos(th_4s1)
    d_3s2 = Px*sin(th_1s1) + Pz*cos(th_1s1) - l_2 - l_6*cos(th_4s3)


    #Variable:  th_5
    th_5s4 = atan2(-r_13*sin(th_1s1)*sin(th_4s3) + r_13*sin(th_2s1)*cos(th_1s1)*cos(th_4s3) - r_23*cos(th_2s1)*cos(th_4s3) - r_33*sin(th_1s1)*sin(th_2s1)*cos(th_4s3) - r_33*sin(th_4s3)*cos(th_1s1), -r_13*cos(th_1s1)*cos(th_2s1) - r_23*sin(th_2s1) + r_33*sin(th_1s1)*cos(th_2s1)) + pi
    th_5s8 = atan2(-r_13*sin(th_1s2)*sin(th_4s4) + r_13*sin(th_2s1)*cos(th_1s2)*cos(th_4s4) - r_23*cos(th_2s1)*cos(th_4s4) - r_33*sin(th_1s2)*sin(th_2s1)*cos(th_4s4) - r_33*sin(th_4s4)*cos(th_1s2), -r_13*cos(th_1s2)*cos(th_2s1) - r_23*sin(th_2s1) + r_33*sin(th_1s2)*cos(th_2s1)) + pi
    th_5s1 = atan2(-r_13*sin(th_1s3)*sin(th_4s1) + r_13*sin(th_2s2)*cos(th_1s3)*cos(th_4s1) - r_23*cos(th_2s2)*cos(th_4s1) - r_33*sin(th_1s3)*sin(th_2s2)*cos(th_4s1) - r_33*sin(th_4s1)*cos(th_1s3), -r_13*cos(th_1s3)*cos(th_2s2) - r_23*sin(th_2s2) + r_33*sin(th_1s3)*cos(th_2s2))
    th_5s2 = atan2(-r_13*sin(th_1s3)*sin(th_4s1) + r_13*sin(th_2s2)*cos(th_1s3)*cos(th_4s1) - r_23*cos(th_2s2)*cos(th_4s1) - r_33*sin(th_1s3)*sin(th_2s2)*cos(th_4s1) - r_33*sin(th_4s1)*cos(th_1s3), -r_13*cos(th_1s3)*cos(th_2s2) - r_23*sin(th_2s2) + r_33*sin(th_1s3)*cos(th_2s2)) + pi
    th_5s6 = atan2(-r_13*sin(th_1s4)*sin(th_4s2) + r_13*sin(th_2s2)*cos(th_1s4)*cos(th_4s2) - r_23*cos(th_2s2)*cos(th_4s2) - r_33*sin(th_1s4)*sin(th_2s2)*cos(th_4s2) - r_33*sin(th_4s2)*cos(th_1s4), -r_13*cos(th_1s4)*cos(th_2s2) - r_23*sin(th_2s2) + r_33*sin(th_1s4)*cos(th_2s2)) + pi
    th_5s3 = atan2(-r_13*sin(th_1s1)*sin(th_4s3) + r_13*sin(th_2s1)*cos(th_1s1)*cos(th_4s3) - r_23*cos(th_2s1)*cos(th_4s3) - r_33*sin(th_1s1)*sin(th_2s1)*cos(th_4s3) - r_33*sin(th_4s3)*cos(th_1s1), -r_13*cos(th_1s1)*cos(th_2s1) - r_23*sin(th_2s1) + r_33*sin(th_1s1)*cos(th_2s1))
    th_5s7 = atan2(-r_13*sin(th_1s2)*sin(th_4s4) + r_13*sin(th_2s1)*cos(th_1s2)*cos(th_4s4) - r_23*cos(th_2s1)*cos(th_4s4) - r_33*sin(th_1s2)*sin(th_2s1)*cos(th_4s4) - r_33*sin(th_4s4)*cos(th_1s2), -r_13*cos(th_1s2)*cos(th_2s1) - r_23*sin(th_2s1) + r_33*sin(th_1s2)*cos(th_2s1))
    th_5s5 = atan2(-r_13*sin(th_1s4)*sin(th_4s2) + r_13*sin(th_2s2)*cos(th_1s4)*cos(th_4s2) - r_23*cos(th_2s2)*cos(th_4s2) - r_33*sin(th_1s4)*sin(th_2s2)*cos(th_4s2) - r_33*sin(th_4s2)*cos(th_1s4), -r_13*cos(th_1s4)*cos(th_2s2) - r_23*sin(th_2s2) + r_33*sin(th_1s4)*cos(th_2s2))


    #Variable:  th_6
    th_6s5 = th_56s2 - th_5s6
    th_6s18 = th_56s1 - th_5s8
    th_6s22 = th_56s1 - th_5s3
    th_6s19 = th_56s1 - th_5s1
    th_6s4 = th_56s2 - th_5s2
    th_6s13 = th_56s4 - th_5s6
    th_6s23 = th_56s1 - th_5s7
    th_6s2 = th_56s2 - th_5s8
    th_6s3 = th_56s2 - th_5s1
    th_6s12 = th_56s4 - th_5s2
    th_6s9 = th_56s4 - th_5s4
    th_6s26 = th_56s3 - th_5s8
    th_6s8 = th_56s2 - th_5s5
    th_6s17 = th_56s1 - th_5s4
    th_6s27 = th_56s3 - th_5s1
    th_6s21 = th_56s1 - th_5s6
    th_6s7 = th_56s2 - th_5s7
    th_6s16 = th_56s4 - th_5s5
    th_6s20 = th_56s1 - th_5s2
    th_6s10 = th_56s4 - th_5s8
    th_6s6 = th_56s2 - th_5s3
    th_6s11 = th_56s4 - th_5s1
    th_6s31 = th_56s3 - th_5s7
    th_6s30 = th_56s3 - th_5s3
    th_6s24 = th_56s1 - th_5s5
    th_6s15 = th_56s4 - th_5s7
    th_6s25 = th_56s3 - th_5s4
    th_6s1 = th_56s2 - th_5s4
    th_6s14 = th_56s4 - th_5s3
    th_6s29 = th_56s3 - th_5s6
    th_6s28 = th_56s3 - th_5s2
    th_6s32 = th_56s3 - th_5s5

##################################
#
#package the solutions into a list for each set
#
###################################

    solution_list = []
    #(note trailing commas allowed in python
    solution_list.append( [  d_3s2,  th_1s1,  th_2s1,  th_4s3,  th_56s1,  th_5s4,  th_6s17,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  d_3s4,  th_1s2,  th_2s1,  th_4s4,  th_56s2,  th_5s8,  th_6s2,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  d_3s2,  th_1s1,  th_2s1,  th_4s3,  th_56s1,  th_5s3,  th_6s22,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  d_3s3,  th_1s4,  th_2s2,  th_4s2,  th_56s4,  th_5s6,  th_6s13,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  d_3s3,  th_1s4,  th_2s2,  th_4s2,  th_56s4,  th_5s5,  th_6s16,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  d_3s1,  th_1s3,  th_2s2,  th_4s1,  th_56s3,  th_5s1,  th_6s27,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  d_3s4,  th_1s2,  th_2s1,  th_4s4,  th_56s2,  th_5s7,  th_6s7,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  d_3s1,  th_1s3,  th_2s2,  th_4s1,  th_56s3,  th_5s2,  th_6s28,  ] )
    if(solvable_pose):
        return(solution_list)
    else: 
        return(False)


#
#    TEST CODE
#
if __name__ == "__main__":

#  4x4 transforms which are pure rotations

    def RotX4_N(t):
      return(np.matrix([
        [1,         0,           0,      0],
        [0, np.cos(t),  -np.sin(t),      0],
        [0, np.sin(t),   np.cos(t),      0],
        [0,0,0,1.0]
        ]))

    def RotY4_N(t):
      return(np.matrix([
        [ np.cos(t),   0,      np.sin(t),    0],
        [0,            1,          0    ,    0],
        [-np.sin(t),   0,      np.cos(t),    0],
        [0,0,0,1]
        ]))

    def RotZ4_N(t):
      return(np.matrix([
        [ np.cos(t),  -np.sin(t),       0,    0],
        [ np.sin(t),   np.cos(t),       0,    0],
        [ 0,              0,            1,    0],
        [0,0,0,1]
        ]))

    px = 0.2   # desired EE position
    py = 0.3
    pz = 0.6
    th = np.pi/7  # just a random angle

    # generate a 4x4 pose to test IK

    T1 = RotX4_N(th) * RotY4_N(2*th)  # combine two rotations
    T1[0,3] = px
    T1[1,3] = py
    T1[2,3] = pz

    # try the Puma IK

    list = ikin_Stanford(T1)

    i = 0
    for sol in list:
        print ''
        print 'Solution ', i
        i+=1
        print sol


    
