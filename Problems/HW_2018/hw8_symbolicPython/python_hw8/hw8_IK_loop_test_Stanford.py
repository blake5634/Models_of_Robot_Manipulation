#!/usr/bin/python
#  Python inverse kinematic equations for Wrist

#  This test performs the following:
#
#  1) Generate a reachable 4x4 pose
#  2) run the generated IK Python code for "Wrist" robot
#  3) take resulting list of joint-solutions and put them back into
#          forward kinematics
#  4) compare the generated 4x4 matrices to the original pose
#
#  Running instructions:
#
#   > cd IKBT/
#   > pyton -m tests.Wrist_IK_loop_test
#
#   final results for each solution are error -- should be very small
#
#
import numpy as np
from math import sqrt
from math import atan2
from math import cos
from math import sin
from math import asin
from math import acos

from ikbtbasics.pykinsym import *
from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy
from ikbtfunctions.ik_robots import *


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

    robot = 'Stanford'
    #   Get the robot model
    [dh, vv, params, pvals, unknowns] = robot_params(robot)  # see ikbtfunctions/ik_robots.py

    #
    #     Set up robot equations for further solution by BT
    #
    #   Check for a pickle file of pre-computed Mech object. If the pickle
    #       file is not there, compute the kinematic equations
    #       if the pickle file is there it has pre-computed kinematic eqns to save time
    #
    testing = False
    [M, R, unknowns] = kinematics_pickle(robot, dh, params, pvals, vv, unknowns, testing)
    print 'GOT HERE: robot name: ', R.name

    R.name = robot
    R.params = params

    ##   check the pickle in case DH params were changed
    dhp = M.DH
    check_the_pickle(dhp, dh)   # check stored robot matches current DH params

    #  4x4 transform based on known joint angles:

    deg = sp.pi/180.0

    
    t1 = 1*deg
    t2 = 2*deg
    d3 = .3     #meters
    t3 = 4*deg
    t4 = 5*deg
    t5 = 6*deg
    t6 = 7*deg
    
    
    t1 = 45*deg
    t2 = 37*deg
    d3 = 0.6     #meters
    t3 = -120*deg
    t4 = 17.3*deg
    t5 = 135*deg
    t6 = 0.0*deg

    pose1 = {th_1: t1, th_2: t2, d_3: d3, th_3:t3, th_4:t4, th_5:t5, th_6:t6}

    np.set_printoptions(precision=3)

    T1 = forward_kinematics_N(M, pose1, M.pvals)
    
    print '\n\nDesired T06:'
    print(T1)

    # now generate IK solutions

    sol_list = ikin_Stanford(T1)

    i = 0
    todeg = 180/np.pi 
    
    print '\n    All angles in degrees  '
    print '   th1       th2         d3(m)       th4        th5        th6'
     
    for sol in sol_list:
        print ''
        print 'Solution ', i
        i+=1
        print ('{0:8.2f}  '.format(sol[1]*todeg)),    # th1 
        print ('{0:8.2f}  '.format(sol[2]*todeg)),    # th2
        print ('{0:8.2f}  '.format(sol[0])),    # d3
        print ('{0:8.2f}  '.format(sol[3]*todeg)),    # th4
        print ('{0:8.2f}  '.format(sol[5]*todeg)),    # th5
        print ('{0:8.2f}  '.format(sol[6]*todeg))
        
    #########3 try to plug back into FK model

    robot = 'Stanford'

    i=0
    # for each solution, compare FK(sol) with T01
    print '\n\nMax error for each Solution: '
    for sol in sol_list:
        pose = {th_1 : sol[1], th_2 : sol[2], d_3 : sol[0], th_4:sol[3], th_5:sol[5], th_6:sol[6]} # all joints included
        T2 = forward_kinematics_N(M, pose, M.pvals)
        maxe = -9999999.99
        #print '- - - - - '
        #sp.pprint (T2-T1)
        #print '- - - - - '
        for k in [0,1,2]:
            for j in [0,1,2,3]:
                e = T1[k,j]-T2[k,j]
                #print '<<',e,'>>'
                if np.abs(e) > maxe:
                    maxe = np.abs(e)
        print 'Solution ',i,': ', maxe
        i += 1
