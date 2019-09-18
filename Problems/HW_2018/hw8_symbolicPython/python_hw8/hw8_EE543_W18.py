#!/usr/bin/python
#
#
#    EE543 HW8 Winter 2018   Symbolic Inverse Kinematics
#
#
import sympy as sp
from sys import exit, argv
import pickle     # for storing pre-computed FK eqns
import unittest

# modified by BH, local version in current dir
import b3 as b3          # behavior trees

# local modules
import ikbtfunctions.helperfunctions as hf
from ikbtfunctions.ik_robots import * 
from ikbtbasics import *
 

sp.init_printing()

print ""
print "          Running Kinematics "
print ""
print "" 

# generic variables for any manipulator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('Px Py Pz') 

# move definition of Sum-of-Angles variables to ik-classes.py so they are available everywhere



print 'Starting Prob 8.1'
########################################################
#
#     Robot Parameters
#
########################################################    NEW Style robot param setups

if False: 
    #
    #
    #    Classic Puma 560 Industrial 6DOF all rotary.
    #
    robot = 'Puma'
    dh = sp.Matrix([
        [  0,        0 ,  0.6 ,     th_1  ],
        [-sp.pi/2,   0 ,   0 ,      th_2  ],
        [      0 ,  a_2, d_3 ,      th_3  ],   
        [-sp.pi/2 , a_3, d_4,       th_4  ],      
        [-sp.pi/2 ,   0,  0 ,       th_5  ],
        [ sp.pi/2 ,   0,  0 ,       th_6  ]
        ])
    vv = [1,1,1,1,1,1]

    variables =  [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]
    params = [a_2, a_3, d_3, d_4]
    pvals = {a_2:0.432, a_3:0.0203, d_3:0.1245, d_4:0.432}  # meters
            
    m = kc.mechanism(dh, params, vv)

    m.forward_kinematics()
    print "Completed Forward Kinematics"
    print m.T_06

    print '\n\n\n\n'
    sp.pprint(notation_squeeze(m.T_06))

    print ' w66'
    sp.pprint(notation_squeeze(m.w_66))
    print '-------------'

    print ' v66'
    sp.pprint(notation_squeeze(m.v_66)) 
     

    print '\n\nManipulator Jacobian Matrix: '+robot
    J = ManipJacobian_S(m.v_66, m.w_66, [qd_1, qd_2,qd_3,qd_4,qd_5,qd_6])

    sp.pprint(notation_squeeze(J))

    #print '------------- Latex Output --------------------'
    #print sp.latex(notation_squeeze(ManipJacobian_S(m.v_66, m.w_66, [qd_1, qd_2,qd_3,qd_4,qd_5,qd_6])))
     
    print ''

    th_5_setting = sp.pi/2
    print 'Jacobian with theta_5 = ', th_5_setting
    Jp = sp.zeros(6,6)

    for i in range(6):
        for j in range(6):
            Jp[i,j] = J[i,j].subs(th_5, th_5_setting)
            
    sp.pprint(notation_squeeze(Jp))

    print 'element ratios:'
    sp.pprint( sp.simplify(Jp[3,0]/Jp[4,0]))


#########################################################################
print 'Starting Craig417'
name = 'Craig417'

dh = sp.Matrix([
    [    0   ,    0 ,   0   ,     th_1  ],
    [-sp.pi/2,    0 ,   0   ,     th_2  ],
    [ sp.pi/4 ,   0,    d_2 ,     th_3  ],   
    [    0    ,   a_3,  d_3 ,     th_4  ],      
    [    0    ,   0 ,   0,         0    ],      
    [    0    ,   0 ,   0,         0    ]
    ])
vv = [1,1,1,1,1,1]

variables =  [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4)]
params = [d_2, d_3, a_3]
pvals = {d_2:1, d_3:1,  a_3:1}  # meters

        
        
m = kc.mechanism(dh, params, vv)

m.forward_kinematics()
print "Completed Forward Kinematics"

T_06a = sp.zeros(4,4)

sp.var('sq2')

for i in range(3):
    for j in range(4):
        T_06a[i,j] = m.T_06[i,j].subs(sp.sqrt(2), sq2)

print m.T_06

print '\n\n\n\n'
sp.pprint(notation_squeeze(T_06a ))

#print ' w66'
#sp.pprint(notation_squeeze(m.w_66))
#print '-------------'

#print '------------- Latex Output --------------------'
#print sp.latex(notation_squeeze(ManipJacobian_S(m.v_66, m.w_66, [qd_1, qd_2,qd_3,qd_4,qd_5,qd_6])))
 

          