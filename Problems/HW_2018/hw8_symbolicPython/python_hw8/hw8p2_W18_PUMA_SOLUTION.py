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


TEST_DATA_GENERATION = False

sp.init_printing()
 

# generic variables for any manipulator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('Px Py Pz') 

# move definition of Sum-of-Angles variables to ik-classes.py so they are available everywhere

########################################################
#
#     Robot Parameters
#
########################################################    NEW Style robot param setups

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
        

print '\n\n  Output for Problem 8.2: '

m = kc.mechanism(dh, params, vv)
m.pvals = pvals  # store numerical values of parameters
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

#sp.pprint(notation_squeeze(ManipJacobian_S(m.v_55, m.w_55, [qd_1, qd_2,qd_3,qd_4,qd_5])))

J = ManipJacobian_S(m.v_55, m.w_55, [qd_1, qd_2,qd_3,qd_4,qd_5])

print '8.2 3a:'
sp.pprint(notation_squeeze(J[2,0]))
         
print '8.2 3b:'
sp.pprint(notation_squeeze(J[0,2]))
          
print '8.2 3c:'
sp.pprint(notation_squeeze(J[2,2]))
          