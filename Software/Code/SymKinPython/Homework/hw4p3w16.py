#!/usr/bin/python


import sympy as sp
import numpy as np

from kin_cl import *


sp.init_printing()

##########################################################################33
#
#        Testing
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))

((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))

((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((h_2, h_3,h_4,h_5)) = sp.symbols(('h_2', 'h_3', 'h_4', 'h_5'))

####################################################
#   Prob 5.1
#                Symbolic inverse kinematics
#
# params for THIS manip
params = [h, l_3, l_4]

# 1 for rotary, 0 for prismatic
v = [1,1,1,1,1,1]

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
dh = sp.Matrix([
    [    sp.pi/2 ,     0,   h,   th_1  ],
    [      0     ,     0,   0,   th_2  ],
    [    -sp.pi/2,     0, l_3,   th_3  ],
    [      0     ,    l_4,  0,   th_4  ],
    [      0     ,     0,   0,   th_5    ],
    [      0     ,     0,   0,   th_6   ]
    ])

M = mechanism(dh, params, v)

M.forward_kinematics()

print '--------------'
sp.pprint(notation_squeeze(M.T_06[:,0:2]))

print ' '

sp.pprint(notation_squeeze(M.T_06[:,2:4]))

print 'Intermediate Results: --- * ---'

print 'T_01'
sp.pprint(notation_squeeze(M.T_01))
print 'T_12'
sp.pprint(notation_squeeze(M.T_12))
print 'T_23'
sp.pprint(notation_squeeze(M.T_23))
print 'T_34'
sp.pprint(notation_squeeze(M.T_34))

