#!/usr/bin/python
import sympy as sp
import numpy as np
from kin_cl import *
sp.init_printing()
##########################################################################33
#
#        EE543  Final Project Template
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))




########################################################
#
#     Robot Parameters

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params_example = [l_1,l_2,l_3]

#  define joint variables:
#         1 for rotary, 0 for prismatic
v_example = [1,1,1,1,1,1]


#  Here are your DB parameters
# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.

dh_example = sp.Matrix([
   [  sp.pi/2,    l_1,  0 ,   th_1  ],
   [      0  ,     0,  0 ,   th_2  ],
   [ -sp.pi/2,    l_2, l_3 ,   th_3  ],    # this is only a 3DOF example
   [      0 ,     0,   0,      0  ],     # must fill remaining 3 rows with zeros
   [      0 ,     0,   0,      0  ],
   [      0 ,     0,   0,      0  ]
   ])

# a manipulator pose for computing numerical Jacobian etc.
pose_example = {th_1:30*deg, th_2:45*deg , th_3:60*deg }

###############################################################################3


name = "Final Project Example"
dh = dh_example
params = params_example
vv = v_example

#  set up a mechanism object
M = mechanism(dh, params, vv)

#  compute the forward kinematics and Jacobian of M
M.forward_kinematics()


print ' Here are the forward kinematic equations for ' + name

sp.pprint(M.T_06)

print ' \n\n Here are the FK equations with simplified notation: '
sp.pprint(notation_squeeze(M.T_06))

print '\n\n here is the latex output corresponding to above: '
print(sp.latex(notation_squeeze(M.T_06)))
      

