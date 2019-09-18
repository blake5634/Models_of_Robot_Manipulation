#!/usr/bin/python
import sympy as sp
import numpy as np
from kin_cl import *
sp.init_printing()
##########################################################################33
#
#        Example:  Puma 560 Arm Manipulator
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4,l_5)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4','l_5'))

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params7p1 = [l_1,l_2,l_3,l_5] 

#params7p1 = {}
# 1 for rotary, 0 for prismatic 

vhandbook = [1,1,1,1,1,1]
dhhandbook = sp.Matrix([
   [   0      ,   0,   0  ,  th_1 ],
   [ -sp.pi/2 ,   0 ,  0  ,  th_2  ],
   [   0      , l_3 ,  0 ,  th_3  ],
   [ -sp.pi/2 ,   0 ,  d_4 ,  th_4  ],
   [  sp.pi/2 ,    0,   0,   th_5  ],
   [ -sp.pi/2 ,    0,   0,   th_6  ]
   ])
 
DOFs = 6

name = "Springer Handbook Example, p24" 
dh = dhhandbook
params = params7p1
vv = vhandbook 

M = mechanism(dh, params, vv)

M.forward_kinematics()

print "\n\n\n           " + name + "\n\n\n---------------------------------  Frame 6 Jacobian:"

sp.pprint(notation_squeeze(M.J66[:,0:DOFs]))
 

# Forward Kinematics
if(1):

  print '  ====  T06'
  sp.pprint(notation_squeeze(M.T_06)) 

  