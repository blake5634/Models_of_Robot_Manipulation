#!/usr/bin/python
import sympy as sp
import numpy as np
from kin_cl import *
sp.init_printing()
##########################################################################33
#
#        EE543  HW7    W 16
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))




########################################################
#
#      7.1 (numerical)

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params7p1 = [l_1,l_2,l_3]
params7p1 = {}
# 1 for rotary, 0 for prismatic
v7p1 = [1,1,1,1,1,1]
dh7p1 = sp.Matrix([
   [  sp.pi/2,     5,  0 ,   th_1  ],
   [  sp.pi/2 ,     0,  0 ,   th_2  ],
   [ -sp.pi/2,     5,  2 ,   th_3  ],
   [      0 ,     0,   0,      0  ],
   [      0 ,     0,   0,      0  ],
   [      0 ,     0,   0,      0  ]
   ])

pose7p1 = {th_1:30*deg, th_2:45*deg , th_3:60*deg }

########################################################
#
#      7.2  (symbolic)

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params7p2 = [l_1,l_2,l_3]
# 1 for rotary, 0 for prismatic
v7p2 = [1,1,1,1,1,1]
dh7p2 = sp.Matrix([
   [      0 ,     0,  l_1,   th_1  ],
   [ -sp.pi/2,   l_2,   0,   th_2  ],
   [    sp.pi,     0, l_3,   th_3  ],
   [      0 ,     0,   0,      0  ],
   [      0 ,     0,   0,      0  ],
   [      0 ,     0,   0,      0  ]
   ])

name = "Prob 7.2"
dh = dh7p2
params = params7p2
vv = v7p2

M = mechanism(dh, params, vv)

M.forward_kinematics()

# Forward Kinematics 

print '\n\n ==== w_33'
sp.pprint(sp.latex(notation_squeeze(M.w_33)))


print '\n\n ==== v_33'
sp.pprint(sp.latex(notation_squeeze(M.v_33)))



R_03 = M.R_01*M.R_12*M.R_23

print '\n\n ==== w_03'
sp.pprint(sp.latex(notation_squeeze(R_03.T*M.w_33)))
print '\n\n ==== v_03'
sp.pprint(sp.latex(notation_squeeze(R_03.T*M.v_33)))



  #print '\n\n ==== J_33'
  #sp.pprint(sp.latex(notation_squeeze((M.J66))))

  #print '\n\n\  ---- Plugging in: '

  #print pose7p1
  #print sp.latex(M.Jacobian_N(pose7p1))

  #print '-+-+-+ w33'
  #print notation_squeeze(M.w_33)