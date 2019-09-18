#!/usr/bin/python
import sympy as sp
import numpy as np
from kin_cl import *
sp.init_printing()
##########################################################################33
#
#        EE543  Test for working on sum-of-angles
#
############################################################################

# generic variables for any maniplator
#((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4,l_5)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4','l_5'))




########################################################
#
#     Example

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params = [l_1,l_2,l_3,l_5] 

#  Joint types
# 1 for rotary, 0 for prismatic
vv = [1,1,1,1,1,1]

# DH parameters
dh = sp.Matrix([
   [      0 ,       0,     0,     th_1 ],
   [      0 ,     l_1,   l_3,    th_2 ],
   [-sp.pi/2,        0,   l_2,    th_3 ],
   [ sp.pi/2,        0,     0,     th_4   ],
   [      0 ,       0,     0,     th_5  ],
   [      0 ,       0,     0,       0  ] 
   ])
 


DOFs = 5

name = "test" 

M = mechanism(dh, params, vv)

M.forward_kinematics() 
#sp.pprint(notation_squeeze(M.J60))
 

def out(x):
  sp.pprint(notation_squeeze(x))
  
 

# Forward Kinematics
if(1):

  print '  ====  T06'
  sp.pprint(notation_squeeze(M.T_06)) 

  print '\n\n\n\n\n'

  x = M.T_06[0,0]
  
  print '=x====='
  out(x)

  #print 'trigsimp(x): '
  #out(sp.trigsimp(x))
   
  
  print '    --- '
  out(sp.simplify( x))
  
  #print 'sq2(x) (again)'
  #out(sq2(sq2(sp.trigsimp(x))))
  
  
  
  #R_03 = M.R_01*M.R_12*M.R_23

  #print '\n\n ==== w_03'
  #sp.pprint(sp.latex(notation_squeeze(R_03.T*M.w_33)))
  #print '\n\n ==== v_03'
  #sp.pprint(sp.latex(notation_squeeze(R_03.T*M.v_33)))



  print '\n\n ==== J_55'
  sp.pprint((notation_squeeze((M.J66))))

  #print '\n\n\  ---- Plugging in: '

  #print pose7p1
  #print sp.latex(M.Jacobian_N(pose7p1))

  print '-+-+-+ w11'
  print notation_squeeze(M.w_11)
  
  print '-+-+-+ w22'
  print notation_squeeze(M.w_22)
  
  print '-+-+-+ w33'
  print notation_squeeze(M.w_33)
  
  print '-+-+-+ w44'
  print notation_squeeze(M.w_44)
  
  print '-+-+-+ w55'
  print notation_squeeze(M.w_55)
  
  