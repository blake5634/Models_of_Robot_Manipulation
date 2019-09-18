#!/usr/bin/python
import sympy as sp
import numpy as np
from kin_cl import *
sp.init_printing()
##########################################################################33
#
#        EE543  TEMPLATE
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4,l_5)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4','l_5'))




########################################################
#
#     Example

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params = [l_1,l_2,l_3,l_5] 

#  Joint types
# 1 for rotary, 0 for prismatic
vv = [1,0,1,1,1,1]

# DH parameters
dh = sp.Matrix([
   [      0 ,       0,     0,     th_1 ],
   [      0 ,     l_1,   d_2,       0  ],
   [  sp.pi/2 ,     0,   l_2,     th_3 ],
   [ -sp.pi/2,    l_3,     0,     th_4 ],
   [      0 ,       0,     0,       0  ],
   [      0 ,       0,     0,       0  ] 
   ])

#
#      Stanford Arm
#

#  Joint types
# 1 for rotary, 0 for prismatic
vv = [1,1,0,1,1,1]

# DH parameters
dh = sp.Matrix([
   [      0 ,    0,   0,     th_1 ],
   [ -sp.pi/2 ,  0,   l_2,   th_2  ],
   [  sp.pi/2 ,  0,   d_3,    0  ],
   [      0 ,    0,   0,     th_4  ],
   [ -sp.pi/2,   0,   0,     th_5  ],
   [  sp.pi/2,   0,   0,     th_6  ]
   ]) 


DOFs = 6

name = "Stanford Arm" 

M = mechanism(dh, params, vv)

M.forward_kinematics()

print name + "---------------------------------  Frame 6 Jacobian:"

sp.pprint(notation_squeeze(M.J66[:,0:DOFs]))

print ""
print (sp.latex(notation_squeeze(M.J66[:,0:DOFs])))
 

#print name + "---------------------------------  Frame 0 Jacobian:"

#sp.pprint(notation_squeeze(M.J60))
 

# Forward Kinematics
if(1):

  print '  ====  T01-T56'
  sp.pprint(notation_squeeze(M.T_01))
  sp.pprint(notation_squeeze(M.T_12))
  sp.pprint(notation_squeeze(M.T_23))
  sp.pprint(notation_squeeze(M.T_34))
  sp.pprint(notation_squeeze(M.T_45))
  sp.pprint(notation_squeeze(M.T_56))



  print '\n\n ==== w_11'
  sp.pprint(sp.latex(notation_squeeze(M.w_11)))

  print '\n\n ==== w_22'
  sp.pprint(sp.latex(notation_squeeze(M.w_22)))

  print '\n\n ==== w_33'
  sp.pprint(sp.latex(notation_squeeze(M.w_33)))

  #print '\n\n ==== w_44'
  #sp.pprint(sp.latex(notation_squeeze(M.w_44)))




  print '\n\n ==== v_11'
  sp.pprint(sp.latex(notation_squeeze(M.v_11)))


  print '\n\n ==== v_22'
  sp.pprint(sp.latex(notation_squeeze(M.v_22)))


  print '\n\n ==== v_33'
  sp.pprint(sp.latex(notation_squeeze(M.v_33)))


  #print '\n\n ==== v_44'
  #sp.pprint(sp.latex(notation_squeeze(M.v_44)))



  #R_03 = M.R_01*M.R_12*M.R_23

  #print '\n\n ==== w_03'
  #sp.pprint(sp.latex(notation_squeeze(R_03.T*M.w_33)))
  #print '\n\n ==== v_03'
  #sp.pprint(sp.latex(notation_squeeze(R_03.T*M.v_33)))



  #print '\n\n ==== J_33'
  #sp.pprint(sp.latex(notation_squeeze((M.J66))))

  #print '\n\n\  ---- Plugging in: '

  #print pose7p1
  #print sp.latex(M.Jacobian_N(pose7p1))

  #print '-+-+-+ w33'
  #print notation_squeeze(M.w_33)