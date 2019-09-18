#!/usr/bin/python
import sympy as sp
import numpy as np
from kin_cl import *
sp.init_printing()
##########################################################################33
#
#        EE543  Book example 5.4    W 16
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,L_0, L_1, L_2, L_3, L_4,L_5)) = sp.symbols(('h','L_0', 'L_1', 'L_2', 'L_3', 'L_4','L_5'))


def output(LHS, RHS):
  print "%       "+LHS
  print "\\["
  print LHS + " = "
  sp.pprint(sp.latex(notation_squeeze(RHS)))
  print "\\]"
  

########################################################
#
#     Example 5.4

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params = [L_1,L_2,L_3,L_5] 

#params7p1 = {}
# 1 for rotary, 0 for prismatic
vv = [1,1,1,1,1,1]
dh = sp.Matrix([
   [      0 ,       0,     0,     th_1 ],
   [  sp.pi/2,     L_1,    0,     th_2 ],
   [      0 ,      L_2,    0,     th_3 ],
   [      0 ,      L_3,    0,       0  ],
   [      0 ,       0,     0,       0  ],
   [      0 ,       0,     0,       0  ] 
   ])
 
DOFs = 3
name = "Book Example 5.4" 

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


  print "Forward Kinematics Link transforms:"
  
  
  
  print "Angular velocity propagations: "
  
  output('{^1_1w}', M.w_11)
  output('{^2_2w}', M.w_22)
  output('{^3_3w}', M.w_33)
  output('{^4_4w}', M.w_44)
  
   