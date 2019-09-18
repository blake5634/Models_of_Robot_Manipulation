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

print "\n\n\n           " + name 
#print "\n\n\n---------------------------------  Frame 6 Jacobian:"

#sp.pprint(notation_squeeze(M.J66[:,0:DOFs]))
 

# Forward Kinematics 
print '  ====  T06'
sp.pprint(notation_squeeze(M.T_06)) 

print ' === solve  test for th_1 :' 
sp.pprint(M.T_06[0,3])

##   Solver tests
sp.var('t21 t22 t14 t24')

eqn = M.T_06[0,3] - t14
print "Solutions: "
for i in range(0,2):
  sp.pprint(notation_squeeze(sp.solve(eqn, th_1)[i]))

print " ---- solving in compact form:\n cos(th_1) = "
eqn = notation_squeeze(M.T_06[0,3]) - t14
sp.pprint(sp.solve(eqn, c_1))
print "\n sin(th_1) = "
eqn = notation_squeeze(M.T_06[1,3]) - t24
sp.pprint(sp.solve(eqn, s_1))
	  
	  
print " === T16:"
tmp = sp.trigsimp(M.T_12*M.T_23*M.T_34*M.T_45*M.T_56)

sp.pprint(notation_squeeze(tmp))
# solve for theta 2

eqn1 = notation_squeeze(sp.Eq(tmp[1,0], t21))
eqn2 = notation_squeeze(sp.Eq(tmp[1,1], t22))

sp.pprint((sp.solve([eqn1,eqn2],[s_6,c_6])))
	   
	  



			   
			   
			   
			   
			   