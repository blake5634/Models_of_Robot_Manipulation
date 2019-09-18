#!/usr/bin/python
import sympy as sp
import numpy as np
from kin_cl import *
sp.init_printing()
##########################################################################33
#
#        EE543  Project tester  W 16
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((l_0, l_1, l_2, l_3, l_4,l_5,l_6,l_7)) = sp.symbols(('l_0', 'l_1', 'l_2', 'l_3', 'l_4','l_5','l_6','l_6'))
((h,h_0, h_1, h_2, h_3,)) = sp.symbols(('h','h_0', 'h_1', 'h_2', 'h_3'))


params = [l_1, l_2, l_3, l_4, l_5, l_6, h_1, h_2, h_3]

  
name = "Yin_Jianzhu"
#params7p1 = {}
# 1 for rotary, 0 for prismatic
vv = [1,1,0,1,1,1]
dh = sp.Matrix([                   #   NO OFFSETS
   [    0       ,      0,  l_1,      th_1 ],
   [   sp.pi/2  ,    l_2,    0,      th_2 ],
   [  -sp.pi/2  ,      0,  d_3,       0   ],
   [     0      ,      0,    0,      th_4 ],
   [   sp.pi/2  ,      0,    0,      th_5 ],
   [   sp.pi/2  ,      0,    0,      th_6 ] 
   ])    

nDOF = 6

print "\n\n        "+name+"\n\n"

M = mechanism(dh, params, vv)

M.forward_kinematics()

print name + "---------------------------------  Forward Kinematics:"


T = M.T_06
#T = T.subs(sp.cos(th_f), c_f)
#T = T.subs(sp.sin(th_f), s_f)
 
sp.pprint(notation_squeeze(T[:,0:1]))
sp.pprint(notation_squeeze(T[:,1:2]))
sp.pprint(notation_squeeze(T[:,2:3]))
sp.pprint(notation_squeeze(T[:,3:4]))
 
#sp.pprint(notation_squeeze(T[:,2:cols]))

#print name + " ---  w55"

#sp.pprint(notation_squeeze(M.w_55))

print name + "---------------------------------  Frame 6 Jacobian:"
cols = nDOF
J = M.J66 
sp.pprint(notation_squeeze(J[:,0:cols]))
