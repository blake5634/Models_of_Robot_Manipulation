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
((h, h_1, h_2, h_3,)) = sp.symbols(('h', 'h_1', 'h_2', 'h_3'))


params = [l_1, l_2, l_3, l_4, l_5, l_6, h_1, h_2, h_3]

((L_1,D_3,L_6)) = sp.symbols(('L_1','D_3','L_6'))
 
name = "Chang_Mark"
#params7p1 = {}
# 1 for rotary, 0 for prismatic
vv = [1,1,0,1,1,1]
dh = sp.Matrix([                   #   NO OFFSETS
   [     0     ,      0,    L_1,     th_1 ],
   [   sp.pi/2 ,      0,    0,          th_2 ],
   [   sp.pi/2 ,      0,    D_3,       0 ],
   [   sp.pi/2 ,      0,    0,         th_4 ],
   [  -sp.pi/2 ,    l_5,    0,         th_5 ],
   [   sp.pi/2 ,      0,    L_6,   th_6 ] 
   ])   

print "\n\n        "+name+"\n\n"

M = mechanism(dh, params, vv)

M.forward_kinematics()

print name + "---------------------------------  Forward Kinematics:"

sp.pprint(notation_squeeze(M.T_06))

#print name + " ---  w55"

#sp.pprint(notation_squeeze(M.w_55))

print name + "---------------------------------  Frame 6 Jacobian:"

sp.pprint(notation_squeeze(M.J66[:,0:6]))
