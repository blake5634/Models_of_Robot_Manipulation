#!/usr/bin/python


import sympy as sp
import numpy as np

from kin_cl import *


sp.init_printing()

##########################################################################33
#
#        EE543  HW6    W 16
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))

((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))

((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))


########################################################
#
#      Prob 6.2 Hard!

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params6p2 = [l_0, l_1, l_2, l_3]
# 1 for rotary, 0 for prismatic
v6p2 = [1,1,1,1,1,1]
dh6p2 = sp.Matrix([
    [      0 ,     0, l_0,   th_1  ],
    [ sp.pi/2,     0,   0,   th_2  ],
    [      0 ,   l_2,   0,   th_3  ],
    [-sp.pi/2,   l_3,   0,   th_4  ],
    [ sp.pi/2,     0,   0,   th_5 ],
    [ sp.pi/2,     0,   0,   th_6 ],
    ])



########################################################
#
#      6.1  (easy algebraic soln)

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params6p1 = [l_3]
# 1 for rotary, 0 for prismatic
v6p1 = [1,1,1,1,1,1]
dh6p1 = sp.Matrix([
   [     0    ,     0 ,     0,   th_1  ],
   [   sp.pi/2,    l_2,    l_1,   th_2  ],
   [   sp.pi/2,    l_3 ,     0,   th_3  ],
   [       0  ,       0,   l_4,   th_4 ],
   [   sp.pi/2,       0,     0,   th_5  ],
   [     0,     0,   0,    0 ],
   ])


name = "MidTerm 2, Prob 1"
dh   = dh6p1
params = dh6p1
vv = v6p1

#name = 'Prob 6.2, EE543 W16'
#dh = dh6p2
#params = params6p2
#vv = v6p2

M = mechanism(dh, params, vv)

M.forward_kinematics()

#print ' ***   Prob'+name+' (M)  ****    '
#print ' - - - T_01'
#sp.pprint(notation_squeeze(M.T_01))
#print ' - - - T_12'
#sp.pprint(notation_squeeze(M.T_12))
#print ' - - - T_23 '
#sp.pprint(notation_squeeze(M.T_23))
#print ' - - - T_02 '
#sp.pprint(notation_squeeze(M.T_01*M.T_12))
#print ' - - - '

#sp.pprint(notation_squeeze(M.T_06))
# Forward Kinematics
if(1):
  print '--------------'
  print 'DH parameter Table: '
  sp.pprint(sp.latex(dh))

  print  '\n\n'

  print(sp.latex(notation_squeeze(M.T_06)))


  print '=============================================\n\n'
  print 'FK Solution: prob: '+name+'\\ Col 1'
  print '\\['
  sp.pprint(sp.latex(notation_squeeze(M.T_06[:,0])))
  print '\\]'
  print ' '
  print '\n'
  print 'Solution: prob: '+name+'\\ Col 2'
  print '\\['
  sp.pprint(sp.latex(notation_squeeze(M.T_06[:,1])))
  print '\\]'

  print ' '

  print ' Cols 3'
  print '\\['
  sp.pprint(sp.latex(notation_squeeze(M.T_06[:,2])))
  print '\\]'
  print ' '

  print ' Col 4'
  print '\\['
  sp.pprint(sp.latex(notation_squeeze(M.T_06[:,3])))
  print '\\]'

  print '--------------\n\n'
  print ' ----- * ----- * ----' + name + ' Cols 1, 2'
  sp.pprint(notation_squeeze(M.T_06[:,0:2]))
  print ' ----- * ----- * ----' + name + ' Cols 3, 4'

  sp.pprint(notation_squeeze(M.T_06[:,2:4]))

#print ' ----- * ----- * ----'
#print 'Intermediate result for inverse kinematics:   T_36, T_16'
#T_36 = (M.T_34*M.T_45*M.T_56)
#T_16 = M.T_12*M.T_23*T_36

#print 'T_36'
#Tdisp = T_36
#print ' Cols 1,2 '
#sp.pprint(sp.latex(notation_squeeze(Tdisp[:,0:2])))
#print ' Cols 3,4'
#sp.pprint(sp.latex(notation_squeeze(Tdisp[:,2:4])))
#print '\n\n --- *** ---\n'



