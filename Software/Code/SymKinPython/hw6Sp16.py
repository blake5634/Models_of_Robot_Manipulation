#!/usr/bin/python


import sympy as sp
import numpy as np

from kin_cl import *


sp.init_printing()

##########################################################################33
#
#        EE543  HW6    Sp 16
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))

((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))

((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))

sp.var('L_2 L_3')



########################################################
#
#      6.1  (easy algebraic soln)

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params6p1 = [L_2, L_3]
# 1 for rotary, 0 for prismatic
v6p1 = [1,1,1,1,1,1]
dh6p1 = sp.Matrix([
   [      0 ,     0,   h,   th_1  ],
   [ sp.pi/2,     0,   0,   th_2  ],
   [-sp.pi/2,   L_2,   0,   th_3  ],
   [ sp.pi/2,     0, L_3,     0   ],
   [      0 ,     0,   0,     0   ],
   [      0 ,     0,   0,     0   ],
   ])
 
name = 'Prob 6.1, EE543 Sp16'
dh = dh6p1
params = params6p1
vv = v6p1

M = mechanism(dh, params, vv)

M.forward_kinematics()
#sp.pprint(notation_squeeze(M.T_06))
# Forward Kinematics

sp.pprint(sp.latex(notation_squeeze(M.T_06)))

if(0):
  print '--------------'
  print 'DH parameter Table: '
  sp.pprint(sp.latex(dh))
  print '\n'
  print 'Solution: prob: '+name+'\\ Col 1'
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

  print '--------------'
  #sp.pprint(notation_squeeze(M.T_06[:,0:2]))
  print ' ----- * ----- * ----' + name + ' Cols 3,4'

  sp.pprint(notation_squeeze(M.T_06[:,2:4]))

print ' ----- * ----- * ----' 


