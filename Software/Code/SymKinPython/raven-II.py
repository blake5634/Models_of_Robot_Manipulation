#!/usr/bin/python
import sympy as sp
import numpy as np
from kin_cl import *
sp.init_printing()
##########################################################################33
#
#        Example:  Raven-II LEFT / Gold Arm(see report UWEETR-2013-0006)
#
############################################################################


# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_w, l_2, l_3, l_4,l_5)) = sp.symbols(('h','l_0', 'l_w', 'l_2', 'l_3', 'l_4','l_5'))

#((la12, la23)) = sp.symbols(('la12', 'la23')); # non-nice alphas
sp.var('la12 la23 s_a12 c_a12 s_a23 c_a23')  #  alphas and their trig fcns

def nsq(x):
  x = notation_squeeze(x)
  x = x.subs(sp.sin(la12), s_a12)
  x = x.subs(sp.cos(la12), c_a12)
  x = x.subs(sp.sin(la23), s_a23)
  x = x.subs(sp.cos(la23), c_a23)
  return x



# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params = [l_w,l_2,l_3,l_5] 

#params7p1 = {}
# 1 for rotary, 0 for prismatic 

vRavenII = [1,1,0,1,1,1]
dhRavenII = sp.Matrix([
   [   0         ,    0 ,   0  ,  th_1 ],
   [  la12       ,    0 ,   0  ,  th_2  ],
   [ sp.pi-la23  ,    0 ,  d_3 ,  sp.pi/2  ],
   [   0         ,    0 ,  d_4 ,  th_4  ],
   [  sp.pi/2    ,    0 ,   0  ,  th_5  ],
   [  sp.pi/2    ,  l_w ,   0  ,  th_6  ]
   ])
 
DOFs = 6

name = "Raven-II" 
dh = dhRavenII
params = params
vv = vRavenII 

M = mechanism(dh, params, vv)

M.forward_kinematics()

print "\n\n\n           " + name + "\n\n\n---------------------------------  Frame 6 Jacobian:"

print "\n\nJacobian $^6J_6$ col 1: element by element:"

for i in range(0,6):
  print "$^J_^$ col 1 row ", i+1
  print "\\begin{equation}"
  sp.pprint(sp.latex(nsq(M.J66[i,0]))) 
  print "\\end{equation}"

print "\n\nJacobian $^6J_6$ col ", 2
print "\\begin{equation}"
sp.pprint(sp.latex(nsq(M.J66[0:6,1]))) 
print "\\end{equation}"

print "\n\nJacobian $^6J_6$ cols 3-6"
print "\\begin{equation}"
sp.pprint(sp.latex(nsq(M.J66[0:6,2:6]))) 
print "\\end{equation}"

# Forward Kinematics
if(0):

  #print '  ====  T06'
  #sp.pprint(notation_squeeze(M.T_06)) 

  print ' ======  T36'
  sp.pprint(notation_squeeze(M.T_34*M.T_45*M.T_56))  
	    
	    