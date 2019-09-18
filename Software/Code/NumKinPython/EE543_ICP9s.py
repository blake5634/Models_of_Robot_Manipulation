#EE543  Solution to ICP 9: part 1  (Winter 18)
#
#

import sympy as sp 
sp.init_printing()
# declare symbolic variables to sympy
sp.var('l_1 l_2 l_4 th_1 th_2 d_3 th_4')

# Homogeneous transform for link based on DH parameters
def Link_S(al, a, d, th):
  t = sp.Matrix ([
    [sp.cos(th),                -sp.sin(th),               0,                a],
    [sp.sin(th)*sp.cos(al),  sp.cos(th)*sp.cos(al), -sp.sin(al), -sp.sin(al)*d],
    [sp.sin(th)*sp.sin(al),  sp.cos(th)*sp.sin(al),  sp.cos(al),  sp.cos(al)*d],
    [0  ,0  ,0  ,1]
    ])
  return t

def arm_FK_S(th_1,th_2,d_3,th_4):     
    T01 = Link_S(       0,     0,  l_1,  th_1)
    T12 = Link_S( sp.pi/2,     0,  l_2,  th_2)
    T23 = Link_S( sp.pi/2,     0,  d_3,  sp.pi/2)
    T34 = Link_S(-sp.pi/2,   l_4,    0,  th_4)

    print 'T01'
    sp.pprint( T01)

    print 'T12'
    sp.pprint( T12)
    
    print 'T23'
    sp.pprint( T23)
    
    print 'T34'
    sp.pprint( T34)
     

    print 'T04'
    sp.pprint (T01*T12*T23*T34)

    
print ' Symbolic FK output'
print '-------------- Both Poses'

sp.pprint(arm_FK_S(th_1,th_2,d_3,th_4)) 

 