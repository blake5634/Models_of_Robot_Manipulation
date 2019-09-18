#EE543  Solution to ICP 8
#
#

#from ikbtbasics.pykinsym import *
import numpy as np 
  
# arm's fixed geometric parameters
l_1 = 1.0
l_2 = 0.5
l_4 = 0.25

deg = np.pi/180  # conversion factor deg to rad

# Homogeneous transform for link based on DH parameters
def Link_N(al, a, d, th):
  t = np.matrix ([
    [np.cos(th),                -np.sin(th),               0,                a],
    [np.sin(th)*np.cos(al),  np.cos(th)*np.cos(al), -np.sin(al), -np.sin(al)*d],
    [np.sin(th)*np.sin(al),  np.cos(th)*np.sin(al),  np.cos(al),  np.cos(al)*d],
    [0  ,0  ,0  ,1]
    ])
  return t

def arm_FK(th_1,th_2,d_3,th_4):     
    T01 = Link_N(       0,     0,  l_1,  th_1)
    T12 = Link_N( np.pi/2,     0,  l_2,  th_2)
    T23 = Link_N( np.pi/2,     0,  d_3,  np.pi/2)
    T34 = Link_N(-np.pi/2,   l_4,    0,  th_4)

    print 'T01'
    print T01

    print 'T12'
    print T12
    
    print 'T23'
    print T23
    
    print 'T34'
    print T34
     

    print 'T04'
    print T01*T12*T23*T34

    
#Pose 1
print '-------------- Pose 1'

arm_FK(37*deg,-122*deg,0.68,192*deg) 

#Pose 2
print '-------------- Pose 2'

arm_FK(120*deg,-27*deg,0.25, 63*deg)
 