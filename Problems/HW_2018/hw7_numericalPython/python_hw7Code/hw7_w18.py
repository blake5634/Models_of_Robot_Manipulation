#!/usr/bin/python
#     EE543  University of Washington
#     hw7    Feb 2018
 

import numpy as np
import random as rand

# for extra credit prob.
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#

deg = np.pi/180  # conversion factor deg to rad

# Homogeneous transform for link based on DH parameters
def Link_N(al, a, d, th):
  al = al*deg
  th = th*deg
  t = np.mat([
    [np.cos(th),                -np.sin(th),               0,                a],
    [np.sin(th)*np.cos(al),  np.cos(th)*np.cos(al), -np.sin(al), -np.sin(al)*d],
    [np.sin(th)*np.sin(al),  np.cos(th)*np.sin(al),  np.cos(al),  np.cos(al)*d],
    [0  ,0  ,0  ,1]
    ])
  return t


############################################################   Set up Puma 560 Robot
#
#


### All DH parameters
##   (including values for the pose: theta_i)
 
###  PUMA parameters from Craig pages 80, 129
 
#angles in degrees, distances in meters
al_0 =  0 ; a_0 =  0     ;  d_1 = 0.660 ;  
al_1 = -90; a_1 =  0     ;  d_2 = 0.0   ; 
al_2 =  0 ; a_2 = 0.432  ;  d_3 = 0.1245; 
al_3 = -90; a_3 = 0.0203 ;  d_4 = 0.432 ;
al_4 =  90; a_4 = 0      ;  d_5 = 0     ;
al_5 = -90; a_5 = 0      ;  d_6 = 0     ; 


##
#          Prob 7.2
#
##

th1 = -125   # will be converted to rad in Link_N
th2 = 12.7
th3 = -40
th4 = 32.7
th5 = -63.7
th6 = 44.2


T01 = Link_N(al_0, a_0, d_1, th1)
T12 = Link_N(al_1, a_1, d_2, th2)
T23 = Link_N(al_2, a_2, d_3, th3)
T34 = Link_N(al_3, a_3, d_4, th4)
T45 = Link_N(al_4, a_4, d_5, th5)
T56 = Link_N(al_5, a_5, d_6, th6)

T06 = T01*T12*T23*T34*T45*T56

print '\n---------\nProblem 7.2: '

np.set_printoptions(precision=4,suppress=True)

print T06

print 'Norm of R row 1'
print 0.9991**2 +  0.004**2 +    0.0429**2

print '\n\nProblem 7.3'

N = 1000   # eventually 100k


# initialize max and min vectors (that's a lot of meters!)
dmax = np.ones(3) * -99999
dmin = np.ones(3) *  99999

points = np.array(np.zeros([N,3]))

# big loop
for i in range(N):
    th1 = rand.uniform(0, 360.0)  # floating point deg
    th2 = rand.uniform(0, 360.0)
    th3 = rand.uniform(0, 360.0)
    th4 = rand.uniform(0, 360.0)
    th5 = rand.uniform(0, 360.0)
    th6 = rand.uniform(0, 360.0)

    T01 = Link_N(al_0, a_0, d_1, th1)
    T12 = Link_N(al_1, a_1, d_2, th2)
    T23 = Link_N(al_2, a_2, d_3, th3)
    T34 = Link_N(al_3, a_3, d_4, th4)
    T45 = Link_N(al_4, a_4, d_5, th5)
    T56 = Link_N(al_5, a_5, d_6, th6)

    T06 = T01*T12*T23*T34*T45*T56
    # find dmin and dmax XYZ values    
    for j in range(3):
        if(T06[j,3] > dmax[j]):
            dmax[j] = T06[j,3]
        if(T06[j,3] < dmin[j]):
            dmin[j] = T06[j,3]
            
print '\nDone! '
print '\n'
print 'Min and Max Ranges: '
print dmin[0], ' < X < ', dmax[0]
print dmin[1], ' < Y < ', dmax[1]
print dmin[2], ' < Z < ', dmax[2]

print '\n7.3.2: the number 0.87 is ~= a2+d4 which is basically the length of the arm.'


print '\n\nProblem 7.4'


# initialize max and min vectors (that's a lot of meters!)
dmax = np.ones(3) * -99999
dmin = np.ones(3) *  99999

# big loop WITH JOINT LIMITS
for i in range(N):
     
    th1 = rand.uniform(-170, 170)  # floating point deg
    th2 = rand.uniform(-225, 45)
    th3 = rand.uniform(-250,  75)
    th4 = rand.uniform(-135, 135)
    th5 = rand.uniform(-100, 100)
    th6 = rand.uniform(-189, 180)

    # points NOT reachable!
    th1 = rand.uniform( 170, 190)  # floating point deg
    th2 = rand.uniform(45, 360-225)
    th3 = rand.uniform(75, 360-250)
    th4 = rand.uniform(135, 360-135)
    th5 = rand.uniform(100, 360-100)
    th6 = rand.uniform(180, 360-189)

    T01 = Link_N(al_0, a_0, d_1, th1)
    T12 = Link_N(al_1, a_1, d_2, th2)
    T23 = Link_N(al_2, a_2, d_3, th3)
    T34 = Link_N(al_3, a_3, d_4, th4)
    T45 = Link_N(al_4, a_4, d_5, th5)
    T56 = Link_N(al_5, a_5, d_6, th6)

    T06 = T01*T12*T23*T34*T45*T56
    # find min and max XYZ values    
    for j in range(3):
        if(T06[j,3] > dmax[j]):
            dmax[j] = T06[j,3]
        if(T06[j,3] < dmin[j]):
            dmin[j] = T06[j,3]
        # store the points
        points[i,j] = T06[j,3]
            
            
print '\nDone! (with joint limits) '
print '\n'
print 'Min and Max Ranges: '
print dmin[0], ' < X < ', dmax[0]
print dmin[1], ' < Y < ', dmax[1]
print dmin[2], ' < Z < ', dmax[2]

print '\nProblem 7.4'
print 'Key difference:  Z does not go very far negative'
print 'Explanation:     Joint limits on th2, th3, prevent hand from striking base'

print'\nEXTRA CREDIT: '
workvolume = ConvexHull(points)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot all points
ax.plot(points.T[0], points.T[1], points.T[2], "ko")

# 12 = 2 * 6 faces are the simplices (2 simplices per square face)
for s in workvolume.simplices:
    s = np.append(s, s[0])  # Here we cycle back to the first coordinate
    ax.plot(points[s, 0], points[s, 1], points[s, 2], "r-")

ax.set_xlim3d(-1,1)
ax.set_ylim3d(-1,1)
ax.set_zlim3d(0,2)

# Make axis label 
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()



