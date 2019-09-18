# EE543  Programming assignment 2:   P2, Rotation Matrices
#
#
#

import numpy as np

deg2rad = 2.0*np.pi/360.00

def colvector(a,b,c):
    m = np.matrix([ [a], [b], [c] ])
    return m

def RotX3(th):
    th = th*deg2rad
    ct = np.cos(th)
    st = np.sin(th)
    m = np.matrix([[1 ,  0,     0 ],
                   [0 ,  ct,  -st ],
                   [0 ,  st,   ct ]
                   ])
    return m


##########################################

#    Complete the Rotation matrices here

#########################################






def colcross(a,b):
    m = np.matrix([[0    , -a[2],  a[1]],
                   [a[2] ,     0, -a[0]],
                   [-a[1],  a[0],    0 ]
                   ])
    return m*b

def colmag(x):
    t=0
    for i in [0, 1, 2]:
        t += x[i]*x[i]
    return np.sqrt(t)


# examples;

# basic tests
print RotX3(45)

print RotX3(20)*RotX3(25)

#required test

TM =  RotX3(27)*RotY3(-16)*RotX3(125)*RotZ3(45)
print TM
##Correct Test Result:

correct = np.matrix([
 [ 0.5200578,  -0.83937153,  0.15809909],
 [-0.70263649, -0.52566641, -0.47955895],
 [ 0.48563551,  0.13831218, -0.86314998]
 ] )

if np.allclose(TM,correct):
    print "CORRECT!"
else:
    print "ERROR in one or more Rotation Matrices!"
    print TM-correct

 # Check if the rows and cols are Mag=1:

print colmag(TM[0:3,0])  # col 1
print colmag(TM[0:3,1])
print colmag(TM[0:3,2])  # col 3
print colmag(TM[0,0:3].T)  # row 1
print colmag(TM[1,0:3].T)  # row 2
print colmag(TM[2,0:3].T)  # row 3


