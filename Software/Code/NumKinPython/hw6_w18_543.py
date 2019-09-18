#EE543  Solution:  hw5_Probs 6.1 Jan 2018
#
#

import numpy as np

TEST = False
deg2rad = 2.0*np.pi/360.0

al0 = 0 ;
deg = np.pi/180  # conversion factor deg to rad

# Homogeneous transform for link based on DH parameters
def Link_N(al, a, d, th):
  al = al*deg2rad
  th = th*deg2rad
  t = np.mat([
    [np.cos(th),                -np.sin(th),               0,                a],
    [np.sin(th)*np.cos(al),  np.cos(th)*np.cos(al), -np.sin(al), -np.sin(al)*d],
    [np.sin(th)*np.sin(al),  np.cos(th)*np.sin(al),  np.cos(al),  np.cos(al)*d],
    [0  ,0  ,0  ,1]
    ])
  return t

def colcross(a,b):
    m = np.matrix([[0    , -a[2],  a[1]],
                   [a[2] ,     0, -a[0]],
                   [-a[1],  a[0],    0 ]
                   ])
    return m*b

def wprop(T,wn,thd):
    R = T[0:3,0:3].T
    wn1 = R*wn + np.matrix([[0,0,thd]]).T
    return wn1
def vprop(T,wn,vn,thd,ddot): #  set wn=0 for prismatic joint, ddot=0 for rotary
    R = T[0:3,0:3].T
    P = T[0:3,3]
    #print 'Shapes:'
    #print np.shape(wn)
    #print np.shape(P)
    vn1 = R*(vn+colcross(wn,P)) + np.matrix([[0,0,ddot]]).T
    return vn1


### All DH parameters
##   (including values for the pose: theta_i)

l_1 = 7
l_2 = 1
l_3 = 2
l_4 = 2

al0 =  0;      a0 = l_1    ; d1 = 0   ; th1 = 90
al1 = -90;     a1 =  0     ; d2 = l_2 ; th2 = 25
al2 = 180;     a2 = l_3    ; d3 = l_4 ; th3 = -68

# a1 = 2

# Joint velocities
# (only one of each pair should be non-zero)

dd1 =    0;  # m/sec
thd1 =  10;  # deg/sec
dd2 =    0;
thd2 =   -15;
dd3  =   0;
thd3 =   3;


### TEST DH parameters

if(TEST):
     #//   (including values for the pose: theta_i)
    al0 = 90;     a0 = 5      ; d1 = 0   ; th1 = 90;
    al1 = 90;     a1 = 0      ; d2 = 0   ; th2 = 0;
    al2 = -90;    a2 = 5      ; d3 = 2   ; th3 = 137;

    #// Joint velocities
    #// (only one of each pair should be non-zero)

    dd1 =    0;  #// m/sec
    thd1 =  20;  #// deg/sec
    dd2 =    0;
    thd2 =   -45;
    dd3  =   0;
    thd3 =  10;

    #  end of Test Params


c1 = np.cos(th1*deg2rad)
c2 = np.cos(th2*deg2rad)
c3 = np.cos(th3*deg2rad)
s1 = np.sin(th1*deg2rad)
s2 = np.sin(th2*deg2rad)
s3 = np.sin(th3*deg2rad)

#  The link transforms
T01 = Link_N(al0,a0,d1,th1);
T12 = Link_N(al1,a1,d2,th2);
T23 = Link_N(al2,a2,d3,th3);

print 'T12: '
print T12

print 'Plugging into symbolic version: '
T12s = np.matrix([[c2, -s2, 0, 0],[0,0,1,l_2],[-s2,-c2,0,0],[0,0,0,1]])
print T12s
print 'T23: '
print T23

print 'Plugging into symbolic version: '
T23s = np.matrix([[c3, -s3, 0, l_3],[-s3,-c3,0,0],[0,0,-1,-l_4],[0,0,0,1]])
print T23s

R03 = T01[0:3,0:3]*T12[0:3,0:3]*T23[0:3,0:3]

# base does not move
w00 = np.matrix([[0,0,0]]).T  # .T = transpose
v00 = np.matrix([[0,0,0]]).T

# angular velocity prop
w11 = wprop(T01,w00,thd1)
w22 = wprop(T12,w11,thd2)
w33 = wprop(T23,w22,thd3)

probnum = "6.2"   # sometimes this changes year to year


#print '\n'+probnum+'.1     w11: '
#print w11

#print '\n'+probnum+'.1     w22: '
#print w22

print '\n'+probnum+'.1     w33: '
print w33


# linear velocity propagation
v11 = vprop(T01,w00,v00,thd1,dd1);
v22 = vprop(T12,w11,v11,thd2,dd2);
v33 = vprop(T23,w22,v22,thd3,dd3);

#print '\n'+probnum+'.2    v11: '
#print v11
#print '\n'+probnum+'.2    v22: '
#print v22



print '\n'+probnum+'.2    v33: '
print v33
 
print '\n'+probnum+'.3   R03*w33'
print R03*w33


print '\n'+probnum+'.4   R03*v33'
print R03*v33



dth = (th2-th3)*deg2rad
sdth = np.sin(dth)
cdth = np.cos(dth)
# Check 5.2.2:

v22s = np.matrix([[-l_2*c2*thd1,l_2*s2*thd1, 0]])

v33s1 = np.matrix([[c3,-s3,0],[-s3,-c3,0],[0,0,-1]])* np.matrix(
    [
     [l_4*c2*thd1-l_2*c2*thd1],
     [l_3*thd2-l_4*s2*thd1+l_2*s2*thd1],
     [l_3*c2*thd1]
    ])

v33s = np.matrix([[(l_4-l_2)*cdth*thd1-l_3*s3*thd2, (l_4-l_2)*sdth*thd1-l_3*c3*thd2, -l_3*c2*thd1]]).T

w33s = np.matrix([[np.sin(dth)*thd1, np.cos(dth)*thd1, thd3-thd2]]).T

w22s = np.matrix([[-s2*thd1, -c2*thd1,thd2]])

print 'Check of symbolic solution for w22: plugging in: '
print w22s
print 'Check of symbolic solution for w33: plugging in: '
print w33s

print 'Check of symbolic solution for V33s: plugging in: '
print v33s

