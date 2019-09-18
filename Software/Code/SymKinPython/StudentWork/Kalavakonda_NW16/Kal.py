
# coding: utf-8

# In[1]:

#CODE written and tested on jupyter nb
#!/usr/bin/python
import sympy as sp
import numpy as np
import math
from kin_cl import *
sp.init_printing()


# In[5]:

def frwd1(d1,th2,th3,th4):
    l1,l2,l3,l4 = 0.3,1,1,1
    params_example = [l_1,l_2,l_3]
    v_example = [1,0,0,1,1,1]
    dh_example1 = sp.Matrix([
            [  0,    0,  d1,   0 ],
            [0 ,     l1,  0 ,   th2  ],
            [ sp.pi/2,    l2, 0 ,   th3  ],    
            [     0,     l3, 0,     th4 ],     
            [      0,     l4,   0,      0],
            [      0,     0,   0,      0]
        ])
    name = "Final Project Extra credit"
    dh1 = dh_example1
    params = params_example
    vv = v_example
    M1 = mechanism(dh1, params, vv)
    #  compute the forward kinematics and Jacobian of M
    M1.forward_kinematics()
    return M1.T_06


# In[6]:

def invKyn_AlexWilson(T):   #Assuming that every angle is desired in degrees
    l1,l2,l3,l4,ns = 0.3,1,1,1,0
    th2 = math.atan2(T[0,2],-T[1,2]) 
    th31 = math.acos(((T[1,3]/T[0,2])-l4*T[2,1]-l2)/l3)
    th32 = -math.acos(((T[1,3]/T[0,2])-l4*T[2,1]-l2)/l3)
    d11 = T[2,3]-l4*T[2,0]-l3*math.sin(th31)
    d12 = T[2,3]-l4*T[2,0]-l3*math.sin(th32)
    th41 = math.atan2(T[2,0],T[2,1])-th31
    th42 = math.atan2(T[2,0],T[2,1])-th32
    sol = []
    T1 = frwd1(d11,th2,th31,th41)
    A1 = T1-T                             #Comparing the two transformation matrices
    #a1 = np.square(A1)
    a1 = A1**2           
    if(np.sum(a1)<10e-5):                 #Checking the tolerance level and deciding if it qualifies as a solution or not
        ns+=1
        sol.append([d11,th2,th31,th41])
    
    T2 = frwd1(d12,th2,th32,th42)
    A2 = T2-T                             #Comparing the two transformation matrices
    #a2 = np.square(A2)
    a2 = A2**2
    if(np.sum(a2)<10e-5):               #Checking the tolerance level and deciding if it qualifies as a solution or not
        ns+=1
        sol.append([d12,th2,th32,th42])
    return ns,sol


# In[7]:

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))

params_example = [0.3,1,1,1]

#  define joint variables:
#         1 for rotary, 0 for prismatic
v_example = [0,1,1,1,1,1]


#  Here are your DB parameters
# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.

dh_example1 = sp.Matrix([
   [  0,    0,  2,   0 ],
   [0 ,     0.3,  0 ,   45*deg  ],
   [ sp.pi/2,    1, 0 ,   30*deg  ],    # this is only a 3DOF example
   [     0,     1, 0,      30*deg ],     # must fill remaining 3 rows with zeros
   [      0,     1,   0,      0],
   [      0,     0,   0,      0]
   ])
name = "Final Project Example"
dh1 = dh_example1
params = params_example
vv = v_example

#  set up a mechanism object
M1 = mechanism(dh1, params, vv)

#  compute the forward kinematics and Jacobian of M
M1.forward_kinematics()


#print ' Here are the forward kinematic equations for ' + name

#sp.pprint(M1.T_06)
T = M1.T_06
invKyn_AlexWilson(T)


# In[23]:

T1 = np.asarray([[0.353554852743743, -0.612372611853925, 0.707105897551811, 6.2], 
                 [0.353553969107008, -0.5, -0.707107664820180, 1], 
                 [0.866024570684938, 0.500001442967887, 0, 3.36602384920047], 
                 [0, 0, 0, 1]])
print T1


# In[24]:

[ns, solns] = invKyn_AlexWilson(T1)

print "# IK solutions: ", ns
