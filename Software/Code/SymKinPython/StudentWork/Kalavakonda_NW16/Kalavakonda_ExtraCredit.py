
# coding: utf-8

# In[19]:

#!/usr/bin/python
import sympy as sp
import numpy as np
import math
from kin_cl2 import * #Specific to the problem in hand
sp.init_printing()


# In[20]:

def frwd(th1,d2,d3,th4,th5,th6): #Forward Kinematics call
    h,l_1,l_2 = 2,0.3,1
    params_example = [l_1,l_2,l_3]
    v_example = [1,0,0,1,1,1]
    dh_example1 = sp.Matrix([
            [  0,    0,  h,   th1 ],
            [-sp.pi/2 ,     0,  d2 ,   0  ],
            [ sp.pi/2,    0, d3 ,   sp.pi/2  ],    
            [     0,     l_1, 0,      th4 ],    
            [     sp.pi/2,     0,   0,      th5],
            [     sp.pi/2,     0,   0,      th6],
            [0,0,l_2,0]            
        ])
    name = "Final Project Extra credit"
    dh1 = dh_example1
    params = params_example
    vv = v_example
    M1 = mechanism(dh1, params, vv)
    #  compute the forward kinematics and Jacobian of M
    M1.forward_kinematics()
    return M1.T_07


# In[21]:

def invKyn_Nkalavak(T): #Assuming that every angle is desired in degrees
    h,l_1,l_2,ns = 2,0.3,1,0 #Constants
    d3 = T[2,3]-h-l_2*T[2,2]
    th5_1 = math.acos(-T[2,2]) 
    th5_2 = -math.acos(-T[2,2]) #Multiple solutions with the various signs
    th6_1 = math.atan2(-T[2,1]/math.sin(th5_1),T[2,0]/math.sin(th5_1))
    th6_2 = math.atan2(-T[2,1]/math.sin(th5_2),T[2,0]/math.sin(th5_2))
    d2_1 = -l_1 + math.sqrt(math.pow((T[0,3]-l_2*T[0,2]),2)+math.pow(T[1,3]-l_2*T[1,2],2))
    d2_2 = -l_1 - math.sqrt(math.pow((T[0,3]-l_2*T[0,2]),2)+math.pow(T[1,3]-l_2*T[1,2],2))
    th1_1 = math.atan2((l_2*T[0,2]-T[0,3])/math.sqrt(math.pow((T[0,3]-l_2*T[0,2]),2)+math.pow(T[1,3]-l_2*T[1,2],2)),(T[1,3]-l_2*T[1,2])/math.sqrt(math.pow((T[0,3]-l_2*T[0,2]),2)+math.pow(T[1,3]-l_2*T[1,2],2)))
    th1_2 = math.atan2(-(l_2*T[0,2]-T[0,3])/math.sqrt(math.pow((T[0,3]-l_2*T[0,2]),2)+math.pow(T[1,3]-l_2*T[1,2],2)),-(T[1,3]-l_2*T[1,2])/math.sqrt(math.pow((T[0,3]-l_2*T[0,2]),2)+math.pow(T[1,3]-l_2*T[1,2],2)))
    th14_1 = math.atan2(-T[0,2]/math.sin(th5_1),T[1,2]/math.sin(th5_1))-th1_1
    th14_2 = math.atan2(-T[0,2]/math.sin(th5_1),T[1,2]/math.sin(th5_1))-th1_2
    th14_3 = math.atan2(-T[0,2]/math.sin(th5_2),T[1,2]/math.sin(th5_2))-th1_1
    th14_4 = math.atan2(-T[0,2]/math.sin(th5_2),T[1,2]/math.sin(th5_2))-th1_2
    sol = []
    T1 = frwd(th1_1,d2_1,d3,th14_1,th5_1,th6_1)
    A1 = T1-T                             #Comparing the two transformation matrices
    #a1 = np.square(A1)
    a1 = A1**2           
    if(np.sum(a1)<10e-5):                 #Checking the tolerance level and deciding if it qualifies as a solution or not
        ns+=1
        sol.append([th1_1,d2_1,d3,th14_1,th5_1,th6_1])
    T2 = frwd(th1_1,d2_1,d3,th14_3,th5_2,th6_2)
    A2 = T2-T                             #Comparing the two transformation matrices
    #a2 = np.square(A2)
    a2 = A2**2
    if(np.sum(a2)<10e-5):               #Checking the tolerance level and deciding if it qualifies as a solution or not
        ns+=1
        sol.append([th1_1,d2_1,d3,th14_3,th5_2,th6_2])
    T3 = frwd(th1_2,d2_2,d3,th14_2,th5_1,th6_1)
    A3 = T3-T                            #Comparing the two transformation matrices
    #a3 = np.square(A3)
    a3 = A3**2
    if(np.sum(a3)<10e-5):                #Checking the tolerance level and deciding if it qualifies as a solution or not
        ns+=1
        sol.append([th1_2,d2_2,d3,th14_2,th5_1,th6_1])
    T4 = frwd(th1_2,d2_2,d3,th14_4,th5_2,th6_2)
    A4 = T4-T                           #Comparing the two transformation matrices
    a4 = np.square(A4)
    if(np.sum(a4)<10e-5):               #Checking the tolerance level and deciding if it qualifies as a solution or not
        ns+=1
        sol.append([th1_2,d2_2,d3,th14_4,th5_2,th6_2])
    return ns,sol


# In[22]:

#!/usr/bin/python
"""import sympy as sp
import numpy as np
from kin_cl2 import *
sp.init_printing()"""
##########################################################################33
#
#        EE543  Final Project Template
#
############################################################################

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))




########################################################
#
#     Robot Parameters

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params_example = [l_1,l_2,l_3]

#  define joint variables:
#         1 for rotary, 0 for prismatic
v_example = [1,0,0,1,1,1]


#  Here are your DB parameters
# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
h,l_1,l_2 = 2,0.3,1
th_1,d_2,d_3,th_4,th_5,th_6 = 30*deg,0.5,0.5,45*deg,35*deg,60*deg
dh_example = sp.Matrix([
   [  0,    0,  h,   th_1 ],
   [-sp.pi/2 ,     0,  d_2 ,   0  ],
   [ sp.pi/2,    0, d_3 ,   sp.pi/2  ],    # this is only a 3DOF example
   [     0,     l_1, 0,      th_4 ],     # must fill remaining 3 rows with zeros
   [     sp.pi/2,     0,   0,      th_5],
   [     sp.pi/2,     0,   0,      th_6],
   [0,0,l_2,0]        
  
        
   ])

###############################################################################3


name = "Final Project Extra credit"
dh = dh_example
params = params_example
vv = v_example
#dh1 = dh_example1
#dh2 = dh_example2
#  set up a mechanism object
M = mechanism(dh, params, vv)

#  compute the forward kinematics and Jacobian of M
M.forward_kinematics()
T = M.T_07
invKyn_Nkalavak(T)


# In[23]:

print M.T_07


# In[33]:

T1 = np.asarray([[-0.9, 0.8, 0.8, 101], 
      [0.67, 0.299354754196832, 0.148453453391305, 2.6], 
      [0.286788647740015, -0.496730597540120, -0.819152601775084, 115], 
      [0, 0, 0, 1]])
print T1


# In[34]:

invKyn_Nkalavak(T1)


# In[ ]:



