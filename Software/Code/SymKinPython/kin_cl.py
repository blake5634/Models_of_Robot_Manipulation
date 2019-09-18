#!/usr/bin/python
#
import sympy as sp
import numpy as np
from pykinsym import *

######################################################################
#
#   Python class for symbolic analysis of serial mechanisms
#        Revision 1      4-Feb-2016
#        Revision 2  1-Apr-2016
#

sp.init_printing()

class mechanism:

  def __init__(self, dh, params, varvect):
    self.DH = dh
    self.vv = varvect
    self.params = params
    self.jlims = np.array([
      [-np.pi, np.pi],
      [-np.pi, np.pi],
      [-np.pi, np.pi],
      [-np.pi, np.pi],
      [-np.pi, np.pi],
      [-np.pi, np.pi]
      ])
    self.jnum = np.matrix(np.zeros(36).reshape(6,6))


  ###############  compute kinematic transforms and equations for the manipulator (including Jacobian)
  def forward_kinematics(self):
    ###   set up symbolic variables

    # angular velocities of each link
    (self.w_00, self.w_11, self.w_22, self.w_33, self.w_44, self.w_55, self.w_66) =           sp.symbols(('self.w_00','self.w_11','self.w_22','self.w_33','self.w_44','self.w_55','self.w_66'))

    # linear velocities of each link
    (self.v_00, self.v_11, self.v_22, self.v_33, self.v_44, self.v_55, self.v_66) =           sp.symbols(('self.v_00','self.v_11','self.v_22','self.v_33','self.v_44','self.v_55','self.v_66'))

    # joint velocities of each link (qd stands for q-dot)
    (qd_0, qd_1, qd_2, qd_3, qd_4, qd_5, qd_6) =           sp.symbols(('qd_0','qd_1','qd_2','qd_3','qd_4','qd_5','qd_6'))

    # standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
    al = 0
    a = 1
    d = 2
    th = 3

    #  symbolic 4x4 transforms for each link
    self.T_01 = Link_S(self.DH[0,al], self.DH[0,a], self.DH[0,d], self.DH[0,th])
    self.T_12 = Link_S(self.DH[1,al], self.DH[1,a], self.DH[1,d], self.DH[1,th])
    self.T_23 = Link_S(self.DH[2,al], self.DH[2,a], self.DH[2,d], self.DH[2,th])
    self.T_34 = Link_S(self.DH[3,al], self.DH[3,a], self.DH[3,d], self.DH[3,th])
    self.T_45 = Link_S(self.DH[4,al], self.DH[4,a], self.DH[4,d], self.DH[4,th])
    self.T_56 = Link_S(self.DH[5,al], self.DH[5,a], self.DH[5,d], self.DH[5,th])

    #  here is the full FK derivation:
    self.T_06 = sp.trigsimp(self.T_01 * self.T_12 * self.T_23 * self.T_34 * self.T_45 * self.T_56)

    # Rotation sub matrices:
    self.R_01 = self.T_01[0:3, 0:3]
    self.R_12 = self.T_12[0:3, 0:3]
    self.R_23 = self.T_23[0:3, 0:3]
    self.R_34 = self.T_34[0:3, 0:3]
    self.R_45 = self.T_45[0:3, 0:3]
    self.R_56 = self.T_56[0:3, 0:3]
 
    # position offset vectors
    self.P_01 = self.T_01[0:3, 3]
    self.P_12 = self.T_12[0:3, 3]
    self.P_23 = self.T_23[0:3, 3]
    self.P_34 = self.T_34[0:3, 3]
    self.P_45 = self.T_45[0:3, 3]
    self.P_56 = self.T_56[0:3, 3]
    
    
    ###################################################
    #
    #   Select axes for application of sum-of-angles
    #
    # propagation steps below will only be trig simplified if consecutive axes 
    # are parallel to save time
    #   if \theta_j =0, then we should look for sin(theta_j-1 + theta_j) etc. 
    #
   
    simp = np.zeros(6)
    for j in range(1,5):  # we will only trigsimp if \alpha_N-1 == {0,pi} signifying 
                          # parallel axes
      if(self.DH[j,al] == 0 or self.DH[j,al] ==  sp.pi):
         simp[j] = 1

    # velocity propagation for the Jacobian matrix
    ##########################
    # angular
    self.v_00 = sp.Matrix([0,0,0]) 
    self.w_00 = sp.Matrix([0,0,0])

    self.w_11 = self.R_01.T * self.w_00
    if(self.vv[0] == 1):
      self.w_11 += sp.Matrix([0,0,qd_1])
    
    self.w_22 = self.R_12.T * self.w_11
    if(self.vv[1] == 1):
	self.w_22 += sp.Matrix([0,0,qd_2])
    if(simp[1]):
        self.w_22 = sp.trigsimp(self.w_22)
    
    self.w_33 = self.R_23.T * self.w_22
    if(self.vv[2] == 1):
      self.w_33 += sp.Matrix([0,0,qd_3])
    if(simp[2]):
      self.w_33 = sp.trigsimp(self.w_33)

    self.w_44 = self.R_34.T * self.w_33
    if(self.vv[3] == 1):
      self.w_44 += sp.Matrix([0,0,qd_4])
    if(simp[3]):
      self.w_44 = sp.trigsimp(self.w_44)
    
    self.w_55 = self.R_45.T * self.w_44
    if(self.vv[4] == 1):
      self.w_55 += sp.Matrix([0,0,qd_5])
    if(simp[4]):
      self.w_55 = sp.trigsimp(self.w_55)

    self.w_66 = self.R_56.T * self.w_55
    if(self.vv[5] == 1):
      self.w_66 += sp.Matrix([0,0,qd_6])
    if(simp[5]):
      self.w_66 = sp.trigsimp(self.w_66)
      
    ##########################
    #linear
    self.v_00 = sp.Matrix([0,0,0]) 

    self.v_11 = self.R_01.T*(self.v_00 + self.w_00.cross(self.P_01))
    if(self.vv[0] == 0):
      self.v_11 += sp.Matrix([0,0,qd_1])

    self.v_22 = self.R_12.T*(self.v_11 + self.w_11.cross(self.P_12))
    if(self.vv[1] == 0):
      self.v_22 += sp.Matrix([0,0,qd_2])
    if(simp[1]):
      self.v_22 = sp.trigsimp(self.v_22)
      
    self.v_33 = self.R_23.T*(self.v_22 + self.w_22.cross(self.P_23))
    if(self.vv[2] == 0):
      self.v_33 += sp.Matrix([0,0,qd_3])
    if(simp[2]):
      self.v_33 = sp.trigsimp(self.v_33)

    self.v_44 = self.R_34.T*(self.v_33 + self.w_33.cross(self.P_34))
    if(self.vv[3] == 0):
      self.v_44 += sp.Matrix([0,0,qd_4])
    if(simp[3]):
      self.v_44 = sp.trigsimp(self.v_44)

    self.v_55 = self.R_45.T*(self.v_44 + self.w_44.cross(self.P_45))
    if(self.vv[4] == 0):
      self.v_55 += sp.Matrix([0,0,qd_5])
    if(simp[4]):
      self.v_55 = sp.trigsimp(self.v_55)

    self.v_66 = self.R_56.T*(self.v_55 + self.w_55.cross(self.P_56))
    if(self.vv[5] == 0):
      self.v_66 += sp.Matrix([0,0,qd_6])
    if(simp[5]):
      self.v_66 = sp.trigsimp(self.v_66)

    self.qdot = sp.Matrix([qd_1, qd_2, qd_3, qd_4, qd_5, qd_6])


    self.J66  = ManipJacobian_S(self.v_66, self.w_66, self.qdot)
 
  #########################################################################################
  #
  #      Numerical Functions
  #

  def forward_kinematics_N(self, pose):
    pp = pose.copy()
    pp.update(self.params)    # combine the pose and the params
    T1 = self.T06.subs(pp)    # substitue for all symbols

    # test to make sure all symbols are substituted with numeric values
    Num_check(T1)     # this quits if fails

    #generate the numerical link transforms
    self.Tn_01 = self.T_01.subs(pp)
    self.Tn_12 = self.T_12.subs(pp)
    self.Tn_23 = self.T_23.subs(pp)
    self.Tn_34 = self.T_34.subs(pp)
    self.Tn_45 = self.T_45.subs(pp)
    self.Tn_56 = self.T_56.subs(pp)

    T2 = np.matrix(T1)   # convert from sympy to numpy matrix
    return T2

  ##################
  #
  #  check joint limits
  #
  def jlcheck_N(self, ns, solutions):
   valid_solns = []
   for i in range(0,ns):
     this_soln = True
     for j in range(0,6):
       if((solutions[i,j] < self.jlims[j,0]) | (solutions[i,j] > self.jlims[j,1])):
            this_soln = False
       if(this_soln):
         #print 'Solution ', i, ' is valid'
         valid_solns.append(i)
   return(valid_solns)

  ##########################
  #
  #  evaluate Jacobian numerically
  #
  def Jacobian_N(self, pose):
    print "J: pose: "
    print pose
    print "----\nself.params: "
    print self.params
    #quit()
    pp = pose.copy()
    pp.update(self.params)  # combine params and pose variables for substitution
    self.jnum = self.J66.evalf(subs=pp)
    #pose.update(self.params)
    #self.jnum = self.J66.evalf(subs=pose)
    T1 = np.matrix(self.jnum)
    Num_check(T1)
    return T1


if __name__ == '__main__':
  ##########################################################################33
  #
  #        Testing
  #
  ############################################################################

  ((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))

  ((h, l_3, l_4)) = sp.symbols(('h', 'l_3', 'l_4'))

  params = [h, l_3, l_4]
  params = {h:5, l_3: 2, l_4: 6}

  # 1 for rotary, 0 for prismatic
  v = [1,1,1,1,1,1]

  # standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
  dh = sp.Matrix([
      [sp.pi/2, 0, h,   th_1],
      [0,       0, 0,   th_2   ],
      [-sp.pi/2, 0, l_3, th_3  ],
      [0,     l_4, 0,   th_4  ],
      [0,       0,  0,   0],
      [0,       0,  0,   0]
      ])

  M = mechanism(dh, params, v)

  M.forward_kinematics()

  #print ' - - - '
  #sp.pprint(notation_squeeze(M.T_01))
  #print ' - - - '
  #sp.pprint(notation_squeeze(M.T_12))
  #print ' - - - '
  #sp.pprint(notation_squeeze(M.T_23))
  print ' - - - '
  sp.pprint(notation_squeeze(M.T_01*M.T_12))
  print ' - - - '

  #sp.pprint(notation_squeeze(M.T_06))

  print '--------------'
  sp.pprint(sp.latex(notation_squeeze(M.T_06[:,0:2])))

  print ' '

  sp.pprint(sp.latex(notation_squeeze(M.T_06[:,2:4])))

  print ' --- Numerical Jacobian ---'
  pose = {th_1: 20*deg, th_2:45*deg, th_3:15*deg, th_4:-21.7*deg}
  print M.Jacobian_N(pose)

