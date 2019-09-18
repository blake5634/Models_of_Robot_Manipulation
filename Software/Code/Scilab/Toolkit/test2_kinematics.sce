// Kinematics test cases
//
//   EE 543 University of Washington
//
//     18-Mar-2013
//
//    Updated with Pass/Fail    April 2016


clear all
global NTests NPass


function check(X,name) 
  global NTests NPass
  W = 25
  if length(name) > W then
      name = part(name, [1:25])
  else
      t = part(" ",ones(1:W-length(name)))
      name = strcat([name, t])  
  end
  NTests = NTests+1
  if (norm(X,1) < E_num_tol) then
      mprintf("Test %d, [%25s]: pass\n",NTests,name)
      NPass = NPass + 1
  else
      mprintf("Test %d, [%25s]: FAIL\n",NTests,name)
  end
endfunction

mprintf("\n EE543 Kinematics Functions TEST\n          %s\n\n",date())


E_num_tol = 1.0D-06

NTests = 0
NPass = 0

exec('kinfunc.sce');
exec('quaternion.sci');

//  1.  Rotations I

A = rotx(30) * roty(60) * rotz(17.5)
 
Ac = [ 0.4768585   -0.1503529    0.8660254  ;
       0.6733904    0.6957337    -0.25       ;
      -0.5649348    0.7023878    0.4330127   ];
      
check(A-Ac,"Multiplication basic rotation matrices");

// compute roll pitch yaw matrix two ways:
B = rotz(30) * roty(40) * rotx(50);
C = rrpy(50,40,30)

check(B-C,"Roll-Pitch-Yaw two ways")

//   2.  Rotations II

// Equivalent Axis-Angle rotation matrix
K = [1 2 3]';
R_kequiv = equiv(K, 47);

Rc = [
    0.7047128   -0.5409587    0.4590682  ;
    0.6318163    0.7728560   -0.0591761  ;
  - 0.3227818    0.3317489    0.8864280  ]

check(R_kequiv-Rc,"Equiv axis/angle 3x3")



// Quaternion
q1 = quaternion(K, 45);
q2 = qcomp(q1);
q3 = qtimes(q1,q2);

check(q3-[1 0 0 0 ]',"Quaternion 01: times and complement")

/////////////////////////////////////////////////////////////////////

//    Some example computations


deg = %pi/180.0;

th1 = 30*deg;   K1 = [0,1,0];
th2 = 45*deg;   K2 = [1,0,0];

q1 = quaternion(K1,th1);
q2 = quaternion(K2,th2);

r1 = quat2rot1(q1);
r2 = quat2rot1(q2);

e1 = r2*r1 - quat2rot1(qtimes(q2,q1));
check(e1,"Quaternion 02: quaternion product")



////////////////////////////////////////////////////////////////

//   example \ref{quaternionmultiplication}

th = 191
K = [27.9, -87.1, 42.3]';
R = equiv(K, th);
T1 = rot4(R);
R = roft(T1);
th1 = equivth(R);
K1 =  equivK(R,th1);
R1 = equiv(K1,th1);
 
check(R-R1, "R: 3x3 fwd inverse equiv fcns")
 
//   2. Homogeneous Transforms

K = [0 1 0 ]'
T1 = rot4(equiv(K, 45));
T2 = rot4(roty(45));

check(T1-T2,"4x4 rotation matrices construction");

// 3.  Forward Kinematics

th1= 0; th2=30;th3=45;th4=37;th5=32;

T01 = link(0,     0,  0, th1);
T12 = link(90,    0,  0, th2);
T23 = link(-90, .37,  0, th3);
T34 = link(90,  0.5,.25, th4);
T45 = link(0,    .1,  0, th5);

T05 = T01*T12*T23*T45

C = [  0.1948133  -0.8438292      -0.5       0.3816666  ;  
    0.9743701      0.2249511       0.        0.0707107  ;
    0.1124755     -0.4871850       0.8660254 0.2203553  ;  
    0.           0.           0.           1.         ];

check(T05-C,"Forward Kinematics T05");

n1 = norm(T05(1,1:3));
n2 = norm(T05(1:3,2));
check(n1-1.0,"Norm of    row == 1 ");
check(n2-1.0,"Norm of column == 1 ");

mprintf("\n\n========= End of Testing =============")
mprintf("\nNumber of Tests:  %d",NTests)
mprintf("\nNumber Passed:    %d", NPass)
mprintf("\n\n")




