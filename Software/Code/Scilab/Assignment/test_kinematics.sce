// Kinematics test cases
//
//   EE 543 University of Washington
//
//     18-Mar-2013


exec('kinfunc.sce');
exec('quaternion.sci');

//  1.  Rotations I


A = rotx(30) * roty(60) * rotz(17.5)

// compute roll pitch yaw matrix two ways:
B = rotz(30) * roty(40) * rotx(50);
C = rrpy(50,40,30)

B-C

//   2.  Rotations II

// Equivalent Axis-Angle rotation matrix
K = [1 2 3]';
R = equiv(K, 45);

// Quaternion
q1 = quaternion(K, 45);
q2 = qcomp(q1);
q3 = qtimes(q1,q2);

/////////////////////////////////////////////////////////////////////

//    Some example computations


deg = %pi/180.0;

th1 = 30*deg;   K1 = [0,1,0];
th2 = 45*deg;   K2 = [1,0,0];

q1 = quaternion(K1,th1);
q2 = quaternion(K2,th2);

e1 = norm(quat2rot1(q1) - quat2rot2(q1) );

e2 = norm(quat2rot1(q2) - quat2rot2(q2) );

r1 = quat2rot1(q1);
r2 = quat2rot1(q2);

e3 = norm(r2*r1 - quat2rot1(qtimes(q2,q1)));


////////////////////////////////////////////////////////////////

//   example \ref{quaternionmultiplication}


//   2. Homogeneous Transforms

T1 = trans4(K, 10);
T2 = rot4(roty(45));

// 3.  Forward Kinematics

th1= 0; th2=30;th3=45;th4=37;th5=32;

T01 = link(0,     0,  0, th1);
T12 = link(90,    0,  0, th2);
T23 = link(-90, .37,  0, th3);
T34 = link(90,  0.5,.25, th4);
T45 = link(0,    .1,  0, th5);

T05 = T01*T12*T23*T45





