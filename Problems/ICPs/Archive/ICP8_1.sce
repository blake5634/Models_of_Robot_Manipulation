// ICP 8.1   EE543 Univ of Washington

clear all;
 

//exec('kinfunc.sce', -1)

// gravity
g0 = [0,0,-9.8]';

// manpulator mass parameters

// link masses
m1 = 30; // kg
m2 = 20;
m3 = 10;

// centers of mass in link frame
c11 = [ 0, 0,-1.5,1]';  // add 4th row = 1 for homogenous coordinates
c22 = [.1, 0,-2.0,1]';
c33 = [.1,.1, .1, 1]';

// manipulator pose
th1 = 45;
d2 = 4;
th3 = 30;

//  Derive link transforms using DH params and pose
T01 = link(0,0,3,th1);
T12 = link(90,.1,d2,0);
T23 = link(-90,.8,0,th3);

// Convert the gravity vector to Frame 2

T02 = T01*T12;
R02 = T02(1:3,1:3);
g2 = R02' * g0;

// Convert c33 to c23

c23 = T23*c33;

// convert the torque

mm = m2*c22 + m3*c23;

t22 = cross(mm,g2);

printf ("\nJoint 2 gravity torque: %6.3d Nm\n", t22(3));  // joint torque is Z component
