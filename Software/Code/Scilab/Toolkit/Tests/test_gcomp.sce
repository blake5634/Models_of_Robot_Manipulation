// test case 5: gravity comp for manip of Example 3.2

clear all;
exec('tg_gravcomp.sce',-1)
 
l0 = 0.75;
l1 = 1.7;
l2 = 1.337;

al0 = -90;     al1 = 90;    al2 = 90;
a0  = 0;      a1 = l1;     a2 = l2;
d1  = l0;     d2 = 0.2;    d3 = 0;

// Pose: 
th1 = 10; th2 = 12.7; th3 = -27.6;

// coms
C11 = [0.5,0,0]';  C22 = [.2,.7,.15]';  C33 = [.4,0.1,0]';

// link masses

M(1) = 1.0;
M(2) = 1.0;
M(3) = 0.5;

// Link Transforms

T01 = link(al0, a0, d1, th1);
T12 = link(al1, a1, d2, th2);
T23 = link(al2, a2, d3, th3);

// build the T matrix vector

T = tcat(T01,T12);
T = tcat(  T,T23);

// build the COM vector

C(:,1) = C11;
C(:,2) = C22;
C(:,3) = C33;

// gravity (Planet Earth!)
g00 = [0,0,-9.8]';

// compute the gravity torques
t = jtorque(T,C,M,g00);

disp(t)

M(2) = 2.5;  // should change only t1 and t2 but not t3

// compute the gravity torques
t = jtorque(T,C,M,g00);

disp(t)
