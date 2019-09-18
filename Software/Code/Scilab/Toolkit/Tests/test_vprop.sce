//  *lab assignment 4: velocity propagation
//   test cases

exec('kinfunc.sce',-1);

// All DH parameters

al0 = 0;    al1 = -90;    al2 = 180;    al3 = 90;
a0 = 0;     a1 = 1;      a2 = 0;      a3 = 2;
d1 = 3;     d2 = 1.5;    d3 = 1;      d4 = 0;

//Pose
th1 = 90;   th2 = 0;     th3 = 45;    th4 = 137;

// Joint velocities
// (only one of each pair should be non-zero)

dd1 =    0;  // m/sec
thd1 =  20;  // deg/sec

dd2 =    0.7;   
thd2 =   0;

dd3  =   0;
thd3 =  10;    

dd4  =   0;
thd4 = -16;   

//  The link transforms

T01 = link(al0,a0,d1,th1);
T12 = link(al1,a1,d2,th2);
T23 = link(al2,a2,d3,th3);
T34 = link(al3,a3,d4,th4);

w00 = [0,0,0]';  // base is not moving
v00 = [0,0,0]';

//angular velocity propagation
w11 = wprop(T01,w00,thd1);
w22 = wprop(T12,w11,thd2);
w33 = wprop(T23,w22,thd3);
w44 = wprop(T34,w33,thd4);

// linear velocity propagation
v11 = vprop(T01,w00,v00,dd1);
v22 = vprop(T12,w11,v11,dd2);
v33 = vprop(T23,w22,v22,dd3);
v44 = vprop(T34,w33,v33,dd4);



// printf("Velocities: w44: %6.2f  \n",w44);
printf("Velocities: w44: %6.2f    v44: %6.2f\n",w44,v44);

