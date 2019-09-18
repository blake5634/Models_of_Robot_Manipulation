// hw 7.1  EE543 W16
exec('kinfunc.sce',-1);  // load the needed functions
//
//// All DH parameters
//   (including values for the pose: theta_i)
al0 = 90;     a0 = 5      ; d1 = 0   ; th1 = 90;
al1 = 90;     a1 = 0      ; d2 = 0   ; th2 = 0;
al2 = -90;    a2 = 5      ; d3 = 2   ; th3 = 137;

// Joint velocities
// (only one of each pair should be non-zero)

dd1 =    0;  // m/sec
thd1 =  20;  // deg/sec
dd2 =    0;
thd2 =   -45;
dd3  =   0;
thd3 =  10;

//  The link transforms
T01 = link(al0,a0,d1,th1);
T12 = link(al1,a1,d2,th2);
T23 = link(al2,a2,d3,th3);

// we will use this to rotate the velocities
R03 = T01(1:3,1:3)*T12(1:3,1:3)*T23(1:3,1:3)
disp(R03)


w00 = [0,0,0]';  // base is not moving
v00 = [0,0,0]';

//angular velocity propagation
w11 = wprop(T01,w00,thd1);
w22 = wprop(T12,w11,thd2);
w33 = wprop(T23,w22,thd3);

// linear velocity propagation
v11 = vprop(T01,w00,v00,dd1);
v22 = vprop(T12,w11,v11,dd2);
v33 = vprop(T23,w22,v22,dd3);

// printf("Velocities: w44: %6.2f  \n",w44);
printf("Velocities: w33: %6.2f \n",w33);
printf("            v33: %6.2f \n",v33);
//  transform to frame 0 by rotation
printf("Velocities: w03: %6.2f \n",R03*w33);
printf("            v03: %6.2f \n",R03*v33);
