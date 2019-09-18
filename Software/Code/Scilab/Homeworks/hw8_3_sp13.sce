// hw 8  EE543 W16
 

exec('tg_gravcomp.sce',-1);   // load all functions needed

//   8.1  
t1 =  0:0.02:0.250;   // plot all graphs 0.0 -- 0.75
t2 =  0:0.02:0.50;   // plot all graphs 0.0 -- 0.75

t=t1
a= polyparams(0, 5, 0.25);
y1  = a(1) + a(2)*t + a(3)*t.*t + a(4)*t.*t.*t;
yd1 =        a(2)   + 2*a(3)*t  + 3*a(4)*t.*t
ydd1 =                2*a(3)    + 6*a(4)*t

t=t2
a= polyparams(0,10, 0.5);
y2 = a(1) + a(2)*t + a(3)*t.*t + a(4)*t.*t.*t;
yd2 =        a(2)   + 2*a(3)*t  + 3*a(4)*t.*t
ydd2 =                2*a(3)    + 6*a(4)*t

t=t2
a= polyparams(0,20, 0.5);
y3 = a(1) + a(2)*t + a(3)*t.*t + a(4)*t.*t.*t;
yd3 =        a(2)   + 2*a(3)*t  + 3*a(4)*t.*t
ydd3 =                2*a(3)    + 6*a(4)*t

t=t2
a= polyparams(0,40, 0.5);
y4 = a(1) + a(2)*t + a(3)*t.*t + a(4)*t.*t.*t;
yd4 =        a(2)   + 2*a(3)*t  + 3*a(4)*t.*t
ydd4 =                2*a(3)    + 6*a(4)*t

t=t1
a= polyparams(0,40, 0.25);
y5 = a(1) + a(2)*t + a(3)*t.*t + a(4)*t.*t.*t;
yd5 =        a(2)   + 2*a(3)*t  + 3*a(4)*t.*t
ydd5 =                2*a(3)    + 6*a(4)*t


//
scf(1)
plot(t1,y1, t2, y2, t2,y3, t2, y4, t1, y5);
title("Position: 3rd order polynomial");
//
scf(2)
plot(t1, yd1, t2, yd2, t2, yd3, t2, yd4, t1, yd5);
title("Velocity: 3rd order polynomial");
//
scf(3)
plot(t1,ydd1, t2, ydd2, t2, ydd3, t2, ydd4, t1,ydd5);
title("Acceleration: 3rd order polynomial");

abort



// Manipulator Model

// DH parameters
al0 =   0;  a0 =      0;   d1 = 0 ;
al1 = -90;  a1 =      0;   d2 = .2435;
al2 =   0;  a2 = 0.4318;   d3 = -.0934; 
al3 =  90;  a3 = -0.0203;  d4 = 0.4331;
al4 = -90;  a4 = 0     ;   d5 = 0;
al5 =  90;  a5 = 0     ;   d6 = 0;

// Link masses
M(1) =  0.0;
M(2) = 17.4;  // kg
M(3) =  4.8;
M(4) =  0.82;
M(5) =  0.534;
M(6) =  0.09;  

// Link Centers of Mass
Cii = zeros(3,6);

// the non-zero entries.
Cii(:,2) = [0.068, 0.006, -0.016]';
Cii(:,3) = [0.0, -0.070, 0.014]';
Cii(:,4) = [0.0,0.0, -0.019]';
Cii(:,6) = [0.0,0.0, 0.032]';

g = [0,0,-9.8]';  // acceleration of gravity on earth

// Pose P1:
th1 = 25; th2 = 0; th3 = 45 ; th4 = 20; th5 = 0; th6 = 0;

// T matrices

T01 = link(al0, a0, d1, th1);
T12 = link(al1, a1, d2, th2);
T23 = link(al2, a2, d3, th3);
T34 = link(al3, a3, d4, th4);
T45 = link(al4, a4, d5, th5);
T56 = link(al5, a5, d6, th6);

T = tcat(T01,T12);
T = tcat(T,  T23);
T = tcat(T,  T34);
T = tcat(T,  T45);
T = tcat(T,  T56); 

t = jtorque(T,Cii,M,g);

disp(t, "Torques for P1: ")

// Pose P2:
th1 = -87; th2 = 0; th3 = 45 ; th4 = 20; th5 = 0; th6 = 0;

// T matrices

T01 = link(al0, a0, d1, th1);
T12 = link(al1, a1, d2, th2);
T23 = link(al2, a2, d3, th3);
T34 = link(al3, a3, d4, th4);
T45 = link(al4, a4, d5, th5);
T56 = link(al5, a5, d6, th6);

T = tcat(T01,T12);
T = tcat(T,  T23);
T = tcat(T,  T34);
T = tcat(T,  T45);
T = tcat(T,  T56); 

t = jtorque(T,Cii,M,g);

disp(t, "Torques for P2: ")



// Pose P3:
th1 = 25; th2 = 90; th3 = -45 ; th4 = -75; th5 = 10; th6 = 10;

// T matrices

T01 = link(al0, a0, d1, th1);
T12 = link(al1, a1, d2, th2);
T23 = link(al2, a2, d3, th3);
T34 = link(al3, a3, d4, th4);
T45 = link(al4, a4, d5, th5);
T56 = link(al5, a5, d6, th6);

T = tcat(T01,T12);
T = tcat(T,  T23);
T = tcat(T,  T34);
T = tcat(T,  T45);
T = tcat(T,  T56); 

t = jtorque(T,Cii,M,g);

disp(t, "Torques for P3: ")


// Pose P4:
th1 = 25; th2 = 135; th3 = 0 ; th4 = 50; th5 = 0; th6 = 0;

// T matrices

T01 = link(al0, a0, d1, th1);
T12 = link(al1, a1, d2, th2);
T23 = link(al2, a2, d3, th3);
T34 = link(al3, a3, d4, th4);
T45 = link(al4, a4, d5, th5);
T56 = link(al5, a5, d6, th6);

T = tcat(T01,T12);
T = tcat(T,  T23);
T = tcat(T,  T34);
T = tcat(T,  T45);
T = tcat(T,  T56); 

t = jtorque(T,Cii,M,g);

disp(t, "Torques for P4: ")
