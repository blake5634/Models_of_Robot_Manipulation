// EE543 HW 6.1  sp2013

clear all;

a1 = polyparams(-10,  0, 0.5);
a2 = polyparams(-10, 10, 0.5);
a3 = polyparams(-10, 20, 0.5);
a4 = polyparams(-10, 40, 0.25);
a5 = polyparams(-10, 40, 1.0);

t=0:0.02:0.50;
t2=0:0.02:0.25;
t3=0:0.02:1.00;

y1 = a1(1) + a1(2)*t + a1(3)*t.*t + a1(4)*t.*t.*t;
y2 = a2(1) + a2(2)*t + a2(3)*t.*t + a2(4)*t.*t.*t;
y3 = a3(1) + a3(2)*t + a3(3)*t.*t + a3(4)*t.*t.*t;
y4 = a4(1) + a4(2)*t2 + a4(3)*t2.*t2 + a4(4)*t2.*t2.*t2;
y5 = a5(1) + a5(2)*t3 + a5(3)*t3.*t3 + a5(4)*t3.*t3.*t3;

scf(1)
plot(t,y1,t,y2,t,y3,t2,y4,t3,y5)
title("Position: Theta(t)")

yd1 = a1(2) + 2*a1(3)*t + a1(4)*3*t.*t;
yd2 = a2(2) + 2*a2(3)*t + a2(4)*3*t.*t;
yd3 = a3(2) + 2*a3(3)*t + a3(4)*3*t.*t;
yd4 = a4(2) + 2*a4(3)*t2 + a4(4)*3*t2.*t2;
yd5 = a5(2) + 2*a5(3)*t3 + a5(4)*3*t3.*t3;

scf(2)
plot(t,yd1,t,yd2,t,yd3,t2,yd4,t3,yd5)
title("Velocity: Theta-dot(t)")


ydd1 = 2*a1(3) + 6*a1(4)*t;
ydd2 = 2*a2(3) + 6*a2(4)*t;
ydd3 = 2*a3(3) + 6*a3(4)*t;
ydd4 = 2*a4(3) + 6*a4(4)*t2;
ydd5 = 2*a5(3) + 6*a5(4)*t3;

scf(3)
plot(t,ydd1,t,ydd2,t,ydd3,t2,ydd4,t3,ydd5);
title("Acceleration: Theta-double-dot(t)")

disp(max(ydd4), "Max acceleration: ")