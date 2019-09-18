// icp2.4
//clear all;

exec('rotation_functions_EE543.sce',-1)
exec('quaternion.sci', -1)

R1 = rotz(42);
R2 = equiv([3.2, -2.7, 4.4]',135);
R3 = rotx(1.6);

R1*R2*R3

// 2.5

q1 = quaternion(42,  [0,0,1]');   // built-in scilab function
q2 = quaternion(135,[3.2,-2.7,4.4]');
q3 = quaternion(1.6,[1,0,0]');
q4 = qtimes(q1,qtimes(q2,q3))
R = quat2rot2(q4);



T1 = trans4([1,0,0]',4.2);
T2 = rotz4(42);
T3 = rot4(equiv([3.2,-2.7,4.4]', 135));
T4 = trans4([-4, 14.7, 0.27]', -16.2)
T5 = rotx4(1.6);

disp(T5)   // display the answer
