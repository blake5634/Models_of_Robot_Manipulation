// Trajectory Generation Test cases
//   Scilab programming assignment 5

clear all;

exec("tg_gravcomp.sce",-1);

/////////////////////////////
//  3rd order polynomial traj:   
//      ThA = 20,  ThB = 60
//      Dt = 0.45 sec

a= polyparams(20, 60, 0.45);

t = 0:0.02:0.45;

y_poly = a(1) + a(2)*t + a(3)*t.*t + a(4)*t.*t.*t;


/////////////////////////////
//  linear with parabolic blends 
//      ThA = 20,  ThB = 60
//      Dt = 0.45 sec

//      start the movement at t=0.00

ThA = 20;
ThB = 60;

[acc,VM,tacc] = lin_parab_init(ThB-ThA, 0.45);
i=0;
for tt=0:0.02:0.45
    i = i+1;
    y_linparab(i) = lin_parab_run(tt, 0.0, ThA, ThB, 0.45, acc, VM, tacc);
end

scf(1)
plot(t,y_poly, t,y_linparab);
title("Two Trajectories: 3rd order polynomial and linear/parabolic");

 


