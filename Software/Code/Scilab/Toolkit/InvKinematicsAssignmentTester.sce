// EE543  Project 50 Testing Script
//
//     V 0.1     April 2016
//     Only 2 solutions!
//
clear all;
exec('kinfunc.sce',-1);

//
//   Instructor useage instructions:
//
//    1) Edit this file as follows:
//        Global Replace:
//        'Raven16' --->  'StudentLastNamexx'
//    2) Run this script
//

exec('Raven16_invkin.sce',-1);

epsilon = 1.0E-5
 


//IK Solutions:

// 
// q(1,:) =  [ 20.0 30.0 40.0 50.0 60.0 70.0 ];
// q(2,:) =  [ -66.61  -30.00   40.00  -64.43   20.00   75.00 ];
// q(3,:) =  [-160.00   30.00  900.00 -122.89   30.00   83.82 ];
// q(4,:) =  [ 113.39  -30.00  900.00  122.68   40.00   83.82 ];

//  A bunch of test poses

q(1,:) =  [  20.00   30.00   40.00   50.00   60.00   70.00 ];
q(2,:) =  [ -66.61  -30.00   40.00  -64.43   60.00   70.00 ];
q(3,:) =  [  20.1   30.00  900.00   50.00   60.00   70.00 ];
q(4,:) =  [ -66.61  -30.00  900.00  -64.43   60.00   70.00 ];
q(5,:) =  [-20.0  +45.00    60.00   -30.00   45.00   32.00 ];
q(6,:) =  [-50.0  -33.00    75.00   135.00  -20.00   15.00 ];
q(7,:) =  [  40.75   33.00   75.00 -103.53  -20.00   15.00 ];

sol = 4

//qtest = [20, 30, 40, 50, 60, 70];
qtest = q(sol,:);

//qtest = [0,  0 ,  0 , 0 , 0 , 0 ];
mprintf("\n\n EE 543 Fwd-Inv Kinematics Test:\n\n"); 
mprintf("    Raven16   \n\n")
T = Raven16_for_kin(qtest);

mprintf("q initial: ");
disp(qtest)
mprintf("Input Transform:\n");
disp(T);

//
//T56 = link(    90  , lw,  0,   qtest(6));
//T05 = T * inv(T56);
// 
//pos05_FK = T05(1:3,4); 
//mprintf("Cheat position P5_XYZ: %f %f %f\n", pos05_FK(1), pos05_FK(2), pos05_FK(3)); 

s = Raven16_inv_kin(T);

ns = 2

mprintf("\n\n  returned IK Solutions:\n");
for i = 1:ns
   mprintf("q(%d,:) =  [%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f ];\n", ...
     i, s(i,1), s(i,2), s(i,3), s(i,4), s(i,5), s(i,6));
end

//// test the four returned solutions

mprintf("\ntest the returned solutions\n");
TtestTol = 0.2;
for i = 1:ns   
    T2 = Raven16_for_kin(s(i,:)) 
    mprintf("testing solution %d:   ",i) 
    if(norm(T-T2)> TtestTol)
        mprintf(" FAIL (Error = %f)\n",norm(T-T2));
    else        
        mprintf(" pass\n")
    end
//    mprintf("T(s):")
//    disp(T2)
//    
//    T56 = link(    90  , lw,  0,   s(i,6));
//    T05 = T2 * inv(T56);
//    pos05_FK = T05(1:3,4); 
//    mprintf("Cheat position P5_XYZ: %f %f %f\n", pos05_FK(1), pos05_FK(2), pos05_FK(3)); 
//    mprintf("Orientation sample:    %f\n",T05(2,2));

end


