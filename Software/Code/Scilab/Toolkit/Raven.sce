// EE543  Project 50 Testing Script
//
//     V 0.1     April 2016
//     Only 2 solutions!
//
clear all;
exec('kinfunc.sce',-1);

epsilon = 1.0E-5

// Raven Constants
la12 = 75    // deg    alpha_12
la23 = 52    // deg    alpha 23
d4   = -470.0  // mm 
lw   =  12.0  // mm
a3   =  0     //  required for IK 


function T06 = RavenFK16(q)
  
    //   LEFT(GOLD) side DH params  (Table 2)
    T01 = link(   0    ,  0,  0,   q(1));
    T12 = link( la12   ,  0,  0,   q(2));
    T23 = link(180-la23,  0, q(3),  90);
    T34 = link(     0  , a3, d4,   q(4));
    T45 = link(    90  ,  0,  0,   q(5));
    T56 = link(    90  , lw,  0,   q(6));
    
    T06 = T01*T12*T23*T34*T45*T56

endfunction

//   The DH parameters  Example: Raven II Gold

function DH = RavenDH16(q)  // q is vector of joint positions
    DH = [ 0            0   0       q(1) ;
           la12         0   0       q(2);
           180-la23     0   q(3)    90;
           0            a3  d4      q(4);
           90           0   0       q(5);
           90           lw  0       q(6);
           ]
endfunction


/////////////////////////////////////////////////
//
//     Raven II Inverse Kinematics - All solutions
//
function q_return = RavenIK(T06)
    //  T = T06 desired
   
    r = sqrt(T06(1,4)^2 + T06(2,4)^2 + T06(3,4)^2);  // get distance from origin
    if(r < lw)
        mprintf("\n\nIK: requested point is too close to RCM\n    HALTING\n");
        abort
    end     
    
    
    ////////////////////////////////    P 5
    //
    ///////   Find origin of Frame 5 (P5 / P_RCM)
    //   (see eqn 32,33) 
    T60 = tinv(T06); // get RCM in frame 6 (T60 col 4)
    P_6RCM = [T60(1,4) T60(2,4) T60(3,4)]'; // P of RCM in frame 6
    
    P_rcm = [T60(1,4) T60(2,4) 0]' ;// project Prcm into X6 Y6 plane
    
    // Report Step 1
    P5f6a =  -lw * P_rcm / norm(P_rcm)   // eqn 34
    P5f6b =   lw * P_rcm / norm(P_rcm)   // eqn 34    
    
    P5f6a = aug4v(P5f6a)  // get augmented 4-vec
    P5f0a = T06 * P5f6a;  // trans to frame 0

    P5f6b = aug4v(P5f6b)  // get augmented 4-vec
    P5f0b = T06 * P5f6b;  // transform to frame 0
//    
    Xpa = P5f0a(1);
    Ypa = P5f0a(2);
    Zpa = P5f0a(3) ;  
//    
//    mprintf("XYZa: %f %f %f\n",Xpa,Ypa,Zpa);
//    ta = sqrt(Xpa*Xpa+Ypa*Ypa+Zpa*Zpa);
    
    Xpb = P5f0b(1);   
    Ypb = P5f0b(2);
    Zpb = P5f0b(3) ;  
    
    //  Report step 2
    da = sqrt(Xpa*Xpa + Ypa*Ypa + Zpa*Zpa);
    db = sqrt(Xpb*Xpb + Ypb*Ypb + Zpb*Zpb);
    
      // compute closest and furthest points Xpc / Xpf etc.
    if (da < db) then
        Xpc = Xpa;  Ypc = Ypa; Zpc = Zpa;
        Xpf = Xpb;  Ypf = Ypb; Zpf = Zpb;        
    else
        Xpc = Xpb;  Ypc = Ypb; Zpc = Zpb; 
        Xpf = Xpa;  Ypf = Ypa; Zpf = Zpa;
    end
    
    // select which P5 solution is right
    //  See Figure 11 of report
    
    //  compute the projection                       (Eqn 35)
    // use kinfunc.sce dot() function to avoid transpose confusion
    
    pr = dot([1 0 0 ]', P_6RCM) / norm(P_6RCM);
    
    if(pr < 0.0)
      Xp = Xpc; // use closer P5
      Yp = Ypc;
      Zp = Zpc; 
    else
      Xp = Xpf; // user further P5
      Yp = Ypf;
      Zp = Zpf; 
    end
        
    mprintf("Selected P5_XYZ:       %f %f %f\n",Xp,Yp,Zp); 
    
    //  This test only works for testing where you can know the true P05 from 
    //            forward kinematics
    if(norm(pos05_FK-[Xp Yp Zp]') > epsilon)
        mprintf("\n\nWrong P5: exiting\n\n");  // should never happen!
        abort
    end
    
    ////////////////////////////    d3
    // two solns for d3 (q3)
    
    tb = sqrt(Xp*Xp + Yp*Yp + Zp*Zp); 
    mprintf("d4, Distance: %7.2f %7.2f\n", d4,tb) 
 
    q3a =    -d4 - sqrt(Xp*Xp + Yp*Yp + Zp*Zp);;  // eqn(37) two th3's arise from the sqare root!!
    q3b =   (-d4 + sqrt(Xp*Xp + Yp*Yp + Zp*Zp)); 
    
    mprintf("q3/d3 = [%f  %f]\n",q3a, q3b)
//  
//   
    ////////////////////////////    Th2
    //
    //  Find the correct solution for theta 2 (q2)
    //    (notes suggest four solns but there are only 2)
    
    g1 = sind(la12);
    g2 = cosd(la12);
    g3 = sind(la23);
    g4 = cosd(la23);
    
//    mprintf("gs: %5.4f %5.4f %5.4f %5.4f\n", g1,g2,g3,g4);
    
  
    // depending on d3/q3  // sec 4.5 eqn 35a
    da = q3a+d4;
    db = (q3b+d4);
//    //    

    mprintf("da db:>>> %f %f\n", da,db); 

//    disp(-Zpa/db);
//    disp((g2*g4));
//    disp((-Zpa/db - g2*g4)/(g1*g3));
//    
     a = abs((-Zp/db - g2*g4)/(g1*g3));
     b = abs((-Zp/da - g2*g4)/(g1*g3));
     
     if (( a < 1.0) & (b < 1.0))
            mprintf(" Possible 8 solutions???\n");
            mprintf(" Are da and db different??\n");
     end
    
  //  q2 solutions a and b
    if (abs((-Zp/da - g2*g4)/(g1*g3)) < 1.0)
      q2a =  acosd((-Zp/da - g2*g4)/(g1*g3)); 
      d1 = da
      q3 = q3a
      mprintf(" Valid d_3: %f\n", q3a);
//       q2a1 =  acosd((-Zp/db - g2*g4)/(g1*g3));
    elseif (abs((-Zp/db - g2*g4)/(g1*g3)) < 1.0)
//       q2b1 =  acosd((-Zp/da - g2*g4)/(g1*g3)); 
      q2a = acosd((-Zp/db - g2*g4)/(g1*g3)); 
      d1 = db
      q3 = q3b
      mprintf(" Valid d_3: %f\n", q3b);
    else
      mprintf("somethings wrong, no valid theta 2 solutions found.\n");
      abort
    end

    // account for multiplicity of arc-cosine()
    q2b = -q2a; 
    
//    mprintf("th2 = [%f %f]\n", q2a, q2b);
    
    /////////////////////////////////   Th 1
    // Theta 1 (also 2 solns)
    
    // th2a  d3.1
    b1L = sind(q2a)*g3 ;
    b2L = cosd(q2a)*g2*g3-g1*g4 ;
    B = [ b1L  b2L ; -b2L b1L];
    cs1 = 1/d1* inv(B)*[Xp, Yp]'; // eqn (50)
    q1a = 360/(2*%pi)*atan(cs1(2),cs1(1));
    
    //  th2a2  d3.1
    b1L = sind(q2b)*g3 ;
    b2L = cosd(q2b)*g2*g3-g1*g4 ;
    B = [ b1L  b2L ; -b2L b1L];
    cs1 = 1/d1* inv(B)*[Xp, Yp]'; // eqn (50)
    q1b = 360/(2*%pi)*atan(cs1(2),cs1(1));    
//     
    
    mprintf("th1 = [%f %f]  \n", q1a , q1b) 
    
    
    ///////////////////////////////////   Th 4,5,6
    
    function res = eval_theta456(q, T_06)
      epsil = sind(1.0);
      
      // eqns 56-58
      T03 = link(   0    ,  0,  0,   q(1)) * ...
            link( la12   ,  0,  0,   q(2)) * ...
            link(180-la23,  0, q(3),  90) ;

     ///   cheat for testing only:
      T36d_test = link(     0  , a3, d4,   qtest(4)) * ...
                  link(    90  ,  0,  0,   qtest(5)) * ...
                  link(    90  , lw,  0,   qtest(6));
                  
      // legit computation: 
      T36d = inv(T03)*T_06;
//      
//      mprintf("eval th456:");
//      disp(T36d);
//       mprintf("Errors: (only small for initial q value) ");
//       disp(norm(T36d-T36d_test));
    
      ////////////////////// Th 4
      c5 = -1*T36d(3,3);    // eqn (59)
      
//       mprintf("c5: %f\n",c5);
      if(abs(c5) > epsil) 
         c4x = T36d(1,4)/(c5*lw);
         s4x = T36d(2,4)/(c5*lw);
      else
         c4x = T36d(1,3); 
         s4x = T36d(2,3);
      end
       
      q4 = 180.0/(%pi) * atan(s4x,c4x);
      
     
      ////////////////////// Th 5
       
      s4 = sind(q4);
      if(abs(s4) > epsil)
        s5x = T36d(2,3)/sind(q4);
      else
        s5x = T36d(1,3)/cosd(q4);
      end
      
      q5 = 360.0/(2*%pi) * atan(s5x,c5);
     
      //////////////////////  Th 6
      s5 = sind(q5);
      if(abs(s5) > epsil)
         s6x = -T36d(3,2)/s5;
         c6x =  T36d(3,1)/s5;
      else
         abort // we'll deal with this later see report eqn(71)
      end
      
      q6 = 360.0/(2*%pi)*atan(s6x,c6x);
    
      res = [q4 q5 q6];
      
      endfunction // end of eval_theta456
 
    
    r = eval_theta456([q1a  q2a  q3], T06);
 
    q4a = r(1);
    q5a = r(2);
    q6a = r(3);
    
    r = eval_theta456([q1b  q2b  q3], T06);
 
    q4b = r(1);
    q5b = r(2);
    q6b = r(3); 
     
    
    // return solution list
    
    q_return = [q1a q2a q3 q4a q5a q6a;  ...
                q1b q2b q3 q4b q5b q6b  ];   // 
    
endfunction  // end of   RavenIK


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
mprintf("\n\n Raven II Fwd-Inv Kinematics Test:\n\n"); 
//T = RavenFK(q(sol,:))
T = RavenFK(qtest);


mprintf("q initial: ");
disp(qtest)
mprintf("Input Transform:\n");
disp(T);


T56 = link(    90  , lw,  0,   qtest(6));
T05 = T * inv(T56);
 
pos05_FK = T05(1:3,4); 
mprintf("Cheat position P5_XYZ: %f %f %f\n", pos05_FK(1), pos05_FK(2), pos05_FK(3)); 

s = RavenIK(T);

ns = 2

mprintf("IK Solutions:\n");
for i = 1:ns
   mprintf("q(%d,:) =  [%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f ];\n", ...
     i, s(i,1), s(i,2), s(i,3), s(i,4), s(i,5), s(i,6));
end

//// test the four returned solutions

mprintf("\ntest the ns returned solutions\n");
TtestTol = 0.2;
for i = 1:ns   
    T2 = RavenFK(s(i,:)) 
    mprintf("testing solution %d:   ",i) 
    if(norm(T-T2)> TtestTol)
        mprintf(" FAIL (Error = %f)\n",norm(T-T2));
    else        
        mprintf(" pass\n")
    end
    mprintf("T(s):")
    disp(T2)
    
    T56 = link(    90  , lw,  0,   s(i,6));
    T05 = T2 * inv(T56);
    pos05_FK = T05(1:3,4); 
    mprintf("Cheat position P5_XYZ: %f %f %f\n", pos05_FK(1), pos05_FK(2), pos05_FK(3)); 
    mprintf("Orientation sample:    %f\n",T05(2,2));

end


