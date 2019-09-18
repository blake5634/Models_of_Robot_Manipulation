//Traj Gen and Dynamics
//  EE543 Spring 2013

clear all;
exec('kinfunc.sce',-1);

function a= polyparams(thA, thB, dt)
    a(1) = thA;   //  note a0 = a(1) !@#*&!@&)
    a(2) = 0;
    a(3) =  3*(thB-thA)/(dt*dt);
    a(4) = -2*(thB-thA)/(dt*dt*dt);
endfunction

function [acc,VM,tacc] = lin_parab_init(dth,dt)
    AMAX = 10000;  // deg/sec^2
    tacc = 0.25 * dt;
    VM = dth/(dt-tacc);
    acc = VM / tacc;
    if(abs(acc) > AMAX) 
         printf("lin_parab_init: Acceleration is too high...\n\n"); exit
         end
endfunction
 
function th = lin_parab_run(t, t_0, thA, thB, Dt, acc, VM, tacc) 
    th = 0; 
    dt = t-t_0;
    if(dt < tacc)    // acceleration
        th = 0.5*acc*(dt)^2 + thA;
    end
    if ((dt > tacc) & (dt < Dt-tacc)) // cruise
       th = VM*(0.5*tacc+(t-t_0-tacc))+thA;
    end
    if (dt > Dt-tacc) // deceleration
        th=thB-0.5*acc*(t-Dt)^2;
    end
 endfunction



 
 

///////////////////////////////////////////  5 ////////////////  Gravity Comp.

//  combine T matrices into an array of arrays
function T = tcat(T1,T2)    
    T = cat(3,T1,T2);
endfunction



//  Compute Torque due to gravity on all joints
function t = jtorque(T,C,M,g) //  ROTARY ONLY
  // T = 3dim matric of Tmatrices
  // C = array of COMvectors (Cii)
  // M = array of masses (Mi)
  // g = gravity vector in frame 0
  // number of links = length(M)
  
  
  m = length(M)
  
  // t = array of joint torques
  mm = zeros(3,m);
  
  for i=1:1:m      // for each joint
  //    printf("\n\nLink %2d:\n",i);
      gi = g; 
      // transform g0 into current frame
      for j = 1:1:i // for each joint proximal to joint i
          gi = roft(T(:,:,j))'*gi  // rotate the gravity vector
      end
      //a place for the transformed COMs
      Cij = C;
      // compute COM of link j in frame i
      for j = i:1:m  // for each joint distal to joint i  
  //        printf("  COM link %d  %3f %3f %3f:\n",j,Cij(1,j),Cij(2,j),Cij(3,j));
          for k= j:-1:i+1       
              Cij(:,j) = roft(T(:,:,k))*Cij(:,j) // compute Cij 
          end
  //        printf("     here: ij %d %d\n",i,j)
      end
      // compute mass moment
      mm(:,i) = [0,0,0]';
      for k=i:1:m // for each joint distal to joint i (current joint)
          mm(:,i) = mm(:,i) + M(k)*Cij(:,k);   // first moment of mass
      end
      // compute z component of torque 
      m1 = mm(:,i);
      t(i) = [0,0,1]*cross(m1, gi);
 //     disp(t(i), "torque: " )
  end
endfunction





 