// test file

clear all;

l0 = 1;
l1 = 1;
l2 = 1;

al0 = -90;     al1 = 90;    al2 = 90;
a0  = 0;      a1 = l1;     a2 = l2;
d1  = l0;     d2 = 0.2;    d3 = 0;

// Pose: 
th1 = 0; th2 = 0; th3 = 0;

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



g=g00;

  m = length(M)
  for i=1:1:m      // for each joint
      disp (i,"loop");
      gi = g; 
      // transform g0 into current frame
      for j = 1:1:i // for each joint proximal to joint i
          gi = roft(T(:,:,j))'*gi  // rotate the gravity vector
      end
      // compute COM of link j in frame i
      for j = i:1:m  // for each joint distal to joint i
          Cij(:,j) =  C(:,j) ; // start with Cii
          for k= j:-1:i+1
              Cij(:,j) = roft(T(:,:,k))*Cij(:,j) // compute Cij 
          end
           
      end
      // compute mass moment
      mm(:,i) = [0,0,0]';
      for k=i:1:m // for each joint distal to joint i (current joint)
          pause;
          mm(:,i) = mm(:,i) + M(k)*Cij(:,k); // first moment of mass
      end
      // compute z component of torque 
      m1 = mm(:,i);
      t(i) = [0,0,1]*cross(m1, gi);
      disp(t(i), "torque: " );
  end