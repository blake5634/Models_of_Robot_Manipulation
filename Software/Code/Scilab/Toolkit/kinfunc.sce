// EE543 handy functions
//
//   Updated March 2016
//   (students write these for themselves)
//
//    All angles in DEGREES
//
funcprot(0);

// basic vector

function V = dot(a,b)
    V = a'*b;
endfunction

function V = cross(a,b)    
   t = [0,    -a(3),  a(2) ;
        a(3),  0   , -a(1) ;
        -a(2), a(1),   0 ]
   V = t*b;
endfunction

// 3x3 rotation matrices
function M=rotx(th)
    th=th/(180/%pi);
    M=[1   0        0     ;
       0 cos(th) -sin(th) ;
       0 sin(th)  cos(th)  ]
endfunction

function M=roty(th)
    th=th/(180/%pi);
    M=[cos(th) 0 sin(th) ; 0 1  0 ; -sin(th) 0 cos(th) ]
endfunction

function M=rotz(th)
    th=th/(180/%pi);
    M=[cos(th) -sin(th) 0 ; sin(th) cos(th) 0 ; 0 0 1]
endfunction


// unit vectors
xh=[1 0 0 ]';
yh=[0 1 0 ]';
zh=[0 0 1 ]';

xh4=[1 0 0 1 ]';
yh4=[0 1 0 1 ]';
zh4=[0 0 1 1 ]';

// 4x4 homogeneous transforms
function M=rotx4(th)
    th=th/(180/%pi);
    M=[1 0 0 0; 0 cos(th) -sin(th) 0; 0 sin(th) cos(th) 0; 0 0 0 1]
endfunction

function M=roty4(th)
    th=th/(180/%pi);
    M=[ cos(th) 0 sin(th) 0;
        0       1  0      0;
       -sin(th) 0 cos(th) 0;
        0       0  0      1]
endfunction

function M=rotz4(th)
    th=th/(180/%pi);
    M=[cos(th) -sin(th) 0 0; sin(th) cos(th) 0 0; 0 0 1 0; 0 0 0 1]
endfunction

//  Convert a 3x3 rotation matrix R to 4x4
function T=rot4(R)
    T=[ R(1,1)	R(1,2)	R(1,3)	0 ;
        R(2,1)	R(2,2)	R(2,3)	0 ;
        R(3,1)	R(3,2)	R(3,3)	0 ;
        0	0	0	1    ]
endfunction

//   Extract the 3x4 rotation matrix R from a 4x4 transform
function R = roft(T)
    R = T(1:3,1:3);
endfunction

//   generate a 3x3 rotation matrix from equivalent angle (th) and axis (K)
function R = equiv(K, th)
    th = th/(180/%pi);
    K  = K/norm(K);
    c = cos(th); s = sin(th); v = (1-c);
    Kx = K(1);	Ky=K(2);  Kz=K(3);
    R = [Kx*Kx*v+c	Kx*Ky*v-Kz*s	Kx*Kz*v+Ky*s;
	 Kx*Ky*v+Kz*s	Ky*Ky*v+c	Ky*Kz*v-Kx*s;
	 Kx*Kz*v-Ky*s	Ky*Kz*v+Kx*s	Kz*Kz*v+c    ]
endfunction

//  generate equiv-axis angle (th) from 3x3 R
//   (actually 2 solutions from acos())
function th = equivth(R)
  th = acosd((R(1,1)+R(2,2)+R(3,3)-1)/2);
endfunction

//  find the axis of rotation from a rotation matrix R
function K = equivK(R,th)
//    K(1) = sqrt((R(1,1)-cosd(th))/(1-cosd(th)));    
//    K(2) = sqrt((R(2,2)-cosd(th))/(1-cosd(th)));    
//    K(3) = sqrt((R(3,3)-cosd(th))/(1-cosd(th)));
    d = 1/(2*sind(th))
    K(1) = d*(R(3,2)-R(2,3));
    K(2) = d*(R(1,3)-R(3,1));
    K(3) = d*(R(2,1)-R(1,2)); 
endfunction

// translate in direction v by distance r
function M=trans4(v, r)
    v = r * v/norm(v);
    M = [ 1 0 0 v(1) ;
          0 1 0 v(2) ;
          0 0 1 v(3) ;
          0 0 0 1 ]
endfunction

// augment a 3 vector to a 4 vector
function V = aug4v(x)
    V = [x(1) x(2) x(3)  1 ]'
endfunction

// translate by vector v
function M=trans4v(v)
    M = [ 1 0 0 v(1) ;
          0 1 0 v(2) ;
          0 0 1 v(3) ;
          0 0 0 1 ]
endfunction

//  compute inverse of a 4x4 homogeneous transform
function T1 = tinv(T)
  R=roft(T);
  P1 = R'*T(1:3,4);
  T1(1:3,1:3) = R';
  T1(1:3,4) = -P1;
  T1(4,1:4) = [0,0,0,1];
endfunction


//  generate a 3x3 matrix R representing roll, pitch, yaw, angles
function R = rrpy(C,B,A)   
    A = A/(180/%pi);   B = B/(180/%pi);   C = C/(180/%pi);
    ca = cos(A); sa = sin(A);
    cb = cos(B); sb = sin(B);
    cc = cos(C); sc = sin(C);
    R = [
        ca*cb    ca*sb*sc-sa*cc    ca*sb*cc+sa*sc;
        sa*cb    sa*sb*sc+ca*cc    sa*sb*cc-ca*sc;
        -sb       cb*sc            cb*cc         ]
endfunction


/////////////////////////////////////  3  /////////////  Forward Kinematics Functions
//   (Craig's notation) 

function T = link( al, a, d, th)
   T = [ cosd(th)              -sind(th)	         0		       a	;
         sind(th)*cosd(al)	cosd(th)*cosd(al)	-sind(al)	-sind(al)*d     ;
         sind(th)*sind(al)	cosd(th)*sind(al)	cosd(al)	 cosd(al)*d	;
         0			0			0		1		]
endfunction


/////////////////////////////////////  4   ////////////////  Velcity Propogation & Jacobian 

function wn1 = wprop(T,wn,thdot)
   R = T(1:3,1:3)';  // transpose of RN-N+1
   wn1 = R*wn + [0,0,thdot]'; 
endfunction

function vn1 = vprop(T,wn,vn,ddot)  // wn = 0 for prismatic ddot=0 for rotary
   R = T(1:3,1:3)';  // transpose of RN-N+1
   P = T(1:3,4);
   vn1 = R*(vn+cross(wn,P)) + [0,0,ddot]';  
endfunction

