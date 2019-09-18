// EE543 handy functions
//
funcprot(0);

// basic vector

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

function T=rot4(R)
    T=[ R(1,1)	R(1,2)	R(1,3)	0 ;
        R(2,1)	R(2,2)	R(2,3)	0 ;
        R(3,1)	R(3,2)	R(3,3)	0 ;
        0	0	0	1    ]
endfunction

function R = roft(T)
    R = T(1:3,1:3);
endfunction

function R = equiv(K, th)
    th = th/(180/%pi);
    K  = K/norm(K);
    c = cos(th); s = sin(th); v = (1-c);
    Kx = K(1);	Ky=K(2);  Kz=K(3);
    R = [Kx*Kx*v+c	Kx*Ky*v-Kz*s	Kx*Kz*v+Ky*s;
	 Kx*Ky*v+Kz*s	Ky*Ky*v+c	Ky*Kz*v-Kx*s;
	 Kx*Kz*v-Ky*s	Ky*Kz*v+Kx*s	Kz*Kz*v+c    ]
endfunction

function th = equivth(R)
  th = acosd((R(1,1)+R(2,2)+R(3,3)-1)/2);
endfunction

function K = equivK(R,th)
    K(1) = sqrt((R(1,1)-cosd(th))/(1-cosd(th)));    
    K(2) = sqrt((R(2,2)-cosd(th))/(1-cosd(th)));    
    K(3) = sqrt((R(3,3)-cosd(th))/(1-cosd(th)));
endfunction

function M=trans4(v, r)
    v = r * v/norm(v);
    M = [ 1 0 0 v(1) ;
          0 1 0 v(2) ;
          0 0 1 v(3) ;
          0 0 0 1 ]
endfunction

function T1 = tinv(T)
  R=T(1:3, 1:3);
  P1 = R'*T(1:3,4);
  T1(1:3,1:3) = R';
  T1(1:3,4) = -P1;
  T1(4,1:4) = [0,0,0,1];
endfunction


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

///////////////////////////////////////////  3  /////////////  Forward Kinematics Functions


function T = link( al, a, d, th)
   T = [ cosd(th)                 -sind(th)		         0		       a		;
         sind(th)*cosd(al)	cosd(th)*cosd(al)		-sind(al)	-sind(al)*d ;
         sind(th)*sind(al)	cosd(th)*sind(al)		cosd(al)		cosd(al)*d	;
         0			0			0		1		]
endfunction



///////////////////////////////////////////  4   ////////////////  Velcity Propogation & Jacobian 

function wn1 = wprop(T,wn,thdot)
   R = T(1:3,1:3)';  // transpose of RN-N+1
   wn1 = R*wn + [0,0,thdot]'; 
endfunction

function vn1 = vprop(T,wn,vn,ddot)  // wn = 0 for prismatic ddot=0 for rotary
   R = T(1:3,1:3)';  // transpose of RN-N+1
   P = T(1:3,4);
   vn1 = R*(vn+cross(wn,P)) + [0,0,ddot]';  
endfunction
