// EE543 handy functions
//
funcprot(0);

// basic vector cross product
function V = cross(a,b)
   t = [0,    -a(3),  a(2) ;
        a(3),  0   , -a(1) ;
        -a(2), a(1),   0 ]
   V = t*b;
endfunction

// 3x3 rotation matrices
function M=rotx(th)  // theta is in degrees
    th=th/(180/%pi); // convert to radians
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

// equivalent angle-axis

function R = equiv(K, th)
    th = th/(180/%pi);
    K  = K/norm(K);
    c = cos(th); s = sin(th); v = (1-c);
    Kx = K(1);  Ky=K(2);  Kz=K(3);
    R = [Kx*Kx*v+c      Kx*Ky*v-Kz*s    Kx*Kz*v+Ky*s;
         Kx*Ky*v+Kz*s   Ky*Ky*v+c       Ky*Kz*v-Kx*s;
         Kx*Kz*v-Ky*s   Ky*Kz*v+Kx*s    Kz*Kz*v+c    ]
endfunction


// 4x4 homogeneous transforms
function M=rotx4(th)
    th=th/(180/%pi);
    M=[1 0 0 0; 0 cos(th) -sin(th) 0; 0 sin(th) cos(th) 0; 0 0 0 1]
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

function M=trans4(v, r)   // pure translation by magnitude (r) and direction |v|
    v = r * v/norm(v);
    M = [ 1 0 0 v(1) ;
          0 1 0 v(2) ;
          0 0 1 v(3) ;
          0 0 0 1 ]
endfunction
