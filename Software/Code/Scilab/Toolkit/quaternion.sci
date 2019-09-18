// Quaternion functions
//
//   BH  July 3, 2012
//
clear all;
funcprot(0);


//function Q = quaternion(theta, K)
function Q = quaternion( K, theta)
    K = normalize(K);
    Q(1) = cosd(theta/2);
    Q(2) = K(1)*sind(theta/2);
    Q(3) = K(2)*sind(theta/2);
    Q(4) = K(3)*sind(theta/2);
endfunction

// find the quaternion complement
function r = qcomp(q)
    r(1) = q(1);
    r(2) = -q(2);
    r(3) = -q(3);
    r(4) = -q(4);
endfunction

function xn = normalize(x)
    xn = x/ norm(x);
endfunction


function r = mag(q)
    r = norm(q);
endfunction

function r = qtimes(a,b)
    a1 = a(1);  a2 = b(1);
    b1 = a(2);  b2 = b(2);
    c1 = a(3);  c2 = b(3);
    d1 = a(4);  d2 = b(4);
    
    r(1) = a1*a2-b1*b2-c1*c2-d1*d2;
    r(2) = a1*b2+b1*a2+c1*d2-d1*c2; // *i
    r(3) = a1*c2-b1*d2+c1*a2+d1*b2; // *j
    r(4) = a1*d2+b1*c2-c1*b2+d1*a2; // *k
endfunction

////////////////////////////////////////////////////////////////
//
//   Rotation matrix from quaternion (method 1)
//     (source, Wikipedia)
function R = quat2rot1(q)
    th = 2*acosd(q(1)); ct = cosd(th); st = sind(th); st2 = sind(th/2); kx = q(2)/st2; ky = q(3)/st2; kz = q(4)/st2;     
    
    R = [ ct+kx^2*(1-ct),       kx*ky*(1-ct)-kz*st,    kx*kz*(1-ct)+ky*st   ;
          ky*kx*(1-ct)+kz*st,   ct+ky^2*(1-ct),        ky*kz*(1-ct)-kx*st   ;
          kz*kx*(1-ct)-ky*st,   kz*ky*(1-ct)+kx*st,     ct+kz^2*(1-ct)      ]
endfunction


////////////////////////////////////////////////////////////////
//
//   Rotation matrix from quaternion (method 2)
//      (source, Craig equation 2.91 page 50, 3rd Ed.)
function R = quat2rot2(q)
    // strange permutation to allow for Craig's use of LAST param = cos(th/2)
    q4 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);     
    R = [   1-2*(q2^2+q3^2),  2*(q1*q2-q3*q4),  2*(q1*q3+q2*q4) ;
            2*(q1*q2+q3*q4),  1-2*(q1^2+q3^2),  2*(q2*q3-q1*q4) ;
            2*(q1*q3-q2*q4),  2*(q2*q3+q1*q4),  1-2*(q1^2+q2^2) ]
endfunction


