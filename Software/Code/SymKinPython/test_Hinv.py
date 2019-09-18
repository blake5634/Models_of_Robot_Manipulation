#!/usr/bin/python
#
import sympy as sp
import numpy as np
from pykinsym import *

((x,th)) = sp.symbols(('x','th'))

T2 = RotX4_S(th)
T1 = Trans4_S(sp.Matrix([0, x, 0])) 

print "T2: "
sp.pprint(T2)

T = T1*T2

print "T: "
sp.pprint(T)

print "T inverse: "
sp.pprint(H_inv_S(T))

print "T times its inverse:"
sp.pprint(sp.simplify(T*H_inv_S(T)))
