#!/usr/bin/python
#
import sympy as sp
import numpy as np
from pykinsym import *

((x,th)) = sp.symbols(('x','th'))

((th_1,th_2,th_3)) = sp.symbols(('th_1','th_2','th_3'))

R1 = RotX4_S(th_2)
R2 = RotY4_S(th_1)
R3 = RotX4_S(th_3)

R = R1*R2*R3

sp.pprint(notation_squeeze(R1))
sp.pprint(notation_squeeze(R2))
sp.pprint(notation_squeeze(R3))
sp.pprint(notation_squeeze(R))
 
