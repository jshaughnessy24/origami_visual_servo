#!/usr/bin/env python3
# coding: utf-8

# Symbolic python script to generate Jacobian function as binary file
# Binary file name: single_module_Jacobian

# A single module is modeled as 5 joints R-P-R-P-R and the kinematic model is computed below
# We add a 6th virtual link at the end to compensate for the difference between
# the ee and marker center

#* environment

# Copied from merlab_ws sym_single_module_jacobian in vs_control
from sympy import sin, cos, tan, simplify, atan, diff, asin
import sympy as sp
import numpy as np
from sympy import *
#* D-H
q1, q2, q3, q4, q5, theta, alpha, a, d = sp.symbols("q1 q2 q3 q4 q5 theta alpha a d")

rot = sp.Matrix([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha)],
                 [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha)],
                 [0, sin(alpha), cos(alpha)]])

trans = sp.Matrix([a*cos(theta),a*sin(theta),d])

last_row = sp.Matrix([[0, 0, 0, 1]])

m = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)

#** the table
# link | a, alpha, d, theta
# 1    | 0, 0,     0, q1
# 2    | 0, 0,     q2, 0
# 3    | 0, -pi/2, 0, q3
# 4    | 0, pi/2,  q4, 0
# 5    | 0, 0, 0,  0, q5
m01 =  m.subs({a:0, alpha:0, d:0, theta:q1})
m12 =  m.subs({a:0, alpha:-sp.pi/2, d:q2, theta:0})
m23 =  m.subs({a:0, alpha:sp.pi/2, d:0, theta:q3})
m34 =  m.subs({a:0, alpha:0, d:q4, theta:0})
m45 =  m.subs({a:0, alpha:0, d:0, theta:q5})

# # CAN CHANGE TRANSLATION VALUES - ASK ABT UNITS
# x = 0       
# z = 25       #was 25
# y = -25      #was -25
# marker_translation = sp.Matrix([[1, 0, 0, x],
#                                 [0, 1, 0, y],
#                                 [0, 0, 1, z]])

A = m01*m12*m23*m34*m45

#** test
phi = 0
s = 1
kappa = sp.pi/2
theta = kappa * s
if theta == 0:
    lt = s/2
else:
    lt = (s/theta)* tan(theta/2)

A.subs({q1:phi, q2:lt, q3:theta, q4:lt, q5:-phi})
Jv1 = diff(A[0:3, 3], q1)

Jv2 = diff(A[0:3, 3], q2)

Jv3 = diff(A[0:3, 3], q3)

Jv4 = diff(A[0:3, 3], q4)

Jv5 = diff(A[0:3, 3], q5)

#** Jw
MatZ = sp.eye(3)
Jw1 = MatZ[0:3, 2]
MatZ = m01
Jw2 = sp.Matrix([0, 0, 0]) # prismatic joint
MatZ = m01*m12
Jw3 = MatZ[0:3, 2]
MatZ = m01*m12*m23
Jw4 = sp.Matrix([0, 0, 0]) # prismatic joint
MatZ = m01*m12*m23*m34
Jw5 = MatZ[0:3, 2]

#** eqn26
J_DH = sp.Matrix([(Jv1, Jv2, Jv3, Jv4, Jv5), (Jw1, Jw2, Jw3, Jw4, Jw5)])

s, kappa, phi = sp.symbols('s, kappa, phi')
theta = kappa * s
element1 = phi
element2 = (s/theta)* tan(theta/2)
element3 = theta
element4 = element2
element5 = -element1


A_skp =  A.subs({q1:element1, q2:element2, q3:element3, q4:element4, q5:element5})



#** J_f1
f1 = sp.Matrix([element1, element2, element3, element4, element5])
J_f1_1 = diff(f1, phi)
J_f1_2 = diff(f1, kappa)
J_f1_3 = diff(f1, s)
J_f1 = np.hstack((J_f1_1, J_f1_2, J_f1_3))

J_DH_skp =  J_DH.subs({q1:element1, q2:element2, q3:element3, q4:element4, q5:element5})

J_DH_f1 = J_DH_skp @ J_f1
#** eval
from numpy import shape
shape(J_DH_f1)

l1, l2, l3, d =  sp.symbols('l1, l2, l3, d')

arclength =  (l1 + l2 + l3)/3
curvature =  2 * sp.sqrt(l1**2+l2**2+l3**2-l1*l2-l2*l3-l3*l1) / (d*(l1+l2+l3))
bend_dire =  sp.atan2((l3+l2-2*l1) , (sp.sqrt(3)*(l2-l3))) # Modified from atan - atan2

f2 =  sp.Matrix([bend_dire, curvature, arclength])
J_f2_1 = diff(f2, l1)
J_f2_2 = diff(f2, l2)
J_f2_3 = diff(f2, l3)

J_f2 =  np.hstack((J_f2_1, J_f2_2, J_f2_3))
#simplify(J_f2)
shape(J_DH_f1)
J_DH_f2 =  J_DH_f1.subs({phi:bend_dire, kappa:curvature, s:arclength})
#simplify(J_DH_f2)

shape(J_DH_f2)

J_continuum = J_DH_f2 @ J_f2
simplify(J_continuum)

###### Added by Krishna - Code to generate Jacobian Matrix elements in C code #########

# with open('Jacobian_equation.txt', 'w') as f:
#     for i in range(18):
#         f.write('\n')
#         f.write(ccode(J_continuum[i]))
#         f.write('\n')


J_continuum.evalf(subs={l1:50, l2:30, l3:50, d:40})
lambda_J_continuum = sp.lambdify((l1, l2, l3, d), J_continuum)

# #** test
# lambda_J_continuum(50, 30, 50, 40)
import dill
dill.settings['recurse'] =  True
dill.dump(lambda_J_continuum, open("single_module_Jacobian", "wb"))


