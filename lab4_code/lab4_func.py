#!/usr/bin/env python
import numpy as np
import math
from math import radians
from scipy.linalg import expm
#from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
PI = 180
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))
	
	# find M and S by measuring the UR3 robot arm
	M = np.array([[0, -1 , 0 ,0.390 ],
				  [0, 0 , -1 ,0.401 ],
				  [1, 0 , 0 ,0.2155],
				  [0, 0 , 0 ,1    ]])

	S = np.array([[0    ,0      , 0    ,  0    ,  1    ,  0     ],
				  [0    ,1      , 1    ,  1    ,  0    ,  1     ],
				  [1    ,0      , 0    ,  0    ,  0    ,  0     ],
				  [0.15 ,-0.162 ,-0.162,-0.162 ,  0    ,  -0.162],
				  [0.15 ,0      , 0    ,  0    , 0.162 ,  0     ],
				  [0    ,-0.15  , 0.094, 0.307 , -0.26 ,  0.39  ]])
	
	# ==============================================================#
	return M, S

def S_screw(S,i):


	screw_i = np.array([[0,-S[2][i],S[1][i],S[3][i]],[S[2][i],0,-S[0][i],S[4][i]],[-S[1][i],S[0][i],0,S[5][i]],[0,0,0,0]])

	return screw_i

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	s1_mat = S_screw(S,0)
	s2_mat = S_screw(S,1)
	s3_mat = S_screw(S,2)
	s4_mat = S_screw(S,3)
	s5_mat = S_screw(S,4)
	s6_mat = S_screw(S,5)

	T = np.dot(expm(s1_mat*theta1*math.pi/180),np.dot(expm(s2_mat*theta2*math.pi/180),np.dot(expm(s3_mat*theta3*math.pi/180),np.dot(expm(s4_mat*theta4*math.pi/180),np.dot(expm(s5_mat*theta5*math.pi/180),np.dot(expm(s6_mat*theta6*math.pi/180),M))))))


	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
M, S = Get_MS()
print(M,S)
lab_fk(10,-25,35,-45,-90,10)