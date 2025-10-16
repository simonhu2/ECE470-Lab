#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	q1 = np.array([-0.150, .150, 0.010])
	q2 = q1 + np.array([0, .120, .152])
	q3 = q2 + np.array([.244,0,0])
	q4 = q3 + np.array([.213,-0.093,0])
	q5 = q4 + np.array([0,.083,0])
	q6 = q5 + np.array([.083,0,0])

	w1 = np.array([0.0, 0.0, 1.0])
	w2 = np.array([])
	w3 = np.array([])
	w4 = np.array([])
	w5 = np.array([])
	w6 = np.array([])



 



	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()




	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	yaw_radians = np.radians(yaw_WgripDegree)

	x_cen = xWgrip-0.0535*np.cos(yaw_radians)			#0.0535 from link 9
	y_cen = yWgrip-0.0535*np.sin(yaw_radians)
	z_cen = zWgrip										#same height as stated in the constraint

	#Solving for Theta1
	theta1 = np.atan2(y_cen, x_cen)


	#Solving for Theta6
	temp_angle = theta1 + np.radians(90)
	theta6 = temp_angle - yaw_radians					#deriving theta6 using yaw and theta1


	#Solving for x_3end, y_3end, and z_3end
	z_3end = zWgrip + 0.059 + 0.082						#59mm from gripper to aluminum plate and 82mm from Link 8
	


	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
