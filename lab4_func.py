#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *
import math


"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
#Link angles for the UR3 robotic arm
L1 = 0.152
L2 = 0.120
L3 = 0.244
L4 = 0.093
L5 = 0.213
L6 = 0.083
L7 = 0.083
L8 = 0.082
L9 = 0.0535
L10 = 0.059


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


    w1 = np.array([0.0, 0.0, 1.0])  # base yaw (z)
    w2 = np.array([0.0, 1.0, 0.0])  # shoulder pitch (y)
    w3 = np.array([0.0, 1.0, 0.0])  # elbow pitch (y)
    w4 = np.array([0.0, 1.0, 0.0])  # wrist pitch (y)
    w5 = np.array([1.0, 0.0, 0.0])  # wrist roll (x)
    w6 = np.array([0.0, 1.0, 0.0])  # wrist yaw/pitch (y)


    v1 = -np.cross(w1, q1)
    v2 = -np.cross(w2, q2)
    v3 = -np.cross(w3, q3)
    v4 = -np.cross(w4, q4)
    v5 = -np.cross(w5, q5)
    v6 = -np.cross(w6, q6)


    M = np.array([
        [1, 0, 0, 0.390],
        [0, 1, 0, 0.401],
        [0, 0, 1, 0.2155],
        [0, 0, 0, 1]
    ])


    S[:,0] = np.hstack((w1, v1))
    S[:,1] = np.hstack((w2, v2))
    S[:,2] = np.hstack((w3, v3))
    S[:,3] = np.hstack((w4, v4))
    S[:,4] = np.hstack((w5, v5))
    S[:,5] = np.hstack((w6, v6))

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
    for i in range(6):
        omega = S[0:3, i]
        v = S[3:6, i]

        skew = np.array([
            [0, -omega[2], omega[1]],
            [omega[2], 0, -omega[0]],
            [-omega[1], omega[0], 0]
        ])

        S_matrix = np.zeros((4, 4))
        S_matrix[0:3, 0:3] = skew
        S_matrix[0:3, 3] = v
       
        exponentials = expm(S_matrix * theta[i])
        T = T @ exponentials

    T = T @ M
    print(str(T) + "\n")

    # ==============================================================#

    return_value = [None]*6
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
    yaw_radians = np.radians(yaw_WgripDegree)    #conver the degrees into radians for cosine, sine, and more
    xgrip = xWgrip + 0.15      #base offset
    ygrip = yWgrip - 0.15      #more offsets
    zgrip = zWgrip - 0.01  


    x_cen = xgrip-L9*np.cos(yaw_radians)                #0.0535m from link 9
    y_cen = ygrip-L9*np.sin(yaw_radians)                
    z_cen = zgrip                                       #same height as stated in the constraint


    #Solving for Theta1
    theta1 = np.arctan2(y_cen, x_cen) - (np.arcsin((L2-L4+L6) / np.sqrt(x_cen**2+y_cen**2)))    #derive the angle from base origin to wrist center plus angle correction


    #Solving for Theta6
    temp_angle = PI - (PI/2 - theta1)
    theta6 = temp_angle - yaw_radians                   #deriving theta6 using yaw and theta1



    #Solving for x_3end, y_3end, and z_3end
    z_3end = z_cen + L10 + L8                           #59mm from gripper to aluminum plate (L10) and 82mm from Link 8
   
    x_3end = x_cen - (L7*np.cos(theta1)) + ((L6+0.027)*np.sin(theta1))    #moving backwards and perpendicular along the gripper x orientation
    y_3end = y_cen - (L6+0.027)*np.cos(theta1) - (L7*np.sin(theta1))


    #Solving for theta2, theta3, and theta4
    hor_dist = ((x_3end**2) + (y_3end**2))              #horizontal distance squared from base to 3end
    vert_dist = ((z_3end-L1)**2)                        #vertical distance squared from joint 2 to 3end
    csquared = hor_dist + vert_dist                     #distanced squared from joint 2 to 3end

    c = np.sqrt(csquared)                               #line from Joint 2 to 3end


    hor_angle = -np.arccos(((L5**2) - csquared - (L3**2)) / (-2*L3*c))     #use Law of Cosines to find the angle at the shoulder in the triangle formed by L3, L5, and c
    vert_angle = np.arcsin((z_3end-L1) / c)                                #use arcsin to find the angle of elevation from the shoulder point to the virtual point
    theta2 = hor_angle - vert_angle                    #derive the actual shoulder joint angle

    #Law of Cosine
    theta3 = PI - np.arccos(((L3**2) + (L5**2) - csquared) / (2*L3*L5))    #use Law of Cosines to find the internal angle at the elbow between L3 and L5


    # x_shoulder = -0.120*np.sin(theta1) + 0.0*np.cos(theta1)  
    # y_shoulder =  0.120*np.cos(theta1) + 0.0*np.sin(theta1)
    # z_shoulder = L1                                          


    # x_length =  (x_3end - x_shoulder)*np.cos(theta1) + (y_3end - y_shoulder)*np.sin(theta1)
    # z_length =  z_3end - z_shoulder


    # #offset values
    # a1 = L2 + L3        
    # a2 = L5 + L4    
    # hyp  = np.sqrt(x_length**2 + z_length**2)  


    #plug in to Law of cosines for elbow angle (theta3)
    #cos_theta3 = (hyp**2 - a1**2 - a2**2)/(2*a1*a2)
    #clamp to [-1,1] to avoid numerical errors
    #theta3 = np.clip(theta3, -1.0, 1.0)

    #Law of cosines for shoulder angle (theta2)

    # Wrist pitch (theta4) so the end effector stays level
    theta4 = -(theta2 + theta3)


    # theta1 = 0.0
    # theta2 = 0.0
    # theta3 = 0.0
    # theta4 = 0.0
    theta5 = -PI/2
    # theta6 = 0.0
    print("theta computed: ", theta1, theta2, theta3, theta4, theta5, theta6)
    # ==============================================================#
    return lab_fk(float(theta1), float(theta2), float(theta3), float(theta4), float(theta5), float(theta6))






