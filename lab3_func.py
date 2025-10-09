#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm
from lab3_header import *

PI = 3.1415926535

# def _skew(w):
#     """Return 3x3 skew-symmetric matrix of 3-vector w"""
#     return np.array([[0, -w[2], w[1]],
#                      [w[2], 0, -w[0]],
#                      [-w[1], w[0], 0]])

# def _twist_to_se3(S):
#     """Convert 6-vector twist (omega, v) to 4x4 se(3) matrix"""
#     w = S[0:3]
#     v = S[3:6]
#     se3 = np.zeros((4,4))
#     se3[0:3,0:3] = _skew(w)
#     se3[0:3,3] = v
#     return se3

def Get_MS():
    """
    Return M (4x4) and S (6x6) screw-axis matrix (each column is a 6-vector).
    NOTE: The numeric values are a best-effort instantiation taken from the
    lab manual figures (distances in meters). Verify/adjust these numbers
    with your TA/robot if you need higher accuracy.
    """
    # =================== Your code starts here ====================#
    # Fill in the correct values for S1~6, as well as the M matrix
    # --- Geometry (meters) from Lab3 figures ---
    z1 = 0.152   # 152 mm
    z2 = z1 + 0.244  # +244 mm
    z3 = z2 + 0.213  # +213 mm
    z4 = z3 + 0.083  # +83 mm (wrist start)
    x_wrist = 0.093  # 93 mm lateral offset
    ee_up = 0.059 + 0.082    # 59 mm (suction cup up offset)
    ee_forward = 0.0535  # 53.5 mm (forward offset)

    # Joint centers q_i (base frame) - approximate
    q1 = np.array([0.0, 0.0, z1])
    q2 = np.array([0.0, 0.12, z1])
    q3 = np.array([0.244, 0.12, z1])
    q4 = np.array([0.244+0.213, 0.12-0.093, z1])
    q5 = np.array([0.244+0.213, 0.12-0.093+0.083, z1])  # small offset above q4
    q6 = np.array([0.244+0.213+0.083, 0.12-0.093+0.083, z1])

    # Axis directions (omega) chosen to match pictures in the manual:
    w1 = np.array([0.0, 0.0, 1.0])  # base yaw (z)
    w2 = np.array([0.0, 1.0, 0.0])  # shoulder pitch (y)
    w3 = np.array([0.0, 1.0, 0.0])  # elbow pitch (y)
    w4 = np.array([0.0, 1.0, 0.0])  # wrist pitch (y)
    w5 = np.array([1.0, 0.0, 0.0])  # wrist roll (x)
    w6 = np.array([0.0, 1.0, 0.0])  # wrist yaw/pitch (y)

    # compute v = -omega x q
    v1 = -np.cross(w1, q1)
    v2 = -np.cross(w2, q2)
    v3 = -np.cross(w3, q3)
    v4 = -np.cross(w4, q4)
    v5 = -np.cross(w5, q5)
    v6 = -np.cross(w6, q6)

    # S matrix: 6x6, each column is [omega; v]
    S = np.zeros((6,6))
    S[:,0] = np.hstack((w1, v1))
    S[:,1] = np.hstack((w2, v2))
    S[:,2] = np.hstack((w3, v3))
    S[:,3] = np.hstack((w4, v4))
    S[:,4] = np.hstack((w5, v5))
    S[:,5] = np.hstack((w6, v6))

    # Home configuration M: end-effector pose at zero joint angles
    p_home = q6 + np.array([0.0, ee_up, ee_forward])
    R_home = np.eye(3)
    M = [np.eye(4)]
    M[0:3,0:3] = R_home
    M[0:3,3] = p_home
    # ==============================================================#

    return M, S

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    """Compute forward kinematics using product of exponentials.
    Angles are in radians; returns the encoder-like joint list expected by lab3_exec.
    Also prints the 4x4 homogeneous transform T.
    """
    # Initialize the return_value
    return_value = [None, None, None, None, None, None]
    # =========== Implement joint angle to encoder expressions here ===========
    print("Foward kinematics calculated:\n")
    
    # =================== Your code starts here ====================#
    thetas = [theta1*np.pi/180, theta2*np.pi/180, theta3*np.pi/180, theta4*np.pi/180, theta5*np.pi/180, theta6*np.pi/180]
    
    M, S = Get_MS()
    print(M)
    print(S)

    # Compute T = e^{S1*th1} ... e^{S6*th6} M
    T = np.eye(4)
    # for i in range(6):
    #     Si = S[:,i]
    #     se3 = _twist_to_se3(Si)
    #     T = T.dot(expm(se3 * thetas[i]))
    # T = T.dot(M)

    for i in range(6):
        omega = S[i][0:3]
        v = S[i][3:6]
        
        skew = np.array([
            [0,-omega[2], omega[1]],
            [omega[2], 0, -omega[0]],
            [-omega[1], omega[0], 0]
        ])
        
        S_matrix = np.zeros((4,4))
        S_matrix[0:3, 0:3] = skew
        S_matrix[0:3, 3] = v
        
        exponentials = expm(S_matrix * thetas[i])
        T = T @ exponentials

    T = T @ M
    

    # ==============================================================#
    print(str(T) + "\n")

    return_value = [None]*6
    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value
