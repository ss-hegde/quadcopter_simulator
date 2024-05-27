import numpy as np
import parameters
from rot_mat import rotation_matrix, rotation_matrix_body_euler

#---------------------------------------------------------------------------
# X - state 
# dX - State derivatives
#---------------------------------------------------------------------------

def quad_model(X, U, T, l, m, n):
    dX = np.empty((12,),dtype=float)   # pre allocate state derivatives
    # U = np.empty((4,),dtype=float)

    x_m = X[0]
    y_m = X[1]
    z_m = X[2]
    u_mps = X[3]
    v_mps = X[4]
    w_mps = X[5]
    phi_rad = X[6]
    theta_rad = X[7]
    psi_rad = X[8]
    p_rps = X[9]
    q_rps = X[10]
    r_rps = X[11]

    # mass and moment of inertia
    m_kg = parameters.m
    Ixx_b_kgm2 = parameters.Ixx
    Iyy_b_kgm2 = parameters.Iyy
    Izz_b_kgm2 = parameters.Izz


    # gravity 
    gz_n_mps2 = parameters.g

    # gravity in Body Fixed Coordinates
    g_b_mps2 = np.transpose(rotation_matrix(phi_rad, theta_rad, psi_rad)) @ np.array([[0],[0],[gz_n_mps2]])

    gx_b_mps2 = g_b_mps2[0]
    gy_b_mps2 = g_b_mps2[1]
    gz_b_mps2 = g_b_mps2[2]

    # Kinematics
    translation_derv = rotation_matrix(phi_rad, theta_rad, psi_rad) @ np.array([[u_mps], [v_mps], [w_mps]])
    angular_derv = rotation_matrix_body_euler(phi_rad, theta_rad, psi_rad) @ np.array([[p_rps], [q_rps], [r_rps]])
    
    # Forces and moments 
    T_b_N = T

    l_b_Nm = l
    m_b_Nm = m 
    n_b_Nm = n   


    dX[0] = translation_derv[0]
    dX[1] = translation_derv[1]
    dX[2] = translation_derv[2]

    dX[3] = gx_b_mps2 + (r_rps*v_mps) - (q_rps*w_mps)
    dX[4] = gy_b_mps2 + (p_rps*w_mps) - (u_mps*r_rps)
    dX[5] = (-T_b_N/m_kg) + gz_b_mps2 + (q_rps*u_mps) - (p_rps*v_mps) 

    dX[6] = angular_derv[0]
    dX[7] = angular_derv[1]
    dX[8] = angular_derv[2]

    dX[9] = ((Iyy_b_kgm2-Izz_b_kgm2)*q_rps*r_rps / Ixx_b_kgm2) + (1/Ixx_b_kgm2 * l_b_Nm)
    dX[10] = ((Izz_b_kgm2-Ixx_b_kgm2)*p_rps*r_rps / Iyy_b_kgm2) + (1/Iyy_b_kgm2 * m_b_Nm)
    dX[11] = ((Ixx_b_kgm2-Iyy_b_kgm2)*p_rps*q_rps / Izz_b_kgm2) + (1/Izz_b_kgm2 * n_b_Nm)


    return dX