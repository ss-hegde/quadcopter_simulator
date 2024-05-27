import numpy as np
from reference_command import refCoordinates
import parameters

def position_control(X, T, des_pos, des_vel, des_acc, psi):

    x_d = des_pos[0]
    y_d = des_pos[1]
    z_d = des_pos[2]

    x_dot_d = des_vel[0]
    y_dot_d = des_vel[1]
    z_dot_d = des_vel[2]

    x_ddot_d = des_acc[0]
    y_ddot_d = des_acc[1]
    z_ddot_d = des_acc[2]

    x = X[0]
    y = X[1]
    z = X[2]

    x_dot = X[3]
    y_dot = X[4]
    z_dot = X[5]

    # Gain (zeta and omega_n) for translational error dynamics
    zeta_t_x = 0.2
    zeta_t_y = 0.2
    zeta_t_z = 0.95
    omega_n_t_x = 8
    omega_n_t_y = 8
    omega_n_t_z = 8

    # solving for x_ddot

    x_ddot = x_ddot_d + (2*zeta_t_x*omega_n_t_x*(x_dot_d-x_dot)) + (np.square(omega_n_t_x) * (x_d - x))
    y_ddot = y_ddot_d + (2*zeta_t_y*omega_n_t_y*(y_dot_d-y_dot)) + (np.square(omega_n_t_y) * (y_d - y))
    z_ddot = z_ddot_d + (2*zeta_t_z*omega_n_t_z*(z_dot_d-z_dot)) + (np.square(omega_n_t_z) * (z_d - z))


    # Thrust, phi_d, theta_d
    psi_d = psi

    T = parameters.m * np.sqrt((x_ddot*x_ddot) + (y_ddot*y_ddot) + ((parameters.g - z_ddot)*(parameters.g - z_ddot)))
    u_x = -parameters.m*x_ddot/T
    u_y = -parameters.m*y_ddot/T
    phi_d = np.arcsin((u_x*np.sin(psi_d)) - (u_y*np.cos(psi_d)))
    theta_d = np.arcsin(((u_x*np.cos(psi_d)) + (u_y*np.sin(psi_d)))/ np.cos(phi_d))
    
    U_ol = [float(phi_d), float(theta_d), psi_d]

    return U_ol, T