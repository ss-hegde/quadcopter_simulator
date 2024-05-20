# Inner loop controller - Attitude controller - NDI
# Input - phi_c, theta_c, psi_c (command) calculated from the outer loop
# Output - l, m, m - moments - goes to the plant model

import numpy as np
import parameters
from rot_mat import rotation_matrix_body_euler

class DerivativeCalculator:
    def __init__(self, dt):
        self.dt = dt
        self.previous_angle = None
        self.current_angle = None
        self.next_angle = None

    def update_angle(self, new_angle):
        # Shift the angles to store the new one
        self.previous_angle = self.current_angle
        self.current_angle = self.next_angle
        self.next_angle = new_angle

    def calculate_first_derivative(self):
        if self.current_angle is None or self.previous_angle is None:
            return 0  # Can't compute derivative yet
        return (self.next_angle - self.previous_angle) / (2 * self.dt)

    def calculate_second_derivative(self):
        if self.previous_angle is None or self.current_angle is None or self.next_angle is None:
            return 0  # Can't compute second derivative yet
        return (self.next_angle - 2 * self.current_angle + self.previous_angle) / (self.dt**2)


def attitude_control(X, U_c, h_s):
    dt = h_s  # Time step

    # Phi
    phi_calculator = DerivativeCalculator(dt)
    # During each update cycle in your control loop:
    new_phi_c = U_c[0]  # Scale joystick input
    phi_calculator.update_angle(new_phi_c)

    phi_dot_d = phi_calculator.calculate_first_derivative()
    phi_ddot_d = phi_calculator.calculate_second_derivative()

    # Theta
    theta_calculator = DerivativeCalculator(dt)
    new_theta_c = U_c[1]
    theta_calculator.update_angle(new_theta_c)

    theta_dot_d = theta_calculator.calculate_first_derivative()
    theta_ddot_d = theta_calculator.calculate_second_derivative()

    # Psi
    psi_calculator = DerivativeCalculator(dt)
    new_psi_c = U_c[2]
    psi_calculator.update_angle(new_psi_c)

    psi_dot_d = psi_calculator.calculate_first_derivative()
    psi_ddot_d = psi_calculator.calculate_second_derivative()

    # Eurler angles (Feedback from the plant)
    phi = X[6]; #phi_dot = X[9]
    theta = X[7]; #theta_dot = X[10]
    psi = X[8]; #psi_dot = X[11]

    p = X[9]
    q = X[10]
    r = X[11]

    angle_rates = rotation_matrix_body_euler(phi, theta, psi) @ np.array([p, q, r])

    phi_dot = angle_rates[0]
    theta_dot = angle_rates[1]
    psi_dot = angle_rates[2]


    # Gain (zeta and omega_n) for rotational error dynamics
    zeta_r_x = 0.95
    zeta_r_y = 0.95
    zeta_r_z = 0.95
    omega_n_r_x = 18
    omega_n_r_y = 18
    omega_n_r_z = 12

    # Error dynamics
    phi_ddot = phi_ddot_d + (2*zeta_r_x*omega_n_r_x*(phi_dot_d-phi_dot)) + (np.square(omega_n_r_x) * (new_phi_c-phi))
    theta_ddot = theta_ddot_d + (2*zeta_r_y*omega_n_r_y*(theta_dot_d-theta_dot)) + (np.square(omega_n_r_y) * (new_theta_c-theta))
    psi_ddot = psi_ddot_d + (2*zeta_r_z*omega_n_r_z*(psi_dot_d-psi_dot)) + (np.square(omega_n_r_z) * (new_psi_c-psi))

    # Eularian angular acceleration (p_dot,q_dot,r_dot)

    A1 = np.array([[1, 0, -np.sin(theta)],
               [0, np.cos(phi), np.sin(phi) * np.cos(theta)],
               [0, -np.sin(phi), np.cos(phi) * np.cos(theta)]])

    d_dot = np.array([[phi_ddot],
                    [theta_ddot],
                    [psi_ddot]])

    A2 = np.array([[0, 0, -np.sin(theta)],
                [0, np.sin(phi) * phi_dot, -np.sin(phi) * np.sin(theta) * theta_dot - np.cos(phi) * np.cos(theta) * phi_dot],
                [0, np.cos(phi) * phi_dot, -np.sin(theta) * np.cos(phi) * theta_dot + np.sin(phi) * np.cos(theta) * phi_dot]])

    d = np.array([[phi_dot],
                [theta_dot],
                [psi_dot],])
    
    # print("A1 shape:", A1.shape)
    # print("A2 shape:", A2.shape)
    # print("d_dot shape:", d_dot.shape)
    # print("d shape:", d.shape)


    angular_accln = A1 @ d_dot + A2 @ d
    
    # Moment and rate of change of moments
    p_dot = angular_accln[0]
    q_dot = angular_accln[1]
    r_dot = angular_accln[2]

    l = (parameters.Ixx*p_dot) + ((parameters.Izz - parameters.Iyy) * q * r)
    m = (parameters.Iyy*q_dot) + ((parameters.Ixx - parameters.Izz) * p * r)
    n = (parameters.Izz*r_dot) + ((parameters.Iyy - parameters.Ixx) * p * q)

    return l, m, n



