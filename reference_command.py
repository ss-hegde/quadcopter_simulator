import numpy as np
import parameters

from rot_mat import rotation_matrix 

class UpdateThrust:
    def __init__(self):
        # self.dt = dt
        self.previous_thrust = None
        self.current_thrust = None
        self.next_thrust = None

    def update_thrust(self, new_thrust):
        # Shift the thrusts to store the new one
        self.previous_thrust = self.current_thrust
        self.current_thrust = self.next_thrust
        self.next_thrust = new_thrust

    # def calculate_first_integral(self):
    #     if self.current_thrust is None or self.previous_thrust is None:
    #         return self.velocity  # Can't compute integral yet
    #     self.velocity += self.current_thrust * self.dt
    #     return self.velocity

    # def calculate_second_integral(self):
    #     if self.current_thrust is None or self.previous_thrust is None:
    #         return self.position  # Can't compute integral yet
    #     self.position += self.velocity * self.dt
    #     return self.position

def calculate_velocity(acc, vel, dt):
    vel += acc*dt
    
    return vel

def calculate_pos(vel, pos, dt):
    pos += vel*dt
    
    return pos

def refCoordinates(U, T, dt):

    phi = U[0]
    theta = U[1]
    psi = U[2]

    g = parameters.g
    m = parameters.m

    thrust_updater = UpdateThrust()
    current_thrust = T
    updated_thrust = thrust_updater.update_thrust(current_thrust)
    updated_thrust = T

    R_b_i = rotation_matrix(phi, theta, psi)
    thrust_vec = np.array([[0], [0], [-updated_thrust/m]], dtype=float)
    gravity_vec = np.array([[0], [0], [g]], dtype=float)

    # thrust_flat = []
    # for item in thrust_vec.flatten():
    #     if isinstance(item, np.ndarray):
    #         thrust_flat.append(item.item())  # Extract the scalar value from the numpy array
    #     else:
    #         thrust_flat.append([item])

    # Convert to numpy array
    # thrust_vec = np.array(thrust_flat)

    # print("R_b_i", R_b_i)
    # print("thrust_vec", thrust_vec)
    # print("g_vec", gravity_vec)


    # acceleration
    acceleration = (R_b_i @ thrust_vec) + gravity_vec
    # acceleration = (rotation_matrix(phi, theta, psi) @ np.array([[0], [0], [-updated_thrust/m]])) + np.array([[0], [0], [g]])

    x_ddot = acceleration[0]
    y_ddot = acceleration[1]
    z_ddot = acceleration[2]
    desired_acc = np.array([[x_ddot], [y_ddot], [z_ddot]], dtype=float)
    # print("desired_acc: ", desired_acc)
    
    # velocities
    x_dot = 0.0
    y_dot = 0.0
    z_dot = 0.0

    x_dot = calculate_velocity(x_ddot, x_dot, dt)
    y_dot = calculate_velocity(y_ddot, y_dot, dt)
    z_dot = calculate_velocity(z_ddot, z_dot, dt)
    desired_vel = np.array([[x_dot], [y_dot], [z_dot]], dtype=float)
    # print("desired_vel: ", desired_vel)

    # position
    x = 0.0
    y = 0.0
    z = 0.0

    x = calculate_pos(x_dot, x, dt)
    y = calculate_pos(y_dot, y, dt)
    z = calculate_pos(z_dot, z, dt)
    desired_pos = np.array([[x], [y], [z]], dtype=float)
    # print("desired_pos: ", desired_pos)

    return desired_pos, desired_vel, desired_acc


