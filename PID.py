import numpy as np

class PID:
# PID controller
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

def pid_control_attitude(X, U, h_s):   
# controllers
    kp = 0.1
    ki = 0.1
    kd = 0.01

    phi_controller = PID(kp=kp, ki=ki, kd=kd, setpoint=U[0])  # Setpoint is the desired angle, here joystick input
    theta_controller = PID(kp=kp, ki=ki, kd=kd, setpoint=U[1])
    psi_controller = PID(kp=kp, ki=ki, kd=kd, setpoint=U[2])

    # Get current angles from state
    phi, theta, psi, T = X[6], X[7], X[8], X[2]

    # Update control outputs from PID based on current angles and desired setpoints
    phi_c = phi_controller.update(phi, h_s)
    theta_c = theta_controller.update(theta, h_s)
    psi_c = psi_controller.update(psi, h_s)

    U_pid_att = np.array([phi_c, theta_c, psi_c])

    return U_pid_att

def pid_control_position(X, U, h_s):   
# controllers
    kp = 0.1
    ki = 0.1
    kd = 0.01

    x_controller = PID(kp=kp, ki=ki, kd=kd, setpoint=U[0])  # Setpoint is the desired angle, here joystick input
    y_controller = PID(kp=kp, ki=ki, kd=kd, setpoint=U[1])
    z_controller = PID(kp=kp, ki=ki, kd=kd, setpoint=U[2])

    # Get current angles from state
    x, y, z = X[0], X[1], X[2]

    # Update control outputs from PID based on current angles and desired setpoints
    x_c = x_controller.update(x, h_s)
    y_c = y_controller.update(y, h_s)
    z_c = z_controller.update(z, h_s)


    U_pid_pos = np.array([x_c, y_c, z_c])

    return U_pid_pos