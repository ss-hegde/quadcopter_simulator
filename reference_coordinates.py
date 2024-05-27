import numpy as np

class DerivativeCalculator:
    def __init__(self, dt):
        self.dt = dt
        self.previous_coordinate = None
        self.current_coordinate = None
        self.next_coordinate = None

    def update_coordinate(self, new_coordinate):
        # Shift the coordinates to store the new one
        self.previous_coordinate = self.current_coordinate
        self.current_coordinate = self.next_coordinate
        self.next_coordinate = new_coordinate

    def calculate_first_derivative(self):   # Velocity
        if self.current_coordinate is None or self.previous_coordinate is None:
            return 0  # Can't compute derivative yet
        return (self.next_coordinate - self.previous_coordinate) / (2 * self.dt)

    def calculate_second_derivative(self):  # Acceleration
        if self.previous_coordinate is None or self.current_coordinate is None or self.next_coordinate is None:
            return 0  # Can't compute second derivative yet
        return (self.next_coordinate - 2 * self.current_coordinate + self.previous_coordinate) / (self.dt**2)

def map_joystick_to_setpoints(x, y, z, current_position, dt):
    # Define sensitivity or scaling factors
    scale_x, scale_y, scale_z = 5.0, 5.0, 5.0

    x_calculator = DerivativeCalculator(dt)
    y_calculator = DerivativeCalculator(dt)
    z_calculator = DerivativeCalculator(dt)

    # Calculate new position setpoints based on joystick input
    new_x = current_position[0] + x * scale_x
    new_y = current_position[1] + y * scale_y
    new_z = current_position[2] + z * scale_z

    # print("x: ", new_x, " y: ", new_y, " z: ", new_z)
    # print("Current_pos", current_position)

    
    x_calculator.update_coordinate(new_x)
    x_dot_d = x_calculator.calculate_first_derivative()
    # print("x_dot_val: ", x_dot_d, ",", "x_dot_type", type(x_dot_d))
    
    x_ddot_d = x_calculator.calculate_second_derivative()

    
    y_calculator.update_coordinate(new_y)
    y_dot_d = y_calculator.calculate_first_derivative()
    y_ddot_d = y_calculator.calculate_second_derivative()

    
    z_calculator.update_coordinate(new_z)
    z_dot_d = z_calculator.calculate_first_derivative()
    z_ddot_d = z_calculator.calculate_second_derivative()

    desired_pos = np.array([[new_x], [new_y], [new_z]], dtype=float)
    desired_vel = np.array([[x_dot_d], [y_dot_d], [z_dot_d]])
    desired_acc = np.array([[x_ddot_d], [y_ddot_d], [z_ddot_d]])
    # desired_pos = [new_x, new_y, new_z]
    # desired_vel = [x_dot_d, y_dot_d, z_dot_d]
    # desired_acc = [x_ddot_d, y_ddot_d, z_ddot_d]

    return desired_pos, desired_vel, desired_acc