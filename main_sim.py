import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  

# from forward_euler import integration
from six_DOF_model import quad_model
from quad_interface import get_joystick_inputs
from PID import pid_control_attitude, pid_control_position
from inner_loop import attitude_control
# from filtering import input_filtering
# from reference_command import refCoordinates
from outer_loop import position_control
from reference_coordinates import map_joystick_to_setpoints

# Initialization

# Initial conditions
x0_m = 0.1
y0_m = 0.0
z0_m = 0.1
u0_mps = 0
v0_mps = 0
w0_mps = 0
phi0_rad = 0*np.pi/180
theta0_rad = 0*np.pi/180
psi0_rad = 0*np.pi/180
p0_rps = 0
q0_rps = 0
r0_rps = 0

# Assigning initial conditions to an array

X0 = np.array([
    x0_m,
    y0_m,
    z0_m,
    u0_mps,
    v0_mps,
    w0_mps,
    phi0_rad,
    theta0_rad,
    psi0_rad,
    p0_rps,
    q0_rps,
    r0_rps,
])

# Length of X0
nX0 = len(X0)

# Time conditions
h_s = 0.005

# Pre allocate the solution array X
X = X0.copy()
U = np.zeros([4])
U_c = np.zeros([4])
U_ol = np.zeros([3])
# T = np.array([1.0])
T = 0.0

# Visualization

# Prepare 2D plotting
fig1, axs = plt.subplots(3, 2, figsize=(12, 12))  # 3x2 grid for 2D plots
axs = axs.flatten()  # Flatten the array to make indexing easier
lines = [ax.plot([], [])[0] for ax in axs]
dotted_lines = [ax.plot([], [], 'r--')[0] for ax in axs]  # Dotted lines for inputs
labels = ['x (m)', 'y (m)', 'z (m)', 'phi', 'theta', 'psi']
labels_c = ['x_c (m)', 'y_c (m)', 'z_c (m)', 'phi_c', 'theta_c', 'psi']
y_limits = [(-10, 10), (-10, 10), (-10, 10), (-180, 180), (-180, 180), (-180, 180)]
# y_limits = [(-1, 1), (-1, 1), (-1, 1), (-180, 180), (-180, 180), (-180, 180)]

for ax, label, label_c, ylim in zip(axs, labels, labels_c, y_limits):
    ax.set_xlim(0, 10)
    ax.set_ylim(*ylim)
    ax.legend([label, label_c])

# Prepare 3D plotting
fig2 = plt.figure()
ax3d = fig2.add_subplot(111, projection='3d')
ax3d.set_xlim([-10, 10])
ax3d.set_ylim([-10, 10])
ax3d.set_zlim([-10, 10])
ax3d.set_xlabel('X')
ax3d.set_ylabel('Y')
ax3d.set_zlabel('Z')
point, = ax3d.plot([], [], [], 'ro')


def init():
    for line in lines + dotted_lines:
        line.set_data([], [])
    return lines + dotted_lines

# Time marching
def update(frame):
    global X, U, U_c, T, U_ol

    # inputs (x, y, z, and psi in this case)
    x, y, z, psi = get_joystick_inputs()

    current_position = np.array([[X[0]], [X[1]], [X[2]]], dtype=float)
    pos_d, vel_d, acc_d = map_joystick_to_setpoints(x, y, z, current_position, h_s)

    # pos_d_pid = pid_control_position(X, pos_d, h_s)

    U_ol, T = position_control(X, T, pos_d, vel_d, acc_d, psi)

    U_pid = pid_control_attitude(X, U_ol, h_s)

    l, m, n = attitude_control(X, U_pid, h_s)

    state_derivative = quad_model(X, U, T, l, m, n)
    
    X += h_s * state_derivative  # Euler forward integration
    
    # Update 2D plots
    data_points = [X[0], X[1], X[2], X[6] * (180 / np.pi), X[7] * (180 / np.pi), X[8] * (180 / np.pi)]
    input_points = [pos_d[0], pos_d[1], pos_d[2], U_pid[0] * (180 / np.pi), U_pid[1] * (180 / np.pi), U_pid[2] * (180 / np.pi)]
    for line, dotted_line, data, input_data in zip(lines, dotted_lines, data_points, input_points):
        xdata, ydata = line.get_data()
        xdata_input, ydata_input = dotted_line.get_data()
        xdata.append(frame * h_s)
        ydata.append(data)
        xdata_input.append(frame * h_s)
        ydata_input.append(input_data)
        line.set_data(xdata, ydata)
        dotted_line.set_data(xdata_input, ydata_input)

    # Update 3D plot
    point.set_data([X[0]], [X[1]]) 
    point.set_3d_properties(X[2])

    return lines + dotted_lines + [point]

ani = FuncAnimation(fig1, update, frames=np.linspace(0, 10/h_s, 1000), init_func=init, blit=True)
plt.show()