import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  

# from forward_euler import integration
from six_DOF_model import quad_model
# from input_map import stick_inputs
from quad_interface import get_joystick_inputs
from PID import pid_control
from inner_loop import attitude_control
from filtering import input_filtering

# Initialization

# Initial conditions
x0_m = 0
y0_m = 0
z0_m = 0
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

# Visualization

# Prepare 2D plotting
fig1, axs = plt.subplots(3, 2, figsize=(12, 12))  # 3x2 grid for 2D plots
axs = axs.flatten()  # Flatten the array to make indexing easier
lines = [ax.plot([], [])[0] for ax in axs]
labels = ['x (m)', 'y (m)', 'z (m)', 'Roll (phi)', 'Pitch (theta)', 'Yaw (psi)']
y_limits = [(-10, 10), (-10, 10), (-10, 10), (-180, 180), (-180, 180), (-180, 180)]

for ax, label, ylim in zip(axs, labels, y_limits):
    ax.set_xlim(0, 10)
    ax.set_ylim(*ylim)
    ax.legend([label])

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
    for line in lines:
        line.set_data([], [])
    return lines


# Time marching
def update(frame):
    global X, U, U_c

    # inputs (phi, theta, and psi in this case)
    U_raw = get_joystick_inputs()

    U = input_filtering(U_raw)

    U_pid = pid_control(X, U, h_s)

    l, m, n = attitude_control(X, U_pid, h_s)

    state_derivative = quad_model(X, U, l, m, n)
    X += h_s * state_derivative  # Euler forward integration

    # print(X[6]*(180/np.pi), ",", X[7]*(180/np.pi), ",", X[8]*(180/np.pi))
    print(X[0], ",", X[1], ",", X[2])
    
    
    # Update 2D plots
    data_points = [X[0], X[1], X[2], X[6] * (180 / np.pi), X[7] * (180 / np.pi), X[8] * (180 / np.pi)]
    for line, data in zip(lines, data_points):
        xdata, ydata = line.get_data()
        xdata.append(frame * h_s)
        ydata.append(data)
        line.set_data(xdata, ydata)

    # Update 3D plot
    point.set_data([X[0]], [X[1]]) 
    point.set_3d_properties(X[2])

    return lines + [point]

ani = FuncAnimation(fig1, update, frames=np.linspace(0, 10/h_s, 1000), init_func=init, blit=True)
plt.show()