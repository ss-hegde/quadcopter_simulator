# Low pass filter to filter the joystick inputs - avoid the noisy inputs due to small scale fluctuations
import numpy as np

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha  # Smoothing factor, 0 < alpha < 1
        self.state = None

    def update(self, input_value):
        if self.state is None:
            self.state = input_value
        else:
            self.state = self.alpha * input_value + (1 - self.alpha) * self.state
        return self.state

def input_filtering(U):
    # Define filters for each joystick axis
    phi_filter = LowPassFilter(alpha=0.1)
    theta_filter = LowPassFilter(alpha=0.1)
    psi_filter = LowPassFilter(alpha=0.1)

    phi = phi_filter.update(U[0])
    theta = theta_filter.update(U[1])
    psi = psi_filter.update(U[2])

    U_filtered = np.array([phi, theta, psi, U[3]])

    return U_filtered