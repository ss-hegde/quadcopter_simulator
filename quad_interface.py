import pygame
import numpy as np

# Initialize Pygame and joystick
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

def get_joystick_inputs():
    pygame.event.pump()  # Update pygame events
    
    
    # Assuming the joystick has at least two axes
    # roll_axis = joystick.get_axis(0) * 90*np.pi/180  # Scale factor for demonstration
    # pitch_axis = joystick.get_axis(1) * 90*np.pi/180
    # yaw_axis = joystick.get_axis(2) * 90*np.pi/180
    # throttle = joystick.get_axis(3) * 1
    # return roll_axis, pitch_axis, yaw_axis, throttle # No yaw control for simplicity

    x = joystick.get_axis(0) * 0.1  # Scale factor for demonstration
    y = joystick.get_axis(1) * 0.1
    z = 0.0
    psi = joystick.get_axis(2) * 90*np.pi/180

    return x, y, z, psi