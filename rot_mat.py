# Rotation from body frame to inertial frame

import numpy as np

def rotation_matrix(phi, theta, psi):
    cphi = np.cos(phi)
    cthe = np.cos(theta)
    cpsi = np.cos(psi)

    sphi = np.sin(phi)
    sthe = np.sin(theta)
    spsi = np.sin(psi)

    R_b_i = np.array([
        [cthe*cpsi, (sphi*sthe*cpsi)-(cphi*spsi), (cphi*sthe*cpsi)+(sphi*spsi)],
        [cthe*spsi, (sphi*sthe*spsi)+(cphi*cpsi), (cphi*sthe*spsi)-(sphi*cpsi)],
        [-sthe, sphi*cthe, cphi*cthe]
    ])

    return R_b_i

def rotation_matrix_body_euler(phi, theta, psi):
    cphi = np.cos(phi)
    cthe = np.cos(theta)
    cpsi = np.cos(psi)

    sphi = np.sin(phi)
    sthe = np.sin(theta)
    spsi = np.sin(psi)

    R_b_e = np.array([
        [1, (sphi*sthe)/cthe, (cphi*sthe)/cthe],
        [0, (cphi), (-sphi)],
        [0, sphi/cthe, cphi/cthe]
    ])

    return R_b_e