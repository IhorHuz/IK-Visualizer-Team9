import numpy as np

SEGMENT_LENGTHS = [1.2, 3.1, 0.85, 1.3, 5.25, 0.825]
DOF_AXES = ["Y", "Z", "Z", "Y", "Z", "Z"]
ANGLE_LIMITS = [
    None,             # J0: unbounded yaw
    (-1.57, 1.57),    # J1: shoulder pitch ±90°
    (-1.57, 1.57),    # J2: elbow pitch ±90°
    None,             # J3: unbounded yaw
    (-1.57, 1.57),    # J4: wrist pitch ±90°
    (-1.57, 1.57),    # J5: wrist Z ±90°
]


def _rot_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fk(angles):
    joints = [np.array([0.0, 0.0, 0.0])]
    R = np.eye(3)
    for i in range(len(SEGMENT_LENGTHS)):
        angle = angles[i] if i < len(angles) else 0.0
        if DOF_AXES[i] == "Y":
            world_dir = np.array([0.0, 1.0, 0.0])
            R = np.dot(R, _rot_y(angle))
        else:
            local_dir = np.array([np.sin(angle), np.cos(angle), 0.0])
            world_dir = np.dot(R, local_dir)
            R = np.dot(R, _rot_z(angle))
        joints.append(joints[-1] + world_dir * SEGMENT_LENGTHS[i])
    return joints


def _world_axis(R_parent, axes_idx):
    if DOF_AXES[axes_idx] == "Y":
        local = np.array([0.0, 1.0, 0.0])
    else:
        local = np.array([0.0, 0.0, 1.0])
    return np.dot(R_parent, local)


def _jacobian(joints, angles):
    n = len(SEGMENT_LENGTHS)
    ee = joints[-1]
    J = np.zeros((3, n))
    R = np.eye(3)
    for i in range(n):
        axis = _world_axis(R, i)
        r = ee - joints[i]
        J[:, i] = np.cross(axis, r)
        if DOF_AXES[i] == "Y":
            R = np.dot(R, _rot_y(angles[i]))
        else:
            R = np.dot(R, _rot_z(angles[i]))
    return J


def solve(target, current_angles=None, damping=0.3, steps=200, tol=0.01):
    if current_angles is None:
        current_angles = np.array([0.0] * len(SEGMENT_LENGTHS))
    current_angles = np.array(current_angles, dtype=float)

    target = np.array(target, dtype=float)
    if target[1] < 0:
        target[1] = 0.0

    for _ in range(steps):
        joints = fk(current_angles)
        ee = joints[-1]
        error = target - ee
        dist = np.linalg.norm(error)
        if dist < tol:
            break

        J = _jacobian(joints, current_angles)

        # DLS
        I = np.eye(3)
        dls_inv = np.linalg.inv(np.dot(J, J.T) + damping ** 2 * I)
        delta_theta = np.dot(np.dot(J.T, dls_inv), error)

        # Apply damping to limit step size
        max_delta = 0.3
        dn = np.linalg.norm(delta_theta)
        if dn > max_delta:
            delta_theta = delta_theta * max_delta / dn

        for i in range(len(current_angles)):
            current_angles[i] += delta_theta[i]
            # Normalize Y-pivot to [-pi, pi]
            if DOF_AXES[i] == "Y":
                current_angles[i] = np.arctan2(np.sin(current_angles[i]),
                                                np.cos(current_angles[i]))
            else:
                lim = ANGLE_LIMITS[i]
                if lim is not None:
                    current_angles[i] = np.clip(current_angles[i], lim[0], lim[1])

    joints = fk(current_angles)
    return joints, list(current_angles)
