import numpy as np

def _local_to_world(joints, local_axes, i):
    R = np.eye(3)
    for k in range(i):
        v = joints[k+1] - joints[k]
        norm = np.linalg.norm(v)
        if norm > 1e-5:
            v_dir = v / norm
            v_local = np.dot(R.T, v_dir)
            if local_axes[k][0] == 1:
                angle = np.arctan2(v_local[2], v_local[1])
                c = np.cos(angle)
                s = np.sin(angle)
                R_j = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
            elif local_axes[k][1] == 1:
                angle = np.arctan2(v_local[0], v_local[2])
                c = np.cos(angle)
                s = np.sin(angle)
                R_j = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
            else:
                angle = np.arctan2(v_local[0], v_local[1])
                c = np.cos(angle)
                s = np.sin(angle)
                R_j = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            R = np.dot(R, R_j)
    return np.dot(R, local_axes[i])

def _lock_inactive(joints, active):
    if active >= len(joints) - 1:
        return
    dir_vec = joints[-1] - joints[active]
    dir_norm = np.linalg.norm(dir_vec)
    if dir_norm > 1e-5:
        dir_u = dir_vec / dir_norm
        pos = joints[active].copy()
        lengths = [np.linalg.norm(joints[i+1] - joints[i]) for i in range(active, len(joints) - 1)]
        for i in range(active, len(joints) - 1):
            pos = pos + dir_u * lengths[i - active]
            joints[i+1] = pos

def _clamp_above_ground(joints):
    for i in range(1, len(joints)):
        if joints[i][1] < 0:
            joints[i][1] = 0.0

def jacobian_iteration_3d(joints, target, method='dls', learning_rate=0.2, damping=0.1, active_joints=5):
    joints = np.array(joints, dtype=float)
    target = np.array(target, dtype=float)
    if target[1] < 0:
        target[1] = 0.0

    end_effector = joints[-1]
    delta_target = target - end_effector

    if np.linalg.norm(delta_target) < 0.1:
        return [list(j) for j in joints]

    num_joints = len(joints) - 1
    local_axes = [
        np.array([0, 1, 0]),
        np.array([0, 0, 1]),
        np.array([0, 0, 1]),
        np.array([0, 1, 0]),
        np.array([0, 0, 1]),
        np.array([0, 0, 1]),
    ]

    J = np.zeros((3, num_joints))

    for i in range(num_joints):
        if i < active_joints:
            r = end_effector - joints[i]
            axis = _local_to_world(joints, local_axes, i)
            J[:, i] = np.cross(axis, r)

    if method == 'dls':
        lambda_sq = damping ** 2
        I = np.eye(3)
        dls_matrix = np.dot(J, J.T) + lambda_sq * I
        dls_inv = np.linalg.inv(dls_matrix)
        J_dls = np.dot(J.T, dls_inv)
        delta_theta = np.dot(J_dls, delta_target) * learning_rate
    elif method == 'transpose':
        delta_theta = learning_rate * np.dot(J.T, delta_target)
    elif method == 'pseudoinverse':
        J_pinv = np.linalg.pinv(J)
        delta_theta = np.dot(J_pinv, delta_target) * learning_rate
    else:
        delta_theta = np.zeros(num_joints)

    for i in range(num_joints):
        angle = delta_theta[i]
        if abs(angle) < 1e-5:
            continue

        if i >= active_joints:
            continue

        axis = _local_to_world(joints, local_axes, i)
        K = np.array([
            [       0, -axis[2],  axis[1]],
            [ axis[2],        0, -axis[0]],
            [-axis[1],  axis[0],        0]
        ])
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

        current_joint = joints[i].copy()
        for j in range(i + 1, len(joints)):
            v = joints[j] - current_joint
            joints[j] = current_joint + np.dot(R, v)

    _lock_inactive(joints, active_joints)
    _clamp_above_ground(joints)
    return [list(j) for j in joints]
