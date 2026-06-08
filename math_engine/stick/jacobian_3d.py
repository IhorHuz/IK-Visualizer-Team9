import numpy as np

def jacobian_iteration_3d(joints, target, method='dls', learning_rate=0.2, damping=0.1):
    joints = np.array(joints, dtype=float)
    target = np.array(target, dtype=float)

    end_effector = joints[-1]
    delta_target = target - end_effector

    if np.linalg.norm(delta_target) < 0.1:
        return [list(j) for j in joints]

    num_joints = len(joints) - 1
    J = np.zeros((3, 3 * num_joints))

    for i in range(num_joints):
        r = end_effector - joints[i]
        rx, ry, rz = r
        J_i = np.array([
            [  0,  rz, -ry],
            [-rz,   0,  rx],
            [ ry, -rx,   0]
        ])
        J[:, i*3:(i+1)*3] = J_i

    if method == 'dls':
        lambda_sq = damping ** 2
        I = np.eye(3)
        dls_matrix = np.dot(J, J.T) + lambda_sq * I
        dls_inv = np.linalg.inv(dls_matrix)
        J_dls = np.dot(J.T, dls_inv)
        delta_omega = np.dot(J_dls, delta_target) * learning_rate
    elif method == 'transpose':
        delta_omega = learning_rate * np.dot(J.T, delta_target)
    elif method == 'pseudoinverse':
        J_pinv = np.linalg.pinv(J)
        delta_omega = np.dot(J_pinv, delta_target) * learning_rate
    else:
        delta_omega = np.zeros(3 * num_joints)

    for i in range(num_joints):
        omega = delta_omega[i*3:(i+1)*3]
        angle = np.linalg.norm(omega)
        if angle < 1e-5:
            continue

        axis = omega / angle
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

    return [list(j) for j in joints]
