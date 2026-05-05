import numpy as np
import math


def forward_kinematics(angles, lengths, base_pos=(0, 0)):

    joints = [base_pos]
    current_angle = 0.0

    for i in range(len(angles)):
        current_angle += angles[i]

        x = joints[-1][0] + lengths[i] * math.cos(current_angle)
        y = joints[-1][1] + lengths[i] * math.sin(current_angle)

        joints.append((x, y))

    return joints


def jacobian_iteration(angles, lengths, target, method='dls', learning_rate=0.05, damping=0.1):

    angles = np.array(angles, dtype=float)
    joints = forward_kinematics(angles, lengths)

    end_effector = np.array(joints[-1])
    target_pos = np.array(target)

    delta_target = target_pos - end_effector

    if np.linalg.norm(delta_target) < 1.0:
        return angles

    num_joints = len(angles)
    J = np.zeros((2, num_joints))

    for i in range(num_joints):
        joint_pos = np.array(joints[i])

        J[0, i] = -(end_effector[1] - joint_pos[1])
        J[1, i] = end_effector[0] - joint_pos[0]

    if method == 'transpose':
        J_T = J.T
        delta_theta = learning_rate * np.dot(J_T, delta_target)

    elif method == 'pseudoinverse':
        J_pinv = np.linalg.pinv(J)
        delta_theta = np.dot(J_pinv, delta_target)

        delta_theta = np.clip(delta_theta, -0.5, 0.5)

    elif method == 'dls':
        J_T = J.T
        lambda_sq = damping ** 2
        I = np.eye(2)

        dls_matrix = np.dot(J, J_T) + lambda_sq * I
        dls_inv = np.linalg.inv(dls_matrix)

        J_dls = np.dot(J_T, dls_inv)
        delta_theta = np.dot(J_dls, delta_target)

        delta_theta = np.clip(delta_theta, -0.5, 0.5)

    else:
        raise ValueError("Unknown method! Use 'transpose', 'pseudoinverse' or 'dls'.")

    new_angles = angles + delta_theta

    return new_angles