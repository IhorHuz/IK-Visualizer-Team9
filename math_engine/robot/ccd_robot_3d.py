import numpy as np

ANGLE_LIMITS = [
    None,             # J0: unbounded yaw
    (-1.57, 1.57),    # J1: shoulder pitch ±90°
    (-1.57, 1.57),    # J2: elbow pitch ±90°
    None,             # J3: unbounded yaw
    (-1.57, 1.57),    # J4: wrist pitch ±90°
    (-1.57, 1.57),    # J5: wrist Z ±90°
]

LOCAL_AXES = [
    np.array([0, 1, 0]),
    np.array([0, 0, 1]),
    np.array([0, 0, 1]),
    np.array([0, 1, 0]),
    np.array([0, 0, 1]),
    np.array([0, 0, 1]),
]


def _clamp_angle(angle, limits):
    if limits is None:
        return angle
    return float(np.clip(angle, limits[0], limits[1]))


def _world_axis(joints, i):
    R = np.eye(3)
    for k in range(i):
        v = joints[k+1] - joints[k]
        norm = np.linalg.norm(v)
        if norm > 1e-5:
            v_dir = v / norm
            v_local = np.dot(R.T, v_dir)
            if LOCAL_AXES[k][1] == 1:
                a = np.arctan2(v_local[0], v_local[2])
                c, s = np.cos(a), np.sin(a)
                R_j = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
            else:
                a = np.arctan2(v_local[0], v_local[1])
                c, s = np.cos(a), np.sin(a)
                R_j = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            R = np.dot(R, R_j)
    return np.dot(R, LOCAL_AXES[i])


def _rotation_matrix(axis, angle):
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)


def _clamp_above_ground(joints):
    for i in range(1, len(joints)):
        if joints[i][1] < 0:
            joints[i][1] = 0.0


def _fk_from_angles(angles, segment_lengths):
    joints = [np.array([0.0, 0.0, 0.0])]
    R = np.eye(3)
    for i in range(len(angles)):
        angle = angles[i]
        if LOCAL_AXES[i][1] == 1:
            world_dir = np.array([0.0, 1.0, 0.0])
        else:
            local_dir = np.array([np.sin(angle), np.cos(angle), 0.0])
            world_dir = np.dot(R, local_dir)
        R = np.dot(R, _rotation_matrix(LOCAL_AXES[i], angle))
        seg_len = segment_lengths[i] if i < len(segment_lengths) else 0.0
        joints.append(joints[-1] + world_dir * seg_len)
    return joints


def ccd_iteration_3d(joints, target, accumulated_angles=None, active_joints=6, step_size=0.3, segment_lengths=None):
    joints = np.array(joints, dtype=float)
    target = np.array(target, dtype=float)
    if target[1] < 0:
        target[1] = 0.0

    if accumulated_angles is None:
        accumulated_angles = [0.0] * active_joints

    for i in range(len(joints) - 2, -1, -1):
        tip = joints[-1]
        current_joint = joints[i]

        v_tip = tip - current_joint
        v_target = target - current_joint

        v_tip_norm = np.linalg.norm(v_tip)
        v_target_norm = np.linalg.norm(v_target)

        if v_tip_norm < 1e-5 or v_target_norm < 1e-5:
            continue

        v_tip_dir = v_tip / v_tip_norm
        v_target_dir = v_target / v_target_norm

        axis_dir = _world_axis(joints, i) if i < active_joints else np.zeros(3)

        if np.linalg.norm(axis_dir) < 1e-5:
            continue

        v_tip_proj = v_tip_dir - np.dot(v_tip_dir, axis_dir) * axis_dir
        v_target_proj = v_target_dir - np.dot(v_target_dir, axis_dir) * axis_dir

        tp_norm = np.linalg.norm(v_tip_proj)
        tv_norm = np.linalg.norm(v_target_proj)

        if tp_norm < 1e-5 or tv_norm < 1e-5:
            continue

        v_tip_proj /= tp_norm
        v_target_proj /= tv_norm

        dot_prod = np.clip(np.dot(v_tip_proj, v_target_proj), -1.0, 1.0)
        delta_angle = np.arccos(dot_prod) * step_size

        cross_prod = np.cross(v_tip_proj, v_target_proj)
        if np.dot(cross_prod, axis_dir) < 0:
            delta_angle = -delta_angle

        if abs(delta_angle) < 1e-5:
            continue

        # Accumulate and clamp angle
        old_angle = accumulated_angles[i]
        new_angle = old_angle + delta_angle
        limits = ANGLE_LIMITS[i] if i < len(ANGLE_LIMITS) else None
        clamped_new = _clamp_angle(new_angle, limits)

        # Compute actual rotation to apply (respecting limits)
        actual_delta = clamped_new - old_angle
        accumulated_angles[i] = clamped_new

        if abs(actual_delta) < 1e-5:
            continue

        R_mat = _rotation_matrix(axis_dir, actual_delta)

        for j in range(i + 1, len(joints)):
            v = joints[j] - current_joint
            joints[j] = current_joint + np.dot(R_mat, v)

    _clamp_above_ground(joints)

    # FK reinitialization: compute FK from accumulated angles to keep positions consistent
    if segment_lengths is not None:
        fk_pos = _fk_from_angles(accumulated_angles, segment_lengths)
        return [list(p) for p in fk_pos], accumulated_angles

    return [list(j) for j in joints], accumulated_angles
