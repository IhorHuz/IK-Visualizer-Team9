import numpy as np

def ccd_iteration_3d(joints, target, damping=0.25):
    joints = np.array(joints, dtype=float)
    target = np.array(target, dtype=float)
    num_joints = len(joints)

    snapshot = joints.copy()
    stored_updates = [None] * (num_joints - 1)

    for i in range(num_joints - 2, -1, -1):
        tip = snapshot[-1]
        current_joint = snapshot[i]

        v_tip = tip - current_joint
        v_target = target - current_joint

        v_tip_norm = np.linalg.norm(v_tip)
        v_target_norm = np.linalg.norm(v_target)

        if v_tip_norm < 1e-5 or v_target_norm < 1e-5:
            continue

        v_tip_dir = v_tip / v_tip_norm
        v_target_dir = v_target / v_target_norm

        axis = np.cross(v_tip_dir, v_target_dir)
        axis_norm = np.linalg.norm(axis)

        if axis_norm < 1e-5:
            continue

        axis_dir = axis / axis_norm

        dot_prod = np.clip(np.dot(v_tip_dir, v_target_dir), -1.0, 1.0)
        angle = np.arccos(dot_prod)

        stored_updates[i] = (axis_dir, angle * damping)

    R_cum = np.eye(3)

    for i in range(num_joints - 1):
        if stored_updates[i] is None:
            continue

        axis_dir, angle = stored_updates[i]

        current_axis = np.dot(R_cum, axis_dir)

        K = np.array([
            [0, -current_axis[2], current_axis[1]],
            [current_axis[2], 0, -current_axis[0]],
            [-current_axis[1], current_axis[0], 0]
        ])
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

        current_joint = joints[i]
        for j in range(i + 1, num_joints):
            v = joints[j] - current_joint
            joints[j] = current_joint + np.dot(R, v)

        R_cum = np.dot(R, R_cum)

    return [list(j) for j in joints]
