import numpy as np

def ccd_iteration_3d(joints, target):
    joints = np.array(joints, dtype=float)
    target = np.array(target, dtype=float)
    
    # DOF: J0=Y, J1=Z, J2=Z, J3=Z, J4=Y
    dof_axes = [
        np.array([0, 1, 0]),
        np.array([0, 0, 1]),
        np.array([0, 0, 1]),
        np.array([0, 0, 1]),
        np.array([0, 1, 0]),
    ]

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

        axis_dir = dof_axes[i] if i < len(dof_axes) else np.array([0, 0, 1])

        v_tip_proj = v_tip_dir - np.dot(v_tip_dir, axis_dir) * axis_dir
        v_target_proj = v_target_dir - np.dot(v_target_dir, axis_dir) * axis_dir

        tp_norm = np.linalg.norm(v_tip_proj)
        tv_norm = np.linalg.norm(v_target_proj)

        if tp_norm < 1e-5 or tv_norm < 1e-5:
            continue

        v_tip_proj /= tp_norm
        v_target_proj /= tv_norm

        dot_prod = np.clip(np.dot(v_tip_proj, v_target_proj), -1.0, 1.0)
        angle = np.arccos(dot_prod)

        cross_prod = np.cross(v_tip_proj, v_target_proj)
        if np.dot(cross_prod, axis_dir) < 0:
            angle = -angle

        K = np.array([
            [0, -axis_dir[2], axis_dir[1]],
            [axis_dir[2], 0, -axis_dir[0]],
            [-axis_dir[1], axis_dir[0], 0]
        ])
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

        for j in range(i + 1, len(joints)):
            v = joints[j] - current_joint
            joints[j] = current_joint + np.dot(R, v)

    return [list(j) for j in joints]