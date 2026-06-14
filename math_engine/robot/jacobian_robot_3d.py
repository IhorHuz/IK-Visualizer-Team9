import numpy as np
import math

ANGLE_LIMITS = [
    None,  # J0: unbounded yaw
    (-1.57, 1.57),  # J1: shoulder pitch ±90°
    (-1.57, 1.57),  # J2: elbow pitch ±90°
    None,  # J3: unbounded yaw
    (-1.57, 1.57),  # J4: wrist pitch ±90°
    (-1.57, 1.57),  # J5: wrist Z ±90°
]

DOF_AXES = ["Y", "Z", "Z", "Y", "Z", "Z"]


def get_rotation_matrix(axis, angle):
    angle = float(angle)
    c = math.cos(angle)
    s = math.sin(angle)
    if axis == "Y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == "Z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.eye(3)


def forward_kinematics(angles, lengths, origin):
    n_joints = len(lengths) + 1
    joints = [np.zeros(3) for _ in range(n_joints)]
    joints[0] = np.array(origin, dtype=float)

    current_R = np.eye(3)
    R_frames = []

    for i in range(len(lengths)):
        R_frames.append(current_R.copy())
        current_R = np.dot(current_R, get_rotation_matrix(DOF_AXES[i], angles[i]))
        world_dir = np.dot(current_R, np.array([0.0, 1.0, 0.0]))
        joints[i + 1] = joints[i] + world_dir * lengths[i]

    return joints, R_frames


class JacobianSolver3d:
    def __init__(self, joints, target, initial_angles=None, locked_joints=None):
        self.origin = np.array([joints[0].x, joints[0].y, joints[0].z], dtype=float)
        self.target = np.array([target.x, target.y, target.z], dtype=float)
        self.lengths = [
            np.linalg.norm(
                np.array([joints[i + 1].x, joints[i + 1].y, joints[i + 1].z]) -
                np.array([joints[i].x, joints[i].y, joints[i].z])
            ) for i in range(len(joints) - 1)
        ]
        self.n = len(joints)
        self.tol = 0.05

        self.damping = 0.2
        self.step_size = 0.1

        if initial_angles is not None:
            self.angles = list(initial_angles)
        else:
            self.angles = [0.0] * (self.n - 1)
        self.locked_joints = locked_joints if locked_joints else [False] * (self.n - 1)

    def solve(self):
        if self.target[1] < 0:
            self.target[1] = 0.0
        joints, R_frames = forward_kinematics(self.angles, self.lengths, self.origin)
        p_ee = joints[-1]

        delta_x = self.target - p_ee
        if np.linalg.norm(delta_x) < self.tol:
            return joints, self.angles

        num_axes = len(joints) - 1
        J = np.zeros((3, num_axes))

        for i in range(num_axes):
            local_axis = np.array([0.0, 1.0, 0.0]) if DOF_AXES[i] == "Y" else np.array([0.0, 0.0, 1.0])
            world_axis = np.dot(R_frames[i], local_axis)

            r_vector = p_ee - joints[i]
            J[:, i] = np.cross(world_axis, r_vector)

        # Zero out locked joints so they don't affect the solve
        for i in range(num_axes):
            if self.locked_joints[i]:
                J[:, i] = 0.0

        J_JT = np.dot(J, J.T)
        damping_matrix = J_JT + (self.damping ** 2) * np.eye(3)
        inv_part = np.linalg.inv(damping_matrix)

        delta_theta = np.dot(J.T, np.dot(inv_part, delta_x))

        proposed_angles = []
        for i in range(num_axes):
            new_ang = self.angles[i] + delta_theta[i] * self.step_size
            lim = ANGLE_LIMITS[i]
            if lim is not None:
                new_ang = np.clip(new_ang, lim[0], lim[1])
            proposed_angles.append(new_ang)

        actual_deltas = [proposed_angles[i] - self.angles[i] for i in range(num_axes)]

        test_angles = [self.angles[i] + actual_deltas[i] for i in range(num_axes)]
        test_joints, _ = forward_kinematics(test_angles, self.lengths, self.origin)

        safe_scale = 1.0
        if any(j[1] < -1e-4 for j in test_joints):
            low, high = 0.0, 1.0
            safe_scale = 0.0

            for _ in range(8):
                mid = (low + high) / 2.0
                chk_angles = [self.angles[i] + mid * actual_deltas[i] for i in range(num_axes)]
                chk_joints, _ = forward_kinematics(chk_angles, self.lengths, self.origin)

                if any(j[1] < -1e-4 for j in chk_joints):
                    high = mid
                else:
                    low = mid
                    safe_scale = mid

        for i in range(num_axes):
            if self.locked_joints[i]:
                continue
            self.angles[i] += safe_scale * actual_deltas[i]

            if ANGLE_LIMITS[i] is None:
                self.angles[i] = math.atan2(math.sin(self.angles[i]), math.cos(self.angles[i]))

        next_joints, _ = forward_kinematics(self.angles, self.lengths, self.origin)
        return next_joints, self.angles
