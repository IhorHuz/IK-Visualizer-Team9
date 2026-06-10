import numpy as np
import math

ANGLE_LIMITS = [
    None,
    (-1.57, 1.57),
    (-1.57, 1.57),
    None,
    (-1.57, 1.57),
    (-1.57, 1.57),
]

DOF_AXES = ["Y", "Z", "Z", "Y", "Z", "Z"]


def get_rotation_matrix(axis, angle):
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


class CcdSolver3d:
    def __init__(self, joints, target, initial_angles=None):
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

        if initial_angles is not None:
            self.angles = list(initial_angles)
        else:
            self.angles = [0.0] * (self.n - 1)

    def solve(self):
        if self.target[1] < 0:
            self.target[1] = 0.0

        joints, R_frames = forward_kinematics(self.angles, self.lengths, self.origin)
        if np.linalg.norm(joints[-1] - self.target) < self.tol:
            return joints, self.angles

        for i in range(self.n - 2, -1, -1):
            joints, R_frames = forward_kinematics(self.angles, self.lengths, self.origin)

            e_world = joints[-1] - joints[i]
            t_world = self.target - joints[i]

            R_parent_T = R_frames[i].T
            e_local = np.dot(R_parent_T, e_world)
            t_local = np.dot(R_parent_T, t_world)

            if DOF_AXES[i] == "Y":
                ang_e = math.atan2(-e_local[2], e_local[0])
                ang_t = math.atan2(-t_local[2], t_local[0])
                delta = ang_t - ang_e
            else:
                ang_e = math.atan2(-e_local[0], e_local[1])
                ang_t = math.atan2(-t_local[0], t_local[1])
                delta = ang_t - ang_e

            delta = math.atan2(math.sin(delta), math.cos(delta))

            old_angle = self.angles[i]
            new_angle = old_angle + delta
            lim = ANGLE_LIMITS[i]
            if lim is not None:
                new_angle = np.clip(new_angle, lim[0], lim[1])

            actual_delta = new_angle - old_angle

            self.angles[i] = new_angle
            test_joints, _ = forward_kinematics(self.angles, self.lengths, self.origin)

            if any(j[1] < -1e-4 for j in test_joints):
                low, high = 0.0, 1.0
                safe_angle = old_angle

                for _ in range(8):
                    mid = (low + high) / 2.0
                    self.angles[i] = old_angle + mid * actual_delta
                    chk_joints, _ = forward_kinematics(self.angles, self.lengths, self.origin)

                    if any(j[1] < -1e-4 for j in chk_joints):
                        high = mid
                    else:
                        low = mid
                        safe_angle = self.angles[i]

                self.angles[i] = safe_angle

            if lim is None:
                self.angles[i] = math.atan2(math.sin(self.angles[i]), math.cos(self.angles[i]))

        joints, _ = forward_kinematics(self.angles, self.lengths, self.origin)
        return joints, self.angles
