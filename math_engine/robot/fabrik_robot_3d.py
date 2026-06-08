import numpy as np

ANGLE_LIMITS = [
    None,             # J0: unbounded yaw
    (-1.57, 1.57),    # J1: shoulder pitch ±90°
    (-1.57, 1.57),    # J2: elbow pitch ±90°
    None,             # J3: unbounded yaw
    (-1.57, 1.57),    # J4: wrist pitch ±90°
    (-1.57, 1.57),    # J5: wrist Z ±90°
]

DOF_AXES = [
    np.array([0, 1, 0]),
    np.array([0, 0, 1]),
    np.array([0, 0, 1]),
    np.array([0, 1, 0]),
    np.array([0, 0, 1]),
    np.array([0, 0, 1]),
]

IS_Y_PIVOT = [True, False, False, True, False, False]


class fabrik_iteration_3d:
    def __init__(self, joints, target, active_joints=5):
        self.joints = [np.array([j.x, j.y, j.z]) for j in joints]
        self.target = np.array([target.x, target.y, target.z])
        self.n = len(joints)
        self.active = active_joints
        self.tol = 0.05
        self.origin = np.array([joints[0].x, joints[0].y, joints[0].z])
        self.lengths = [np.linalg.norm(np.array([joints[i].x, joints[i].y, joints[i].z]) -
                                       np.array([joints[i+1].x, joints[i+1].y, joints[i+1].z]))
                        for i in range(self.n - 1)]
        self.total_length = sum(self.lengths)

    # ── Pure FABRIK ──────────────────────────────────────────────

    def _pure_backward(self):
        self.joints[-1] = self.target
        for i in range(self.n - 2, -1, -1):
            d = self.joints[i+1] - self.joints[i]
            dn = np.linalg.norm(d)
            if dn < 1e-5:
                continue
            self.joints[i] = self.joints[i+1] - (d / dn) * self.lengths[i]

    def _pure_forward(self):
        self.joints[0] = self.origin
        for i in range(self.n - 1):
            d = self.joints[i+1] - self.joints[i]
            dn = np.linalg.norm(d)
            if dn < 1e-5:
                continue
            self.joints[i+1] = self.joints[i] + (d / dn) * self.lengths[i]
        self._lock_inactive()

    def _lock_inactive(self):
        if self.active >= self.n - 1:
            return
        d = self.joints[-1] - self.joints[self.active]
        dn = np.linalg.norm(d)
        if dn < 1e-5:
            return
        du = d / dn
        p = self.joints[self.active].copy()
        for i in range(self.active, self.n - 1):
            p = p + du * self.lengths[i]
            self.joints[i+1] = p

    # ── Ground clamp ────────────────────────────────────────────

    def _clamp_above_ground(self):
        for i in range(1, self.n):
            if self.joints[i][1] < 0:
                self.joints[i][1] = 0.0

    # ── Solve ───────────────────────────────────────────────────

    def _solve_pure(self):
        prev_d = float('inf')
        stalled = 0
        for it in range(200):
            self._pure_backward()
            self._pure_forward()
            d = np.linalg.norm(self.joints[-1] - self.target)
            if d < self.tol:
                break
            if d >= prev_d - 1e-6:
                stalled += 1
                if stalled > 10:
                    break
            else:
                stalled = 0
            prev_d = d

    def solve(self):
        if self.target[1] < 0:
            self.target[1] = 0.0

        if np.linalg.norm(self.target - self.origin) > self.total_length:
            for i in range(self.n - 1):
                d = self.target - self.joints[i]
                dn = np.linalg.norm(d)
                if dn < 1e-5:
                    continue
                l = self.lengths[i] / dn
                self.joints[i+1] = (1 - l) * self.joints[i] + l * self.target
            self._clamp_above_ground()
            return

        self._solve_pure()
        self._clamp_above_ground()
