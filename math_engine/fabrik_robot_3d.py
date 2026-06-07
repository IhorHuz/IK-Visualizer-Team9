import numpy as np

class fabrik_iteration_3d:
    def __init__(self, joints, target):
        self.joints = [np.array([j.x, j.y, j.z]) for j in joints]
        self.target = np.array([target.x, target.y, target.z])
        self.n = len(joints)
        self.tol = 0.1
        self.origin = np.array([joints[0].x, joints[0].y, joints[0].z])
        self.lengths = [np.linalg.norm(np.array([joints[i].x, joints[i].y, joints[i].z]) -
                                       np.array([joints[i+1].x, joints[i+1].y, joints[i+1].z]))
                        for i in range(self.n - 1)]
        self.total_length = sum(self.lengths)

        # DOF: J0=Y, J1=Z, J2=Z, J3=Z, J4=Y
        self.dof_axes = [
            np.array([0, 1, 0]),
            np.array([0, 0, 1]),
            np.array([0, 0, 1]),
            np.array([0, 0, 1]),
            np.array([0, 1, 0]),
        ]

    def constrain(self, i, parent_pos, child_pos):
        axis = self.dof_axes[i] if i < len(self.dof_axes) else np.array([0, 0, 1])
        v = child_pos - parent_pos
        v_proj = v - np.dot(v, axis) * axis
        norm = np.linalg.norm(v_proj)

        if norm < 1e-5:
            neutral = np.array([0, 1, 0]) if axis[1] == 0 else np.array([1, 0, 0])
            return parent_pos + neutral * self.lengths[i]

        return parent_pos + (v_proj / norm) * self.lengths[i]

    def backward(self):
        self.joints[-1] = self.target
        for i in range(self.n - 2, -1, -1):
            r = self.joints[i+1] - self.joints[i]
            dist = np.linalg.norm(r)
            if dist == 0: continue
            l = self.lengths[i] / dist
            self.joints[i] = (1 - l) * self.joints[i+1] + l * self.joints[i]

    def forward(self):
        self.joints[0] = self.origin
        for i in range(self.n - 1):
            r = self.joints[i+1] - self.joints[i]
            dist = np.linalg.norm(r)
            if dist == 0: continue
            l = self.lengths[i] / dist
            target_pos = (1 - l) * self.joints[i] + l * self.joints[i+1]
            self.joints[i+1] = self.constrain(i, self.joints[i], target_pos)

    def solve(self):
        if np.linalg.norm(self.target - self.origin) > self.total_length:
            for i in range(self.n - 1):
                r = self.target - self.joints[i]
                dist = np.linalg.norm(r)
                l = self.lengths[i] / dist
                self.joints[i+1] = (1 - l) * self.joints[i] + l * self.target
                self.joints[i+1] = self.constrain(i, self.joints[i], self.joints[i+1])
            return
        for _ in range(20):
            self.backward()
            self.forward()
            if np.linalg.norm(self.joints[-1] - self.target) < self.tol:
                break
