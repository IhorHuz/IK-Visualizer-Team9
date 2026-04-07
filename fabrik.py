class FabrikChain:
    def __init__(self, joints, target):
        self.joints = joints[:]
        self.target = target
        self.n = len(joints)
        self.tol = 0.1
        self.origin = joints[0]

        self.lengths = [
            (joints[i] - joints[i+1]).magnitude
            for i in range(self.n - 1)
        ]
        self.total_length = sum(self.lengths)

    def backward(self):
        self.joints[-1] = self.target

        for i in range(self.n - 2, -1, -1):
            r = self.joints[i+1] - self.joints[i]
            dist = r.magnitude

            if dist == 0:
                continue

            l = self.lengths[i] / dist
            self.joints[i] = (1 - l) * self.joints[i+1] + l * self.joints[i]

    def forward(self):
        self.joints[0] = self.origin

        for i in range(self.n - 1):
            r = self.joints[i+1] - self.joints[i]
            dist = r.magnitude

            if dist == 0:
                continue

            l = self.lengths[i] / dist
            self.joints[i+1] = (1 - l) * self.joints[i] + l * self.joints[i+1]

    def solve(self):
        if (self.target - self.origin).magnitude > self.total_length:
            for i in range(self.n - 1):
                r = self.target - self.joints[i]
                dist = r.magnitude
                l = self.lengths[i] / dist
                self.joints[i+1] = (1 - l) * self.joints[i] + l * self.target
            return

        for _ in range(20):
            self.backward()
            self.forward()

            if (self.joints[-1] - self.target).magnitude < self.tol:
                break
