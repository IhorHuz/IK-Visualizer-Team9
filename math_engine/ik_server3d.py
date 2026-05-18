import socket
import json
import math

class Vec3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    @property
    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    def __sub__(self, other):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    def __mul__(self, scalar):
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)
    def __rmul__(self, scalar):
        return self.__mul__(scalar)
    def to_list(self):
        return [self.x, self.y, self.z]

class FabrikChain3D:
    def __init__(self, joints, target):
        self.joints = joints[:]
        self.target = target
        self.n = len(joints)
        self.tol = 0.1
        self.origin = joints[0]
        self.lengths = [(joints[i] - joints[i+1]).magnitude for i in range(self.n - 1)]
        self.total_length = sum(self.lengths)

    def backward(self):
        self.joints[-1] = self.target
        for i in range(self.n - 2, -1, -1):
            r = self.joints[i+1] - self.joints[i]
            dist = r.magnitude
            if dist == 0: continue
            l = self.lengths[i] / dist
            self.joints[i] = (1 - l) * self.joints[i+1] + l * self.joints[i]

    def forward(self):
        self.joints[0] = self.origin
        for i in range(self.n - 1):
            r = self.joints[i+1] - self.joints[i]
            dist = r.magnitude
            if dist == 0: continue
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

# --- The UDP Server ---
HOST = '127.0.0.1'
PORT = 5005
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((HOST, PORT))

print(f"Python 3D FABRIK Server running on {HOST}:{PORT}...")

# 3D Initial arm setup (X, Y, Z)
initial_joints = [Vec3(x * 5, 0, 0) for x in range(10)]

while True:
    try:
        data, address = server_socket.recvfrom(1024)
        message = json.loads(data.decode('utf-8'))
        
        # Grab all 3 coordinates!
        tx, ty, tz = message.get("target_pos", [0, 0, 0])
        target_vec = Vec3(tx, ty, tz)
        
        chain = FabrikChain3D(initial_joints, target_vec)
        chain.solve()
        
        initial_joints = chain.joints
        
        response = {"positions": [j.to_list() for j in chain.joints]}
        server_socket.sendto(json.dumps(response).encode('utf-8'), address)
        
    except KeyboardInterrupt:
        print("\nShutting down server.")
        break
    except Exception as e:
        pass