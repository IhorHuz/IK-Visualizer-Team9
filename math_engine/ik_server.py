import socket
import json
import math

# --- Helper Vector Class ---
class Vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    @property
    def magnitude(self):
        return math.hypot(self.x, self.y)
    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)
    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)
    def __mul__(self, scalar):
        return Vec2(self.x * scalar, self.y * scalar)
    def __rmul__(self, scalar):
        return self.__mul__(scalar)
    def to_list(self):
        return [self.x, self.y]

class FabrikChain:
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

print(f"Python FABRIK Server running on {HOST}:{PORT}...")

# Initial arm setup
initial_joints = [Vec2(500, 300), Vec2(650, 300), Vec2(800, 300), Vec2(950, 300)]

while True:
    try:
        data, address = server_socket.recvfrom(1024)
        message = json.loads(data.decode('utf-8'))
        print(f"Received from Godot: {message}")
        
        tx, ty = message.get("target_pos", [0, 0, 0])[0:2]
        target_vec = Vec2(tx, ty)
        
        chain = FabrikChain(initial_joints, target_vec)
        chain.solve()
        
        initial_joints = chain.joints
        
        response = {"positions": [j.to_list() for j in chain.joints]}
        server_socket.sendto(json.dumps(response).encode('utf-8'), address)
        
    except KeyboardInterrupt:
        print("\nShutting down server.")
        break
    except Exception as e:
        pass