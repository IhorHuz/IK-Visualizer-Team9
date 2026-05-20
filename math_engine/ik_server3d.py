import socket
import json
import math

from ccd_3d import ccd_iteration_3d
from jacobian_3d import jacobian_iteration_3d
from fabrik_3d import fabrik_iteration_3d
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

# --- The UDP Server ---
HOST = '127.0.0.1'
PORT = 5005
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((HOST, PORT))

print(f"Python 3D FABRIK Server running on {HOST}:{PORT}...")

initial_joints = [Vec3(x * 5, 0, 0) for x in range(10)]
# initial_joints = [Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(10, 0, 0), Vec3(15, 0, 0)]

while True:
    try:
        data, address = server_socket.recvfrom(1024)
        message = json.loads(data.decode('utf-8'))
        # print(f"Received from Godot: {message}")
        
        tx, ty, tz = message.get("target_pos", [0, 0, 0])
        target_vec = Vec3(tx, ty, tz)
        
        algorithm_choice = message.get("algo", "FABRIK")
        
        max_reach = sum((initial_joints[i+1] - initial_joints[i]).magnitude for i in range(len(initial_joints)-1))
        
        base_pos = initial_joints[0]
        vector_to_target = target_vec - base_pos
        distance_to_target = vector_to_target.magnitude
        
        if distance_to_target > max_reach:
            clamped_vector = vector_to_target * ((max_reach - 0.01) / distance_to_target)
            target_vec = base_pos + clamped_vector
            
            tx, ty, tz = target_vec.x, target_vec.y, target_vec.z
        
        if algorithm_choice == "FABRIK":
            chain = fabrik_iteration_3d(initial_joints, target_vec)
            chain.solve()
            initial_joints = chain.joints
            positions_to_send = [j.to_list() for j in initial_joints]
            
        elif algorithm_choice == "CCD":
            joints_list = [j.to_list() for j in initial_joints]
            new_joints = ccd_iteration_3d(joints_list, [tx, ty, tz])
            initial_joints = [Vec3(j[0], j[1], j[2]) for j in new_joints]
            positions_to_send = new_joints
            
        elif algorithm_choice == "JACOBIAN":
            joints_list = [j.to_list() for j in initial_joints]
            new_joints = jacobian_iteration_3d(joints_list, [tx, ty, tz], method='dls')
            initial_joints = [Vec3(j[0], j[1], j[2]) for j in new_joints]
            positions_to_send = new_joints
        
        response = {"positions": positions_to_send}
        server_socket.sendto(json.dumps(response).encode('utf-8'), address)
        
    except KeyboardInterrupt:
        print("\nShutting down server.")
        break
    except Exception as e:
        pass