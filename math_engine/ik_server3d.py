import socket
import json
import math
import numpy as np

from ccd_3d import ccd_iteration_3d
from jacobian_3d import jacobian_iteration_3d
from fabrik_3d import fabrik_iteration_3d

from ccd_robot_3d import ccd_iteration_3d as ccd_robot
from jacobian_robot_3d import jacobian_iteration_3d as jacobian_robot
from fabrik_robot_3d import fabrik_iteration_3d as fabrik_robot


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


# --- CONFIGURATION ---
SEGMENT_LENGTHS = [3.455, 6.103, 1.463, 8.280, 2.699]
JOINT_COUNT = len(SEGMENT_LENGTHS) + 1
MAX_REACH = sum(SEGMENT_LENGTHS)

DOF_AXES = ["Y", "Z", "Z", "Z", "Y"]

def build_vertical_chain(segment_lengths):
    joints = [Vec3(0, 0, 0)]
    y = 0.0
    for length in segment_lengths:
        y += length
        joints.append(Vec3(0, y, 0))
    return joints

def get_rotation_matrix(axis, angle):
    # axis is "Y" or "Z"
    c = math.cos(angle)
    s = math.sin(angle)
    if axis == "Y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == "Z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.eye(3)

def compute_joint_angles(joints):
    angles = []
    current_R = np.eye(3)

    for i in range(len(SEGMENT_LENGTHS)):
        v_world = np.array([
            joints[i+1].x - joints[i].x,
            joints[i+1].y - joints[i].y,
            joints[i+1].z - joints[i].z
        ])

        v_local = np.dot(current_R.T, v_world)

        axis = DOF_AXES[i]

        if axis == "Y":
            angle = math.atan2(v_local[0], v_local[2])
        elif axis == "Z":
            angle = math.atan2(v_local[0], v_local[1])

        angles.append(float(angle))

        current_R = np.dot(current_R, get_rotation_matrix(axis, angle))

    return angles


# --- The UDP Server ---
HOST = '127.0.0.1'
PORT = 5005

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((HOST, PORT))

print(f"Python 3D IK Server running on {HOST}:{PORT}...")
print(f"  Joints: {JOINT_COUNT} ({' → '.join(f'{l}' for l in SEGMENT_LENGTHS)} units)")
print(f"  Max reach: {MAX_REACH}")

initial_joints = build_vertical_chain(SEGMENT_LENGTHS)

while True:
    try:
        data, address = server_socket.recvfrom(1024)
        message = json.loads(data.decode('utf-8'))

        tx, ty, tz = message.get("target_pos", [0, 0, 0])
        target_vec = Vec3(tx, ty, tz)

        algorithm_choice = message.get("algo", "FABRIK")
        mode = message.get("mode", "STICK")

        base_pos = initial_joints[0]
        vector_to_target = target_vec - base_pos
        distance_to_target = vector_to_target.magnitude

        if distance_to_target > MAX_REACH:
            clamped_vector = vector_to_target * ((MAX_REACH - 0.01) / distance_to_target)
            target_vec = base_pos + clamped_vector
            tx, ty, tz = target_vec.x, target_vec.y, target_vec.z

        if mode == "ROBOT":
            if algorithm_choice == "FABRIK":
                chain = fabrik_robot(initial_joints, target_vec)
                chain.solve()
                initial_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in chain.joints]
                positions_to_send = [[float(coord) for coord in j] for j in chain.joints]
            elif algorithm_choice == "CCD":
                joints_list = [j.to_list() for j in initial_joints]
                new_joints = ccd_robot(joints_list, [tx, ty, tz])
                initial_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in new_joints]
                positions_to_send = [[float(coord) for coord in j] for j in new_joints]
            elif algorithm_choice == "JACOBIAN":
                joints_list = [j.to_list() for j in initial_joints]
                new_joints = jacobian_robot(joints_list, [tx, ty, tz], method='dls')
                initial_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in new_joints]
                positions_to_send = [[float(coord) for coord in j] for j in new_joints]
        else: # STICK mode
            if algorithm_choice == "FABRIK":
                chain = fabrik_iteration_3d(initial_joints, target_vec)
                chain.solve()
                initial_joints = [Vec3(float(j.x), float(j.y), float(j.z)) for j in chain.joints]
                positions_to_send = [j.to_list() for j in initial_joints]
            elif algorithm_choice == "CCD":
                joints_list = [j.to_list() for j in initial_joints]
                new_joints = ccd_iteration_3d(joints_list, [tx, ty, tz])
                initial_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in new_joints]
                positions_to_send = [[float(coord) for coord in j] for j in new_joints]
            elif algorithm_choice == "JACOBIAN":
                joints_list = [j.to_list() for j in initial_joints]
                new_joints = jacobian_iteration_3d(joints_list, [tx, ty, tz], method='dls')
                initial_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in new_joints]
                positions_to_send = [[float(coord) for coord in j] for j in new_joints]

        angles_to_send = compute_joint_angles(initial_joints)
        response = {
            "positions": positions_to_send,
            "angles": angles_to_send
        }
        server_socket.sendto(json.dumps(response).encode('utf-8'), address)

    except KeyboardInterrupt:
        print("\nShutting down server.")
        break
    except Exception as e:
        print(f"Server error: {e}")
