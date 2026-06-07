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


# --- CONFIGURATION ---
# Segment lengths for the 6-DOF robot arm (standing vertically along Y axis).
# J0 (base) at origin, each subsequent joint offset by the segment length.
SEGMENT_LENGTHS = [2.0, 5.0, 4.0, 0.5, 0.5]
# Labels for reference (not used in computation):
#   base=2.0, arm_1=5.0, arm_2=4.0, wrist_pitch=0.5, wrist_yaw/tool=0.5
# Results in 6 joints (J0..J5). Total reach = 12.0 units.

JOINT_COUNT = len(SEGMENT_LENGTHS) + 1
MAX_REACH = sum(SEGMENT_LENGTHS)


def build_vertical_chain(segment_lengths):
    joints = [Vec3(0, 0, 0)]
    y = 0.0
    for length in segment_lengths:
        y += length
        joints.append(Vec3(0, y, 0))
    return joints


def compute_joint_angles(joints):
    angles = []
    for i in range(1, len(joints) - 1):
        v1 = joints[i] - joints[i - 1]
        v2 = joints[i + 1] - joints[i]
        d1 = v1.magnitude
        d2 = v2.magnitude
        if d1 < 1e-8 or d2 < 1e-8:
            angles.append(0.0)
            continue
        dot = (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / (d1 * d2)
        dot = max(-1.0, min(1.0, dot))
        angles.append(math.acos(dot))
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

        base_pos = initial_joints[0]
        vector_to_target = target_vec - base_pos
        distance_to_target = vector_to_target.magnitude

        if distance_to_target > MAX_REACH:
            clamped_vector = vector_to_target * ((MAX_REACH - 0.01) / distance_to_target)
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