import socket
import json
import math
import numpy as np

from stick.ccd_3d import ccd_iteration_3d
from stick.jacobian_3d import jacobian_iteration_3d
from stick.fabrik_3d import fabrik_iteration_3d

from robot.ccd_robot_3d import CcdSolver3d as CcdRobot
from robot.jacobian_robot_3d import jacobian_iteration_3d as jacobian_robot
from robot.fabrik_robot_3d import fabrik_iteration_3d as fabrik_robot


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
SEGMENT_LENGTHS = [1.2, 3.1, 0.85, 1.3, 5.25, 0.825]
JOINT_COUNT = len(SEGMENT_LENGTHS) + 1
MAX_REACH = sum(SEGMENT_LENGTHS)

ACTIVE_JOINTS = 6

DOF_AXES = ["Y", "Z", "Z", "Y", "Z", "Z"]

def build_vertical_chain(segment_lengths):
    joints = [Vec3(0, 0, 0)]
    y = 0.0
    for length in segment_lengths:
        y += length
        joints.append(Vec3(0, y, 0))
    return joints

def get_rotation_matrix(axis, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    if axis == "Y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == "Z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    elif axis == "X":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return np.eye(3)

def compute_joint_angles(joints):
    angles = []
    current_R = np.eye(3)

    for i in range(len(SEGMENT_LENGTHS)):
        if i >= ACTIVE_JOINTS:
            angles.append(0.0)
            continue

        axis = DOF_AXES[i]

        if axis == "Y":
            if i + 1 < len(joints):
                w_world = np.array([
                    joints[-1].x - joints[i+1].x,
                    joints[-1].y - joints[i+1].y,
                    joints[-1].z - joints[i+1].z
                ])
                wn = np.linalg.norm(w_world)
                if wn > 1e-8:
                    w_local = np.dot(current_R.T, w_world / wn)
                    w_local[1] = 0.0
                    pn = np.linalg.norm(w_local)
                    if pn > 1e-8:
                        angle = math.atan2(w_local[0], w_local[2])
                    else:
                        angle = 0.0
                else:
                    angle = 0.0
            else:
                angle = 0.0
        else:
            v_world = np.array([
                joints[i+1].x - joints[i].x,
                joints[i+1].y - joints[i].y,
                joints[i+1].z - joints[i].z
            ])
            v_local = np.dot(current_R.T, v_world)
            v_local[2] = 0.0
            pn = np.linalg.norm(v_local)
            if pn > 1e-8:
                v_local = v_local / pn
                angle = math.atan2(v_local[0], v_local[1])
            else:
                angle = 0.0

        angles.append(float(angle))
        current_R = np.dot(current_R, get_rotation_matrix(axis, angle))

    return angles


# --- Angle limit table (mirrors fabrik_robot_3d) ---
ANGLE_LIMITS = [
    None,             # J0: unbounded yaw
    (-1.57, 1.57),    # J1: shoulder pitch ±90°
    (-1.57, 1.57),    # J2: elbow pitch ±90°
    None,             # J3: unbounded yaw
    (-1.57, 1.57),    # J4: wrist pitch ±90°
    (-1.57, 1.57),    # J5: wrist Z ±90°
]


def fk_from_angles(angles):
    joints = [np.array([0.0, 0.0, 0.0])]
    R = np.eye(3)
    for i in range(len(SEGMENT_LENGTHS)):
        angle = angles[i] if i < len(angles) else 0.0
        axis = DOF_AXES[i]
        if axis == "Y":
            world_dir = np.array([0.0, 1.0, 0.0])
            R = np.dot(R, get_rotation_matrix("Y", angle))
        else:
            local_dir = np.array([np.sin(angle), np.cos(angle), 0.0])
            world_dir = np.dot(R, local_dir)
            R = np.dot(R, get_rotation_matrix(axis, angle))
        joints.append(joints[-1] + world_dir * SEGMENT_LENGTHS[i])
    return joints


def clamp_angles(angles):
    clamped = list(angles)
    for i in range(len(clamped)):
        lim = ANGLE_LIMITS[i] if i < len(ANGLE_LIMITS) else None
        if lim is not None:
            clamped[i] = np.clip(clamped[i], lim[0], lim[1])
    return clamped

def wrap_angles(angles):
    wrapped = list(angles)
    for i in range(len(wrapped)):
        lim = ANGLE_LIMITS[i] if i < len(ANGLE_LIMITS) else None
        if lim is None:
            wrapped[i] = math.atan2(math.sin(wrapped[i]), math.cos(wrapped[i]))
    return wrapped


# --- Solver dispatch ---
def solve_single(message, initial_joints, accumulated_angles=None):
    tx, ty, tz = message.get("target_pos", [0, 0, 0])
    target_vec = Vec3(tx, ty, tz)

    algorithm_choice = message.get("algo", "FABRIK")
    mode = message.get("mode", "STICK")

    if accumulated_angles is None:
        accumulated_angles = [0.0] * ACTIVE_JOINTS

    if message.get("reset", False):
        initial_joints = build_vertical_chain(SEGMENT_LENGTHS)
        accumulated_angles = [0.0] * ACTIVE_JOINTS

    base_pos = initial_joints[0]
    vector_to_target = target_vec - base_pos
    distance_to_target = vector_to_target.magnitude

    if distance_to_target > MAX_REACH:
        clamped_vector = vector_to_target * ((MAX_REACH - 0.01) / distance_to_target)
        target_vec = base_pos + clamped_vector
        tx, ty, tz = target_vec.x, target_vec.y, target_vec.z

    if mode == "ROBOT":
        if algorithm_choice == "CCD":
            robot = CcdRobot(initial_joints, target_vec, initial_angles=accumulated_angles)
            result, accumulated_angles = robot.solve()
            accumulated_angles = wrap_angles(accumulated_angles)
            new_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in result]
            angles = list(accumulated_angles)
            clamped = clamp_angles(angles)
        elif algorithm_choice == "FABRIK":
            chain = fabrik_robot(initial_joints, target_vec, active_joints=ACTIVE_JOINTS)
            chain.solve()
            new_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in chain.joints]
            angles = compute_joint_angles(new_joints)
            clamped = clamp_angles(angles)
        elif algorithm_choice == "JACOBIAN":
            joints_list = [j.to_list() for j in initial_joints]
            result = jacobian_robot(joints_list, [tx, ty, tz], method='dls', active_joints=ACTIVE_JOINTS)
            new_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in result]
            angles = compute_joint_angles(new_joints)
            clamped = clamp_angles(angles)
        else:
            new_joints = initial_joints
            angles = compute_joint_angles(new_joints)
            clamped = clamp_angles(angles)
    else:
        if algorithm_choice == "FABRIK":
            chain = fabrik_iteration_3d(initial_joints, target_vec)
            chain.solve()
            new_joints = [Vec3(float(j.x), float(j.y), float(j.z)) for j in chain.joints]
        elif algorithm_choice == "CCD":
            joints_list = [j.to_list() for j in initial_joints]
            result = ccd_iteration_3d(joints_list, [tx, ty, tz])
            new_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in result]
        elif algorithm_choice == "JACOBIAN":
            joints_list = [j.to_list() for j in initial_joints]
            result = jacobian_iteration_3d(joints_list, [tx, ty, tz], method='dls')
            new_joints = [Vec3(float(j[0]), float(j[1]), float(j[2])) for j in result]
        else:
            new_joints = initial_joints
        angles = compute_joint_angles(new_joints)
        clamped = clamp_angles(angles)

    positions = [j.to_list() for j in new_joints]
    return new_joints, positions, clamped, accumulated_angles


if __name__ == '__main__':
    HOST = '127.0.0.1'
    PORT = 5005

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind((HOST, PORT))

    print(f"Python 3D IK Server running on {HOST}:{PORT}...")
    print(f"  Joints: {JOINT_COUNT} ({' → '.join(f'{l}' for l in SEGMENT_LENGTHS)} units)")
    print(f"  Max reach: {MAX_REACH}")

    initial_joints = build_vertical_chain(SEGMENT_LENGTHS)
    accumulated_angles = [0.0] * ACTIVE_JOINTS

    while True:
        try:
            data, address = server_socket.recvfrom(1024)
            message = json.loads(data.decode('utf-8'))
            initial_joints, positions, angles, accumulated_angles = solve_single(
                message, initial_joints, accumulated_angles)
            response = {"positions": positions, "angles": angles}
            server_socket.sendto(json.dumps(response).encode('utf-8'), address)
        except KeyboardInterrupt:
            print("\nShutting down server.")
            break
        except Exception as e:
            print(f"Server error: {e}")
