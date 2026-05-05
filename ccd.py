import math

def ccd_iteration(joints, target):

    for i in range(len(joints) - 2, -1, -1):
        tip = joints[-1]
        current_joint = joints[i]

        dx_tip = tip[0] - current_joint[0]
        dy_tip = tip[1] - current_joint[1]

        dx_target = target[0] - current_joint[0]
        dy_target = target[1] - current_joint[1]

        angle_to_tip = math.atan2(dy_tip, dx_tip)
        angle_to_target = math.atan2(dy_target, dx_target)

        rotation_angle = angle_to_target - angle_to_tip

        for j in range(i + 1, len(joints)):
            qx = joints[j][0] - current_joint[0]
            qy = joints[j][1] - current_joint[1]

            new_x = qx * math.cos(rotation_angle) - qy * math.sin(rotation_angle)
            new_y = qx * math.sin(rotation_angle) + qy * math.cos(rotation_angle)

            joints[j] = (current_joint[0] + new_x, current_joint[1] + new_y)

    return joints
