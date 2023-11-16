import math
import numpy as np
from extensions.tools import getAngle



def turnCalc(pose, target):
    # Calculate the desired orientation for the robot to point to the target
    desired_orientation = getAngle(target[0] - pose[0], target[1] - pose[1])
    # Calculate the angular difference between the robot's current orientation and the desired orientation
    angular_difference = desired_orientation - pose[2]
    # Adjust the angular difference for wrap-around at 2*pi
    while angular_difference > math.pi:
        angular_difference -= 2 * math.pi
    while angular_difference < -math.pi:
        angular_difference += 2 * math.pi
    # Determine the turn command based on the angular difference (reversed directions)
    return max(min(-angular_difference, 1), -1)


def pf_diff(pose, target):
    pose = np.array(pose)
    target = np.array(target)
    xy = pose[:2]
    z = pose[2]

    diff_xy = target - xy  # Get point difference
    # Regulate to always be capped at 1 in any direction
    norm = np.linalg.norm(diff_xy)
    reg_xy = diff_xy / norm if norm != 0 else diff_xy  # Note: the abs might fuck things up. TODO: Please check.
    desired_theta = getAngle(*reg_xy)
    theta_diff = desired_theta - z

    # Wrap the angle difference to the range [-pi, pi]
    theta_diff = (theta_diff + math.pi) % (2 * math.pi) - math.pi
    return max(min(-theta_diff, 1), -1)


if __name__ == "__main__":
    pose = (0, 0, 0)
    target = lambda target_angle: (math.cos(math.radians(target_angle)), math.sin(math.radians(target_angle)))
    angles = [target(i) for i in range(361)]
    for angle in angles:
        a = turnCalc(pose, angle)
        b = pf_diff(pose, angle)
        if abs(a-b) > 0.1:
            print(f"Error found. {a} {b} {angle}")
