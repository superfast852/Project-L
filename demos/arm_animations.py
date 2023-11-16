import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
from matplotlib.animation import FuncAnimation, FFMpegFileWriter
from matplotlib import pyplot as plt

chain = ikpy.chain.Chain.from_urdf_file("../Resources/arm.urdf", active_links_mask=[bool(i%2) for i in range(10)])

def p2v(coords):
    d = np.linalg.norm(coords)
    azimuth = np.arctan2(coords[1], coords[0])
    elevation = np.arcsin(coords[2] / d)
    return d, azimuth, elevation


def v2p(vec):
    d, a, e = vec
    x = d * np.cos(a) * np.cos(e)
    y = d * np.sin(a) * np.cos(e)
    z = d * np.sin(e)
    return x, y, z


def moveTo(target, orientation=None, threshold=1, starting=None):
    joints = chain.inverse_kinematics(target, orientation, initial_position=starting)
    fk_check = chain.forward_kinematics(joints)[:3, 3]
    if abs(fk_check-target).all() > threshold:
        print("Warning: Could not reach the target.")
    return joints


fig, ax = plot_utils.init_3d_figure()
# Animation update function
prev_joints = np.array([0]*10)


def chain2pose(joints: np.ndarray):
    return np.append(((joints - [link.bounds[0] for link in chain.links])*180/np.pi).round()[1::2], [0])


def pose2chain(pose):
    return np.vstack(
        ([float('-inf')]*len(pose[:5]), pose[:5]*np.pi/180)
    ).reshape(-1, order='F') + [link.bounds[0] for link in chain.links]


checkpoints = []
arm_checks = []
xyz = False


def update(frame):
    global prev_joints

    if xyz:
        target = [1, frame/20, 0]
    else:
        vec = [frame/20, np.pi/2, np.pi/6]
        # can also be described as: Travel speed, base_servo angle (0, pi), incline rate
        target = v2p(vec)



    joints = chain.inverse_kinematics(target, initial_position=prev_joints)
    prev_joints = joints
    ax.clear()  # Clear the previous plot
    if joints is not None:
        chain.plot(joints, ax, target=target)
        print(chain2pose(joints))
    else:
        chain.plot([0]*10, ax, target=target)

    if not frame%5:
        checkpoints.append(target)
        arm_checks.append(chain.forward_kinematics(joints)[:3, 3])

    checklist = np.array(checkpoints)
    arm_list = np.array(arm_checks)
    ax.scatter(checklist[:, 0], checklist[:, 1], checklist[:, 2])
    ax.scatter(arm_list[:, 0], arm_list[:, 1], arm_list[:, 2])


# Create animation
ani = FuncAnimation(fig, update, frames=100, interval=1/60)
FFWriter = FFMpegFileWriter(fps=30)
ani.save("arm_move.mp4", writer=FFWriter)
plt.show()