from extensions.NavStack import PathFollow, array
import matplotlib.pyplot as plt


def visualize_trajectory(trajectory, waypoints):
    """
    Visualizes the trajectory of the robot along with the waypoints.

    Parameters:
    - trajectory: List of robot poses [(x1, y1, theta1), (x2, y2, theta2), ...]
    - waypoints: List of waypoints [[x1, y1], [x2, y2], ...]
    """
    # Extracting X, Y coordinates from trajectory and waypoints
    traj_x, traj_y, _ = zip(*trajectory)
    wp_x, wp_y = zip(*waypoints)

    plt.figure(figsize=(10, 6))
    plt.plot(traj_x, traj_y, '-o', label='Robot Trajectory', markersize=4)
    plt.scatter(wp_x, wp_y, c='red', marker='x', label='Waypoints')
    plt.title('Robot Trajectory and Waypoints')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.show()


# Define initial robot pose and waypoints
initial_pose = [0, 0, 0]  # [x, y, theta]
waypoints = array([[[0, 0], [5, 5]], [[5, 5], [10, 0]], [[10, 0], [15, 5]]])  # Example waypoints

# Initialize PathFollow
path_follower = PathFollow(waypoints)

# Run the simulation
trajectory = [initial_pose]
pose = initial_pose
for _ in range(1000):  # Simulating for 100 time steps
    x_speed, y_speed, theta_speed = path_follower(pose)
    # Update pose (for simplicity, we'll just add the speeds to current pose)
    pose = [pose[0] + x_speed, pose[1] + y_speed, pose[2] + theta_speed]
    trajectory.append(pose)

# Visualize the results
visualize_trajectory(trajectory, [wp[1] for wp in waypoints])

