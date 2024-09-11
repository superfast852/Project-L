import time
import numpy as np
import matplotlib.pyplot as plt
from Robots.RM_HAL import RP_A1, RPLidarException
from extensions.ICP import iterative_closest_point, transform_to_pose, transform_points
from scipy.spatial import KDTree


class AnimatedWindow:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.gca()
        self.figid = id(self.fig)

    def scatter(self, points):
        if len(points) > 0:
            self.ax.scatter(*zip(*points))

    def refresh(self):
        if id(plt.gcf()) != self.figid:
            raise ValueError("Window does not exist.")
        plt.draw()
        plt.pause(0.0001)

    def clear(self):
        self.ax.cla()


def preprocess_points(points):
    """ Preprocess the point cloud data: filtering and downsampling """
    points = np.array(points)
    # Filter out points that are too close or too far
    distances = np.linalg.norm(points, axis=1)
    mask = (distances > 100) & (distances < 3000)
    points = points[mask]
    # Downsample points using a voxel grid filter
    voxel_size = 50  # Example voxel size in mm
    voxel_grid = {}
    for point in points:
        voxel_key = tuple((point // voxel_size).astype(int))
        if voxel_key not in voxel_grid:
            voxel_grid[voxel_key] = point
    downsampled_points = np.array(list(voxel_grid.values()))
    return downsampled_points


def run():
    lidar = RP_A1(scan_type="express")
    window = AnimatedWindow()
    last_scan = preprocess_points(lidar.readCartesian())
    pose = np.array([0.0, 0.0, 0.0])
    dt = 0.1
    last_time = time.perf_counter()
    last_pose = np.array([0, 0, 0])
    last_v = np.array([0, 0, 0])
    v = np.array([0, 0, 0])

    try:
        while True:
            # Timing
            curr = time.perf_counter()
            dt = curr - last_time
            last_time = curr

            # Clear plot
            window.clear()

            # Data processing
            points = preprocess_points(lidar.readCartesian())

            # Use KDTree for fast nearest neighbor search
            tree = KDTree(last_scan)
            indices = tree.query(points, k=1)[1]
            u = iterative_closest_point(points, last_scan[indices], max_iterations=50,
                                                                tolerance=1e-5)

            x, y, t = transform_to_pose(u)
            transformed = transform_points(u, points)

            # Visualization
            window.scatter(last_scan)
            window.scatter(transformed)
            window.scatter(points)
            window.scatter(((x, y),))  # see the center
            window.refresh()

            # Pose and velocity estimation
            last_pose = pose.copy()
            pose += np.array([x, y, t])

            v = (pose - last_pose) / dt
            a = (v - last_v) / dt
            last_v = v.copy()

            # Extract 2D components
            pose_2d = pose[:2]
            last_pose_2d = last_pose[:2]

            # Calculate covariance matrix of 2D points
            cov_matrix = np.cov(last_scan[:, :2].T)
            cov_inv = np.linalg.inv(cov_matrix)

            # Mahalanobis distance
            delta = pose_2d - last_pose_2d
            mahalanobis_distance = np.sqrt(delta.T @ cov_inv @ delta)

            if mahalanobis_distance > 3:
                pose = last_pose
                print("Alignment quality poor, pose reset")
            else:
                print(
                    f"Pose: {pose.astype(float).round(2).tolist()}")

            last_scan = points

    except RPLidarException:
        pass
    except ValueError as e:
        if str(e) == "Window does not exist.":
            pass
        else:
            print(e)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.exit()


if __name__ == '__main__':
    run()