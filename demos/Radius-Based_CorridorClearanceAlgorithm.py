"""

This code is a test for a Corridor Clearance and Path Optimization algorithm.
The corridor clearance checks for obstacles in a radius around the path, and
the path optimizer tries to remove unnecessary line segments in the path.

"""


from extensions.NavStack import RRT, Map
import numpy as np
from time import time, sleep
single = 1
map = Map("random")
planner = RRT(map, 500, 15, 20)


def customWithin(points: np.ndarray, x: np.ndarray, r: float) -> np.ndarray:
    # Compute squared distances using vectorized operations and broadcasting
    squared_distances = np.sum((points - x) ** 2, axis=1)

    # Find indices of points within the radius
    within_radius = squared_distances < r ** 2

    # Return points within radius
    return points[within_radius]


def sample_points_on_line(line_segment, d=None, p=None):
    # Extract start and end points
    start, end = np.array(line_segment[0]), np.array(line_segment[1])
    diff = end-start
    # Calculate the distance between the start and end points
    line_length = np.linalg.norm(end - start)
    if d is None:
        p = p if p is not None else 10
        d = line_length/p

    # Determine the number of points to sample
    num_samples = int(np.floor(line_length / d)) + 1

    # Compute the points
    coeff = d*diff/line_length
    sampled_points = np.round(np.array([i * coeff for i in range(num_samples)])+start)

    # Ensure the end point is included
    if not np.isclose(sampled_points[-1], end, atol=d/2).all():
        sampled_points = np.concatenate((sampled_points, np.expand_dims(end, 0)), 0)

    return np.unique(sampled_points, axis=0).astype(int)


def intervalObstacleCheck(map, path, r=10, img=None):
    obstacles = np.argwhere(map.map==1)
    neighbors = []
    for line in path:
        out = []
        samples = sample_points_on_line(line, r)  # arrays with shape Nx2, points.
        try:
            prev_sample = neighbors[-1][-1]
            if np.isclose(prev_sample, samples[0], atol=r/2).all() and prev_sample.size > 0:
                samples = samples[1:]
        except IndexError:
            pass

        for sample in samples:
            nearby = customWithin(obstacles, sample, r)
            if nearby.size > 0:
                out.append(nearby)  # also Nx2

                if img is not None:
                    #map.drawPoint(img, sample, r, (0, 0, 255))  # where you sampled it
                    for point in nearby:
                        map.drawPoint(img, point, 1, (0, 255, 0))  # the obstacles near the sample
        if out:
            neighbors.append(np.vstack(out))  # therefore the shape of neighbors is (lines, samples, obstacle_clumps, 2)
        else:
            neighbors.append(np.array([]))
    return neighbors


def pathOptimizer(path, img, r=25):
    unique_points = planner.getEndPoints(path)
    if len(unique_points) == 2:
        return path
    
    result_path = path
    for i, point in enumerate(unique_points):
        ## Sampling
        if i == len(unique_points)-1:
            continue
        testable = unique_points[i+1:]  # remove the current and previous points. They've already been analyzed.
        nearby = customWithin(testable, point, r)  # sample for the endpoints nearby
        # remove the points that have an obstacle between them.
        nearby = np.array([i for i in nearby if planner.planner.collisionfree(np.rot90(planner.map.map), point, i)])

        ## Candidate Selection
        if nearby.size > 0:  # This means that the path can be shortened.
            # get the best point to shorten to
            if len(nearby) > 1:
                print(f"Multiple nearby points: {nearby}")
                candidate = nearby[np.argmin(np.linalg.norm(nearby-point, axis=1))]
            else:
                print(f"One nearby point: {nearby}")
                candidate = nearby[0]


            ## Optimization
            # shorten the path
            shortened = shortenPath(unique_points, point, candidate)
            print("Found an optimization!")
            # run the optimizer on the shortened path
            result_path = pathOptimizer(planner.endpointsToPath(shortened), img, r)
            break  # leave
    return result_path


def idxPoint(point, array):
    try:
        return np.where(np.all(array == point, axis=1))[0][0]
    except IndexError:
        return -1


def shortenPath(endpoints, a, b):
    a_idx = idxPoint(a, endpoints)
    b_idx = idxPoint(b, endpoints)
    if a_idx == -1 or b_idx == -1:
        raise ValueError("Point not found in endpoints.")
    if a_idx > b_idx:
        a_idx, b_idx = b_idx, a_idx
    return np.concatenate((endpoints[:a_idx], endpoints[b_idx:]))


while True:
    map.paths = []
    tInit = time()
    img = map.tocv2()

    if single:
        # start, stop = (1, 450), (0, 0)
        start, stop = map.getValidRoute(2)
        path = planner.plan(start, stop)
        if not planner.isValidPath(path):
            continue
        map.addPath(path)
        #map.drawPoint(img, start, 4)
        #map.drawPoint(img, stop, 4)
        #a = intervalObstacleCheck(map, path, 25)
        # First dim is each line.
        # Second dim is all the obstacles found in that line
        # Third dim is the point itself.
        optimized = pathOptimizer(path, img, 25)
        intervalObstacleCheck(map, optimized, 25, img)
        map.addPath(path)
        for i in range(len(optimized)):
            map.drawLine(img, optimized[i], (0, 0, 255))
            map.drawPoint(img, optimized[i][0], 4, (255, 0, 0))
            map.drawPoint(img, optimized[i][1], 4, (255, 0, 0))

    else:
        route = map.getValidRoute(3)  # + [(1, 450)]
        chain = planner.chainroute(route)
        for path in chain:
            if not planner.isValidPath(path):
                continue
        for point in route:
            map.drawPoint(img, point, 10)
        map.addPath(chain)
    print(f"Time: {time()-tInit} seconds.")
    for i in range(5):
        map.animate(img)
    sleep(5)