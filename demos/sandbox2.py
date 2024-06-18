from extensions.NavStack import RRT, Map
from extensions.tools import line2dots
import numpy as np
from time import time, sleep
map = Map("random")
planner = RRT(map, 500, 15, 20)


def customWithin(points: np.ndarray, x: np.ndarray, r: float) -> np.ndarray:
    # Compute squared distances using vectorized operations and broadcasting
    squared_distances = np.sum((points - x) ** 2, axis=1)

    # Find indices of points within the radius
    within_radius = squared_distances < r ** 2

    # Return points within radius
    return points[within_radius]


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


def pointwisePathOptimizer(path, r):
    endpoints = planner.getEndPoints(path)
    print(f"Endpoints: {endpoints.tolist()}")
    if len(endpoints) == 2:
        return path
    for i, point in enumerate(endpoints[1:-1]):
        searchable = endpoints[i+1:]
        print(f"\nIteration {i}: ")
        print(f"\tSearch Space: {searchable.tolist()}")
        print(f"\tReference Point: {point}")
        nearby, segmented = getNearbyPoints(searchable, point, r)
        # fuck lines, just work with endpoints. Figure out what you cut off and stitch the line back together.
        # Please do a manual example so that you can wrap your head around it.
        if nearby.size > 0:
            if len(nearby) > 1:
                candidate = nearby[np.argmax(np.linalg.norm(nearby-point, axis=1))]
            else:
                candidate = nearby[0]
            # ok so we get an available stitching candidate.
            # now how do we integrate it into our path?
            # Data:
            # endpoints
            # reference point
            # candidate
            # Search Space

            print(f"\tCandidate: {candidate}")
            location = search2D(candidate, segmented)
            print(f"\tLocation: {i+location[0]}")
            print(f"\tVerification: {searchable[location[0]]}, {point}, {endpoints[i+location[0]+1]}")
        else:
            print("No points found nearby.")


def getNearbyPoints(searchable, point, r):
    # it's a 2dim set, so no searching here.
    dots = []
    _2d_dots = []
    for line in planner.endpointsToPath(searchable):  # turn back into a path for it to cooperate with l2c
        n = line2dots(*line)
        dots += n
        _2d_dots.append(n)
    dots = np.array(dots)
    return np.array([i for i in customWithin(dots, point, r) if planner.planner.collisionfree(np.rot90(planner.map.map), point, i)]), _2d_dots


def search2D(point, arr):
    for i, seg in enumerate(arr):
        idx = idxPoint(point, seg)
        if idx != -1:
            print("Found a match!")
            print(f"Segment: {seg[i]}")
            return i, idx
    return -1, -1


while True:
    map.paths = []
    img = map.tocv2()
    path = planner.plan(*map.getValidRoute(2))
    if not planner.isValidPath(path) or len(path) < 4:
        continue
    print(len(planner.getEndPoints(path)))
    map.addPath(path)
    pointwisePathOptimizer(path, 25)
    #map.addPath(optimized)
    #[map.drawPoint(img, i, 4, (0, 255, 0)) for i in planner.getEndPoints(optimized)]
    map.animate(img)
    sleep(5)

