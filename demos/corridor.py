import numpy as np
from extensions.tools import evaluate_bezier, line2dots
from extensions.NavStack import Map, RRT, ndarray
from _pickle import load
from time import sleep

with open("./valid_path.pkl", "rb") as file:
    chain = load(file)

# Initialization.
map = Map("random")
rrt = RRT(map)
image = map.tocv2()
distance = 100
draw_closest = True
draw_furthest = True
map.animate(image, drawLines=False)
sleep(0.1)
map.animate(image, drawLines=False)
sleep(1)


def drawSmoothPath(path):
    # Extract the second element of each line, and add to the first spot the starting point.
    endpoints = np.append([path[0][0]], path[:, 1], 0)
    print(endpoints)
    for point in endpoints[1:-1]:
        map.drawPoint(image, point, 1)
    map.drawPoint(image, endpoints[0], 1, (255, 0, 0))
    map.drawPoint(image, endpoints[-1], 1, (255, 0, 0))

    curve = evaluate_bezier(endpoints, 50).astype(np.int16)
    map.drawLineOfDots(image, curve)
    map.animate(image)


# Self-Contained method
def getCorridors(path: ndarray, img: ndarray = None, distance: int = 100, params:dict = None):

    """

    :param path: A ndarray containing an RRT-Resultant path.
    :param img: A standard opencv 2d image in the form of (h, w, c)
    :param distance: The distance to search left and right (along the X-Axis always) of every point in the path
    :param params: Dictionary or list containing the drawing parameters, if you choose to draw the collisions.
    :return:
    (Furthest, Closest): Two lists containing the closest and furthest collision points relative to the line.
    A point located immediately on the line indicates that no obstacle was found in a certain direction.
    """

    if params is None:
        params = {'closest': True, 'furthest': True, 'cvdot': False, 'collision_only': True}
    draw_closest, draw_furthest, cvdot, collision_only = list(params.values()) if isinstance(params, dict) else params
    if not path.size:
        print("Found an invalid path!!")
        return 0, 0
    closest = rrt.find_corridor(path, distance, False)
    furthest = rrt.find_corridor(path, distance, True)
    dotted_line = [line2dots(line) for line in path]
    if img is not None:
        for i in range(len(path)):
            c_corridor = closest[i]
            f_corridor = furthest[i]
            dotted = dotted_line[i]
            for n in range(len(c_corridor)):  # Every dot is the furthest obstacle possible.
                if draw_closest:
                    if c_corridor[n][0] != dotted[n][0] or c_corridor[n][1] != dotted[n][1]:
                        if cvdot:
                            map.drawPoint(img, c_corridor[n], 1)
                        else:
                            map.drawPx(img, c_corridor[n])
                    else:
                        if not collision_only:
                            if cvdot:
                                map.drawPoint(img, c_corridor[n], 1)
                            else:
                                map.drawPx(img, c_corridor[n])

                if draw_furthest:
                    if f_corridor[n][0] != dotted[n][0] or f_corridor[n][1] != dotted[n][1]:
                        if cvdot:
                            map.drawPoint(img, f_corridor[n], 1, (255, 0, 0), 1)
                        else:
                            img = map.drawPx(img, f_corridor[n], (255, 0, 0))
                    else:
                        if not collision_only:
                            if cvdot:
                                map.drawPoint(img, f_corridor[n], 1, (255, 0, 0), 1)
                            else:
                                img = map.drawPx(img, f_corridor[n], (255, 0, 0))
    return closest, furthest


while True:
    chain = rrt.chainroute(map.getValidRoute(5))
    for path in chain:
        map.paths = []
        map.addPath(path)
        image = map.tocv2()
        close, far = getCorridors(path, image)
        # drawSmoothPath(path)
        map.animate(image)
        sleep(5)