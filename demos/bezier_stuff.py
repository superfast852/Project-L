# TODO: Upgrade the bezier curve to take into account obstacles.
# To do this, add the closest points to obstacles as control points.
# NOTE: The bezier curve don't seem to work under bresenham lines.

from extensions.NavStack import RRT, Map
from time import time, sleep
single = 1
map = Map("random")
planner = RRT(map, 500, 15, 20)


def drawSmoothPath(img, path):
    # Extract the second element of each line, and add to the first spot the starting point.
    endpoints = planner.getEndPoints(path)
    #print(endpoints)
    curve = planner.smoothPath(path)
    for point in endpoints[1:-1]:
        map.drawPoint(img, point, 4)
    for point in [endpoints[0], endpoints[-1]]:
        map.drawPoint(img, point, 4, (255, 0, 0))
    #print(curve)
    map.drawLineOfDots(img, curve)
    map.animate(img, drawLines="points")


while True:
    start = time()
    img = map.tocv2()
    if single:
        start, stop = (1, 450), (0, 0)
        path = planner.plan(start, stop)
        print(path)
        drawSmoothPath(img, path)
    else:
        route = map.getValidRoute(3) + [(1, 450)]
        print(route)
        chain = planner.chainroute(route)
        print(chain)
        for path in chain:
            drawSmoothPath(img, path)
        for point in route:
            map.drawPoint(img, point, 4)
    print(f"Time: {time()-start} seconds.")