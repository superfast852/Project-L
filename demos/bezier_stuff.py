# TODO: Upgrade the bezier curve to take into account obstacles.
# To do this, add the closest points to obstacles as control points.
# NOTE: The bezier curve don't seem to work under bresenham lines.

from extensions.NavStack import RRT, Map
from time import time, sleep
single = 0
map = Map("random")
planner = RRT(map, 500, 15, 20)


def drawSmoothPath(img, path):
    # Extract the second element of each line, and add to the first spot the starting point.
    endpoints = planner.getEndPoints(path)
    print(endpoints)
    curve = planner.smoothPath(path)
    for point in endpoints[1:-1]:
        map.drawPoint(img, point, 4)
    for point in [endpoints[0], endpoints[-1]]:
        map.drawPoint(img, point, 4, (255, 0, 0))
    print(curve)
    map.drawLineOfDots(img, curve)
    map.animate(img, drawLines="points")


while True:
    start = time()
    img = map.tocv2()
    if single:
        start, stop = map.getValidRoute(2)
        path = planner.plan(start, stop)
        drawSmoothPath(img, path)
    else:
        route = map.getValidRoute(3)
        chain = planner.chainroute(route)
        for path in chain:
            drawSmoothPath(img, path)
    print(f"Time: {time()-start} seconds.")
    sleep(5)