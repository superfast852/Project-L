from extensions.NavStack import Map
import time
import numpy as np
import cv2


def gen_binary_cost(og, inflation_r=8.5, start=None, stop=None):
    '''
    Inflates the obstacles in the map to create a binary cost map. This is useful for path planning.
    :param og: The occupancy grid (map.map)
    :param inflation_r: The inflation radius around the obstacles. A higher radius mean more centered paths, but less likely to generate a path. A smaller one means shorter paths, but closer to walls.
    :param start: The start point of the path
    :param stop: The end point of the path
    :return: Inflated binary cost map
    '''
    if inflation_r < 0:
        raise ValueError("Inflation coefficient must be positive")

    cut = round(inflation_r)

    sample = np.logical_not(og).astype(np.uint8)
    edt = cv2.distanceTransform(sample, cv2.DIST_L2, 3)
    sample[edt < inflation_r] = 0
    out = np.logical_not(sample).astype(np.uint8)
    if start is not None:
        out[start[1]-cut:start[1]+cut, start[0]-cut:start[0]+cut] = 0
    if stop is not None:
        out[stop[1]-cut:stop[1]+cut, stop[0]-cut:stop[0]+cut] = 0
    return out

def test():
    from extensions.NavStack import RRT

    map = Map("random")
    inflated = Map(gen_binary_cost(map.map))
    planner = RRT(map, inflate=20)

    while cv2.waitKey(1) != ord('q'):
        start, stop = map.getValidRoute(2)
        s = time.time()
        path = planner.plan(start, stop)
        print("Time taken:", 1/(time.time()-s))

        map.addPath(path)
        inflated.addPath(path)
        inflated.animate(show="Inflated Obstacles.")
        map.animate()


def measure_inflation():
    img = np.ones((800, 800), np.uint8)*255
    cv2.circle(img, (400, 400), 200, 0, -1)
    map = Map(img)
    inflated = Map(gen_binary_cost(map.map))

    def on_track(val):
        c = val
        inflated.map = gen_binary_cost(map.map, inflation_coeff=c)
        img = inflated.tocv2()
        try:
            pass
            cv2.line(img, (400, 400), (400, 400+c+200), (0, 0, 255), 1)
        except Exception as e:
            print(e)
        cv2.imshow("Inflated Obstacles.", img)

    cv2.namedWindow("Inflated Obstacles.")
    cv2.createTrackbar("Inflation Coefficient", "Inflated Obstacles.", 0, 100, on_track)
    on_track(0)
    cv2.waitKey(0)


test()