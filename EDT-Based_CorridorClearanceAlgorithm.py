from extensions.NavStack import Map, RRT
from scipy.ndimage import distance_transform_edt
import numpy as np
import cv2

map = Map("random")
rrt = RRT(map)
img1 = map.tocv2()
edt = distance_transform_edt(np.logical_not(np.rot90(map.map)).astype(int))
img2 = cv2.normalize(edt, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)

dst = cv2.addWeighted(img1, 0.5, img2, 0.7, 0)

img_arr = np.hstack((img1, img2))
cv2.imshow('Input Images',img_arr)
cv2.imshow('Blended Image',dst)

points = (map.getValidPoint(), map.getValidPoint())
path = rrt.plan(*points)
map.addPath(path)
map.animate(dst)

"""
Notes:
Ok, so using this algorithm can bring us a lot of centering benefits, its just a matter of making the line adjustment.
from now on, the river is the lines in the center that the edt algo makes.
Idea: Nearest Center Algorithm:
Make a circle around both start and finish, and find the nearest available "river" point.
Once you do that, pass the points to A* algorithm for a "Filling" effect.

"""

cv2.waitKey(0)
cv2.destroyAllWindows()
