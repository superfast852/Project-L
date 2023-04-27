import cv2
from matplotlib import pyplot as plt
from skimage import morphology
import numpy as np

plt.rcParams["figure.figsize"] = [7.00, 3.50]
plt.rcParams["figure.autolayout"] = True
path = r'lmap2.png'
# Load the input image
img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
# Convert to binary
img_bin = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)[1]
bin_map = img_bin//255


def centerline(bin, type=int, return_dist=False):
    edt = cv2.distanceTransform(bin, cv2.DIST_L2, 5)
    skel = morphology.skeletonize(bin)*edt
    norm_skel = cv2.normalize(skel, None, 0, 1.0, cv2.NORM_MINMAX)
    if return_dist:
        return cv2.threshold(norm_skel, 0.1, 1.0, cv2.THRESH_BINARY)[1].astype(type), edt
    else:
        return cv2.threshold(norm_skel, 0.1, 1.0, cv2.THRESH_BINARY)[1].astype(type)


skel, dist = centerline(img_bin, return_dist=True)
smooth_dist = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
overlay = img.copy()
smooth_dist[skel == 1] = 1
overlay[smooth_dist>0.1] = smooth_dist[smooth_dist > 0.1]*255

#plt.imshow(img_bin, cmap="gray", interpolation="none")
plt.imshow(overlay, cmap="magma", alpha=0.5)
plt.show()
plt.imshow(smooth_dist, cmap="gray")
plt.show()
"""
plt.imshow(skel)
plt.show()
plt.imshow(dist, cmap="gray", interpolation="none")
plt.show()
overlay = img.copy()
overlay[skel == 1] = 0
plt.imshow(img, cmap="gray", interpolation="none")
plt.imshow(overlay, alpha=0.5)
plt.show()
"""
