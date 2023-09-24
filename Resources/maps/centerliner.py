import cv2
from matplotlib import pyplot as plt
from skimage import morphology
import numpy as np
from _pickle import load

# TODO: Turn this into a library. Please.
# TODO: Turn centerline pixel coords into lines.
plt.rcParams["figure.autolayout"] = True
fig, ax = plt.subplots(2, 2)
path = "classroom.pkl"  # "classroom.pkl"
# Load the input image
if path.endswith(".pkl"):
    with open(path, "rb") as f:
        bmap = load(f)[1]
    img = np.array(bmap).reshape(int(len(bmap)**0.5), int(len(bmap)**0.5))
else:
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


def skeleton2coords(map):
    line = []
    for n, i in enumerate(map):
        for n2, j in enumerate(i):
            if j:
                line.append((n2, n))
    return line


skel, dist = centerline(img_bin, return_dist=True)
smooth_dist = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)
overlay = img.copy()
smooth_dist[skel == 1] = 1
overlay[smooth_dist > 0.1] = smooth_dist[smooth_dist > 0.1]*255

ax[0,0].imshow(overlay, cmap="magma", alpha=0.5)
ax[0,1].imshow(smooth_dist, cmap="gray")
coords = skeleton2coords(skel)
ax[1,0].scatter(*zip(*coords))
ax[1,0].invert_yaxis()
ax[1,1].imshow(skel)
plt.show()

