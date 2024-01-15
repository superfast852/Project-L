from extensions.tools import xyz_matrix, np
import freenect
import cv2
from extensions.Detector import Detector
#detector = Detector("../Resources/yolov7-tiny-nms.trt")


def getCoordinate(u, v, depth):
    """
    Get the 3D world coordinates (XYZ) of the given pixel coordinates (u, v)
    Parameters:
      u, v - Pixel coordinates
      depth - Depth data from the Kinect sensor
    Returns:
      xyz - 3D world coordinates in meters
    """
    # Ensure u and v are arrays
    u, v = np.array([u]), np.array([v])

    # Get the depth value at (u, v)
    depth_value = depth[v, u]

    # Convert to 3D world coordinates
    C = np.vstack((u.flatten(), v.flatten(), depth_value.flatten(), np.ones_like(u.flatten())))
    X, Y, Z, W = np.dot(xyz_matrix(), C)
    X, Y, Z = X / W, Y / W, Z / W

    return np.array([X[0], Y[0], Z[0]])


def convert_depth_to_uint8(depth_image, max_depth=None):
    """
    Convert a uint16 depth image to uint8 for display with OpenCV.
    Parameters:
        depth_image: numpy array of uint16 depth data.
        max_depth: maximum depth value for normalization. If None, uses the max value in the image.
    Returns:
        depth_image_8bit: depth image converted to uint8.
    """
    if max_depth is None:
        max_depth = np.max(depth_image)

    # Normalize the depth image to float values between 0 and 1
    normalized_depth = depth_image / max_depth

    # Scale to uint8
    depth_image_8bit = (normalized_depth * 255).astype(np.uint8)

    return depth_image_8bit


coords = 100, 150
# Example usage
while cv2.waitKey(1) != ord('q'):
    rgb = cv2.cvtColor(freenect.sync_get_video()[0], cv2.COLOR_RGB2BGR)
    depth = convert_depth_to_uint8(freenect.sync_get_depth()[0])
    cv2.drawMarker(rgb, coords, (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
    print(getCoordinate(*coords, depth.copy()))
    #dets = detector(rgb)
    #if dets is not None:
    #    dets.draw(rgb)
    #    print(dets)
    #else:
    #    continue
    cv2.imshow("hi", rgb)
    cv2.imshow("depth", cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR))

cv2.destroyAllWindows()