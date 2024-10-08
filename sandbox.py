import cv2
import numpy as np


def hypot(shape):
    return np.sqrt(shape[0]**2 + shape[1]**2)


def warpMatrix(sz, theta, phi, gamma, scale, fovy):

    st = np.sin(np.radians(theta))
    ct = np.cos(np.radians(theta))
    sp = np.sin(np.radians(phi))
    cp = np.cos(np.radians(phi))
    sg = np.sin(np.radians(gamma))
    cg = np.cos(np.radians(gamma))

    halfFovy = fovy*0.5
    d = hypot(sz)
    sideLength = scale*d/np.cos(np.radians(halfFovy))
    h = d/(2.0*np.sin(np.radians(halfFovy)))
    n = h-(d/2.0)
    f = h+(d/2.0)

    F = np.zeros((4, 4))  # 4x4 transformation matrix F

    Rtheta = np.eye(4)    # 4x4 rotation matrix around Z-axis by theta degrees
    Rphi = np.eye(4)      # 4x4 rotation matrix around X-axis by phi degrees
    Rgamma = np.eye(4)    # 4x4 rotation matrix around Y-axis by gamma degrees
    T = np.eye(4)         # 4x4 translation matrix along Z-axis by -h units
    P = np.zeros((4, 4))  # Allocate 4x4 projection matrix

    # Rtheta
    Rtheta[0, 0] = Rtheta[1, 1] = ct
    Rtheta[0, 1] = -st
    Rtheta[1, 0] = st

    # Rphi
    Rphi[1, 1] = Rphi[2, 2] = cp
    Rphi[1, 2] = -sp
    Rphi[2, 1] = sp

    # Rgamma
    Rgamma[0, 0] = Rgamma[2, 2] = cg
    Rgamma[0, 2] = -sg
    Rgamma[2, 0] = sg

    # T
    T[2, 3] = -h

    # P
    P[0, 0] = P[1, 1] = 1.0/np.tan(np.radians(halfFovy))
    P[2, 2] = -(f+n)/(f-n)
    P[2, 3] = -(2.0*f*n)/(f-n)
    P[3, 2] = -1.0

    # Compose transformations
    F = np.dot(np.dot(np.dot(np.dot(P, T), Rphi), Rtheta), Rgamma)

    # Transform 4x4 points
    halfW = sz[1]/2
    halfH = sz[0]/2

    ptsIn = np.array([-halfW, halfH, 0,
                      halfW, halfH, 0,
                      halfW, -halfH, 0,
                      -halfW, -halfH, 0])

    ptsInMat = np.reshape(ptsIn, (4, 1, 3))

    ptsOutMat = cv2.perspectiveTransform(ptsInMat, F)  # Transform points

    ptsInPt2f = np.zeros((4, 2)).astype('float32')
    ptsOutPt2f = np.zeros((4, 2)).astype('float32')

    for i in range(4):
        ptsInPt2f[i] = ptsInMat[i, 0, :2] + np.array([halfW, halfH])
        ptsOutPt2f[i] = (ptsOutMat[i, 0, :2] + np.ones(2)) * (sideLength * 0.5)

    return cv2.getPerspectiveTransform(ptsInPt2f, ptsOutPt2f)


def warpImage(src, theta, phi, gamma, scale, fovy):

    halfFovy = fovy*0.5
    d = hypot(src.shape)
    sideLength = int(scale*d/np.cos(np.radians(halfFovy)))

    # Compute warp matrix
    M = warpMatrix(src.shape, theta, phi, gamma, scale, fovy)

    # Do actual image war0
    return cv2.warpPerspective(src, M, (sideLength, sideLength))


def main():

    cap = cv2.VideoCapture(0)
    i = 0
    while(True):

        ok, frame = cap.read()

        if not ok:
            print('*** ERROR: No camera detected ***')
            exit(1)

        # yz, xz, xy rotations,
        frame = warpImage(frame, 0, i/10,0,1,30)
        i += 1
        print(i)
        cv2.imshow('frame', frame)

        if cv2.waitKey(1) == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':

    main()