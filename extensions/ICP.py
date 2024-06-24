import numpy as np
import faiss


def best_fit_transform(A, B):
    # assert A.shape == B.shape
    m = A.shape[1]
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:
        Vt[m - 1, :] *= -1
        R = np.dot(Vt.T, U.T)
    t = centroid_B.reshape(-1, 1) - np.dot(R, centroid_A.reshape(-1, 1))
    T = np.eye(m + 1)
    T[:m, :m] = R
    T[:m, -1] = t.ravel()
    return T

def nearest_neighbor(src, dst):
    # assert src.shape == dst.shape

    # Ensure arrays are C-contiguous
    src = np.ascontiguousarray(src.astype(np.float32))
    dst = np.ascontiguousarray(dst.astype(np.float32))

    index = faiss.IndexFlatL2(dst.shape[1])
    index.add(dst)
    distances, indices = index.search(src, 1)
    return distances.ravel(), indices.ravel()


def iterative_closest_point(data, target, max_iterations=20, tolerance=0.001):
    assert len(data.shape) == len(target.shape)
    assert data.shape[1] == target.shape[1] == 2
    m = data.shape[1]
    src = np.ones((m + 1, data.shape[0]))
    dst = np.ones((m + 1, target.shape[0]))
    src[:m, :] = np.copy(data.T)
    dst[:m, :] = np.copy(target.T)
    nn_dst = dst[:m, :].T
    prev_error = 0

    for i in range(max_iterations):
        distances, indices = nearest_neighbor(src[:m, :].T, nn_dst)
        T = best_fit_transform(src[:m, :].T, dst[:m, indices].T)
        src = np.dot(T, src)
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    T = best_fit_transform(data, src[:m, :].T)
    return T


def transform_points(T, points):
    R, t = T[0:-1, 0:-1], T[0:-1, -1]
    return np.dot(R, points.T).T + t


def transform_to_pose(T):
    R, t = T[0:-1, 0:-1], T[0:-1, -1]
    return np.array([t[0], t[1], np.arctan2(R[1, 0], R[0, 0])])


if __name__ == "__main__":
    from time import perf_counter
    from matplotlib import pyplot as plt
    import numpy as np
    from extensions.tools import gen_atsushi, gen_sines

    dev = 0.05
    tg_x_shift = 0
    n_pts = 1000
    translation, rotation = np.array([1, 1]), np.pi / 2
    for i in range(100):

        #target, scan = gen_sines(n_pts, 0, 0, 3.14159265/2, np.array([2, 3]))
        target, scan = gen_atsushi()

        st = perf_counter()
        t = iterative_closest_point(scan, target, 100, 0.001)
        print("Framerate: ", 1 / (perf_counter() - st))

    # Plot the process

    # Before

    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(15, 5))
    ax[0].scatter(*target.T, label='Original')
    ax[0].scatter(*scan.T, label='Noised')

    ax[1].scatter(*target.T, label='Original')
    ax[1].scatter(*transform_points(t, scan).T, label='Transformed')
    ax[0].legend()
    ax[1].legend()
    plt.show()
