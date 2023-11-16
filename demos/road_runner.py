# from extensions.NavStack import RRT
import noise
import numpy as np


def create_map(dims, ceil=100, clumping=110, **kwargs):
    return np.array([[noise.pnoise2(x/clumping, y/clumping, **kwargs)*ceil for x in range(dims[0])] for y in range(dims[1])])


# Uses:
#   - Project depths in 3d
#   - Primitive way to enable depth-based rrt
# Note: The algorithm converts the depth map into binary obstacles for the RRT* Algorithm. Let's try it out!
#   For the algorithm to work, the tolerance must be the absolute height difference of the target point.
def map2og(map, start=None, stop=None, default=0.5):
    if start is not None and stop is not None:
        startP, stopP = abs(map[start[1]][start[0]]), abs(map[stop[1]][stop[0]])
        selection = startP if startP >= stopP else stopP
        if selection < 0.2:
            selection = abs(map).mean()
    else:
        selection = default
    dims = map.shape
    return np.array([[1 if abs(map[y][x]) >= selection else 0 for x in range(dims[1])] for y in range(dims[0])])


if __name__ == "__main__":
    from extensions.NavStack import RRT, Map, random_point_og, line2dots
    import time
    mpl = 0
    ply = 1
    rpl = 0.5
    if mpl:
        from matplotlib import pyplot as plt
    elif ply:
        from plotly import graph_objects as go
    depthMap = create_map((800, 800), 1, 500)
    start_og = map2og(depthMap)  # To get a starting point to pick a random point in the map.
    # Note: what if I free the valid point calc from map, and avoid this step entirely?

    # The random point pick doesn't work because of height tolerance readjustment.
    start, stop = tuple(random_point_og(start_og)), tuple(random_point_og(start_og))
    # start, stop = (191, 103), (369, 99)
    map = Map(map2og(depthMap, start, stop))  # Reload the map with the updated height restrictions.
    rrt = RRT(map)

    while True:
        if abs(depthMap[start]) >= rpl or abs(depthMap[stop]) >= rpl:
            continue
        map.paths = []
        map.update(map2og(depthMap, start, stop))
        img = map.tocv2()
        for i in range(5):
            map.animate(img)
        route = rrt.plan(start, stop)
        if route is None:
            pass
        else:
            print(len(route))
            print(route)
            map.addPath(route)
            if len(route) > 1:  # If there is more than 1 line:
                smooth = rrt.smoothPath(route)
                map.drawLineOfDots(img, smooth)
            else:
                map.drawLine(img, route[0])

        map.animate(img)
        if mpl and route is not None:  # TODO: Finish what you started.
            # Create X and Y coordinate arrays based on the shape of your depth array
            x = np.arange(0, depthMap.shape[0])
            y = np.arange(0, depthMap.shape[1])
            X, Y = np.meshgrid(x, y)

            # Create a 3D plot
            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.plot_surface(X, Y, depthMap, cmap='viridis')  # You can change the colormap as needed
            for line in route:  # The line is defined by 2 points
                dotted_line = line2dots(line)
                for i in range(0, len(dotted_line), 10):
                    dot = dotted_line[i]
                    print(depthMap[dot])
                    ax.scatter(dot[0], dot[1], depthMap[dot]+0.1)


            # Display both plots in separate windows
            ax.invert_yaxis()
            plt.show()
        elif ply and route is not None and len(route)>3:
            print(len(route))
            while True:
                try:
                    surface_plot = go.Figure(data=go.Surface(z=depthMap, colorscale='Viridis'))
                    break
                except ValueError:
                    pass
            for line in route:
                dots = np.array(line2dots(line))
                z_points = depthMap[dots[:,1], dots[:,0]]
                surface_plot.add_trace(
                    go.Scatter3d(x=dots[:, 0], y=dots[:, 1], z=z_points, mode='markers', marker=dict(size=3, color='red')))

            surface_plot.update_layout(title='Depth Map as 3D Surface Plot',
                                       scene=dict(xaxis_title='X', yaxis_title='Y', zaxis_title='Depth'))
            surface_plot.show()
            time.sleep(5)
        start, stop = tuple(random_point_og(map.map)), tuple(random_point_og(map.map))