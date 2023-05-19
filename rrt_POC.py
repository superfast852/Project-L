from extensions.NavStack import RRT, Map
from rrtplanner.plots import plot_og
from matplotlib.pyplot import subplots, show
from numpy import load

map = Map("random")
map.map = load("map.npy")
planner = RRT(n=500)
fig, ax = subplots(1, 1)
start = 0, 799
goal = 775, 10

print(start, goal)
plot_og(ax, map.map)
coords = planner.plan(start, goal, map)
for coord in coords:
    print(map.isValidPoint(coord[0]), map.isValidPoint(coord[1]))
""" Plot the lines. """
for line in coords:
    ax.plot((line[0][0], line[1][0]), (line[0][1], line[1][1]), color="red")

""" Plot the start and goal points. """
ax.scatter(start[0], start[1], color="green")
ax.scatter(goal[0], goal[1], color="blue")
show()
