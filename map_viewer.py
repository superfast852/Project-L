from _pickle import load
from matplotlib.pyplot import imshow, show
from extensions.NavStack import Map

map = Map("random")
print(map.map)
imshow(map.map, cmap="Greys")
show()
