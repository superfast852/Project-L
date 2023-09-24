from _pickle import load
from matplotlib.pyplot import imshow, show
from extensions.NavStack import Map

map = Map("classroom.pkl")
imshow(map.map, cmap="Greys")
show()
