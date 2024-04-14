import cv2
import socket
from _pickle import dumps, loads
from numpy import zeros

empty = zeros((480, 640, 3))
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.55.1", 9160))

while True:
    img = loads(sock.recv(2*21))
    cv2.imshow("hi", img)
    cv2.waitKey(1)
    sock.send(b'OK')