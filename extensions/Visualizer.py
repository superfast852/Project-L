from networktables import NetworkTables
from threading import Thread


class UnityVisualizer:
    def __init__(self, ip="127.0.0.1", mapHandle = None, botHandle = None):
        NetworkTables.initialize(ip)
        self.mapTable = NetworkTables.getTable("map")
        self.botTable = NetworkTables.getTable("bot")
