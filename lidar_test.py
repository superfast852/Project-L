from async_interface import Lidar
import time

lidar = Lidar(simOverride=True)
try:
    for i in range(100):
        print(f"{i}: {lidar.read()}")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")

lidar.exit()