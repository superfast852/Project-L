from rplidar import RPLidar as rpl, RPLidarException

lidar = rpl("/dev/ttyUSB0")

iterate = lidar.iter_scans(max_buf_meas=5)

for scan in iterate:
    print(len(scan))