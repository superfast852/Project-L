from extensions.tools import find_port_by_vid_pid
from rplidar import RPLidar, RPLidarException, MAX_MOTOR_PWM
from extensions.NavStack import SLAM, Map
from breezyslam.sensors import RPLidarA1
from matplotlib import pyplot as plt
from operator import itemgetter
from itertools import groupby
from time import sleep, perf_counter
from threading import Thread
import numpy as np
import atexit
import math


class RP_A1(RPLidarA1):
    VID = 0x10c4
    PID = 0xea60

    def __init__(self, com="/dev/ttyUSB2", baudrate=115200, timeout=3, rotation=0, scan_type="normal", threaded=True):
        super().__init__()
        try:
            port = find_port_by_vid_pid(self.VID, self.PID)
            self.lidar = RPLidar(port, baudrate, timeout)
        except RPLidarException:
            self.lidar = RPLidar(com, baudrate, timeout)
        print(self.lidar.get_info(), self.lidar.get_health())
        self.t = threaded
        self.lidar.motor_speed = MAX_MOTOR_PWM//2
        self.lidar.clean_input()
        self.scanner = self.lidar.iter_scans(scan_type, False, 10)

        next(self.scanner)
        self.rotation = rotation % 360
        if self.t:
            self.latest = [[0], [0]]
            self.scans = Thread(target=self.threaded_read, daemon=True)
            self.scans.start()
        # last_scan = self.read()  # Return this for when we must clear the buffer if we don't read fast enough.

    def threaded_read(self):
        try:
            while self.t:
                self.latest = self.read()
        except RPLidarException:
            pass

    def read(self, rotate=False):  # It's totally possible to move this to a thread.
        items = next(self.scanner)
        angles, distances = list(zip(*items))[1:]
        if rotate:
            distances, angles = list(zip(*self.rotate_lidar_readings(zip(distances, angles))))
        return [list(distances), list(angles)]

    def exit(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        if self.t:
            self.t = False
            self.scans.join()

    def autoStopCollision(self, collision_threshold):
        # This returns only the clear angles.
        distance, angle = self.getScan()
        clear = []
        for i, dist in enumerate(distance):
            if dist > collision_threshold:
                clear.append(angle[i])
        return clear

    def getScan(self):
        return self.latest if self.t else self.read()

    def self_nav(self, collision_angles, collision_threshold, lMask=(180, 271), rMask=(270, 361)):
        self.map = [collision_threshold-1]*360
        distances, angles = self.getScan()
        for i, angle in enumerate(angles):
            self.map[int(angle)] = distances[i]

        if min(self.map[collision_angles[0]:collision_angles[1]]) < collision_threshold:
            ls_clear = []
            rs_clear = []

            for i, distance in enumerate(self.map[rMask[0]:rMask[1]]):  # Right Side Check
                if distance > collision_threshold:  # If the vector_distance is greater than the threshold
                    rs_clear.append(i + rMask[0])  # append the angle of the vector to the list of available angles

            for i, distance in enumerate(self.map[lMask[0]:lMask[1]]):  # Left Side Check
                if distance > collision_threshold:
                    ls_clear.append(i + lMask[0])

            # From the list, extract all numerical sequences (e.g. [3, 5, 1, 2, 3, 4, 5, 8, 3, 1] -> [[1, 2, 3, 4, 5]])
            ls_sequences = self._extract_sequence(ls_clear)
            rs_sequences = self._extract_sequence(rs_clear)

            # Find the longest sequence, as there might be multiple sequences.
            # If there were no sequences, the longest sequence would be 0.
            ls_space = max(ls_sequences, key=len) if len(ls_sequences) > 0 else [0, 0]
            rs_space = max(rs_sequences, key=len) if len(rs_sequences) > 0 else [0, 0]

            if len(ls_space) > len(rs_space):  # If the left side has more space
                return "Left", ls_space  # Return the direction and the angles of the obstacle for future optional odometry.
            else:  # If the right side has more space
                return "Right", rs_space  # Return the direction and the angles of the obstacle for future optional odometry.
        else:
            return "Forward", self.map[collision_angles[0]:collision_angles[1]]

    @staticmethod
    def _extract_sequence(array):
        sequences = []
        for k, g in groupby(enumerate(array), lambda x: x[0] - x[1]):
            seq = list(map(itemgetter(1), g))
            sequences.append(seq) if len(seq) > 1 else None
        return sequences

    def rotate_lidar_readings(self, readings):
        """
        Rotate lidar angle readings by the specified angle while keeping distances intact.

        Args:
            readings (list): List of tuples containing distances and angles.

        Returns:
            list: Rotated lidar readings with preserved distances.
        """
        if self.rotation % 360 == 0:
            return readings
        else:
            return [(readings[i][0], (readings[i][1] + self.rotation + 360) % 360) for i in range(len(readings))]


def measure_speed():
    '''Main function'''
    lidar = RP_A1(scan_type="express")
    old_t = None
    data = []
    try:
        print('Press Ctrl+C to stop')
        for _ in lidar.scanner:
            now = perf_counter()
            if old_t is None:
                old_t = now
                continue
            delta = now - old_t
            print('%.2f Hz, %.2f RPM' % (1/delta, 60/delta))
            data.append(delta)
            old_t = now
    except KeyboardInterrupt:
        print('Stopping. Computing mean...')
        lidar.exit()
        delta = sum(data)/len(data)
        print('Mean: %.2f Hz, %.2f RPM' % (1/delta, 60/delta))


if __name__ == "__main__":
    from os.path import exists
    lidar = RP_A1(threaded=True)
    atexit.register(lidar.exit)
    name = ""
    if name:
        if exists(name):  # Navigating a made map
            map = Map(name)
            slam = SLAM(lidar, map, update_map=False)
        else:  # Creating a new map
            map = Map(800)
            slam = SLAM(lidar, map)
    else:  # Just that instance
        map = Map(800)
        slam = SLAM(lidar, map)
    while True:
        try:
            start = perf_counter()
            distances, angles = lidar.latest

            pose = list(slam.update(distances, angles))
            pose[2] *= 3.14159 / 180

            map.animate(None, pose)
            print(f"FPS: {1/(perf_counter() - start)}")
            #sleep(1/60)
        except RPLidarException:
            break
        except KeyboardInterrupt:
            if name and not exists(name):
                map.save(name)

else:
    measure_speed()