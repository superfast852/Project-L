from time import sleep
from Robots.RM_HAL import Drive, MecanumKinematics, IMU
from pickle import dump
import atexit

drive = Drive()
atexit.register(drive.brake)
mk = MecanumKinematics()
imu = IMU()
directions, poses, readings = [], [], []
sequence = [(1, 0, 1, 0), (0, 1, 1, 0), (-1, 0, 1, 0), (0, -1, 1, 0)]


def log():
    directions.append(drive.get_directions())
    poses.append(mk.pose)
    readings.append(list(zip(*[imu.getGyro(), imu.getAccel(), imu.getMag()])))  # fed in by axis.


count = 0
i = 0
while True:
    while count < 100:
        drive.drive(*sequence[i])
        log()
        count += 1
        sleep(1/120)
    drive.brake()
    i += 1
    count = 0
    if i == 4:
        break
    sleep(1)
drive.brake()
with open("data.pkl", "wb") as f:
    dump([poses, directions, readings], f)