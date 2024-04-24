from time import sleep
from Robots.RM_HAL import Drive, MecanumKinematics, IMU
from pickle import dump

drive = Drive()
mk = MecanumKinematics()
imu = IMU()
directions, poses, readings = [], [], []
sequence = [(1, 0, 1, 0), (0, 1, 1, 0), (-1, 0, 1, 0), (0, -1, 1, 0)]
move = lambda x, y, p, t: directions.append(drive.drive(x, y, p, t))
pose = lambda: poses.append(mk.pose)
read_sense = lambda: readings.append(list(zip(*[imu.getGyro(), imu.getAccel(), imu.getMag()])))  # fed in by axis.
count = 0
i = 0
while True:
    while count < 100:
        move(*sequence[i])
        pose()
        read_sense()
        sleep(1/120)
    drive.brake()
    i += 1
    count = 0
    if i==4:
        break
    sleep(1)

with open("data.pkl", "wb") as f:
    dump([poses, directions, readings], f)