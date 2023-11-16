from numpy import array
with open("data", "r") as f:
    data = f.read()

data = data.split("\n")
data_points = array([eval(point) for point in data])

"""
Ok, so our data. We have about 19 datapoints, including speed and ticks. Each datapoint was collected 1 second apart.
We know:
delta = 1 second
rpm = 12000
final_rpm = 333.33
"""
rpm = 11000
reduction = 30
motors = array([[data_points[:, 0, i].tolist(), data_points[:, 1, i].tolist()] for i in range(3)])
motor_data = []
for i, motor in enumerate(motors):
    data = []
    print(f"Motor: {i}")
    speeds, ticks = motor

    # Ticks per Second
    tps = round(sum(speeds)/len(speeds), 6)
    data.append(tps)
    print(f"Average Speed: {tps} ticks per second")

    # Tick Increment per second (mostly verifying)
    increment = [ticks[i + 1] - ticks[i] for i in range(len(ticks) - 1)]
    avg_increment = round(sum(increment) / len(increment), 6)
    data.append(avg_increment)
    print(f"Increments: {avg_increment}")
    print()
    # Ticks per Rotation and Rotations per Tick.
    rps = round(rpm / 60, 6)
    print(f"Rotations per second: {rps}")
    print(f"If there were {rps} rotations per second, and we had {tps} ticks per second...")
    print(f"TPR: {round(tps/(rps), 7)}, RPT: {round(rps/tps, 7)}")
    data.append(tps/rps)

    rps = (rpm/reduction) / 60
    print(f"Reduced TPR: {round(tps/rps, 7)}, Reduced RPT: {round(rps/tps, 7)}")
    data.append(tps/rps)
    print("\n\n\n")
    motor_data.append(data)

motor_data = array(motor_data)
avg = lambda x: sum(x)/len(x)
print(f"Average TPS: {avg(motor_data[:,0])}")
print(f"Average Inc: {avg(motor_data[:,1])}")
print(f"Average TPR: {avg(motor_data[:,2])}")
print(f"Average R-TPR: {avg(motor_data[:,3])}")