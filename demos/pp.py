import numpy as np
import math
import matplotlib.pyplot as plt

# Parameters
look_forward_gain = 0.1  # look forward gain
look_ahead_distance = 0.2  # [m] look-ahead distance
WB = 2.9  # [m] wheel base of vehicle
DT = 0.1  # [s] time tick
T = 500.0  # max simulation time


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class PurePursuitController:
    def __init__(self, look_forward_gain, look_ahead_distance):
        self.k = look_forward_gain
        self.Lfc = look_ahead_distance
        self.last_target_idx = None
        self.WB = WB

    def get_steering_control(self, state, cx, cy):
        if self.is_endpoint_in_lookahead_circle(state.x, state.y, cx[-1], cy[-1]):
            return None, None
        target_idx, Lf = self._search_target_index(state, cx, cy)

        if target_idx < len(cx):
            tx = cx[target_idx]
            ty = cy[target_idx]
        else:
            tx = cx[-1]
            ty = cy[-1]
            target_idx = len(cx) - 1

        state.yaw = math.atan2(ty - state.y, tx - state.x)

        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
        delta = math.atan2(2.0 * self.WB * math.sin(alpha) / Lf, 1.0)
        # Normalized delta to the range [0, 1]
        normalized_delta = (delta + math.pi) / (2 * math.pi)
        print(normalized_delta)
        return normalized_delta, target_idx

    def _search_target_index(self, state, cx, cy):
        if self.last_target_idx is None:
            dx = [state.x - icx for icx in cx]
            dy = [state.y - icy for icy in cy]
            d = np.hypot(dx, dy)
            target_idx = np.argmin(d)
            self.last_target_idx = target_idx
        else:
            target_idx = self.last_target_idx
            distance_this_index = np.hypot(state.x - cx[target_idx], state.y - cy[target_idx])

            while (target_idx + 1) < len(cx):
                distance_next_index = np.hypot(state.x - cx[target_idx + 1], state.y - cy[target_idx + 1])
                if distance_this_index < distance_next_index:
                    break
                distance_this_index = distance_next_index
                target_idx += 1

        Lf = self.k * 1.0 + self.Lfc  # Assuming constant velocity of 1.0 for Lf calculation
        while (target_idx + 1) < len(cx) and Lf > np.hypot(state.x - cx[target_idx], state.y - cy[target_idx]):
            target_idx += 1

        self.last_target_idx = target_idx
        return target_idx, Lf

    def is_endpoint_in_lookahead_circle(self, current_x, current_y, end_x, end_y):
        distance = np.sqrt((current_x - end_x) ** 2 + (current_y - end_y) ** 2)
        return distance <= self.Lfc


# Create target path
cx = np.arange(0, np.pi*8, 0.1)
cy = [2*math.sin(ix) for ix in cx]

# Initialize the controller and robot state
controller = PurePursuitController(look_forward_gain, look_ahead_distance)
robot = State(x=-0.0, y=-3.0, yaw=0.0)
history_x = []
history_y = []

# Simulation loop
for _ in np.arange(0.0, T, DT):
    normalized_delta, _ = controller.get_steering_control(robot, cx, cy)
    if normalized_delta is None:
        print("ye got there")
        break
    V_left = 1 - normalized_delta
    V_right = normalized_delta

    robot_speed = (V_left + V_right) / 2.0
    robot.yaw += (V_right - V_left) / controller.WB * DT
    robot.x += robot_speed * math.cos(robot.yaw) * DT
    robot.y += robot_speed * math.sin(robot.yaw) * DT

    history_x.append(robot.x)
    history_y.append(robot.y)

# Plotting
plt.plot(cx, cy, "-r", label="Target path")
plt.plot(history_x, history_y, "-b", label="Robot trajectory")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Pure Pursuit Controller Simulation with T=200")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()
