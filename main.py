#from robot import Robot
from controller import XboxController

#robot = Robot()
joy = XboxController(0.15)
exit_state = False
mode_state = False
mode = 0
while True:
    sig, exit_state = joy.edge(joy.Start, exit_state)
    change, mode_state = joy.edge(joy.LB, mode_state)
    if sig:
        #robot.exit()
        print("Exit Called!")
        break
    if change:
        mode = not mode
    if mode:
        print("Teleop")
        #robot.teleop(joy)
    else:
        print("Autonomous")
        #robot.autonomous()