import math


def smoothSpeed(current, target, speed_lim=1, min_speed=0.1, smoothing_spread=10):
    distance = current-target
    try:
        direction = -distance/abs(distance)
        speed = min(9, (distance/10)**2/smoothing_spread)
        output = (1+speed)*direction/10*speed_lim
        return output if abs(output) > min_speed else min_speed*direction
    except ZeroDivisionError:
        return 0


def getAngle(x, y):
    # Calculate the angle in radians and rotate it counterclockwise by 90 degrees
    # Angle grows counterclockwise
    if x == 0 and y == 0:
        return 0.0
    angle = math.degrees(math.atan2(y, x) - math.pi / 2)
    if angle < 0:
        angle += 360
    # Return the angle in degrees
    return round(angle)


def inTolerance(a, b, tol=1):
    return abs(a - b) <= tol


def getCoordinates(angle):
    # Convert the angle from degrees to radians
    angle = math.radians(angle)
    # Calculate the x and y coordinates
    x = round(math.cos(angle + math.pi/2), 3)
    y = round(math.sin(angle + math.pi/2), 3)
    # Return the coordinates as a tuple
    return x, y


if __name__ == "__main__":
    angle = list(range(0, 361, 10))
    for i in angle:
        x, y = getCoordinates(i)
        print(i, (x, y), getAngle(x, y))
