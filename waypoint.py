import math
from robot import *


def navigateToWaypoint(waypoint, position):
    vector = (waypoint[0] - position[0], waypoint[1] - position[1])

    alpha = math.atan2(vector[1], vector[0])
    beta = alpha - position[2]
    Robot.turn(math.remainder(beta, math.tau))

    distance = math.hypot(vector[0], vector[1])
    Robot.forward(distance)

    return waypoint + (alpha,)


def inputWaypoint():
    while True:
        try:
            x, y = input("Target: ").split(",")
            return float(x), float(y)
        except ValueError:
            pass


if __name__ == "__main__":
    try:
        position = (0, 0, 0)
        while True:
            position = navigateToWaypoint(inputWaypoint(), position)
            print(
                f"\t(x: {position[0]:.1f}, y: {position[1]:.1f}, theta: {position[2] * 180 / math.pi:.1f}°/{position[2]:.2f}rad)"
            )
    except KeyboardInterrupt:
        pass
    BP.reset_all()
