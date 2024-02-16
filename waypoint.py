import math
from robot import *


def getBeta(waypoint, position):
    alpha = math.atan2(waypoint[1] - position[1], waypoint[0] - position[0])
    beta = alpha - position[2]
    return math.remainder(beta, math.tau)

def getDistance(waypoint, position):
    return math.hypot(waypoint[0] - position[0], waypoint[1] - position[1])

def navigateToWaypoint(waypoint, position):
    alpha = math.atan2(waypoint[1] - position[1], waypoint[0] - position[0])
    Robot.turn(getBeta(waypoint, position))
    Robot.forward(getDistance(waypoint, position))
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
