from robot import *
import sys
import math

WAYPOINTS = [
    (84, 30),
    (180, 30),
    (180, 54),
    (138, 54),
    (138, 168),
    (114, 168),
    (114, 84),
    (84, 84),
    (84, 30),
]

def inputWaypoint():
    while True:
        try:
            x, y = input("Target: ").split(",")
            return float(x), float(y)
        except ValueError:
            pass


if __name__ == "__main__":
    
    try:
        Robot.setPosition(WAYPOINTS[0]+(0,))
        for w in WAYPOINTS[1:]:
            Robot.navigateToWaypoint(w)
            # input()
    except KeyboardInterrupt:
        pass
    BP.reset_all()

    # x = 0 if len(sys.argv) < 2 else int(sys.argv[1])
    # y = 0 if len(sys.argv) < 3 else int(sys.argv[2])
    # theta = 0 if len(sys.argv) < 4 else math.radians(int(sys.argv[3]))
    # try:
    #     Robot.setPosition((x, y, theta))
    #     while True:
    #         Robot.navigateToWaypoint(inputWaypoint())
    #         print(Robot())
    # except KeyboardInterrupt:
    #     pass
    # BP.reset_all()
