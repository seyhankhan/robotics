from robot import *


def inputWaypoint():
    while True:
        try:
            x, y = input("Target: ").split(",")
            return float(x), float(y)
        except ValueError:
            pass


if __name__ == "__main__":
    try:
        Robot.setPosition((0, 0, 0))
        while True:
            Robot.navigateToWaypoint(inputWaypoint())
            print(Robot())
    except KeyboardInterrupt:
        pass
    BP.reset_all()
