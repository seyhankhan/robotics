import math
import sys
from robot import *

try:
    if len(sys.argv) == 1:
        for i in range(4):
            Robot.forward(40)
            Robot.turn(math.pi / 2)
    else:
        if len(sys.argv) > 2:
            Robot.turn(math.radians(int(sys.argv[2])))
        Robot.forward(int(sys.argv[1]))

except KeyboardInterrupt:
    pass
BP.reset_all()
