import brickpi3
from time import sleep
from robot import *

try:
    Robot.setupSonar()
    while True:
        try:
            print(BP.get_sensor(SONAR_PORT))
        except brickpi3.SensorError as error:
            print(error)
        sleep(0.1)

except KeyboardInterrupt:
    pass
BP.reset_all()
