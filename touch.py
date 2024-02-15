import brickpi3
import math
from time import sleep
from robot import *

try:
    BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.TOUCH)

    while True:
        try:
            senL = BP.get_sensor(BP.PORT_3)
            senR = BP.get_sensor(BP.PORT_2)

            Robot.reset_encoders()
            Robot.forward(-150)
            if senL and senR:
                Robot.turn(math.pi / 2)
                print("Both")
            elif senL:
                Robot.turn(-math.pi / 2)
                print("left")
            elif senR:
                Robot.turn(math.pi / 2)
                print("right")
        except brickpi3.SensorError as error:
            print(error)
        sleep(0.02)

except KeyboardInterrupt:
    pass
BP.reset_all()
