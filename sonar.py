import brickpi3
from time import sleep
from robot import *

try:
    BP.set_sensor_type(SENSOR_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    while True:
        try:
            print(BP.get_sensor(SENSOR_PORT))
        except brickpi3.SensorError as error:
            print(error)
        sleep(0.1)

except KeyboardInterrupt:
    pass
BP.reset_all()
