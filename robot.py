import brickpi3
import math
from time import sleep

BP = brickpi3.BrickPi3()

LEFT_WHEEL = BP.PORT_B
RIGHT_WHEEL = BP.PORT_C
SENSOR_PORT = BP.PORT_3

EPSILON = 10
WHEEL_ROTATION_PER_METER = 2050
WHEEL_ROTATION_PER_RADIAN = 470 / math.pi

# Unconfigure sensors, disable motors, restore LED to BP3 firmware's control
BP.reset_all()

WHEELS = [LEFT_WHEEL, RIGHT_WHEEL]
try:
    # set power limit (%) and speed limit (Degrees/sec)
    for wheel in WHEELS:
        BP.set_motor_limits(wheel, 25, 200)
except KeyboardInterrupt:
    BP.reset_all()


class Robot:
    def forward(distanceCM):
        Robot.reset_encoders()
        wheelRotation = WHEEL_ROTATION_PER_METER * distanceCM / 100
        if wheelRotation == 0:
            return
        Robot._set_targets(
            [(BP.get_motor_encoder(wheel) + wheelRotation) for wheel in WHEELS]
        )

    def turn(radians):
        Robot.reset_encoders()
        wheelRotation = WHEEL_ROTATION_PER_RADIAN * radians
        if wheelRotation == 0:
            return
        Robot._set_targets(
            [
                BP.get_motor_encoder(LEFT_WHEEL) - wheelRotation,
                BP.get_motor_encoder(RIGHT_WHEEL) + wheelRotation,
            ]
        )

    def reset_encoders():
        try:
            for wheel in WHEELS:
                BP.offset_motor_encoder(wheel, BP.get_motor_encoder(wheel))
        except IOError as error:
            print(error)

    def _set_targets(targets):
        for i in range(2):
            BP.set_motor_position(WHEELS[i], targets[i])
        while True:
            if all(
                abs(BP.get_motor_encoder(WHEELS[i]) - targets[i]) < EPSILON
                for i in range(2)
            ):
                break
            sleep(0.02)
