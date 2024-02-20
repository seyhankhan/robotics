import brickpi3
import math
from time import sleep

BP = brickpi3.BrickPi3()

LEFT_WHEEL = BP.PORT_B
RIGHT_WHEEL = BP.PORT_C
SONAR_PORT = BP.PORT_3

SONAR_RELIABILITY_CEILING = 200
SONAR_MAX_ANGLE = math.radians(34)

EPSILON = 10
WHEEL_ROTATION_PER_METER = 2050
WHEEL_ROTATION_PER_RADIAN = 490.0 / math.pi
WHEELS = [LEFT_WHEEL, RIGHT_WHEEL]

# Unconfigure sensors, disable motors, restore LED to BP3 firmware's control
BP.reset_all()
try:
    # set power limit (%) and speed limit (Degrees/sec)
    for wheel in WHEELS:
        BP.set_motor_limits(wheel, 25, 200)
except KeyboardInterrupt:
    BP.reset_all()


class Robot:
    x, y, theta = (0, 0, 0)

    def __str__(self):
        return f"\t(x: {Robot.x:.1f}, y: {Robot.y:.1f}, theta: {math.degrees(Robot.theta):.1f}°/{Robot.theta:.2f}rad)"

    def setPosition(position):
        Robot.x, Robot.y, Robot.theta = position

    def setupSonar():
        BP.set_sensor_type(SONAR_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)

    def readSonar():
        while True:
            try:
                z = BP.get_sensor(SONAR_PORT)
                if z < SONAR_RELIABILITY_CEILING:
                    break
                print(f"UNRELIABLE SONAR: {z}cm")
            except brickpi3.SensorError as error:
                print(error)
            sleep(0.2)
        return z

    def forward(distanceCM):
        Robot.reset_encoders()
        wheelRotation = WHEEL_ROTATION_PER_METER * distanceCM / 100
        if wheelRotation == 0:
            return
        Robot._set_targets(
            [(BP.get_motor_encoder(wheel) + wheelRotation) for wheel in WHEELS]
        )
        Robot.x += distanceCM * math.cos(Robot.theta)
        Robot.y += distanceCM * math.sin(Robot.theta)

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
        Robot.theta = math.remainder(Robot.theta + radians, math.tau)

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

    def navigateToWaypoint(waypoint):
        vector = (waypoint[0] - Robot.x, waypoint[1] - Robot.y)
        alpha = math.atan2(vector[1], vector[0])
        beta = alpha - Robot.theta

        Robot.turn(math.remainder(beta, math.tau))
        Robot.forward(math.hypot(vector[0], vector[1]))
