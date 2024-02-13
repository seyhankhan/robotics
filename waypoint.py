import math
import brickpi3
from time import sleep    # import the time library for the sleep function

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
ports = [BP.PORT_B, BP.PORT_C]
EPSILON = 10
def reset_encoders():
	try:
		BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder A
		BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder D
	except IOError as error:
		print(error)

def forward(distanceCM):
	power = 20.5 * distanceCM
	reset_encoders()
	targets = [(BP.get_motor_encoder(port) + power) for port in ports]
	for i in range(2):
		BP.set_motor_position(ports[i], targets[i])
	while True:
		if (abs(BP.get_motor_encoder(ports[0]) - targets[0]) < EPSILON
			and abs(BP.get_motor_encoder(ports[1]) - targets[1]) < EPSILON):
			break
		sleep(0.02)

def turn(rad):
	power = (480 / math.pi) * rad
	reset_encoders()
	targetB = BP.get_motor_encoder(BP.PORT_B) + power
	targetC = BP.get_motor_encoder(BP.PORT_C) - power
	BP.set_motor_position(BP.PORT_B, targetB)
	BP.set_motor_position(BP.PORT_C, targetC)
	while True:
		if (abs(BP.get_motor_encoder(BP.PORT_B) - targetB) < EPSILON
			and abs(BP.get_motor_encoder(BP.PORT_C) - targetC) < EPSILON):
			break
		sleep(0.02)

def navigateToWaypoint(targetX, targetY, position):
	vector = (targetX - position[0], targetY - position[1])
	distance = math.sqrt(vector[0] ** 2 + vector[1] ** 2)

	alpha = math.atan2(vector[1], vector[0])
	beta = alpha - position[2]
	if beta < -math.pi:
		beta += 2 * math.pi
	if beta > math.pi:
		beta -= 2 * math.pi
	turn(beta)
	forward(distance)
	return (targetX, targetY, alpha)

try:
	BP.set_motor_limits(BP.PORT_B, 20, 180)		  # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
	BP.set_motor_limits(BP.PORT_C, 20, 180)		  # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
	position = (0,0,0)
	while True:
		targetX = int(input("Target x: "))
		targetY = int(input("Target y: "))
		position = navigateToWaypoint(targetX, targetY, position)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
	BP.reset_all()  # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
		
finally:
	BP.reset_all()  