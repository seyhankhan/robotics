# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for running a motor to a target position set by the encoder of another motor.
# 
# Hardware: Connect EV3 or NXT motors to the BrickPi3 motor ports A and D. Make sure that the BrickPi3 is running on a 9v power supply.
#
# Results:  When you run this program, motor A will run to match the position of motor D. Manually rotate motor D, and motor A will follow.

# from __future__ import print_function # use python 3 syntax but make it compatible with python 2
# from __future__ import division       #                           ''

from time import sleep    # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import sys

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP.reset_all()  # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.


try:
	# reset_encoders()
	# BP.set_sensor_type configures the BrickPi3 for a specific sensor.
	BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.NXT_ULTRASONIC)
	while True:
		# read and display the sensor value
		# BP.get_sensor retrieves a sensor value.
		# BP.PORT_1 specifies that we are looking for the value of sensor port 1.
		# BP.get_sensor returns the sensor value (what we want to display).
		try:
			value = BP.get_sensor(BP.PORT_2)
			print(value)                         # print the distance in CM
		except brickpi3.SensorError as error:
			print(error)
		sleep(0.2)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
    
finally:
    BP.reset_all()  
