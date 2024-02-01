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

EPSILON = 10

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

ports = [BP.PORT_B, BP.PORT_C]


def reset_encoders():
    #reset_encoders motors
    try:
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder A
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder D
    except IOError as error:
        print(error)

def forward(deg):
    reset_encoders()
    targets = [(BP.get_motor_encoder(port) + deg) for port in ports]
    for i in range(2):
        BP.set_motor_position(ports[i], targets[i])
    while True:
        if (abs(BP.get_motor_encoder(ports[0]) - targets[0]) < EPSILON
            and abs(BP.get_motor_encoder(ports[1]) - targets[1]) < EPSILON):
            break
        sleep(0.02)

def turn(deg):
    reset_encoders()
    targetB = BP.get_motor_encoder(BP.PORT_B) + deg
    targetC = BP.get_motor_encoder(BP.PORT_C) - deg
    BP.set_motor_position(BP.PORT_B, targetB)
    BP.set_motor_position(BP.PORT_C, targetC)
    while True:
        if (abs(BP.get_motor_encoder(BP.PORT_B) - targetB) < EPSILON
            and abs(BP.get_motor_encoder(BP.PORT_C) - targetC) < EPSILON):
            break
        sleep(0.02)
    

try:
    # reset_encoders()
    BP.set_motor_limits(BP.PORT_B, 25, 200)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
    BP.set_motor_limits(BP.PORT_C, 25, 200)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)

    # BP.set_sensor_type configures the BrickPi3 for a specific sensor.
    BP.set_sensor_type(BP.PORT_2, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.TOUCH)

    
    
    # if len(sys.argv) > 1:
    #     forward(int(sys.argv[1]))
    # else:
    #     for i in range(4):
    #         forward(412 * 2)
    #         if i < 3:
    #             turn(-232)

    while True:

        # frwd
        BP.set_motor_dps(BP.PORT_B, 200)
        BP.set_motor_dps(BP.PORT_C, 200)

        try:
            senR = BP.get_sensor(BP.PORT_2)
            senL = BP.get_sensor(BP.PORT_3)
            
            if senL and senR:
                reset_encoders()
                forward(-150)
                turn(232)
                print("Both")
            elif senL:
                reset_encoders()
                forward(-150)
                turn(-232)
                print("left")
            elif senR:
                reset_encoders()
                forward(-150)
                turn(+232)
                print("right")


        except brickpi3.SensorError as error:
            print(error)
        
        sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.
    

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
    
finally:
    BP.reset_all()  
