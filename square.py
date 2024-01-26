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

def reset_encoders():
    #reset_encoders motors
    try:
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder A
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder D
    except IOError as error:
        print(error)

def forward(deg):
    reset_encoders()
    targetB = BP.get_motor_encoder(BP.PORT_B) + deg
    targetC = BP.get_motor_encoder(BP.PORT_C) + deg
    BP.set_motor_position(BP.PORT_B, targetB)
    BP.set_motor_position(BP.PORT_C, targetC)
    while True:
        if (abs(BP.get_motor_encoder(BP.PORT_B) - targetB) < 5
            and abs(BP.get_motor_encoder(BP.PORT_C) - targetC) < 5):
            break
        sleep(0.02)

def turn(deg):
    reset_encoders()
    targetB = BP.get_motor_encoder(BP.PORT_B) + deg
    targetC = BP.get_motor_encoder(BP.PORT_C) - deg
    BP.set_motor_position(BP.PORT_B, targetB)
    BP.set_motor_position(BP.PORT_C, targetC)
    while True:
        if (abs(BP.get_motor_encoder(BP.PORT_B) - targetB) < 5
            and abs(BP.get_motor_encoder(BP.PORT_C) - targetC) < 5):
            break
        sleep(0.02)
    

try:
    # reset_encoders()

    #BP.set_motor_power(BP.PORT_C, BP.MOTOR_FLOAT)    # float motor D
    BP.set_motor_limits(BP.PORT_B, 25, 200)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
    BP.set_motor_limits(BP.PORT_C, 25, 200)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
    
    # forward(int(sys.argv[1]))
    for i in range(1):
        forward(400)
        turn(285)


    # sleep(3)

    
    print('Doasdfasdfne')

    
    '''while True:
        # Each of the following BP.get_motor_encoder functions returns the encoder value.
        try:
            target = BP.get_motor_encoder(BP.PORT_C) # read motor D's position
        except IOError as error:
            print(error)
        
        BP.set_motor_position(BP.PORT_B, target)    # set motor A's target position to the current position of motor D
        
        try:
            print("Motor A target: %6d  Motor A position: %6d" % (target, BP.get_motor_encoder(BP.PORT_B)))
        except IOError as error:
            print(error)
        
        time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.'''
        
    

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
    
finally:
    BP.reset_all()  
