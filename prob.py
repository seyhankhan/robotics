from time import sleep  # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import sys
import random
import math

EPSILON = 10

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

PX_PER_CM = 10

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

class Particle:
    def __init__(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.theta = pos[2]

    def changeDistance(self, D):
        sigmaE = 2
        sigmaF = math.pi / 90
        e = random.gauss(0, sigmaE)
        f = random.gauss(0, sigmaF)
        self.x = round(self.x + (D + e) * math.cos(self.theta), 2)
        self.y = round(self.y + (D + e) * math.sin(self.theta), 2)
        self.theta = round(self.theta + f, 2)

    def changeAngle(self, alpha):
        sigmaG = (math.pi / 90)
        g = random.gauss(0, sigmaG)
        self.theta = self.theta + alpha + g

    def __str__(self):
        return f"({self.x}, {self.y}, {self.theta})"
        
def drawParticles(particles):
    ps= list(map(lambda p: (p.x, p.y, p.theta), particles))
    print("drawParticles:" + str(ps))

try:
    BP.set_motor_limits(BP.PORT_B, 25, 200)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
    BP.set_motor_limits(BP.PORT_C, 25, 200)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)

    pos = (400, 500, 0)
    NUM_PARTICLES = 100 # 10cm is 100
    particles = [Particle(pos) for _ in range(NUM_PARTICLES)]
    drawParticles(particles)
    
    for _ in range(4):
        for _ in range(4):
            forward(200)

            (x, y, theta) = pos
            # move frwd
            for p in particles:
                p.changeDistance(10 * PX_PER_CM)
                
            xs = 0
            ys = 0            
            for p in particles:            
                xs += p.x
                ys += p.y
                
            xs /= NUM_PARTICLES
            ys /= NUM_PARTICLES

            # print
            print(f"drawLine:{(x, y, xs, ys)}")
            drawParticles(particles)

            pos = (xs, ys, theta)
            sleep(2)
            # wait for enter
            
        turn(-237)
        [p.changeAngle(-math.pi / 2) for p in particles]
        (x, y, _) = pos
        theta = sum(map(lambda p: p.theta, particles)) / NUM_PARTICLES
        pos = (x, y, theta)
        sleep(1)
        
            
    # print(f"drawLine:{(500, 400, 600, 400)}")
    # for particle in particles:
    #     print(f"drawParticles:" + str((particle.x, particle.y, particle.theta)))
    #     particle.changeDistance(100)
    #     print(f"drawParticles:" + str((particle.x, particle.y, particle.theta)))
    #     print("NEW:", particle.x)
        

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()  # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
        
finally:
    BP.reset_all()  
